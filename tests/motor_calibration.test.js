// 执行真实C函数的有界分支检查；不代替Keil编译、硬件Flash时序或实机运动验证。

/** 函数作用：把被测C函数转为JavaScript。source为源码，name为函数名。 */
function calibrationExtract(source, name) {
    const match = new RegExp('(?:static\\s+)?(?:void|u8|u16|u32)\\s+' + name + '\\s*\\(([^)]*)\\)\\s*\\{').exec(source); // 函数定义位置。
    if (!match) throw new Error('缺少功能函数：' + name);
    let end = match.index + match[0].length; // 函数体扫描位置。
    let depth = 1; // 花括号嵌套深度。
    while (depth && end < source.length) { if (source[end] === '{') depth++; if (source[end] === '}') depth--; end++; }
    const args = match[1].replace(/\b(?:const|void|u8|u16|u32)\b|\*/g, '').trim(); // 去除C类型后的参数。
    let body = source.slice(match.index + match[0].length, end - 1) // 被测函数体。
        .replace(/^\s*#(?:if.*|endif)\s*$/gm, '')
        .replace(/\((?:u8|u16|u32|unsigned long)(?:\s*\*)?\)/g, '')
        .replace(/\bu(?:8|16|32)\s+(\w+)\[([^\]]+)\];/g, 'let $1 = Array($2).fill(0);')
        .replace(/\b(?:u8|u16|u32|FLASH_Status)\s+([^;]+);/g, 'let $1;')
        .replace(/\bsizeof\((?:record|verify)\)/g, '36')
        .replace(/record\[(\d+)\] \* 10 \/ (\w+)/g, 'Math.trunc(record[$1] * 10 / $2)')
        .replace(/Mot(\d)\.MaxPulseNum \/ 2/g, 'Math.trunc(Mot$1.MaxPulseNum / 2)')
        .replace(/Mot2\.PulseNum \* c_PulseScaleFactor \/ c_MaxHeight/g, 'Math.trunc(Mot2.PulseNum * c_PulseScaleFactor / c_MaxHeight)')
        .replace(/>>/g, '>>>'); // 有界检查保留无符号右移及原控制分支。
    if (/\bu32\s/.test(match[0])) body = body.replace(/return\s+([^;]+);/g, 'return ($1) >>> 0;'); // 模拟32位无符号返回值。
    return 'function ' + name + '(' + args + ') {' + body + '}';
}

/** 函数作用：检查断电保存、记录校验和快速归位。input含当前C源码和电机头文件。 */
function runMotorCalibrationTests(input) {
    const sources = [input.header, input.motor].join('\n').replace(/\/\/[^\n]*/g, ''); // 电机管理常量来源，先解析头文件依赖。
    const macros = Array.from(sources.matchAll(/^#define\s+(\w+)\s+([^\r\n]+)/gm))
        .filter(entry => /^(?:Motor[123]Enable|MotorCalibration\w+|Motor2CalibrationHeight|Mot[13]MaxAngle|C_VerifyMinimumPulseNum)$/.test(entry[1]))
        .map(entry => 'const ' + entry[1] + ' = ' + entry[2].replace(/(0x[\da-f]+|\d+)[uUlL]+\b/gi, '$1') + ';').join('\n'); // 数值及启用掩码的真实定义。
    const functions = ['MotorCalibrationChecksum', 'MotorCalibrationValidate', 'MotorCalibrationLoad', 'MotorCalibrationSave', 'MotorCalibrationService']; // 被测持久化逻辑。
    const globals = Array.from(input.motor.matchAll(/^(?:static\s+)?u8\s+(Motor\w+)\s*(?:=\s*([^;]+))?;/gm), entry => 'let ' + entry[1] + ' = ' + (entry[2] || '0') + ';').join('\n'); // 电机管理状态初值。
    const makeStore = new Function('Mot1', 'Mot2', 'Mot3', 'io', macros + globals + `
        const FLASH_COMPLETE = 0, motor_stop = 0, NoFault = 0; // 驱动成功状态及无故障停机状态。
        const FLASH_FLAG_EOP = 1, FLASH_FLAG_PGERR = 2, FLASH_FLAG_WRPRTERR = 4; // 测试用Flash标志。
        function printf() {} // 忽略串口输出。
        function Flash_Read(address, record, length) { if (address !== 0x0803F800 || length !== 36) throw Error('读取区域错误'); for (let i=0;i<9;i++) record[i]=io.words[i]; } // 模拟断电后仍存在的存储内容。
        function FLASH_Unlock() { io.locked = false; } // 模拟解锁。
        function FLASH_Lock() { io.locked = true; } // 模拟上锁。
        function FLASH_ClearFlag() {} // 模拟状态清除。
        function FLASH_ErasePage(address) { if (address !== 0x0803F800) throw Error('擦除区域错误'); io.erases++; if (io.eraseFails) return 1; io.words.fill(0xFFFFFFFF); return 0; } // 仅替代硬件擦除。
        function FLASH_ProgramWord(address, value) { const index=(address-0x0803F800)/4; io.writes.push(index); if (io.failAt===index) return 1; if (!io.dropWrite) io.words[index]=value>>>0; return 0; } // 模拟写入失败和读回错误。
    ` + functions.map(name => calibrationExtract(input.motor, name)).join('\n') + '; return { load: MotorCalibrationLoad, save: MotorCalibrationSave, service: MotorCalibrationService, loaded: () => MotorCalibrationLoaded };'); // 真实存储函数执行环境。
    const failures = []; // 失败用例描述。
    let passed = 0; // 通过用例数量。
    /** 函数作用：运行一个用例。name为场景名，body为测试逻辑。 */
    function check(name, body) { try { body(); passed++; } catch (error) { failures.push(name + '：' + error.message); } }
    /** 函数作用：核对预期值。actual为实际值，expected为人工推导的预期。 */
    function equal(actual, expected) { if (JSON.stringify(actual) !== JSON.stringify(expected)) throw Error('实际 ' + JSON.stringify(actual) + '，期望 ' + JSON.stringify(expected)); }
    /** 函数作用：建立新上电环境。io为可跨重启复用的硬件存储。 */
    function boot(io) {
        const motors = [{ MaxPulseNum: 3600, PulseNumPerAngle: 200 }, { MaxPulseNum: 3000, PulseNumPerMm: 200 }, { MaxPulseNum: 3600, PulseNumPerAngle: 200 }]; // 初始化默认值。
        motors.forEach(motor => Object.assign(motor, { Reset: 1, En: 0, Faultflag: 0, Action: 0 }));
        io = io || { words: Array(9).fill(0xFFFFFFFF), writes: [], erases: 0, locked: true }; // 外部存储在重启时保留。
        return { motors, io, control: makeStore(...motors, io) };
    }
    /** 函数作用：模拟三路实测自检成功。state为当前上电环境。 */
    function calibrated(state) { state.motors.forEach(motor => Object.assign(motor, { Reset: 0, En: 1 })); Object.assign(state.motors[0], { MaxPulseNum: 7200, PulseNumPerAngle: 400 }); Object.assign(state.motors[1], { MaxPulseNum: 51000, PulseNumPerMm: 1000 }); Object.assign(state.motors[2], { MaxPulseNum: 2600, PulseNumPerAngle: 400 }); }
    check('空白Flash保留首次自检', () => { const state=boot(); state.control.load(); equal(state.control.loaded(),0); equal(state.motors.map(motor=>motor.Reset),[1,1,1]); });
    check('独立CRC32样本可恢复实测标定', () => { const state=boot(); state.io.words=[0x4D43414C,0x00010007,7200,400,51000,1000,2600,400,0xB89C33A6]; state.control.load(); equal(state.control.loaded(),1); equal(state.motors.map(motor=>motor.MaxPulseNum),[7200,51000,2600]); });
    check('自检未完成不写Flash', () => { const state=boot(); state.control.service(); equal(state.io.erases,0); });
    check('任一路故障不写Flash', () => { const state=boot(); calibrated(state); state.motors[2].Faultflag=4; state.control.service(); equal(state.io.erases,0); });
    check('成功只保存一次且最后提交有效标记', () => { const state=boot(); calibrated(state); state.control.service(); state.control.service(); equal(state.io.erases,1); equal(state.io.writes,[1,2,3,4,5,6,7,8,0]); equal(state.io.locked,true); });
    check('第二次上电恢复三路标定并继续归位', () => { const state=boot(); calibrated(state); state.control.service(); const next=boot(state.io); next.control.load(); equal(next.control.loaded(),1); equal(next.motors.map(motor=>motor.MaxPulseNum),[7200,51000,2600]); equal(next.motors.map(motor=>motor.Reset),[1,1,1]); next.control.service(); equal(next.io.erases,1); });
    check('非整除实测脉冲按C整数规则保存恢复', () => { const state=boot(); calibrated(state); state.motors.forEach(motor=>motor.MaxPulseNum++); state.control.service(); const next=boot(state.io); next.control.load(); equal(next.control.loaded(),1); equal(next.motors.map(motor=>motor.MaxPulseNum),[7201,51001,2601]); });
    check('数据位损坏被校验拒绝', () => { const state=boot(); calibrated(state); state.control.service(); state.io.words[4]=51001; const next=boot(state.io); next.control.load(); equal(next.control.loaded(),0); });
    check('CRC本身损坏被拒绝', () => { const state=boot(); calibrated(state); state.control.service(); state.io.words[8]^=1; const next=boot(state.io); next.control.load(); equal(next.control.loaded(),0); });
    check('版本变化回退完整自检', () => { const state=boot(); calibrated(state); state.control.service(); state.io.words[1]=0x00020007; const next=boot(state.io); next.control.load(); equal(next.control.loaded(),0); });
    check('写入中断不留下成功标记', () => { const state=boot(); calibrated(state); state.io.failAt=5; state.control.service(); const next=boot(state.io); next.control.load(); equal(next.control.loaded(),0); equal(state.io.words[0],0xFFFFFFFF); equal(state.io.locked,true); });
    check('读回错误不提交成功标记', () => { const state=boot(); calibrated(state); state.io.dropWrite=true; state.control.service(); equal(state.io.words[0],0xFFFFFFFF); equal(state.io.locked,true); });
    check('擦除失败只尝试一次', () => { const state=boot(); calibrated(state); state.io.eraseFails=true; state.control.service(); state.control.service(); equal(state.io.erases,1); equal(state.io.writes,[]); equal(state.io.locked,true); });
    check('零换算系数不能保存', () => { const state=boot(); calibrated(state); state.motors[1].PulseNumPerMm=0; state.control.service(); equal(state.io.erases,0); });
    for (let axis=1; axis<=3; axis++) {
        const source=input.axes[axis-1]; // 对应电机的实际自检状态机。
        const service=calibrationExtract(source,'Mot'+axis+'MoveService'); // 保留生产状态机分支。
        const environment = 'let MotorCalibrationLoaded = loaded; const Motor1Enable=1, Motor2Enable=1; const c_MoveTimeOver='+([20000,40000,30000][axis-1])+'; const c_PulseScaleFactor=10, c_MaxAngle='+ (axis===1?180:65) +', c_MaxHeight=510, C_VerifyMinimumPulseNum=20; const motor_stop=0, motor_brake=1, motor_forward=2, motor_backward=3, hall_backward_brake=3, hall_forward_brake=1, pulse_forward_brake=0, pulse_backward_brake=2, Hall1Fault=1, Hall2Fault=2, MotorFault=4; function printf(){} function delay_ms(){}'; // 状态机所用真实枚举值。
        const setup = ['LeftHallScan','RightHallScan','UpHallScan','DownHallScan','ActionHandle','MoveTask'].map(name=>'function Mot'+axis+name+'() {}').join('\n') + 'function Mot'+axis+'ActionState(){return Mot'+axis+'.Action;} function Mot'+axis+'ExcuteTime(){return Mot'+axis+'.PidTimerB;} function Mot'+axis+'SetCurrentPosition(position){Mot'+axis+'.CurPosition=position;} function Mot'+axis+'MoveForward(position,speed){io.moves.push(["forward",position]); Mot'+axis+'.Action=2;} function Mot'+axis+'MoveBackward(position,speed){io.moves.push(["backward",position]); Mot'+axis+'.Action=3;} function Mot'+axis+'MoveStop(){io.stopped=true;Mot'+axis+'.Action=0;}'; // 仅代替动作硬件及传感器输入。
        const factory=new Function('Mot1','Mot2','Mot3','io','loaded',environment+setup+service+'; return Mot'+axis+'MoveService;'); // 实际归位函数环境。
        /** 函数作用：建立指定电机的归位场景。loaded为保存记录是否有效。 */
        function homing(loaded) { const motors=[{Reset:0},{Reset:0},{Reset:0}]; const motor=motors[axis-1]; Object.assign(motor,{Reset:1,En:0,Step:1,Action:0,BrakeType:3,MaxPulseNum:axis===2?51000:7200,PulseNum:100,PidTimerB:0,Faultflag:0,Hall1Sta:0,Hall2Sta:0}); const io={moves:[]}; return {motor,io,motors,run:factory(...motors,io,loaded)}; }
        check('MOT'+axis+'已保存时只执行归位',()=>{const state=homing(1); state.run(); equal(state.io.moves,axis===2?[]:[['forward',3600]]); equal(state.motor.Step,axis===2?1:3); if(axis===2)equal([state.motor.Reset,state.motor.En,state.motor.CurPosition],[0,1,0]);});
        check('MOT'+axis+'首次启动仍标定另一端',()=>{const state=homing(0);state.run();equal(state.io.moves,[['forward',20000]]);equal(state.motor.Step,2);});
        check('MOT'+axis+'没有确认限位不接受归位',()=>{const state=homing(1);state.motor.BrakeType=2;state.run();equal(state.motor.En,0);equal(state.io.moves,[]);equal(state.motor.Faultflag,1);});
        check('MOT'+axis+'最终归位超时关闭PWM',()=>{const state=homing(1);state.motor.Step=3;state.motor.Action=2;state.motor.PidTimerB=40001;state.run();equal(state.io.stopped,true);equal(state.motor.En,0);equal(state.motor.Faultflag,4);});
        check('MOT'+axis+'最终归位成功才使能',()=>{const state=homing(1);state.motor.Step=3;state.motor.BrakeType=axis===2?3:0;state.run();equal([state.motor.Reset,state.motor.En],[0,1]);});
        check('MOT'+axis+'最终被错误限位打断不使能',()=>{const state=homing(1);state.motor.Step=3;state.motor.BrakeType=1;state.run();equal(state.motor.En,0);equal(state.motor.Faultflag,4);});
        check('MOT'+axis+'首次完整自检归位保持成功',()=>{const state=homing(0);state.motor.Step=3;state.motor.BrakeType=axis===2?3:2;state.run();equal([state.motor.Reset,state.motor.En],[0,1]);});
        if (axis!==2) {
            for (const loaded of [0,1]) { // 分别覆盖首次反向回中与历史数据正向回中。
            check('MOT'+axis+(loaded?'快速':'首次')+'目标脉冲与端点霍尔同时到达不接受回中',()=>{
                const state=homing(loaded); // 构造同时满足脉冲和霍尔停机的场景。
                const stopTicks=Number(new RegExp('^#define Mot'+axis+'StopDebTicks\\s+(\\d+)','m').exec(source)?.[1] || 1000); // 兼容原版本与工作区的停稳计数定义。
                Object.assign(state.motor,{Step:3,Action:loaded?2:3,PulseNum:3600,TarPulseNum:3600,TarPosition:3600,CurPosition:loaded?0:7200,Hall1Sta:loaded?0:1,Hall2Sta:loaded?1:0,PulseDebB:stopTicks,PulseNumPerAngle:400});
                const hallNames=axis===1?['ReadLeftHallState','ReadRightHallState']:['ReadUpHallState','ReadDownHallState']; // 实际动作函数依赖的霍尔接口。
                const realAction=hallNames.map(name=>calibrationExtract(source,'Mot'+axis+name)).join('\n')+calibrationExtract(source,'Mot'+axis+'ActionHandle'); // 执行真实动作停机分支。
                const legacyConstants=Array.from(source.replace(/\/\/[^\n]*/g,'').matchAll(/^#define\s+(Mot[13](?:PosCorr\w+|StopDebTicks))\s+([^\r\n]+)/gm),entry=>'const '+entry[1]+'='+entry[2]+';').join('\n'); // 原版本动作处理可能依赖的停稳与位置补偿常量。
                const legacyGlobals=Array.from(source.matchAll(/^static\s+u(?:8|16|32)\s+(Mot[13]PosCorr\w+)\s*=\s*([^;]+);/gm),entry=>'let '+entry[1]+'='+entry[2]+';').join('\n'); // 原版本位置补偿状态初值。
                const legacyFunctions=new RegExp('static void Mot'+axis+'PositionCorrectionHandle\\(void\\)\\s*\\{').test(source)?calibrationExtract(source,'Mot'+axis+'AngleToPulse')+calibrationExtract(source,'Mot'+axis+'PositionCorrectionHandle'):''; // 存在位置补偿时执行真实函数，覆盖干净提交版本。
                const realFactory=new Function('Mot1','Mot2','Mot3','io','loaded',environment+setup+' const C_MotorStopDeb=1000,c_AngleScaleFactor=10; function Mot'+axis+'VelocityControl(){} function Mot'+axis+'DisablePidControl(){io.pwm=0;} '+legacyConstants+legacyGlobals+legacyFunctions+realAction+service+';return Mot'+axis+'MoveService;'); // 只替代速度计算及硬件PWM输出。
                const run=realFactory(...state.motors,state.io,loaded); // 联合运行真实动作处理和归位状态机。
                run(); run();
                equal(state.motor.En,0); equal(state.motor.Faultflag,4);
            });
            }
        } else {
            check('MOT2实测系数会截断为零时不能继续自检',()=>{const state=homing(0);state.motor.Step=2;state.motor.BrakeType=1;state.motor.PulseNum=50;state.run();equal(state.motor.Faultflag,4);equal(state.io.moves,[]);});
        }
    }
    if(failures.length)throw Error(failures.join('\n'));
    return {passed,scope:'真实持久化和复位状态机；未模拟C整数溢出、Flash硬件时序或机械行为'};
}

// 有Node.js时可直接运行本文件；当前工具也可调用runMotorCalibrationTests执行。
if(typeof require!=='undefined' && require.main===module){
    const fs=require('node:fs'); // 文件读取接口。
    const path=require('node:path'); // 路径拼接接口。
    const root=path.resolve(__dirname,'..'); // 工程根目录。
    const decoder=new TextDecoder('gbk'); // 工程C源码编码。
    /** 函数作用：读取被测源码。name为相对工程路径。 */
    function read(name){return decoder.decode(fs.readFileSync(path.join(root,name)));}
    console.log(runMotorCalibrationTests({motor:read('USER/Sourse/motor.c'),header:read('USER/Include/motor.h'),axes:[1,2,3].map(axis=>read('USER/Sourse/mot'+axis+'.c'))}));
}
