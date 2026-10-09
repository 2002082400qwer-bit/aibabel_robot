// 执行真实C函数的有界行为检查，不代替Keil编译和硬件验证。
/** 函数作用：提取实际C函数；source为源码，name为函数名。 */
function positionExtract(source,name) {
    const match=new RegExp('(?:static\\s+)?(?:void|u8|u16|u32)\\s+'+name+'\\s*\\(([^)]*)\\)\\s*\\{').exec(source); // 检查场景的输入、状态或执行环境。
    if(!match) throw Error('缺少功能函数：'+name);
    let end=match.index+match[0].length,depth=1; // 检查场景的输入、状态或执行环境。
    while(depth && end<source.length){if(source[end]==='{')depth++;if(source[end]==='}')depth--;end++;}
    const args=match[1].replace(/\b(?:const|void|u8|u16|u32|FlashCalibrationData|FlashMotorPositionData)\b|\*/g,'').trim(); // 检查场景的输入、状态或执行环境。
    let body=source.slice(match.index+match[0].length,end-1) // 检查场景的输入、状态或执行环境。
      .replace(/^\s*#(?:if.*|endif)\s*$/gm,'')
      .replace(/\*\(__IO\s+u32\s*\*\)\(([^;]+)\)/g,'FlashReadWord($1)')
      .replace(/\((?:u8|u16|u32|unsigned long)(?:\s*\*)?\)/g,'')
      .replace(/\bFlashCalibrationData\s+(\w+);/g,'let $1={MaxPulseNum:[0,0,0],PulseScale:[0,0,0]};') // 检查场景的输入、状态或执行环境。
      .replace(/\bFlashMotorPositionData\s+(\w+);/g,'let $1={Motor1Pulse:0,Motor3Pulse:0};') // 检查场景的输入、状态或执行环境。
      .replace(/->/g,'.').replace(/&(calibration|position)\b/g,'$1')
      .replace(/\b(calibration|position|data) == 0/g,'$1 == null')
      .replace(/\bu(?:8|16|32)\s+(\w+)\[([^\]]+)\];/g,'let $1=Array($2).fill(0);') // 检查场景的输入、状态或执行环境。
      .replace(/\b(?:u8|u16|u32|FLASH_Status)\s+([^;]+);/g,'let $1;') // 检查场景的输入、状态或执行环境。
      .replace(/calibration\.MaxPulseNum\[(\d+)\] \* 10 \/ (\w+)/g,'Math.trunc(calibration.MaxPulseNum[$1] * 10 / $2)')
      .replace(/Mot(\d)\.MaxPulseNum\s*\/\s*2/g,'Math.trunc(Mot$1.MaxPulseNum / 2)')
      .replace(/Mot(\d)\.CurPosition \* 100 \/ Mot\1\.PulseNumPerAngle/g,'Math.trunc(Mot$1.CurPosition * 100 / Mot$1.PulseNumPerAngle)')
      .replace(/>>/g,'>>>');
    if(/\bu32\s/.test(match[0]))body=body.replace(/return\s+([^;]+);/g,'return ($1) >>> 0;');
    return 'function '+name+'('+args+'){'+body+'}';
}
/** 函数作用：运行位置日志、断电恢复和启动状态机检查；input包含工程源码。 */
function runMotorPositionTests(input){
    const failures=[]; let passed=0; // 检查场景的输入、状态或执行环境。
    function equal(a,b){if(JSON.stringify(a)!==JSON.stringify(b))throw Error('实际'+JSON.stringify(a)+'，期望'+JSON.stringify(b));}
    function check(name,fn){try{fn();passed++;}catch(e){failures.push(name+'：'+e.message);}}
    let factory; // 检查场景的输入、状态或执行环境。
    try{
      const sources=[input.header,input.flashHeader,input.flash,input.motor].join('\n').replace(/\/\/[^\n]*/g,''); // 检查场景的输入、状态或执行环境。
      const macros=Array.from(sources.matchAll(/^#define\s+(\w+)\s+([^\r\n]+)/gm)).filter(m=>/^(?:Flash\w+|Motor[123]Enable|MotorCalibrationMask|Motor2CalibrationHeight|Mot[13]MaxAngle|C_VerifyMinimumPulseNum)$/.test(m[1])).map(m=>'const '+m[1]+'='+m[2]+';').join('\n'); // 检查场景的输入、状态或执行环境。
      const globals=Array.from(input.motor.matchAll(/^(?:static\s+)?(?:u8|uint8_t)\s+(Motor\w+)\s*(?:=\s*([^;]+))?;/gm),m=>'let '+m[1]+'='+(m[2]||'0')+';').join('\n'); // 检查场景的输入、状态或执行环境。
      const flashNames=Array.from(input.flash.matchAll(/^(?:static\s+)?(?:void|u8|u16|u32)\s+(\w+)\(/gm),m=>m[1]); // 检查场景的输入、状态或执行环境。
      const motorNames=['MotorCalibrationValidate','MotorCalibrationLoad','MotorCalibrationSave','MotorCalibrationService','MotorPositionLoad','MotorPositionStopped','MotorPositionPrepareMove','MotorPositionService','MotorInit']; // 检查场景的输入、状态或执行环境。
      factory=new Function('Mot1','Mot2','Mot3','Move1','Move2','Move3','io',macros+globals+String.raw`
        const FLASH_COMPLETE=0,FLASH_FLAG_EOP=1,FLASH_FLAG_PGERR=2,FLASH_FLAG_WRPRTERR=4,motor_stop=0,NoFault=0; // 检查场景的输入、状态或执行环境。
        function printf(){} function Mot1Init(){} function Mot2Init(){} function Mot3Init(){}
        function FlashReadWord(address){if(address<0x0803F000||address>=0x08040000||address%4)throw Error('读取越界');return io.words[(address-0x0803F000)/4];}
        function FLASH_Unlock(){io.locked=false;} function FLASH_Lock(){io.locked=true;} function FLASH_ClearFlag(){}
        function FLASH_ErasePage(address){io.ops.push(['erase',address]);if(io.locked)throw Error('写保护未解锁');if(io.eraseFail)return 1;if(address!==0x0803F000&&address!==0x0803F800)throw Error('擦除越界');io.words.fill(0xFFFFFFFF,(address-0x0803F000)/4,(address-0x0803F000)/4+512);return 0;}
        function FLASH_ProgramWord(address,value){io.ops.push(['write',address,value>>>0]);if(io.locked)throw Error('写保护未解锁');if(io.failAddress===address)return 1;const index=(address-0x0803F000)/4;if(index<0||index>=1024||!Number.isInteger(index))throw Error('写入越界');if((io.words[index]&(value>>>0))!==(value|0))throw Error('禁止0写1');if(!io.dropWrite)io.words[index]=(io.words[index]&value)>>>0;return io.failAfterWriteAddress===address?1:0;} // 检查场景的输入、状态或执行环境。
      `+flashNames.map(n=>positionExtract(input.flash,n)).join('\n')+motorNames.map(n=>positionExtract(input.motor,n)).join('\n')+`;return {init:MotorInit,read:FlashMotorPositionRead,write:FlashMotorPositionWrite,invalidate:FlashMotorPositionInvalidate,prepare:MotorPositionPrepareMove,service:MotorPositionService,calService:MotorCalibrationService,calWrite:FlashCalibrationWrite,loaded:()=>MotorPositionLoaded,calLoaded:()=>MotorCalibrationLoaded};`);
    }catch(e){return {passed,failures:[e.message]};}
    function boot(io){
      io=io||{words:Array(1024).fill(0xFFFFFFFF),ops:[],locked:true};
      const motors=[{MaxPulseNum:7200,PulseNumPerAngle:400},{MaxPulseNum:51000,PulseNumPerMm:1000},{MaxPulseNum:2600,PulseNumPerAngle:400}]; // 检查场景的输入、状态或执行环境。
      motors.forEach(m=>Object.assign(m,{Reset:1,En:0,Action:0,Faultflag:0,CurPosition:0,CurAngle:0}));
      const moves=[{En:0},{En:0},{En:0}],control=factory(...motors,...moves,io); // 检查场景的输入、状态或执行环境。
      control.init();return {io,motors,moves,control};
    }
    function stopped(s){s.motors.forEach(m=>Object.assign(m,{Reset:0,En:1,Action:0}));s.motors[0].CurPosition=1200;s.motors[2].CurPosition=2000;}
    function saved(){const s=boot();stopped(s);s.control.calService();s.control.service();return s;} // 检查场景的输入、状态或执行环境。
    function read(s){const d={Motor1Pulse:99,Motor3Pulse:99};const ok=s.control.read(d,7);return [ok,d.Motor1Pulse,d.Motor3Pulse];} // 检查场景的输入、状态或执行环境。
    check('首次成功保存位置并重启恢复坐标',()=>{const s=saved(),n=boot(s.io);equal([n.control.calLoaded(),n.control.loaded(),n.motors[0].CurPosition,n.motors[2].CurPosition],[1,1,1200,2000]);equal([n.motors[0].CurAngle,n.motors[2].CurAngle],[300,500]);}); // 检查场景的输入、状态或执行环境。
    check('空白位置仍保留校准而不信任坐标',()=>{const s=boot();stopped(s);s.control.calService();const n=boot(s.io);equal([n.control.calLoaded(),n.control.loaded()],[1,0]);}); // 检查场景的输入、状态或执行环境。
    check('运动前失效一次且多轴运动不重复写',()=>{const s=saved(),before=s.io.ops.length;equal(s.control.prepare(),1);equal(read(s)[0],0);equal(s.control.prepare(),1);equal(s.io.ops.length-before,1);}); // 检查场景的输入、状态或执行环境。
    check('运动中断电只回退位置且保留行程',()=>{const s=saved();s.control.prepare();const n=boot(s.io);equal([n.control.calLoaded(),n.control.loaded()],[1,0]);}); // 检查场景的输入、状态或执行环境。
    check('停稳后追加新位置并且不擦校准页',()=>{const s=saved();s.control.prepare();s.motors[0].CurPosition=5000;s.motors[2].CurPosition=800;s.control.service();equal(read(s),[1,5000,800]);equal(s.io.ops.filter(o=>o[0]==='erase').map(o=>o[1]),[0x0803F800]);equal(boot(s.io).control.loaded(),1);}); // 检查场景的输入、状态或执行环境。
    check('一直停稳不重复保存',()=>{const s=saved(),count=s.io.ops.length;for(let i=0;i<10;i++)s.control.service();equal(s.io.ops.length,count);}); // 检查场景的输入、状态或执行环境。
    check('运动中及制动等待期间不写位置',()=>{for(const action of [1,2,3]){const s=saved();s.control.prepare();s.motors[1].Action=action;const count=s.io.ops.length;s.control.service();equal(s.io.ops.length,count);}}); // 检查场景的输入、状态或执行环境。
    check('未完成归位或故障或待执行任务不保存',()=>{for(const change of [s=>s.motors[0].Reset=1,s=>s.motors[2].En=0,s=>s.motors[1].Faultflag=4,s=>s.moves[2].En=1]){const s=saved();s.control.prepare();change(s);const count=s.io.ops.length;s.control.service();equal(s.io.ops.length,count);}}); // 检查场景的输入、状态或执行环境。
    check('失效写失败禁止运动且保存标记不误清RAM',()=>{const s=saved();s.io.failAddress=0x0803F000;equal(s.control.prepare(),0);equal(s.io.locked,true);equal(read(s)[0],1);s.io.failAddress=undefined;equal(s.control.prepare(),1);equal(read(s)[0],0);}); // 检查场景的输入、状态或执行环境。
    check('失效读回失败也禁止运动',()=>{const s=saved();s.io.dropWrite=true;equal(s.control.prepare(),0);equal(s.io.locked,true);}); // 检查场景的输入、状态或执行环境。
    check('最新失效记录不回退较老有效坐标',()=>{const s=saved();s.control.write({Motor1Pulse:600,Motor3Pulse:700},7);s.control.invalidate();equal(read(s)[0],0);equal(boot(s.io).control.loaded(),0);}); // 检查场景的输入、状态或执行环境。
    check('最新未提交记录不回退旧坐标',()=>{const s=saved();s.io.failAddress=0x0803F014;s.control.write({Motor1Pulse:600,Motor3Pulse:700},7);equal(read(s)[0],0);equal(boot(s.io).control.loaded(),0);}); // 检查场景的输入、状态或执行环境。
    check('任一数据字写入中断不回退旧坐标',()=>{for(const offset of [4,8,12,16]){const s=saved();s.io.failAddress=0x0803F014+offset;equal(s.control.write({Motor1Pulse:600,Motor3Pulse:700},7),0);equal(boot(s.io).control.loaded(),0);}}); // 检查场景的输入、状态或执行环境。
    check('位置CRC或版本或组合错误拒绝恢复',()=>{for(const word of [1,2,3,4]){const s=saved();s.io.words[word]^=1;equal(boot(s.io).control.loaded(),0);}}); // 检查场景的输入、状态或执行环境。
    check('CRC有效但越界坐标拒绝恢复',()=>{for(const d of [{Motor1Pulse:7201,Motor3Pulse:700},{Motor1Pulse:600,Motor3Pulse:2601}]){const s=saved();s.control.write(d,7);equal(boot(s.io).control.loaded(),0);}}); // 检查场景的输入、状态或执行环境。
    check('位置日志满后只擦位置页并恢复新坐标',()=>{const s=saved();for(let i=1;i<103;i++){s.control.prepare();s.motors[0].CurPosition=i;s.control.service();}equal(s.io.ops.filter(o=>o[0]==='erase').map(o=>o[1]),[0x0803F800,0x0803F000]);equal(read(s),[1,102,2000]);}); // 检查场景的输入、状态或执行环境。
    check('日志轮换擦除失败锁定且停止周期只尝试一次',()=>{const s=saved();for(let i=1;i<102;i++){s.control.prepare();s.control.service();}s.control.prepare();s.io.eraseFail=true;s.control.service();const count=s.io.ops.length;s.control.service();equal(s.io.ops.length,count);equal(read(s)[0],0);equal(s.io.locked,true);}); // 检查场景的输入、状态或执行环境。
    check('重写校准前必须失效位置',()=>{const s=saved();equal(s.control.calWrite({MaxPulseNum:[7200,51000,2600],PulseScale:[400,1000,400]},7),1);equal(read(s)[0],0);}); // 检查场景的输入、状态或执行环境。
    check('位置失效失败不允许擦除校准页',()=>{const s=saved();s.io.failAddress=0x0803F000;const count=s.io.ops.filter(o=>o[0]==='erase').length;equal(s.control.calWrite({MaxPulseNum:[7200,51000,2600],PulseScale:[400,1000,400]},7),0);equal(s.io.ops.filter(o=>o[0]==='erase').length,count);}); // 检查场景的输入、状态或执行环境。
    check('非法参数不写且读取失败保留输出',()=>{const s=boot(),d={Motor1Pulse:5,Motor3Pulse:6};equal(s.control.read(d,7),0);equal(d,{Motor1Pulse:5,Motor3Pulse:6});for(const mask of [0,8])equal(s.control.write(d,mask),0);equal(s.control.write(null,7),0);equal(s.io.ops.length,0);}); // 检查场景的输入、状态或执行环境。
    check('保存失败后下一次运动可重新开启保存周期',()=>{const s=saved();s.control.prepare();s.io.failAddress=0x0803F018;s.control.service();const count=s.io.ops.length;s.control.service();equal(s.io.ops.length,count);s.io.failAddress=undefined;s.control.prepare();s.motors[0].CurPosition=999;s.control.service();equal(read(s),[1,999,2000]);}); // 检查场景的输入、状态或执行环境。
    check('标记实际写入但驱动报失败后再次运动必须失效',()=>{const s=saved();s.control.prepare();s.io.failAfterWriteAddress=0x0803F014;s.motors[0].CurPosition=777;s.control.service();equal(read(s),[1,777,2000]);s.io.failAfterWriteAddress=undefined;equal(s.control.prepare(),1);equal(read(s)[0],0);equal(boot(s.io).control.loaded(),0);}); // 检查场景的输入、状态或执行环境。
    for(let axis=1;axis<=3;axis++){ // 检查场景的输入、状态或执行环境。
      const source=input.axes[axis-1],prefix='Mot'+axis; // 检查场景的输入、状态或执行环境。
      let axisFactory; // 检查场景的输入、状态或执行环境。
      try{
        const stubNames=['LeftHallScan','RightHallScan','UpHallScan','DownHallScan','ActionHandle','MoveTask']; // 检查场景的输入、状态或执行环境。
        axisFactory=new Function('Mot1','Mot2','Mot3','io','MotorCalibrationLoaded','MotorPositionLoaded','MotorPositionPrepareMove',String.raw`
          const motor_stop=0,motor_brake=1,motor_forward=2,motor_backward=3,NoFault=0,MotorFault=4,Hall1Fault=1,Hall2Fault=2,pulse_forward_brake=0,pulse_backward_brake=2,hall_forward_brake=1,hall_backward_brake=3,c_MoveTimeOver=30000,c_PulseScaleFactor=10,c_MaxAngle=180,c_MaxHeight=510,C_VerifyMinimumPulseNum=20; // 检查场景的输入、状态或执行环境。
          function printf(){}function delay_ms(){}function GPIO_SetBits(){}function GPIO_ResetBits(){}
        `+`const ${prefix}DirIO=0,${prefix}DirPin=0; let ${prefix}PosCorrBusy=0,${prefix}PosCorrTimes=0;`+stubNames.map(n=>`function ${prefix+n}(){}`).join('\n')+` // 检查场景的输入、状态或执行环境。
          function ${prefix}EnablePidControl(){io.events.push('enable');}
          function ${prefix}DisablePidControl(){io.events.push('disable');}
          function ${prefix}ActionState(){return ${prefix}.Action;}
          function ${prefix}ExcuteTime(){return ${prefix}.PidTimerB;}
          function ${prefix}SetCurrentPosition(p){${prefix}.CurPosition=p;}
          function ${prefix}MoveStop(){io.events.push('stop');${prefix}.Action=0;}
        `+['MoveForward','MoveBackward','MoveService'].map(n=>positionExtract(source,prefix+n)).join('\n')+`;return {forward:${prefix}MoveForward,backward:${prefix}MoveBackward,run:${prefix}MoveService};`);
      }catch(e){failures.push(prefix+'：'+e.message);continue;}
      function scenario(position,loaded,ok){const motors=[{Reset:0},{Reset:0},{Reset:0}],m=motors[axis-1];Object.assign(m,{Reset:1,En:0,Step:0,Action:0,Faultflag:0,CurPosition:position,MaxPulseNum:7200,PulseNumPerAngle:400,Hall1Sta:0,Hall2Sta:0,PidTimerB:0});const io={events:[]};const control=axisFactory(...motors,io,1,loaded,()=>{io.events.push('prepare');return ok===false?0:1;});return {m,io,control};} // 检查场景的输入、状态或执行环境。
      for(const direction of ['forward','backward'])check(prefix+direction+'先失效后使能PID',()=>{const s=scenario(direction==='forward'?100:7000,1);s.control[direction](3600,100);equal(s.io.events,['prepare','enable']);}); // 检查场景的输入、状态或执行环境。
      for(const direction of ['forward','backward'])check(prefix+direction+'失效失败禁止使能并报错',()=>{const s=scenario(direction==='forward'?100:7000,1,false);s.control[direction](3600,100);equal(s.io.events,['prepare']);equal([s.m.Action,s.m.En,s.m.Faultflag],[0,0,4]);}); // 检查场景的输入、状态或执行环境。
      if(axis===2){check('MOT2可信位置启动仍寻找下限位',()=>{const s=scenario(1200,1);s.control.run();equal([s.m.Step,s.m.Action,s.m.TarPosition],[1,3,0]);});continue;} // 检查场景的输入、状态或执行环境。
      for(const position of [1000,6000])check(prefix+'可信位置直接回中且跳过case0',()=>{const s=scenario(position,1);s.control.run();equal([s.m.Step,s.m.TarPosition,s.m.Action,s.m.CurPosition],[3,3600,position<3600?2:3,position]);equal(s.io.events,['prepare','enable']);}); // 检查场景的输入、状态或执行环境。
      check(prefix+'已在中点无需运动',()=>{const s=scenario(3600,1);s.control.run();equal([s.m.Reset,s.m.En,s.m.CurPosition],[0,1,3600]);equal(s.io.events.includes('enable'),false);}); // 检查场景的输入、状态或执行环境。
      check(prefix+'位置不可信仍寻一次限位',()=>{const s=scenario(1000,0);s.control.run();equal([s.m.Step,s.m.TarPosition,s.m.Action,s.m.CurPosition],[1,0,3,20000]);}); // 检查场景的输入、状态或执行环境。
      check(prefix+'直接回中失效失败不能假报归位成功',()=>{const s=scenario(1000,1,false);s.control.run();equal([s.m.Step,s.m.Reset,s.m.En,s.m.Faultflag],[4,1,0,4]);}); // 检查场景的输入、状态或执行环境。
      for(const brake of [0,2])check(prefix+'正反方向回中完成均成功',()=>{const s=scenario(3600,1);s.m.Step=3;s.m.BrakeType=brake;s.control.run();equal([s.m.Reset,s.m.En],[0,1]);}); // 检查场景的输入、状态或执行环境。
      check(prefix+'回中被限位打断保持故障',()=>{const s=scenario(3600,1);s.m.Step=3;s.m.BrakeType=1;s.control.run();equal([s.m.Reset,s.m.En,s.m.Faultflag],[1,0,4]);}); // 检查场景的输入、状态或执行环境。
    }
    return {passed,failures};
}

if (typeof require !== "undefined" && typeof module !== "undefined" && require.main === module) {
    const fs = require("node:fs"); // 源文件读取接口。
    const path = require("node:path"); // 工程路径拼接接口。
    const root = path.resolve(__dirname, ".."); // 当前工程根目录。
    /** 函数作用：按GBK读取固件源码；name为相对工程根目录的文件路径。 */
    function readSource(name) { return new TextDecoder("gbk").decode(fs.readFileSync(path.join(root, name))); }
    const result = runMotorPositionTests({ // 调用实际生产函数的行为检查结果。
        flash: readSource("USER/Sourse/flash.c"), flashHeader: readSource("USER/Include/flash.h"),
        motor: readSource("USER/Sourse/motor.c"), header: readSource("USER/Include/motor.h"),
        axes: [1,2,3].map(axis => readSource("USER/Sourse/mot"+axis+".c"))
    });
    console.log(result);
    if (result.failures.length) process.exitCode = 1;
}
