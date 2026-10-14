// 执行源码中的控制分支，检查最低PWM、PID状态同步及停机；不代替实机验证。
const fs = require('node:fs'); // 源码读取接口。
const path = require('node:path'); // 路径拼接接口。
const assert = require('node:assert/strict'); // 行为断言接口。
const root = path.resolve(__dirname, '..'); // 工程根目录。
const decoder = new TextDecoder('gbk'); // 工程源码中文编码。

/** 函数作用：读取工程源码。name为相对工程根目录的路径。 */
function readSource(name) { return decoder.decode(fs.readFileSync(path.join(root, name))); }

/** 函数作用：提取C函数的真实控制逻辑。source为源码，name为函数名。 */
function extractFunction(source, name) {
    const match = new RegExp('(?:void|u8|u16)\\s+' + name + '\\s*\\(([^)]*)\\)\\s*\\{').exec(source); // 函数起点。
    assert.ok(match, '找不到函数：' + name);
    let end = match.index + match[0].length; // 函数体扫描位置。
    let depth = 1; // 花括号深度。
    while (depth && end < source.length) {
        if (source[end] === '{') depth++;
        if (source[end] === '}') depth--;
        end++;
    }
    assert.equal(depth, 0, '函数体不完整');
    const parameters = match[1].replace(/\b(?:void|u8|u16)\b/g, '').trim(); // 去掉参数类型。
    const body = source.slice(match.index + match[0].length, end - 1)
        .replace(/#if\s+1\b([\s\S]*?)#else[\s\S]*?#endif/g, '$1')
        .replace(/^\s*#(?:if\s+1|endif)\s*$/gm, '')
        .replace(/\((?:u8|u16|u32|int32_t)\)/g, '')
        .replace(/\b(?:u8|u16|u32|int32_t)\s+(\w[^;]*);/g, 'let $1;')
        .replace(/\b(brakePulseNum|PidTargetVelocity)\s*=(?!=)\s*([^;]+);/g, '$1 = Math.trunc($2);'); // 保留分支和调用，对减速窗口及速度指令保持C整数截断。
    return 'function ' + name + '(' + parameters + ') {' + body + '}';
}

const source = readSource('USER/Sourse/mot3.c').replace(/\/\/[^\n]*/g, ''); // 去掉行注释后的电机实现。
const headers = readSource('USER/Include/motor.h') + '\n' + readSource('USER/Include/pwm.h'); // 公共宏和动作枚举。
const allSource = (source + '\n' + headers).replace(/\/\/[^\n]*/g, ''); // 数值常量来源。
const macros = Array.from(allSource.matchAll(/^#define\s+(\w+)\s+([0-9(][^\r\n]*)/gm), match => 'const ' + match[1] + ' = ' + match[2].trim() + ';').join('\n'); // 源码中的数值宏。
const globals = Array.from(source.matchAll(/^static\s+int32_t\s+(\w+)\s*=\s*([^;]+);/gm), match => 'let ' + match[1] + ' = ' + match[2] + ';').join('\n'); // PID状态初值。
let enums = ''; // 动作和刹停原因枚举。
for (const match of allSource.matchAll(/enum\s+(?:motorState|motorBrake)\s*\{([^}]+)\}/g)) {
    let value = 0; // 当前枚举值。
    for (const item of match[1].split(',')) {
        const entry = item.trim().match(/^(\w+)(?:\s*=\s*(\d+))?$/); // 当前枚举项。
        if (!entry) continue;
        if (entry[2]) value = Number(entry[2]);
        enums += 'const ' + entry[1] + ' = ' + value++ + ';\n';
    }
}
const names = ['Mot3VelocityFeedbackControl', 'Mot3ReadEncode', 'Mot3ClearEncode', 'Mot3EnablePidControl', 'Mot3DisablePidControl', 'Mot3VelocityControl', 'Mot3ReadUpHallState', 'Mot3ReadDownHallState', 'Mot3MoveStop', 'Mot3ActionHandle']; // 真实控制路径函数。
const factory = new Function('Mot3', 'hardware', macros + enums + globals + `
    /** 函数作用：模拟PWM寄存器写入。duty为写入值。 */
    function TIM1_SetDuty_CH2(duty) { hardware.pwm = Math.trunc(duty); }
    /** 函数作用：单线程测试中忽略中断屏蔽，无参数。 */
    function __disable_irq() {}
    /** 函数作用：单线程测试中忽略中断恢复，无参数。 */
    function __enable_irq() {}
    /** 函数作用：忽略串口打印，无需处理传入内容。 */
    function printf() {}
` + names.map(name => extractFunction(source, name)).join('\n') + `
    const feedback = Mot3VelocityFeedbackControl; // 真实PID函数，记录输入时仍调用原实现。
    /** 函数作用：记录本周期速度指令后执行真实PID。target为速度指令，current为实测速度。 */
    Mot3VelocityFeedbackControl = function(target, current) {
        hardware.target = target;
        return feedback(target, current);
    };
    return { velocity: Mot3VelocityControl, enable: Mot3EnablePidControl, stop: Mot3MoveStop, handle: Mot3ActionHandle,
        /* 函数作用：设置本场景的PID初始输出。value为缩放后的PWM。 */
        seed(value) { Mot3PidOutputScaled = value; },
        /* 函数作用：读取PID内部输出，检查状态同步，无参数。 */
        output() { return Mot3PidOutputScaled; } };
`); // 执行真实逻辑，只替换硬件接口。
const cases = [
    { name: '低于下限时提升实际PWM并同步PID状态', seed: 0, want: [1800, 180000] },
    { name: '反向运动也使用最低PWM', seed: 0, fields: { Action: 3, CurPosition: 100 }, want: [1800, 180000] },
    { name: '高反馈导致输出归零时保持运动下限', seed: 200000, fields: { PulseNumB: 1000 }, want: [1800, 180000] },
    { name: '高于下限时保留PID小数积分', seed: 250037, want: [2601, 260137] },
    { name: '达到最大PWM时保留上限', seed: 480000, want: [4800, 480000] },
    { name: '接近目标减速时保留最低PWM', seed: 0, fields: { PulseNum: 99 }, want: [1800, 180000] },
    { name: '速度为零时关闭PWM而不抬到下限', seed: 250000, fields: { TarVelocity: 0 }, want: [0, 0] },
    { name: '到达目标时关闭PWM', seed: 250000, fields: { PulseNum: 100 }, want: [0, 0] },
    { name: '超过目标时关闭PWM', seed: 250000, fields: { PulseNum: 101 }, want: [0, 0] },
    { name: '启动与MOT1一样初始化PID并等待控制周期', method: 'enable', want: [0, 180000] },
    { name: '零速度启动保持PWM和PID状态为零', method: 'enable', fields: { TarVelocity: 0 }, want: [0, 0] },
    { name: '主动停止清零PWM和PID状态', method: 'stop', seed: 250000, want: [0, 0], pidEn: 0 },
    { name: '正向下限位关闭PWM', method: 'handle', seed: 250000, fields: { Hall2Sta: 1 }, want: [0, 0], pidEn: 0 },
    { name: '反向上限位关闭PWM', method: 'handle', seed: 250000, fields: { Action: 3, CurPosition: 100, Hall1Sta: 1 }, want: [0, 0], pidEn: 0 },
    { name: '短行程前半程不减速', seed: 0, fields: { TarVelocity: 100, PulseNum: 40 }, target: 100 },
    { name: '短行程半程边界不提前减速', seed: 0, fields: { TarVelocity: 100, PulseNum: 50 }, target: 100 },
    { name: '短行程后半程按剩余距离减速', seed: 0, fields: { TarVelocity: 100, PulseNum: 75 }, target: 60 },
    { name: '长行程最多提前500脉冲减速', seed: 0, fields: { TarVelocity: 100, TarPulseNum: 2000, PulseNum: 1600, MaxPulseNum: 2000 }, target: 84 },
    { name: '长行程500脉冲边界不提前减速', seed: 0, fields: { TarVelocity: 100, TarPulseNum: 2000, PulseNum: 1500, MaxPulseNum: 2000 }, target: 100 },
    { name: '初始化寻零限位不减速', seed: 0, fields: { Reset: 1, Step: 1, Action: 3, CurPosition: 100, TarVelocity: 100, PulseNum: 75 }, target: 100 },
    { name: '首次往返自检寻另一限位不减速', seed: 0, fields: { Reset: 1, Step: 2, TarVelocity: 100, PulseNum: 75 }, target: 100 },
    { name: '初始化回中仍按距离减速', seed: 0, fields: { Reset: 1, Step: 3, TarVelocity: 100, PulseNum: 75 }, target: 60 },
    { name: '反向普通定位按距离减速', seed: 0, fields: { Action: 3, CurPosition: 100, TarVelocity: 100, PulseNum: 75 }, target: 60 },
    { name: '低于接近速度的指令不被抬高', seed: 0, fields: { TarVelocity: 10, PulseNum: 75 }, target: 10 },
    { name: '一脉冲行程避免减速窗口除零', seed: 0, fields: { TarVelocity: 100, TarPulseNum: 1 }, target: 100 },
    { name: '奇数行程先对半程窗口进行整数截断', seed: 0, fields: { TarVelocity: 100, TarPulseNum: 101, PulseNum: 76, MaxPulseNum: 101 }, target: 60 },
    { name: '减速插值按C整数规则截断', seed: 0, fields: { TarVelocity: 101, PulseNum: 75 }, target: 60 }
]; // 手工计算的行为期望，缺失下限、同步或停止分支会使检查失败。
const failures = []; // 失败信息集合。
for (const test of cases) {
    const motor = Object.fromEntries(Array.from(readSource('USER/Include/mot3.h').matchAll(/\bu(?:8|16|32)\s+(\w+)\s*;/g), match => [match[1], 0])); // 电机完整字段初值。
    Object.assign(motor, { Action: 2, PidEn: 1, PidTimer: 50, TarPulseNum: 100, TarVelocity: 50, MaxPulseNum: 100, PulseNumPerAngle: 200 }, test.fields);
    const hardware = { pwm: 0 }; // 模拟PWM寄存器。
    const control = factory(motor, hardware); // 当前场景控制环境。
    control.seed(test.seed || 0);
    control[test.method || 'velocity']();
    try {
        if (test.want) assert.deepEqual([hardware.pwm, control.output()], test.want);
        if (test.target !== undefined) {
            assert.equal(hardware.target, test.target);
            assert.equal(hardware.pwm, 1800);
            assert.equal(control.output(), 180000);
        }
        if (test.pidEn !== undefined) assert.equal(motor.PidEn, test.pidEn);
    } catch (error) { failures.push(test.name + ': ' + error.message); }
}
assert.equal(failures.length, 0, failures.join('\n'));
console.log({ passed: cases.length, scope: 'MOT3最低PWM及停机的有界行为；不模拟C整数溢出、芯片时序或机械动作' });
