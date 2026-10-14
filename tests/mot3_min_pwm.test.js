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
        .replace(/\b(?:u8|u16|u32|int32_t)\s+(\w[^;]*);/g, 'let $1;'); // 只移除预处理和C类型，保留分支和调用。
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
    { name: '高于下限时保留PID小数积分', seed: 250037, want: [2576, 257637] },
    { name: '达到最大PWM时保留上限', seed: 480000, want: [4800, 480000] },
    { name: '接近目标减速时保留最低PWM', seed: 0, fields: { PulseNum: 99 }, want: [1800, 180000] },
    { name: '速度为零时关闭PWM而不抬到下限', seed: 250000, fields: { TarVelocity: 0 }, want: [0, 0] },
    { name: '到达目标时关闭PWM', seed: 250000, fields: { PulseNum: 100 }, want: [0, 0] },
    { name: '超过目标时关闭PWM', seed: 250000, fields: { PulseNum: 101 }, want: [0, 0] },
    { name: '启动立即使用PID初值', method: 'enable', want: [2000, 200000] },
    { name: '零速度启动保持PWM和PID状态为零', method: 'enable', fields: { TarVelocity: 0 }, want: [0, 0] },
    { name: '主动停止清零PWM和PID状态', method: 'stop', seed: 250000, want: [0, 0], pidEn: 0 },
    { name: '正向下限位关闭PWM', method: 'handle', seed: 250000, fields: { Hall2Sta: 1 }, want: [0, 0], pidEn: 0 },
    { name: '反向上限位关闭PWM', method: 'handle', seed: 250000, fields: { Action: 3, CurPosition: 100, Hall1Sta: 1 }, want: [0, 0], pidEn: 0 }
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
        assert.deepEqual([hardware.pwm, control.output()], test.want);
        if (test.pidEn !== undefined) assert.equal(motor.PidEn, test.pidEn);
    } catch (error) { failures.push(test.name + ': ' + error.message); }
}
assert.equal(failures.length, 0, failures.join('\n'));
console.log({ passed: cases.length, scope: 'MOT3最低PWM及停机的有界行为；不模拟C整数溢出、芯片时序或机械动作' });
