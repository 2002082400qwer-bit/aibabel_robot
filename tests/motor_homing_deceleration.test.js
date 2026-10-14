// 执行两路真实速度控制，检查寻位减速分支；不代表真实限位距离或实机噪音。
const fs = require('node:fs'); // 源码读取接口。
const path = require('node:path'); // 路径拼接接口。
const assert = require('node:assert/strict'); // 行为断言接口。
const root = path.resolve(__dirname, '..'); // 工程根目录。
const decoder = new TextDecoder('gbk'); // 工程源码编码。
const helper = fs.readFileSync(path.join(__dirname, 'mot3_min_pwm.test.js'), 'utf8'); // 已有C函数转换器。
const extract = new Function('require', '__dirname', helper.slice(0, helper.indexOf('\nconst source =')) + '; return extractFunction;')(require, __dirname); // 只复用转换器，不执行其用例。
let passed = 0; // 已通过的有界行为场景数。
const failures = []; // 失败场景信息。

/** 函数作用：读取源码并去除行注释。name为工程相对路径。 */
function readSource(name) { return decoder.decode(fs.readFileSync(path.join(root, name))).replace(/\/\/[^\n]*/g, ''); }

for (const axis of [1, 3]) {
    const prefix = 'Mot' + axis; // 当前电机符号前缀。
    const source = readSource('USER/Sourse/mot' + axis + '.c'); // 当前电机真实实现。
    const allSource = source + '\n' + readSource('USER/Include/motor.h') + '\n' + readSource('USER/Include/pwm.h'); // 数值常量来源。
    const macros = Array.from(allSource.matchAll(/^#define\s+(\w+)\s+([0-9(][^\r\n]*)/gm), match => 'const ' + match[1] + ' = ' + match[2].trim() + ';').join('\n'); // 保留源码数值宏。
    const globals = Array.from(source.matchAll(/^static\s+int32_t\s+(\w+)\s*=\s*([^;]+);/gm), match => 'let ' + match[1] + ' = ' + match[2] + ';').join('\n'); // 当前PID状态初值。
    const names = ['VelocityFeedbackControl', 'ReadEncode', 'VelocityControl'].map(name => prefix + name); // 本轮需要执行的真实函数。
    const factory = new Function(prefix, 'hardware', macros + globals + `
        const motor_forward = 2; // 公共枚举中的正向运动状态。
        const motor_backward = 3; // 公共枚举中的反向运动状态。
        /** 函数作用：模拟PWM写入，duty为占空比。 */
        function TIM1_SetDuty_CH${axis === 1 ? 3 : 2}(duty) { hardware.pwm = Math.trunc(duty); }
        /** 函数作用：单线程检查忽略中断屏蔽，无参数。 */
        function __disable_irq() {}
        /** 函数作用：单线程检查忽略中断恢复，无参数。 */
        function __enable_irq() {}
        /** 函数作用：忽略串口输出，无需处理传入内容。 */
        function printf() {}
    ` + names.map(name => extract(source, name)).join('\n') + `
        const feedback = ${prefix}VelocityFeedbackControl; // 原PID实现。
        /** 函数作用：记录速度指令并继续调用真实PID，target为指令，current为反馈。 */
        ${prefix}VelocityFeedbackControl = function(target, current) { hardware.target = target; return feedback(target, current); };
        return ${prefix}VelocityControl;
    `); // 保留真实分支，仅替换硬件边界。
    const cases = [
        { name: '寻零阶段进入减速窗口', fields: { Step: 1, Action: 3, CurPosition: 100 }, target: 60 },
        { name: '另一侧自检进入减速窗口', fields: { Step: 2 }, target: 60 },
        { name: '回中仍按半程窗口减速', fields: { Step: 3 }, target: 60 },
        { name: '低速寻位不被抬高', fields: { Step: 1, TarVelocity: 10 }, target: 10 },
        { name: '零速度寻位关闭PWM', fields: { Step: 1, TarVelocity: 0 }, target: 0 },
        { name: '目标脉冲到达时关闭PWM', fields: { Step: 1, PulseNum: 100 }, target: 0 }
    ]; // 期望由手工脉冲插值计算，与被测函数独立。
    for (const test of cases) {
        const motor = Object.fromEntries(Array.from(readSource('USER/Include/mot' + axis + '.h').matchAll(/\bu(?:8|16|32)\s+(\w+)\s*;/g), match => [match[1], 0])); // 当前电机完整状态。
        Object.assign(motor, { Reset: 1, Action: 2, PidEn: 1, PidTimer: 50, TarPulseNum: 100, PulseNum: 75, TarVelocity: 100, MaxPulseNum: 100, PulseNumPerAngle: 200 }, test.fields);
        const hardware = { pwm: 0 }; // 模拟PWM寄存器和PID指令。
        factory(motor, hardware)();
        try {
            assert.equal(hardware.target, test.target);
            assert.equal(hardware.pwm, test.target === 0 ? 0 : 1800);
            passed++;
        } catch (error) { failures.push(prefix + ' ' + test.name + ': ' + error.message); }
    }
}
assert.equal(failures.length, 0, failures.join('\n'));
console.log({ passed, scope: 'MOT1/MOT3寻位按设定目标脉冲减速；不模拟实际限位距离、硬件时序及噪音' });
