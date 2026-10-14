// 执行两路真实控制，检查固定低PWM寻限位与PID回中的切换；不代替实机标定和噪音测量。
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
    const names = ['VelocityFeedbackControl', 'ReadEncode', 'ClearEncode', 'EnablePidControl', 'DisablePidControl', 'VelocityControl', 'MoveStop', 'ActionHandle'].map(name => prefix + name); // 控制、启动和停止的真实函数。
    names.push(prefix + (axis === 1 ? 'ReadLeftHallState' : 'ReadUpHallState'), prefix + (axis === 1 ? 'ReadRightHallState' : 'ReadDownHallState')); // 两侧真实限位状态读取。
    const factory = new Function(prefix, 'hardware', macros + globals + `
        const motor_forward = 2; // 公共枚举中的正向运动状态。
        const motor_backward = 3; // 公共枚举中的反向运动状态。
        const motor_stop = 0; // 停止状态。
        const motor_brake = 1; // 刹停状态。
        const pulse_forward_brake = 0; // 正向到目标刹停。
        const hall_forward_brake = 1; // 正向限位刹停。
        const pulse_backward_brake = 2; // 反向到目标刹停。
        const hall_backward_brake = 3; // 反向限位刹停。
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
        return { velocity: ${prefix}VelocityControl, enable: ${prefix}EnablePidControl, handle: ${prefix}ActionHandle, stop: ${prefix}MoveStop,
            /* 函数作用：设定PID状态以验证寻限位不受积分影响，value为缩放后的PWM。 */
            seed(value) { ${prefix}PidOutputScaled = value; } };
    `); // 保留真实分支，仅替换硬件边界。
    const cases = [
        { name: '未知距离寻零固定低PWM', fields: { Step: 1, Action: 3, CurPosition: 20000, TarPulseNum: 20000, TarVelocity: 900 }, target: undefined, pwm: 1800 },
        { name: '首次自检另一端也固定低PWM', fields: { Step: 2, TarPulseNum: 20000, TarVelocity: 900 }, target: undefined, pwm: 1800 },
        { name: '接近保护脉冲终点仍保持低PWM', fields: { Step: 1 }, target: undefined, pwm: 1800 },
        { name: '高反馈及残余积分不改变寻限位PWM', fields: { Step: 1, PulseNumB: 1000 }, seed: 450000, target: undefined, pwm: 1800 },
        { name: '回中使用PID并按半程窗口减速', fields: { Step: 3 }, seed: 250000, target: 60, pwm: 2621 },
        { name: '回中前半程PID输出可高于寻位PWM', fields: { Step: 3, TarVelocity: 900, TarPulseNum: 3600, MaxPulseNum: 7200, PulseNum: 0 }, seed: 180000, target: 900, pwm: 3618 },
        { name: '普通定位使用PID', fields: { Reset: 0, Step: 1 }, seed: 250000, target: 60, pwm: 2621 },
        { name: '零速度寻位关闭PWM', fields: { Step: 1, TarVelocity: 0 }, target: 0, pwm: 0 },
        { name: '保护目标到达时关闭PWM', fields: { Step: 1, PulseNum: 100 }, target: 0, pwm: 0 },
        { name: '寻位启动立即输出低PWM', method: 'enable', fields: { Step: 0 }, pwm: 1800 },
        { name: '零速度寻位启动不输出PWM', method: 'enable', fields: { Step: 0, TarVelocity: 0 }, pwm: 0 },
        { name: '反向限位立即关PWM', method: 'handle', fields: { Step: 1, Action: 3, CurPosition: 100, Hall1Sta: 1 }, pwm: 0, pidEn: 0 },
        { name: '正向限位立即关PWM', method: 'handle', fields: { Step: 2, Hall2Sta: 1 }, pwm: 0, pidEn: 0 },
        { name: '主动停止仍关PWM', method: 'stop', fields: { Step: 1 }, pwm: 0, pidEn: 0 },
        { name: '限位停稳后启动回中会重置PID并切换模式', method: 'transition', fields: { Step: 1 }, seed: 450000, target: 900, pwm: 3618 }
    ]; // 期望由手工脉冲插值计算，与被测函数独立。
    for (const test of cases) {
        const motor = Object.fromEntries(Array.from(readSource('USER/Include/mot' + axis + '.h').matchAll(/\bu(?:8|16|32)\s+(\w+)\s*;/g), match => [match[1], 0])); // 当前电机完整状态。
        Object.assign(motor, { Reset: 1, Action: 2, PidEn: 1, PidTimer: 50, TarPulseNum: 100, PulseNum: 75, TarVelocity: 100, MaxPulseNum: 100, PulseNumPerAngle: 200 }, test.fields);
        const hardware = { pwm: 0 }; // 模拟PWM寄存器和PID指令。
        const control = factory(motor, hardware); // 当前用例的真实控制器。
        control.seed(test.seed || 0);
        if (test.method === 'transition') {
            control.velocity();
            assert.equal(hardware.target, undefined, '寻限位不能运行速度PID');
            assert.equal(hardware.pwm, 1800, '寻限位保持低PWM');
            control.stop();
            assert.equal(hardware.pwm, 0, '停稳切换前关闭PWM');
            Object.assign(motor, { Action: 2, TarVelocity: 900, TarPulseNum: 3600, MaxPulseNum: 7200 });
            control.enable(); // 与真实归位流程一致：先启动回中运动，再进入Step3。
            motor.Step = 3;
            motor.PidTimer = 50;
            control.velocity();
        } else { control[test.method || 'velocity'](); }
        try {
            if (!test.method || test.method === 'transition') assert.equal(hardware.target, test.target);
            assert.equal(hardware.pwm, test.pwm);
            if (test.pidEn !== undefined) assert.equal(motor.PidEn, test.pidEn);
            passed++;
        } catch (error) { failures.push(prefix + ' ' + test.name + ': ' + error.message); }
    }
}
assert.equal(failures.length, 0, failures.join('\n'));
console.log({ passed, scope: 'MOT1/MOT3低PWM寻限位及PID回中；不模拟硬件时序、机械负载及噪音' });
