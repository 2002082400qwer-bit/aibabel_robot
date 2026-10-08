// 复用MOT2检查中的函数提取器，检查MOT1测试模式的实际PWM及停机分支。
// C函数转为JavaScript后只检查有界输入，不代替C编译、整数溢出检查或实机验证。

/** 函数作用：运行MOT1满PWM检查。input为源码及头文件，extract为既有C函数提取器。 */
function runMot1MaxPwmTests(input, extract) {
    const names = ["Mot1VelocityFeedbackControl", "Mot1ReadEncode", "Mot1ClearEncode", "Mot1EnablePidControl", "Mot1DisablePidControl", "Mot1ReadLeftHallState", "Mot1ReadRightHallState", "Mot1VelocityControl", "Mot1MoveForward", "Mot1MoveBackward", "Mot1MoveStop", "Mot1MoveBrake", "Mot1ActionHandle"]; // 执行启动、运行及停机路径所需的真实函数。
    if (/static\s+void\s+Mot1PidReset\s*\([^)]*\)\s*\{/.test(input.source)) names.unshift("Mot1PidReset"); // 同时兼容已提交版本的PID复位辅助函数。
    const globals = Array.from(input.source.matchAll(/^static\s+(?:int32_t|u8|u16|u32)\s+(\w+)\s*=\s*([^;]+);/gm), match => "let " + match[1] + " = " + match[2] + ";").join("\n"); // 源码中的控制状态初值，兼容工作区及已提交版本。
    const source = [input.source, input.motorHeader, input.pwmHeader].join("\n").replace(/\/\/[^\n]*/g, ""); // 数值宏和枚举的来源。
    const macros = Array.from(source.matchAll(/^#define\s+(\w+)\s+([0-9][^\r\n]*)/gm), match => "const " + match[1] + " = " + match[2].trim() + ";").join("\n"); // 源码中的数值宏。
    let enums = ""; // 电机动作和刹停原因枚举。
    for (const match of source.matchAll(/enum\s+(?:motorState|motorBrake)\s*\{([^}]+)\}/g)) {
        let value = 0; // 当前枚举项的值。
        for (const item of match[1].split(",")) {
            const entry = item.trim().match(/^(\w+)(?:\s*=\s*(\d+))?$/); // 当前枚举项。
            if (!entry) continue;
            if (entry[2]) value = Number(entry[2]);
            enums += "const " + entry[1] + " = " + value++ + ";\n";
        }
    }
    const factory = new Function("Mot1", "hardware", macros + enums + globals + `
        const Adc = { Mot1: { Vout: 0, Vref: 0, Current: 0, Hall1: 0, Hall2: 0, Hall3: 0 } }; // 已提交版本串口打印所读取的ADC值。
        const Mot1DirIO = {}; // 测试用方向端口占位。
        const Mot1DirPin = 0; // 测试用方向引脚占位。
        function GPIO_SetBits() {} // 忽略真实方向GPIO写入。
        function GPIO_ResetBits() {} // 忽略真实方向GPIO写入。
        function TIM1_SetDuty_CH3(duty) { hardware.pwm = duty; } // 模拟PWM寄存器，duty为占空比。
        function __disable_irq() {} // 单线程检查不产生编码器中断。
        function __enable_irq() {} // 单线程检查不产生编码器中断。
        function printf() {} // 忽略串口打印。
    ` + names.map(name => extract(input.source, name)).join("\n") + "; return { forward: Mot1MoveForward, backward: Mot1MoveBackward, velocity: Mot1VelocityControl, handle: Mot1ActionHandle, stop: Mot1MoveStop, brake: Mot1MoveBrake };"); // 真实控制路径的执行环境。
    const cases = [
        { name: "正向启动立即输出满PWM", method: "forward", args: [100, 50], fields: { Action: 0, PidEn: 0 }, pwm: 0, want: [4800, 1, 2, 0] },
        { name: "反向启动立即输出满PWM", method: "backward", args: [0, 50], fields: { Action: 0, PidEn: 0, CurPosition: 100 }, pwm: 0, want: [4800, 1, 3, 0] },
        { name: "目标速度50时固定满PWM", method: "velocity", want: [4800, 1, 2, 0] },
        { name: "目标速度900时固定满PWM", method: "velocity", fields: { TarVelocity: 900 }, want: [4800, 1, 2, 0] },
        { name: "测试模式不按速度0降低PWM", method: "velocity", fields: { TarVelocity: 0 }, want: [4800, 1, 2, 0] },
        { name: "接近目标脉冲时仍固定满PWM", method: "velocity", fields: { PulseNum: 99 }, want: [4800, 1, 2, 0] },
        { name: "目标脉冲到达后PWM为0", method: "velocity", fields: { PulseNum: 100 }, pwm: 4800, want: [0, 1, 2, 0] },
        { name: "正向目标脉冲仍触发刹停", method: "handle", fields: { PulseNum: 100 }, pwm: 4800, want: [0, 0, 1, 0] },
        { name: "反向目标脉冲仍触发刹停", method: "handle", fields: { Action: 3, PulseNum: 100 }, pwm: 4800, want: [0, 0, 1, 2] },
        { name: "右霍尔确认仍关闭PWM", method: "handle", fields: { Hall2Sta: 1 }, pwm: 4800, want: [0, 0, 1, 1] },
        { name: "左霍尔确认仍关闭PWM", method: "handle", fields: { Action: 3, Hall1Sta: 1 }, pwm: 4800, want: [0, 0, 1, 3] },
        { name: "主动停止关闭PWM", method: "stop", pwm: 4800, want: [0, 0, 0, 0] },
        { name: "主动刹停关闭PWM", method: "brake", pwm: 4800, want: [0, 0, 1, 0] },
        { name: "控制禁用时不重新输出PWM", method: "velocity", fields: { Action: 0, PidEn: 0 }, pwm: 0, want: [0, 0, 0, 0] },
        { name: "零行程指令不启动PWM", method: "forward", args: [0, 50], fields: { Action: 0, PidEn: 0 }, pwm: 0, want: [0, 0, 0, 0] }
    ]; // 手工确定的PWM、控制使能、动作和刹停原因期望。
    const failures = []; // 未符合预期的用例。
    for (const test of cases) {
        const motor = Object.fromEntries(Array.from(input.mot1Header.matchAll(/\bu(?:8|16|32)\s+(\w+)\s*;/g), match => [match[1], 0])); // 完整电机字段及初始值。
        Object.assign(motor, { Action: 2, PidEn: 1, PidTimer: 50, TarPulseNum: 100, TarVelocity: 50, MaxPulseNum: 100, PulseNumPerAngle: 200 }, test.fields);
        const hardware = { pwm: test.pwm === undefined ? 0 : test.pwm }; // PWM寄存器初始值。
        const control = factory(motor, hardware); // 当前用例的真实控制路径。
        control[test.method](...(test.args || []));
        const actual = [hardware.pwm, motor.PidEn, motor.Action, motor.BrakeType]; // 实际PWM及运动状态。
        if (JSON.stringify(actual) !== JSON.stringify(test.want)) failures.push(test.name + ": 实际 " + actual + "，期望 " + test.want);
    }
    if (failures.length) throw new Error(failures.join("\n"));
    return { passed: cases.length, scope: "满PWM测试模式的有界分支行为；未模拟硬件时序、整数溢出及机械惯性" };
}

// 可使用node tests/mot1_max_pwm.test.js重复检查。
if (typeof require !== "undefined" && typeof module !== "undefined" && require.main === module) {
    const fs = require("node:fs"); // 文件读取接口。
    const path = require("node:path"); // 路径拼接接口。
    const root = path.resolve(__dirname, ".."); // 工程根目录。
    const decoder = new TextDecoder("gbk"); // 工程中文源码的解码方式。
    const helper = fs.readFileSync(path.join(__dirname, "mot2_limit_stop.test.js"), "utf8"); // 既有函数提取器源码。
    const extract = new Function(helper + "; return extractFunction;")(); // 复用既有函数提取器。
    /** 函数作用：读取工程源码。name为相对工程根目录的路径。 */
    function readSource(name) { return decoder.decode(fs.readFileSync(path.join(root, name))); }
    console.log(runMot1MaxPwmTests({ source: readSource("USER/Sourse/mot1.c"), motorHeader: readSource("USER/Include/motor.h"), pwmHeader: readSource("USER/Include/pwm.h"), mot1Header: readSource("USER/Include/mot1.h") }, extract));
}
