// 在无本机C编译器时运行限位分支的行为检查；不代替Keil编译或实机验证。
// 仅转换函数声明、局部变量声明和整数类型转换，保留源码中的判断与调用顺序。

/** 函数作用：提取一个C函数并转换为可执行的JavaScript。source为C源码，name为函数名。 */
function extractFunction(source, name) {
    const signature = new RegExp("(?:static\\s+)?(?:void|u8|u16|u32)\\s+" + name + "\\s*\\(([^)]*)\\)\\s*\\{"); // 函数声明匹配规则。
    const match = signature.exec(source); // 当前函数声明。
    if (!match) throw new Error("找不到函数：" + name);
    let end = match.index + match[0].length; // 函数体扫描位置。
    let depth = 1; // 花括号嵌套深度。
    while (depth && end < source.length) {
        if (source[end] === "{") depth++;
        if (source[end] === "}") depth--;
        end++;
    }
    if (depth) throw new Error("函数体不完整：" + name);
    const parameters = match[1].replace(/\b(?:void|u8|u16|u32)\b/g, "").trim(); // 去除类型后的参数列表。
    const body = source.slice(match.index + match[0].length, end - 1)
        .replace(/^\s*#(?:if\s+1|endif)\s*$/gm, "")
        .replace(/\((?:u8|u16|u32|int32_t|unsigned long)\)/g, "")
        .replace(/\b(?:u8|u16|u32|int32_t)\s+(\w[^;]*);/g, "let $1;"); // 保留原判断和赋值的函数体。
    return "function " + name + "(" + parameters + ") {" + body + "}";
}

/** 函数作用：运行MOT2限位停机回归检查。input包含电机源码及相关头文件正文。 */
function runMot2LimitStopTests(input) {
    const names = ["Mot2UpHallScan", "Mot2DownHallScan", "Mot2ReadUpHallState", "Mot2ReadDownHallState", "Mot2PidReset", "Mot2VelocityFeedbackControl", "Mot2ReadEncode", "Mot2DisablePidControl", "Mot2IsFirstHomingMove", "Mot2IsUpperHomingMove", "Mot2VelocityControl", "Mot2ActionHandle"]; // 执行真实控制路径需要的函数。
    const allSource = [input.source, input.motorHeader, input.sysHeader, input.pwmHeader].join("\n").replace(/\/\/[^\n]*/g, ""); // 去除行注释后的常量来源。
    const macros = Array.from(allSource.matchAll(/^#define\s+(\w+)\s+([0-9][^\r\n]*)/gm)).filter(match => match[1] !== "true" && match[1] !== "false").map(match => "const " + match[1] + " = " + match[2].trim() + ";").join("\n"); // 源码中的数值宏，布尔关键字沿用运行时定义。
    let enumCode = ""; // 电机动作和刹停原因的枚举声明。
    for (const match of allSource.matchAll(/enum\s+(?:motorState|motorBrake)\s*\{([^}]+)\}/g)) {
        let value = 0; // 当前枚举的数值。
        for (const item of match[1].split(",")) {
            const entry = item.trim().match(/^(\w+)(?:\s*=\s*(\d+))?$/); // 当前枚举项。
            if (!entry) continue;
            if (entry[2]) value = Number(entry[2]);
            enumCode += "const " + entry[1] + " = " + value++ + ";\n";
        }
    }
    const factory = new Function("Mot2", "Adc", "hardware", macros + enumCode + `
        let Mot2PidPreviousError = 0; // 上一周期PID误差。
        let Mot2PidPreviousPreviousError = 0; // 上上周期PID误差。
        let Mot2PidOutputScaled = 0; // PID内部输出。
        function TIM1_SetDuty_CH1(duty) { hardware.pwm = duty; } // 模拟PWM寄存器写入，duty为占空比。
        function __disable_irq() {} // 单线程检查不产生编码器中断。
        function __enable_irq() {} // 单线程检查不产生编码器中断。
        function printf() {} // 忽略串口打印，不改变电机控制状态。
    ` + names.map(name => extractFunction(input.source, name)).join("\n") + "; return { scanUp: Mot2UpHallScan, scanDown: Mot2DownHallScan, handle: Mot2ActionHandle };"); // 真实控制函数的执行环境。
    const cases = [
        { name: "首次下行复位，延时计数为0", fields: { Reset: 1, Step: 1, Hall2Sta: 1 }, want: [0, 0, 1, 3] },
        { name: "标定后回下限位，延时计数为0", fields: { Reset: 1, Step: 3, Hall2Sta: 1 }, want: [0, 0, 1, 3] },
        { name: "普通下行，延时计数为300", fields: { Hall2Sta: 1, DelayTimer: 300 }, want: [0, 0, 1, 3] },
        { name: "普通下行，延时计数为999", fields: { Hall2Sta: 1, DelayTimer: 999 }, want: [0, 0, 1, 3] },
        { name: "测速周期到期时下限位仍关PWM", fields: { Reset: 1, Step: 1, Hall2Sta: 1, PidTimer: 50 }, want: [0, 0, 1, 3] },
        { name: "上限位确认后关PWM", fields: { Reset: 1, Step: 2, Action: 2, Hall1Sta: 1 }, want: [0, 0, 1, 1] },
        { name: "下限位未触发时保持运动", fields: {}, want: [4800, 1, 3, 0] },
        { name: "普通下行到目标脉冲仍停机", fields: { PulseNum: 100 }, want: [0, 0, 1, 2] },
        { name: "首次下行保留脉冲限值绕过", fields: { Reset: 1, Step: 1, PulseNum: 100 }, want: [4800, 1, 3, 0] },
        { name: "下霍尔19ms不提前停机", fields: { Hall2Deb: 19 }, adc: 2500, scan: true, want: [4800, 1, 3, 0] },
        { name: "下霍尔20ms确认后立即关PWM", fields: { Hall2Deb: 20 }, adc: 2500, scan: true, want: [0, 0, 1, 3] },
        { name: "下霍尔未达阈值时不误停", fields: { Hall2Deb: 20 }, adc: 2499, scan: true, want: [4800, 1, 3, 0] }
    ]; // 手工确定的动作、消抖和PWM期望，独立于被测实现。
    const failures = []; // 未满足停机行为的用例。
    for (const test of cases) {
        const motor = Object.fromEntries(Array.from(input.mot2Header.matchAll(/\bu(?:8|16|32)\s+(\w+)\s*;/g), match => [match[1], 0])); // 完整电机字段及初始值。
        Object.assign(motor, { Action: 3, PidEn: 1, TarPulseNum: 100, TarVelocity: 50, MaxPulseNum: 100, PulseNumPerMm: 200 }, test.fields);
        const adc = { Mot2: { Vout: 0, Vref: 0, Current: 0, Hall1: 0, Hall2: test.adc || 0, Hall3: 0 } }; // 模拟ADC输入。
        const hardware = { pwm: 4800 }; // 已处于运动时的PWM寄存器。
        const control = factory(motor, adc, hardware); // 当前用例的真实控制路径。
        if (test.scan) control.scanDown();
        control.handle();
        const actual = [hardware.pwm, motor.PidEn, motor.Action, motor.BrakeType]; // 实际PWM、PID使能、动作和刹停原因。
        if (JSON.stringify(actual) !== JSON.stringify(test.want)) failures.push(test.name + ": 实际 " + actual + "，期望 " + test.want);
    }
    if (failures.length) throw new Error(failures.join("\n"));
    return { passed: cases.length, scope: "限位控制分支行为；未模拟C整数溢出、硬件时序及机械惯性" };
}

// 可在有Node.js的环境中使用node tests/mot2_limit_stop.test.js重复检查。
if (typeof require !== "undefined" && typeof module !== "undefined" && require.main === module) {
    const fs = require("node:fs"); // 源文件读取接口。
    const path = require("node:path"); // 路径拼接接口。
    const root = path.resolve(__dirname, ".."); // 工程根目录。
    const decoder = new TextDecoder("gbk"); // 保留工程GB2312中文源码的解码方式。
    /** 函数作用：读取工程源码。name为相对工程根目录的文件路径。 */
    function readSource(name) { return decoder.decode(fs.readFileSync(path.join(root, name))); }
    console.log(runMot2LimitStopTests({ source: readSource("USER/Sourse/mot2.c"), motorHeader: readSource("USER/Include/motor.h"), sysHeader: readSource("USER/Include/sys.h"), pwmHeader: readSource("USER/Include/pwm.h"), mot2Header: readSource("USER/Include/mot2.h") }));
}
