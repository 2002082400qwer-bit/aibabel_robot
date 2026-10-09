# 电机停稳位置保存与可信启动回中实施计划

> **For agentic workers:** 使用 subagent-driven-development 完成本计划；一个任务涵盖本次协同状态机，最终独立审查后由主代理选择性提交。

**Goal:** MOT1/MOT3 在停止后正常断电且断电期间不移动时，从保存位置直接回中；运动中掉电仍寻限位；MOT2 始终回最低限位。

**Architecture:** 运动前同步使最后位置检查点失效，之后才允许PWM启动。全部电机停稳、复位完成且无故障时追加位置记录；读取只认最新占用记录，最新无效/不完整时不得回退旧有效记录。校准页保持0x0803F800，位置日志单独使用0x0803F000的2KB页，程序区收缩到0x3F000；位置页满后只在全部停稳时擦除。
**Tech Stack:** FT32F103VC，厂商标准Flash驱动，Keil ARMCC5，GBK C源文件，UTF8 JavaScript有界行为检查。
**Spec:** RequirementDocument.md（最新第3、5、7条及用户确认“停稳后保存，正常断电直接回中；运动中断电重新寻限位”）。

## Global Constraints

- 所有新函数实现及参数、变量、宏均有中文注释；保持现有GBK编码和用户PID/方向/速度调参。
- 正常断电直接回中仅以可信停稳记录为依据，断电期间不得外力移动；不使用虚构绝对位置，不写入移动中的位置。
- 运动前失效操作成功后才启用PID/PWM，失效失败阻止动作并报告MotorFault。
- 任意轴开始移动都使全局位置记录无效，避免另一个轴运动时再擦写Flash丢失编码采样。
- 只在启用电机全部Reset=0、En=1、Action=stop、Fault=NoFault，且无待执行任务时保存位置；一个停止周期只尝试一次，位置不变不重复保存。
- 校准36字节版本1记录和地址0x0803F800保持兼容；新增位置页0x0803F000，两个页都与程序链接区域隔离。
- 新位置记录建议5个u32：完成标记、版本/mask、MOT1脉冲位置、MOT3脉冲位置、CRC。CRC覆盖后4字中的版本/两位置，标记最后写；无效标记通过只清位的方式原地写0。
- 只认最新占用槽，包括无效/部分写入的槽；旧有效记录不能越过最新无效槽被加载。空白、CRC/版本/mask错误、位置越界时保持旧寻限位归位。
- 首次完整自检、MOT2限位流程、最终归位霍尔保护和超时停机仍有效。
- 不生成或烧录固件；实际尝试Keil构建并报告许可证阻断边界。JavaScript转换检查不代替C编译或实机验证。
- 不操作Git索引或提交；主代理保留原有未提交修改并选择性提交。
- 不再派发子代理；独立审查由主代理执行。

## Task 1: 完整位置检查点生命周期

**Files:**
- Modify: USER/Sourse/flash.c, USER/Include/flash.h
- Modify: USER/Sourse/motor.c, USER/Include/motor.h
- Modify: USER/Sourse/mot1.c, USER/Sourse/mot2.c, USER/Sourse/mot3.c（仅运动准备钩子和MOT1/MOT3启动/最终完成判断）
- Modify: robot_calibration.sct, robot.uvprojx
- Test: tests/motor_calibration.test.js（保留当前46例及MOT2限位12例）；可另建 tests/motor_position.test.js

**Interfaces:**
```c
typedef struct {
    u32 Motor1Pulse; // MOT1停稳后的绝对脉冲坐标。
    u32 Motor3Pulse; // MOT3停稳后的绝对脉冲坐标。
} FlashMotorPositionData;
u8 FlashMotorPositionRead(FlashMotorPositionData *data, u8 motorMask);
u8 FlashMotorPositionWrite(const FlashMotorPositionData *data, u8 motorMask);
u8 FlashMotorPositionInvalidate(void);
extern u8 MotorPositionLoaded; // 本次启动位置检查点是否可信。
u8 MotorPositionPrepareMove(void); // 成功失效后返回1，失败返回0阻止运动。
```
FlashCalibrationWrite还必须先失效位置记录，防止新校准配上旧可信位置。
MotorInit在校准加载后加载位置并验证position<=MaxPulseNum，恢复CurPosition/CurAngle；校准无效时不应用位置。
MotorPositionPrepareMove在每个MotXMoveForward/Backward的TarPulseNum非零分支、EnablePidControl前调用；本次运动阶段已失效后不再写Flash。
MotorService在旧校准保存服务后处理停稳位置服务；记录保存失败每停止周期只尝试一次，待下次运动再重置。
MOT1/MOT3在Reset且Step=0、校准和位置均可信时，先于switch走直接回中分支：CurPosition<Max/2时Forward，>时Backward，相等时保持PWM关闭并标记归位完成；有动作进入Step3。
Step3允许可信位置分支的正/反脉冲制动，仍要求两端霍尔均0、无故障。位置不可信时继续当前case0/1寻一端限位归位。
MOT2不跳过case0，始终向下限位归位。

- [x] Step 1: 写行为测试并在现有源码上观察预期失败。覆盖低/中/高位置的直接回中、首次无记录及最新失效记录走限位、错误CRC/越界回退、移动前失效失败拒绝PWM、多轴运动不再次写Flash、校准重写失效旧位置、追加顺序/标记最后、部分写入及满页擦除期间断电、停稳只写一次。
- [x] Step 2: 实现Flash日志读/追加/原地失效与故障锁回，读取遵循最新占用槽规则，满页只擦位置页，校准格式不变。
- [x] Step 3: 实现motor.c的加载/运动准备/停稳保存生命周期，接入六个运动函数钩子及两轴直接回中。
- [x] Step 4: 更新Keil链接和存储区预留到0x3F000，运行新检查、原46例校准和12例MOT2限位检查；报告具体执行结果及原生编译限制。
- [x] Step 5: 自查所有编码、注释、旧运动流程及用户调参未改变，提交实现报告（不做Git提交）。

## 验证命令与边界

无本地Node环境时，通过functions.exec读取源文件（GBK），测试JS（UTF8），以new Function(testText+ "\\nreturn run...;")调用实际C函数转换后的检查；硬件Flash地址访问/寄存器和PWM输出为最底层模拟，其余记录与状态机执行真实代码。测试必须验证地址范围和顺序，不用源码包含某字样代替行为验证。
Keil以Start-Process UV4.exe -b robot.uvprojx -t FT32F103 -o日志 -WindowStyle Hidden调用；已有许可证错误不等于源码编译通过。



完成记录：主代理直接实现（实现代理额度不可用），独立审查修复一次重要失败边界；51+46+12行为检查通过，授权新版Keil重建0错误7原警告，HEX范围校验通过，未烧录。工作区已有用户调参保持，主代理选择性提交。
