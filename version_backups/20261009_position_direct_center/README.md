# 2026-10-09 停稳位置直接回中版本备份

源码及当前调参保存到本地 Git 标签 `backup-position-direct-center-20261009`，包含本轮备份时的工作区修改。

本版行为：停稳后保存位置；重启可信位置时 MOT1/MOT3 跳过 case0 直接回中，MOT2 回最低限位；运动中断电重新寻限位。断电期间机构不得外力移动。

- `robot.hex`：已编译固件。
- `robot.axf`：同次编译的调试镜像。
- `build.log`：0 错误、7 个已有警告的完整编译记录。
- `SHA256.txt`：备份文件校验值。

备份前 51 个位置、46 个校准及 12 个限位检查通过；未烧录和实机验证。

在新分支恢复本版源码：

```powershell
git switch -c restore-position-direct-center backup-position-direct-center-20261009
```

当前 main 随后的提交恢复“先寻零点限位再回中”流程。
