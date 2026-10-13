#ifndef __FLASH_H__
#define __FLASH_H__ // Flash校准存储接口头文件保护标志。

#include "ft32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FlashCalibrationMotorCount 3 // 校准数据中的电机数量。

typedef struct
{
    u32 MaxPulseNum[FlashCalibrationMotorCount]; // 按MOT1、MOT2、MOT3顺序保存的全行程脉冲数。
    u32 PulseScale[FlashCalibrationMotorCount]; // 对应电机的角度或毫米脉冲换算系数。
} FlashCalibrationData;

/** 函数作用：初始化Flash存储访问状态并锁定写保护，不擦除已有校准；无传入参数。 */
void FlashInit(void);
/**
 * 函数作用：读取并校验持久化记录，成功后填充校准数据。
 * data：输出数据指针，失败时内容不变；motorMask：启用电机位图，bit0至bit2对应MOT1至MOT3。
 * 返回值：1表示读取成功，0表示记录无效或参数错误；电机行程范围由调用方校验。
 */
u8 FlashCalibrationRead(FlashCalibrationData *data, u8 motorMask);
/**
 * 函数作用：将校准数据写入预留页并读回校验，最后写成功标记。
 * data：调用方已校验行程和换算系数的数据；motorMask：启用电机位图，范围1至7。
 * 返回值：1表示保存成功，0表示参数或擦写校验失败；调用前必须确保电机已停止。
 */
u8 FlashCalibrationWrite(const FlashCalibrationData *data, u8 motorMask);

#ifdef __cplusplus
}
#endif

#endif
