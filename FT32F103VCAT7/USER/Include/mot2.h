#ifndef __MOT2_H__
#define __MOT2_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "sys.h"

typedef struct {
	u8  Reset;
	u8  Action;
	u8  Step;
	u8  PidTimer;
	u16 PidTimerB;
	u8  PidEn;
	u8  PulseSta;
	u8  PulseDeb;
	u16 PulseDebB;
	u32 PulseNum; // 本次运动累计脉冲数。
	u32 PulseNumB; // 本测速周期累计脉冲数。
	u8  BrakeType;
	u32 ErrPulseNum; // 目标与实际脉冲误差，预留。
	u32 TarPulseNum; // 本次运动目标脉冲数。
	u32 MaxPulseNum; // 上下限位之间的标定脉冲数。
	u8	Hall1Sta;
	u8  Hall1Deb;
	u8	Hall2Sta;
	u8  Hall2Deb;
	u16 DelayTimer;
	u16 TarVelocity;
	u16 CurVelocity;
	u32 TarPosition; // 目标位置对应的脉冲数。
	u32 CurPosition; // 当前位置对应的脉冲数。
	u16 CurHeight;
	u16 TarHeight;
	u32 PulseNumPerMm; // 每毫米对应的缩放脉冲数。
	u32 PulseNumPerMin; // 当前每分钟脉冲数。
	u8  Faultflag;
	u8  En;
}Motor2St;

typedef struct
{
	u8 En;
	u8 Step;
	u16 Angle;
	u16 Height;
	u16 Speed;
	u16 SpeedB;
}Move2St;


extern Motor2St Mot2;
extern Move2St Move2;

void Mot2Encode(void);
void Mot2Init(void);
void Mot2MoveService(void);
void Move2SetTask( u16 Height, u16 Speed );

#ifdef __cplusplus
}
#endif

#endif
