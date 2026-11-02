#ifndef CTRL_ADVANCED_H
#define CTRL_ADVANCED_H

/* 根据 q 轴电流参考 (A) 计算 MTPA 的 d 轴电流参考 (A)。 */
float CTRL_Calc_MTPA(float iq_target);

/* 根据电压利用率和 q 轴电流状态计算弱磁 d 轴参考 (A)。 */
float CTRL_Calc_FieldWeakening(float vd, float vq, float vbus, float iq_meas, float iq_ref, float current_fw_ref, float dt);

#endif
