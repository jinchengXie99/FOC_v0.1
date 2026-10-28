#ifndef CTRL_ADVANCED_H
#define CTRL_ADVANCED_H

float CTRL_Calc_MTPA(float iq_target);
float CTRL_Calc_FieldWeakening(float vd, float vq, float vbus, float iq_meas, float iq_ref, float current_fw_ref, float dt);

#endif
