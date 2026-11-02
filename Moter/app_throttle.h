#ifndef APP_THROTTLE_H
#define APP_THROTTLE_H

/* Maps low ADC voltage to reverse RPM, center to zero, and high voltage to forward RPM. */
void APP_Throttle_Update(float v_raw, float *target_rpm);

#endif
