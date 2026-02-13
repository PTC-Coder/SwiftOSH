/* StatusLED.h — SwiftOSH (STM32U545RET6Q)
 * LED pattern abstraction via LPTIM2 PWM on LSE.
 *
 * SwiftOSH LED pins (active-high, GPIO_PIN_SET = ON):
 *   RED  = PC1
 *   GREEN = PA1
 *   BLUE  = PA2
 *
 * LPTIM2 output is routed to one or more LED pins via AF.
 * On STM32U5, check the AF mapping for LPTIM2_OUT.
 */
#ifndef __STATUSLED_H__
#define __STATUSLED_H__

void StatusLED_Initialize(void);
void StatusLED_BlinkRedLED(void);
void StatusLED_BlinkGreenLED(void);
void StatusLED_BlinkBlueLED(void);
void StatusLED_BlinkBlueGreenLED(void);
void StatusLED_BlinkRedBlueGreenLED(void);
void StatusLED_SolidBlueLED(void);
void StatusLED_SolidGreenLED(void);
void StatusLED_SolidAllLED(void);
void StatusLED_AllOutputs(void);
void StatusLED_LowBatteryMode(void);
void StatusLED_BlinkRedBlueLED(void);

#endif /* __STATUSLED_H__ */
