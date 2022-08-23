#ifndef PWM_H
#define	PWM_H

// = 2*9999 (see pwm.c)
#define PWM_MAX 19998

// = 6V/9V * PWM_MAX
#define MOTOR_MAX 13332

#ifdef	__cplusplus
extern "C" {
#endif

    void initPWM();


#ifdef	__cplusplus
}
#endif

#endif	/* PWM_H */

