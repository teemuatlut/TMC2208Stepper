#include "TMC2208Stepper.h"
#include "TMC2208Stepper_MACROS.h"

// PWMCONF
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::PWMCONF(uint32_t *data) {
	if (write_only) {
		*data = PWMCONF_sr;
		return 0;
	}
	READ_REG(PWMCONF);
}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::PWMCONF(uint32_t input) {
	PWMCONF_sr = input;
	UPDATE_REG(PWMCONF);
}

template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::pwm_ofs		( uint8_t B ) { MOD_REG(PWMCONF, PWM_OFS); 		}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::pwm_grad		( uint8_t B ) { MOD_REG(PWMCONF, PWM_GRAD); 	}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::pwm_freq		( uint8_t B ) { MOD_REG(PWMCONF, PWM_FREQ); 	}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::pwm_autoscale	( bool 	  B ) { MOD_REG(PWMCONF, PWM_AUTOSCALE);}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::pwm_autograd	( bool    B ) { MOD_REG(PWMCONF, PWM_AUTOGRAD); }
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::freewheel		( uint8_t B ) { MOD_REG(PWMCONF, FREEWHEEL); 	}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::pwm_reg		( uint8_t B ) { MOD_REG(PWMCONF, PWM_REG); 		}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::pwm_lim		( uint8_t B ) { MOD_REG(PWMCONF, PWM_LIM); 		}

template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::pwm_ofs()		{ GET_BYTE(PWMCONF, PWM_OFS);		}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::pwm_grad()		{ GET_BYTE(PWMCONF, PWM_GRAD);		}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::pwm_freq()		{ GET_BYTE(PWMCONF, PWM_FREQ);		}
template<class SERIAL_TYPE> bool 	TMC2208Stepper<SERIAL_TYPE>::pwm_autoscale()	{ GET_BYTE(PWMCONF, PWM_AUTOSCALE);	}
template<class SERIAL_TYPE> bool 	TMC2208Stepper<SERIAL_TYPE>::pwm_autograd()	{ GET_BYTE(PWMCONF, PWM_AUTOGRAD);	}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::freewheel()		{ GET_BYTE(PWMCONF, FREEWHEEL);		}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::pwm_reg()		{ GET_BYTE(PWMCONF, PWM_REG);		}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::pwm_lim()		{ GET_BYTE(PWMCONF, PWM_LIM);		}

