#include "TMC2208Stepper.h"
#include "TMC2208Stepper_MACROS.h"

// GCONF
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::GCONF(uint32_t *data) {
	if (write_only) {
		*data = GCONF_sr;
		return 0;
	}
	READ_REG(GCONF);
}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::GCONF(uint32_t input) {
	GCONF_sr = input;
	UPDATE_REG(GCONF);
}


template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::I_scale_analog(bool B)		{ MOD_REG(GCONF, I_SCALE_ANALOG);	}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::internal_Rsense(bool B)	{ MOD_REG(GCONF, INTERNAL_RSENSE);	}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::en_spreadCycle(bool B)		{ MOD_REG(GCONF, EN_SPREADCYCLE);	}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::shaft(bool B) 				{ MOD_REG(GCONF, SHAFT);			}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::index_otpw(bool B)			{ MOD_REG(GCONF, INDEX_OTPW);		}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::index_step(bool B)			{ MOD_REG(GCONF, INDEX_STEP);		}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::pdn_disable(bool B)		{ MOD_REG(GCONF, PDN_DISABLE);		}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::mstep_reg_select(bool B)	{ MOD_REG(GCONF, MSTEP_REG_SELECT);	}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::multistep_filt(bool B)		{ MOD_REG(GCONF, MULTISTEP_FILT);	}

template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::I_scale_analog()	{ GET_BYTE(GCONF, I_SCALE_ANALOG);	}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::internal_Rsense()	{ GET_BYTE(GCONF, INTERNAL_RSENSE);	}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::en_spreadCycle()	{ GET_BYTE(GCONF, EN_SPREADCYCLE);	}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::shaft()			{ GET_BYTE(GCONF, SHAFT);			}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::index_otpw()		{ GET_BYTE(GCONF, INDEX_OTPW);		}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::index_step()		{ GET_BYTE(GCONF, INDEX_STEP);		}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::pdn_disable()		{ GET_BYTE(GCONF, PDN_DISABLE);		}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::mstep_reg_select()	{ GET_BYTE(GCONF, MSTEP_REG_SELECT);}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::multistep_filt()	{ GET_BYTE(GCONF, MULTISTEP_FILT);	}
