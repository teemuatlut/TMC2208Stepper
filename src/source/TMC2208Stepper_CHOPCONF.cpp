#include "TMC2208Stepper.h"
#include "TMC2208Stepper_MACROS.h"

// CHOPCONF
template<class SERIAL_TYPE>
void TMC2208Stepper<SERIAL_TYPE>::CHOPCONF(uint32_t input) {
	CHOPCONF_sr = input;
	UPDATE_REG(CHOPCONF);
}
template<class SERIAL_TYPE>
bool TMC2208Stepper<SERIAL_TYPE>::CHOPCONF(uint32_t *data) {
	if (write_only) {
		*data = CHOPCONF_sr;
		return 0;
	}
	READ_REG(CHOPCONF);
}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::toff	( uint8_t  B )	{ MOD_REG(CHOPCONF, TOFF); 		}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::hstrt	( uint8_t  B )	{ MOD_REG(CHOPCONF, HSTRT); 	}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::hend	( uint8_t  B )	{ MOD_REG(CHOPCONF, HEND); 		}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::tbl	( uint8_t  B )	{ MOD_REG(CHOPCONF, TBL); 		}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::vsense	( bool     B )	{ MOD_REG(CHOPCONF, VSENSE); 	}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::mres	( uint16_t B )	{ MOD_REG(CHOPCONF, MRES); 		}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::intpol	( bool     B )	{ MOD_REG(CHOPCONF, INTPOL); 	}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::dedge	( bool     B )	{ MOD_REG(CHOPCONF, DEDGE);  	}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::diss2g	( bool     B )	{ MOD_REG(CHOPCONF, DISS2G); 	}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::diss2vs( bool     B )	{ MOD_REG(CHOPCONF, DISS2VS);	}

template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::toff()		{ GET_BYTE(CHOPCONF, TOFF); 	}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::hstrt()		{ GET_BYTE(CHOPCONF, HSTRT); 	}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::hend()		{ GET_BYTE(CHOPCONF, HEND); 	}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::tbl()		{ GET_BYTE(CHOPCONF, TBL); 		}
template<class SERIAL_TYPE> bool 	TMC2208Stepper<SERIAL_TYPE>::vsense()	{ GET_BYTE(CHOPCONF, VSENSE); 	}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::mres()		{ GET_BYTE(CHOPCONF, MRES); 	}
template<class SERIAL_TYPE> bool 	TMC2208Stepper<SERIAL_TYPE>::intpol()	{ GET_BYTE(CHOPCONF, INTPOL); 	}
template<class SERIAL_TYPE> bool 	TMC2208Stepper<SERIAL_TYPE>::dedge()		{ GET_BYTE(CHOPCONF, DEDGE);  	}
template<class SERIAL_TYPE> bool 	TMC2208Stepper<SERIAL_TYPE>::diss2g()	{ GET_BYTE(CHOPCONF, DISS2G); 	}
template<class SERIAL_TYPE> bool 	TMC2208Stepper<SERIAL_TYPE>::diss2vs()	{ GET_BYTE(CHOPCONF, DISS2VS);	}

template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::hysterisis_end(int8_t value) { hend(value+3); }
template<class SERIAL_TYPE> int8_t TMC2208Stepper<SERIAL_TYPE>::hysterisis_end() { return hend()-3; };

template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::hysterisis_start(uint8_t value) { hstrt(value-1); }
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::hysterisis_start() { return hstrt()+1; }
