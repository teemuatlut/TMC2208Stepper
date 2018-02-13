#include "TMC2208Stepper.h"
#include "TMC2208Stepper_MACROS.h"

template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::DRV_STATUS(uint32_t *data) {
	if (write_only) return 1;
	bool b = sendDatagram(TMC2208_READ|REG_DRV_STATUS, data);
	return b;
}
template<class SERIAL_TYPE> uint32_t TMC2208Stepper<SERIAL_TYPE>::DRV_STATUS() {
	uint32_t data = 0;
	DRV_STATUS(&data);
	return data;
}

template<class SERIAL_TYPE> bool 		TMC2208Stepper<SERIAL_TYPE>::otpw()		{ GET_BYTE_R(DRV_STATUS, OTPW); 	}
template<class SERIAL_TYPE> bool 		TMC2208Stepper<SERIAL_TYPE>::ot() 		{ GET_BYTE_R(DRV_STATUS, OT); 	 	}
template<class SERIAL_TYPE> bool 		TMC2208Stepper<SERIAL_TYPE>::s2ga() 		{ GET_BYTE_R(DRV_STATUS, S2GA); 	}
template<class SERIAL_TYPE> bool 		TMC2208Stepper<SERIAL_TYPE>::s2gb() 		{ GET_BYTE_R(DRV_STATUS, S2GB); 	}
template<class SERIAL_TYPE> bool 		TMC2208Stepper<SERIAL_TYPE>::s2vsa() 	{ GET_BYTE_R(DRV_STATUS, S2VSA);	}
template<class SERIAL_TYPE> bool 		TMC2208Stepper<SERIAL_TYPE>::s2vsb() 	{ GET_BYTE_R(DRV_STATUS, S2VSB);	}
template<class SERIAL_TYPE> bool 		TMC2208Stepper<SERIAL_TYPE>::ola() 		{ GET_BYTE_R(DRV_STATUS, OLA);  	}
template<class SERIAL_TYPE> bool 		TMC2208Stepper<SERIAL_TYPE>::olb() 		{ GET_BYTE_R(DRV_STATUS, OLB);  	}
template<class SERIAL_TYPE> bool 		TMC2208Stepper<SERIAL_TYPE>::t120() 		{ GET_BYTE_R(DRV_STATUS, T120); 	}
template<class SERIAL_TYPE> bool 		TMC2208Stepper<SERIAL_TYPE>::t143() 		{ GET_BYTE_R(DRV_STATUS, T143); 	}
template<class SERIAL_TYPE> bool 		TMC2208Stepper<SERIAL_TYPE>::t150() 		{ GET_BYTE_R(DRV_STATUS, T150); 	}
template<class SERIAL_TYPE> bool 		TMC2208Stepper<SERIAL_TYPE>::t157() 		{ GET_BYTE_R(DRV_STATUS, T157); 	}
template<class SERIAL_TYPE> uint16_t 	TMC2208Stepper<SERIAL_TYPE>::cs_actual()	{ GET_BYTE_R(DRV_STATUS, CS_ACTUAL);}
template<class SERIAL_TYPE> bool 		TMC2208Stepper<SERIAL_TYPE>::stealth() 	{ GET_BYTE_R(DRV_STATUS, STEALTH);	}
template<class SERIAL_TYPE> bool 		TMC2208Stepper<SERIAL_TYPE>::stst() 		{ GET_BYTE_R(DRV_STATUS, STST); 	}
