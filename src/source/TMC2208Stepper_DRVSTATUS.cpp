#include "TMC2208Stepper.h"
#include "TMC2208Stepper_MACROS.h"

bool TMC2208Stepper::DRV_STATUS(uint32_t *data) {
	bool b = sendDatagram(READ|REG_DRV_STATUS, data);
	return b;
}

bool TMC2208Stepper::otpw()		{ GET_BYTE_R(DRV_STATUS, OTPW); 	}
bool TMC2208Stepper::ot() 		{ GET_BYTE_R(DRV_STATUS, OT); 	 	}
bool TMC2208Stepper::s2ga() 	{ GET_BYTE_R(DRV_STATUS, S2GA); 	}
bool TMC2208Stepper::s2gb() 	{ GET_BYTE_R(DRV_STATUS, S2GB); 	}
bool TMC2208Stepper::s2vsa() 	{ GET_BYTE_R(DRV_STATUS, S2VSA);	}
bool TMC2208Stepper::s2vsb() 	{ GET_BYTE_R(DRV_STATUS, S2VSB);	}
bool TMC2208Stepper::ola() 		{ GET_BYTE_R(DRV_STATUS, OLA);  	}
bool TMC2208Stepper::olb() 		{ GET_BYTE_R(DRV_STATUS, OLB);  	}
bool TMC2208Stepper::t120() 	{ GET_BYTE_R(DRV_STATUS, T120); 	}
bool TMC2208Stepper::t143() 	{ GET_BYTE_R(DRV_STATUS, T143); 	}
bool TMC2208Stepper::t150() 	{ GET_BYTE_R(DRV_STATUS, T150); 	}
bool TMC2208Stepper::t157() 	{ GET_BYTE_R(DRV_STATUS, T157); 	}
bool TMC2208Stepper::stealth() 	{ GET_BYTE_R(DRV_STATUS, STEALTH);	}
bool TMC2208Stepper::stst() 	{ GET_BYTE_R(DRV_STATUS, STST); 	}

