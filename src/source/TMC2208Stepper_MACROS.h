#ifndef TMC2208Stepper_MACROS_H
#define TMC2208Stepper_MACROS_H
#include "TMC2208Stepper.h"
#include "../TMC2208Stepper_REGDEFS.h"

#define UPDATE_REG(R) sendDatagram(TMC2208_WRITE|REG_##R, R##_sr);
#define READ_REG(R)   bool b = sendDatagram(TMC2208_READ|REG_##R, data); R##_sr = *data; return b;

#define MOD_REG(REG, SETTING) REG##_sr &= ~SETTING##_bm; REG##_sr |= ((uint32_t)B<<SETTING##_bp)&SETTING##_bm; UPDATE_REG(REG);

#define GET_BYTE(REG, SETTING) REG(&REG##_sr); 	return (REG##_sr&SETTING##_bm)	>>SETTING##_bp;
#define GET_BYTE_R(REG, SETTING) REG(&tmp_sr); 	return (tmp_sr&SETTING##_bm)	>>SETTING##_bp;

#endif
