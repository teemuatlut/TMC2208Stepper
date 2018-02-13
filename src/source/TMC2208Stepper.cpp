#include <Stream.h>
#include "TMC2208Stepper.h"
#include "TMC2208Stepper_MACROS.h"

//TMC2208Stepper::TMC2208Stepper(HardwareSerial& SR) : TMC_SERIAL(SR) {}
template<class SERIAL_TYPE>
TMC2208Stepper<SERIAL_TYPE>::TMC2208Stepper(SERIAL_TYPE * ser, bool has_rx) {
	tmc_serial = ser;
	write_only = !has_rx;
}

/*	
	Requested current = mA = I_rms/1000
	Equation for current:
	I_rms = (CS+1)/32 * V_fs/(R_sense+0.02ohm) * 1/sqrt(2)
	Solve for CS ->
	CS = 32*sqrt(2)*I_rms*(R_sense+0.02)/V_fs - 1
	
	Example:
	vsense = 0b0 -> V_fs = 0.325V
	mA = 1640mA = I_rms/1000 = 1.64A
	R_sense = 0.10 Ohm
	->
	CS = 32*sqrt(2)*1.64*(0.10+0.02)/0.325 - 1 = 26.4
	CS = 26
*/
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::rms_current(uint16_t mA, float multiplier, float RS) {
	Rsense = RS;
	uint8_t CS = 32.0*1.41421*mA/1000.0*(Rsense+0.02)/0.325 - 1;
	// If Current Scale is too low, turn on high sensitivity R_sense and calculate again
	if (CS < 16) {
		vsense(true);
		CS = 32.0*1.41421*mA/1000.0*(Rsense+0.02)/0.180 - 1;
	} else if(vsense()) { // If CS >= 16, turn off high_sense_r if it's currently ON
		vsense(false);
	}
	irun(CS);
	ihold(CS*multiplier);
	mA_val = mA;
}

template<class SERIAL_TYPE> uint16_t TMC2208Stepper<SERIAL_TYPE>::rms_current() {
	return (float)(irun()+1)/32.0 * (vsense()?0.180:0.325)/(Rsense+0.02) / 1.41421 * 1000;
}

template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::setCurrent(uint16_t mA, float Rsense, float multiplier) { rms_current(mA, multiplier, Rsense); }
template<class SERIAL_TYPE> uint16_t TMC2208Stepper<SERIAL_TYPE>::getCurrent() {	return mA_val; }

template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::checkOT() {
	uint32_t response;
	DRV_STATUS(&response);
	if (response & OTPW_bm) {
		flag_otpw = true;
		return true; // bit 26 for overtemperature warning flag
	}
	return false;
}

template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::getOTPW() { return flag_otpw; }

template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::clear_otpw() {	flag_otpw = false; }

template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::isEnabled() { return enn() && toff(); }

template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::microsteps(uint16_t ms) {
	switch(ms) {
		case 256: mres(0); break;
		case 128: mres(1); break;
		case  64: mres(2); break;
		case  32: mres(3); break;
		case  16: mres(4); break;
		case   8: mres(5); break;
		case   4: mres(6); break;
		case   2: mres(7); break;
		case   0: mres(8); break;
		default: break;
	}
}

template<class SERIAL_TYPE> uint16_t TMC2208Stepper<SERIAL_TYPE>::microsteps() {
	switch(mres()) {
		case 0: return 256;
		case 1: return 128;
		case 2: return  64;
		case 3: return  32;
		case 4: return  16;
		case 5: return   8;
		case 6: return   4;
		case 7: return   2;
		case 8: return   0;
	}
	return 0;
}

template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::blank_time(uint8_t B) {
	switch (B) {
		case 16: tbl(0b00); break;
		case 24: tbl(0b01); break;
		case 36: tbl(0b10); break;
		case 54: tbl(0b11); break;
	}
}

template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::blank_time() {
	switch (tbl()) {
		case 0b00: return 16;
		case 0b01: return 24;
		case 0b10: return 36;
		case 0b11: return 54;
	}
	return 0;
}

template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::calcCRC(uint8_t datagram[], uint8_t len) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) {
		uint8_t currentByte = datagram[i];
		for (uint8_t j = 0; j < 8; j++) {
			if ((crc >> 7) ^ (currentByte & 0x01)) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc = (crc << 1);
			}
			crc &= 0xff;
			currentByte = currentByte >> 1;
		} 
	}
	return crc;
}

template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::sendDatagram(uint8_t addr, uint32_t regVal, uint8_t len) {
	uint8_t datagram[] = {TMC2208_SYNC, TMC2208_SLAVE_ADDR, addr, (uint8_t)(regVal>>24), (uint8_t)(regVal>>16), (uint8_t)(regVal>>8), (uint8_t)(regVal>>0), 0x00};

	datagram[len] = calcCRC(datagram, len);

	for(int i=0; i<=len; i++){
		bytesWritten += tmc_serial->write(datagram[i]);
	}
}

template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::sendDatagram(uint8_t addr, uint32_t *data, uint8_t len) {
	uint8_t datagram[] = {TMC2208_SYNC, TMC2208_SLAVE_ADDR, addr, 0x00};
	datagram[len] = calcCRC(datagram, len);

	while (tmc_serial->available() > 0) tmc_serial->read(); // Flush

	for(int i=0; i<=len; i++) tmc_serial->write(datagram[i]);
	
	tmc_serial->flush(); // Wait for TX to finish
	for(int byte=0; byte<4; byte++) tmc_serial->read(); // Flush bytes written
	delay(replyDelay);

	uint64_t out = 0x00000000UL;
	while(tmc_serial->available() > 0) {
		uint8_t res = tmc_serial->read();
		out <<= 8;
		out |= res&0xFF;
	}

	uint8_t out_datagram[] = {(uint8_t)(out>>56), (uint8_t)(out>>48), (uint8_t)(out>>40), (uint8_t)(out>>32), (uint8_t)(out>>24), (uint8_t)(out>>16), (uint8_t)(out>>8), (uint8_t)(out>>0)};
	if (calcCRC(out_datagram, 7) == (uint8_t)(out&0xFF)) {
		*data = out>>8;
		return 0;
	} else {
		return 1;
	}
}

// GSTAT
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::GSTAT(uint32_t *data) {
	if (write_only) {
		*data = GSTAT_sr;
		return 0;
	}
	READ_REG(GSTAT);
}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::GSTAT(uint32_t input) {
	GSTAT_sr = input;
	UPDATE_REG(GSTAT);
}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::GSTAT() {
	uint32_t data = 0;
	GSTAT(&data);
	return data;
}

template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::reset(bool B)	{ MOD_REG(GSTAT, RESET); 	}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::drv_err(bool B){ MOD_REG(GSTAT, DRV_ERR); 	}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::uv_cp(bool B)	{ MOD_REG(GSTAT, UV_CP); 	}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::reset()		{ GET_BYTE(GSTAT, RESET);	}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::drv_err()		{ GET_BYTE(GSTAT, DRV_ERR);	}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::uv_cp()		{ GET_BYTE(GSTAT, UV_CP);	}

// IFCNT
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::IFCNT(uint32_t *data) {
	bool b = sendDatagram(TMC2208_READ|REG_IFCNT, data);
	return b;
}

// SLAVECONF
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::SLAVECONF(uint32_t input) {
	SLAVECONF_sr = input&SLAVECONF_bm;
	UPDATE_REG(SLAVECONF);
}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::SLAVECONF(uint32_t *data) {
	*data = SLAVECONF_sr;
	return 0;
}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::senddelay(uint8_t B) 	{ MOD_REG(SLAVECONF, SENDDELAY);	}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::senddelay() 		{ GET_BYTE(SLAVECONF, SENDDELAY); 	}

// OTP_PROG
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::OTP_PROG(uint32_t input) {
	OTP_PROG_sr = input;
	UPDATE_REG(OTP_PROG);
}

// IOIN
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::IOIN(uint32_t *data) {
	bool b = sendDatagram(TMC2208_READ|REG_IOIN, data);
	return b;
}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::enn()			{ GET_BYTE_R(IOIN, ENN);		}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::ms1()			{ GET_BYTE_R(IOIN, MS1);		}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::ms2()			{ GET_BYTE_R(IOIN, MS2);		}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::diag()			{ GET_BYTE_R(IOIN, DIAG);		}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::pdn_uart()		{ GET_BYTE_R(IOIN, PDN_UART);	}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::step()			{ GET_BYTE_R(IOIN, STEP);		}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::sel_a()		{ GET_BYTE_R(IOIN, SEL_A);		}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::dir()			{ GET_BYTE_R(IOIN, DIR);		}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::version() 	{ GET_BYTE_R(IOIN, VERSION);	}

// FACTORY_CONF
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::FACTORY_CONF(uint32_t *data) {
	if (write_only) {
		*data = FACTORY_CONF_sr;
		return 0;
	}
	READ_REG(FACTORY_CONF);
}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::FACTORY_CONF(uint32_t input) {
	FACTORY_CONF_sr = input;
	UPDATE_REG(FACTORY_CONF);
}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::fclktrim(uint8_t B){ MOD_REG(FACTORY_CONF, FCLKTRIM);	}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::ottrim(uint8_t B)	{ MOD_REG(FACTORY_CONF, OTTRIM);	}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::fclktrim()		{ GET_BYTE(FACTORY_CONF, FCLKTRIM);	}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::ottrim()		{ GET_BYTE(FACTORY_CONF, OTTRIM);	}

// IHOLD_IRUN
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::IHOLD_IRUN(uint32_t input) {
	IHOLD_IRUN_sr = input;
	UPDATE_REG(IHOLD_IRUN);
}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::IHOLD_IRUN(uint32_t *data) {
	*data = IHOLD_IRUN_sr;
	return 0;
}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::ihold(uint8_t B) 		{ MOD_REG(IHOLD_IRUN, IHOLD);		}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::irun(uint8_t B)  		{ MOD_REG(IHOLD_IRUN, IRUN); 		}
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::iholddelay(uint8_t B)	{ MOD_REG(IHOLD_IRUN, IHOLDDELAY); 	}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::ihold() 			{ GET_BYTE(IHOLD_IRUN, IHOLD);		}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::irun()  			{ GET_BYTE(IHOLD_IRUN, IRUN); 		}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::iholddelay()  		{ GET_BYTE(IHOLD_IRUN, IHOLDDELAY);	}

// TPOWERDOWN
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::TPOWERDOWN(uint32_t input) {
	TPOWERDOWN_sr = input;
	UPDATE_REG(TPOWERDOWN);
}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::TPOWERDOWN(uint32_t *data) {
	*data = TPOWERDOWN_sr;
	return 0;
}

// TSTEP
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::TSTEP(uint32_t *data) {
	bool b = sendDatagram(TMC2208_READ|REG_TSTEP, data);
	return b;
}

// TPWMTHRS
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::TPWMTHRS(uint32_t input) {
	TPWMTHRS_sr = input;
	UPDATE_REG(TPWMTHRS);
}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::TPWMTHRS(uint32_t *data) {
	*data = TPWMTHRS_sr;
	return 0;
}
template<class SERIAL_TYPE> uint32_t TMC2208Stepper<SERIAL_TYPE>::TPWMTHRS() {
	return TPWMTHRS_sr;
}

// VACTUAL
template<class SERIAL_TYPE> void TMC2208Stepper<SERIAL_TYPE>::VACTUAL(uint32_t input) {
	VACTUAL_sr = input;
	UPDATE_REG(VACTUAL);
}
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::VACTUAL(uint32_t *data) {
	*data = VACTUAL_sr;
	return 0;
}

// MSCNT
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::MSCNT(uint32_t *data) {
	bool b = sendDatagram(TMC2208_READ|REG_MSCNT, data);
	return b;
}

// MSCURACT
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::MSCURACT(uint32_t *data) {
	bool b = sendDatagram(TMC2208_READ|REG_MSCURACT, data);
	return b;
}
template<class SERIAL_TYPE> uint16_t TMC2208Stepper<SERIAL_TYPE>::cur_a() { GET_BYTE_R(MSCURACT, CUR_A);	}
template<class SERIAL_TYPE> uint16_t TMC2208Stepper<SERIAL_TYPE>::cur_b() { GET_BYTE_R(MSCURACT, CUR_B);	}

// MSCNT
template<class SERIAL_TYPE> bool TMC2208Stepper<SERIAL_TYPE>::PWM_SCALE(uint32_t *data) {
	bool b = sendDatagram(TMC2208_READ|REG_PWM_SCALE, data);
	return b;
}
template<class SERIAL_TYPE> uint8_t TMC2208Stepper<SERIAL_TYPE>::pwm_scale_sum() { GET_BYTE_R(PWM_SCALE, PWM_SCALE_SUM); }
template<class SERIAL_TYPE> int16_t TMC2208Stepper<SERIAL_TYPE>::pwm_scale_auto() {
	// Not two's complement? 9nth bit determines sign
	uint32_t d;
	PWM_SCALE(&d);
	int16_t response = (d>>PWM_SCALE_AUTO_bp)&0xFF;
	if (((d&PWM_SCALE_AUTO_bm) >> 24) & 0x1) return -response;
	else return response;
}