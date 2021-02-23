/*! \file  RFFC2072A.h
    \brief Header for communication with RFFC2072a module.
    \author Mohammad Akbarizadeh
    \date november,2,2021
    
    RFFC2072a basic communication Implemented.
*/

#ifndef _RFFC2072A_H
#define _RFFC2072A_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

// Pin defines - Change to suit
#define RFFC2072A_ENX_PORT     GPIOB
#define RFFC2072A_ENX_PIN      GPIO_PIN_5
#define RFFC2072A_CLK_PORT     GPIOB
#define RFFC2072A_CLK_PIN      GPIO_PIN_6
#define RFFC2072A_DATA_PORT    GPIOB
#define RFFC2072A_DATA_PIN     GPIO_PIN_7
#define RFFC2072A_DOUT_PORT    GPIOB
#define RFFC2072A_DOUT_PIN     GPIO_PIN_8

typedef struct regs
{
  uint16_t reg[31];
//	uint16_t lf;
//	uint16_t xo;
//	uint16_t cal_time;
//	uint16_t vco_ctrl;
//	uint16_t ct_cal1;
//	uint16_t ct_cal2;
//	uint16_t pll_cal1;
//	uint16_t pll_cal2;
//	uint16_t vco_auto;
//	uint16_t pll_ctrl;
//	uint16_t pll_bias;
//	uint16_t mix_count;
//	uint16_t p1_freq1;
//	uint16_t p1_freq2;
//	uint16_t p1_freq3;
//	uint16_t p2_freq1;
//	uint16_t p2_freq2;
//	uint16_t p2_freq3;
//	uint16_t fn_ctrl;
//	uint16_t ext_mode;
//	uint16_t fmod;
//	uint16_t sdi_ctrl;
//	uint16_t gpo;
//	uint16_t t_vco;
//	uint16_t iqmod1;
//	uint16_t iqmod2;
//	uint16_t iqmod3;
//	uint16_t iqmod4;
//	uint16_t t_ctrl;
//	uint16_t dev_ctrl;
//	uint16_t test;
} REGS;	  

typedef struct freqs
{
	uint16_t n;
	uint16_t lodiv;
	uint16_t nmsb;
	uint16_t nlsb;
} FREQS;
	  
static void send_1(uint8_t *data, int nbits);

static void rffc_write(uint8_t *addr, uint8_t *data);

static void set_bits(uint16_t *in, uint16_t *out, int inbits, int msb_set_bit);

static void uint16to8(uint16_t *data16, uint8_t *data8);

static void calc_freqs(FREQS *out,int lo_div, int f_PD, int f_LO, int f_VCOMax);

static void init_defaults(REGS *r);

static void setup_device_operation(REGS *r, uint8_t _3wire, uint8_t multiSclice, uint8_t full_duplex, uint8_t p1mixidd, uint8_t p2mixidd);

static void set_additional_features(REGS *r, uint8_t gpo_p1[6], uint8_t gpo_p2[6], int gate, int lockOutputSignal, int _4wire, int freqModulator, int modsetup, int modstep, uint16_t modulation);

static void set_operating_freq(REGS *r,int autoSel, uint16_t autoVCO, uint16_t p1vcosel, uint16_t p2vcosel, int autoCTcal, uint16_t p1ct, uint16_t p1ctdef, uint16_t p2ct, uint16_t p2ctdef, float VCO_freq, int lo_div1, int f_PD1, int f_LO1, int f_VCOMax1, int lo_div2, int f_PD2, int f_LO2, int f_VCOMax2);

static void set_calibration_mode(REGS *r, int en_p1, int en_p2);

static void enable_device(REGS *r, int control_method);

#endif

