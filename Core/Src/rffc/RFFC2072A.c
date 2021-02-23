/*! \file RFFC2072A.c
    \brief Header for communication with RFFC2072a module.
    \author Mohammad Akbarizadeh
    \date november,2,2021
    
    RFFC2072a communication Implemented.
*/

#include "RFFC2072A.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "math.h"

#define LD long double 

#define LF 			0x00
#define XO 			0x01
#define CAL_TIME 	0x02
#define VCO_CTRL 	0x03
#define CT_CAL1 	0x04
#define CT_CAL2 	0x05
#define PLL_CAL1 	0x06
#define PLL_CAL2 	0x07
#define VCO_AUTO 	0x08
#define PLL_CTRL 	0x09
#define PLL_BIAS 	0x0A
#define MIX_CONT 	0x0B
#define P1_FREQ1 	0x0C
#define P1_FREQ2 	0x0D
#define P1_FREQ3 	0x0E
#define P2_FREQ1 	0x0F
#define P2_FREQ2 	0x10
#define P2_FREQ3 	0x11
#define FN_CTRL 	0x12
#define EXT_MOD 	0x13
#define FMOD 		0x14
#define SDI_CTRL 	0x15
#define GPO 		0x16
#define T_VCO 		0x17
#define IQMOD1 		0x18
#define IQMOD2 		0x19
#define IQMOD3 		0x1A
#define IQMOD4 		0x1B
#define T_CTRL 		0x1C
#define DEV_CTRL 	0x1D
#define TEST 		0x1E

// These functions turn a specific port and pin ON and OFF
// Change to fit your design
// In this implementation 2-wire is used
#define CLK_ON(n)		RFFC2072A_CLK_PORT->BSRR = n
#define CLK_OFF(n)		RFFC2072A_CLK_PORT->BSRR  = n << 16
#define DATA_ON(n)		RFFC2072A_DATA_PORT->BSRR = n
#define DATA_OFF(n)		RFFC2072A_DATA_PORT->BSRR  = n << 16
#define READ_B(n)		((GPIOB->IDR & n) != 0)
#define	SPI1_CLK_T			CLK_ON(RFFC2072A_CLK_PIN)
#define	SPI1_CLK_F			CLK_OFF(RFFC2072A_CLK_PIN)
#define	SPI1_DIO_T			DATA_ON(RFFC2072A_DATA_PIN)
#define	SPI1_DIO_F			DATA_OFF(RFFC2072A_DATA_PIN)



static void send_1(uint8_t *data, int nbits)
{
	/*! void send_1(uint8_t *data, int nbits)
		\brief send nbits of data
		
		\param data to be sent
		\param number of bits you want to send
	*/
	int i = nbits%8;
	uint8_t frame;
	if(i!=0)
		frame = 1<<i-1;
	else
		frame = 0;
	uint8_t tx_data=0xFF;
	if(i != 0)
		tx_data = data[(nbits/8)];
	while(frame)
	{
		SPI1_CLK_F;
		if ((tx_data & frame) == frame)
		{
			SPI1_DIO_T;
		}
		else
		{
			SPI1_DIO_F;
		}

		frame = (frame >> 1);
		SPI1_CLK_T;
	}
	for( i = (nbits/8)-1; i>=0;i--)
	{
		frame = 0x80;
		tx_data = data[i];
		while(frame)
		{
			SPI1_CLK_F;
			if ((tx_data & frame) == frame)
			{
				SPI1_DIO_T;
			}
			else
			{
				SPI1_DIO_F;
			}

			frame = (frame >> 1);
			SPI1_CLK_T;
		}
	}
	SPI1_CLK_F;
	SPI1_DIO_F;
}

static void rffc_write(uint8_t *addr, uint8_t *data)
{
	/*! void rffc_write(uint8_t *addr, uint8_t *data)
		\brief send data to the provided address
		
		\param register address
		\param data to be sent
	*/
	// set CLK & DATA to low
	SPI1_CLK_F;
	SPI1_DIO_F;
	// send a CLK
	SPI1_CLK_T;
	SPI1_CLK_F;
	// send the X bit
	SPI1_CLK_T;
	SPI1_CLK_F;
	// send write bit
	SPI1_CLK_T;
	SPI1_CLK_F;
	// send address
	send_1(addr,7);
	send_1(data,16);
}

static void set_bits(uint16_t *in, uint16_t *out, int inbits, int msb_set_bit)
{
	/*
		this function takes two inputs which are input number your want to \
		embed in the output number bits. the other two inputs are the number of\
		bits your input number has and the last one is the MSB of the input data in\
		output data
		\param input binary number to replace in the output
		\param output binary number you want to change bits
		\param number of bits you want to change (number of the input bits)
		\param MSB of the chunk in the output you want to change

		example: 
		in: 0x007E
		out: 0xACBF
		inbits: 7
		msb_set_bit: 14

		changes output:
		0xFEBF
	*/
	uint16_t mask = 0xffff;
	mask = ~((((uint16_t)(~*in)) % (1<<inbits)) << (msb_set_bit - inbits + 1)); // create a mask to make the desired bits 0
	(*out) &= mask;
	mask = 0;
	mask = (*in << (msb_set_bit - inbits + 1)); // create a mask to make the desired bits 1
	(*out) |= mask;
}

static void uint16to8(uint16_t *data16, uint8_t *data8)
{
	/*
		change the input 16-bit data into two bytes
		\param input 16-bit data
		\param output bytes
	*/
	// extract the 2 parts of 8-bit data from the 16-bit input
  	data8[1]= (*data16 >> 8) & 0xff ;
	// printf("(in >> 8) %x\n",(in >> 8));
  	data8[0]= (*data16) & 0xff ;
	// printf("(in) %x\n",(in));
}

static void init_defaults(REGS *r)
{
  printf("load defaults\n");
	r->reg[LF] 			= 0xBEFA;
	r->reg[XO] 			= 0x4064;
	r->reg[CAL_TIME]	= 0x9055;
	r->reg[VCO_CTRL]	= 0x2D02;
	r->reg[CT_CAL1]		= 0xACBF;
	r->reg[CT_CAL2]		= 0xACBF;
	r->reg[PLL_CAL1] 	= 0x0028;
	r->reg[PLL_CAL2] 	= 0x0028;
	r->reg[VCO_AUTO] 	= 0xFF00;
	r->reg[PLL_CTRL] 	= 0x8220;
	r->reg[PLL_BIAS] 	= 0x0202;
	r->reg[MIX_CONT] 	= 0x4800;
	r->reg[P1_FREQ1] 	= 0x1A94;
	r->reg[P1_FREQ2] 	= 0xD89D;
	r->reg[P1_FREQ3] 	= 0x8900;
	r->reg[P2_FREQ1] 	= 0x1E84;
	r->reg[P2_FREQ2]	= 0x89D8;
	r->reg[P2_FREQ3] 	= 0x9D00;
	r->reg[FN_CTRL]		= 0x2A80;
	r->reg[EXT_MOD] 	= 0x0000;
	r->reg[FMOD] 		= 0x0000;
	r->reg[SDI_CTRL] 	= 0x0000;
	r->reg[GPO]			= 0x0000;
	r->reg[T_VCO] 		= 0x4900;
	r->reg[IQMOD1] 		= 0x0281;
	r->reg[IQMOD2] 		= 0xF00F;
	r->reg[IQMOD3] 		= 0x0000;
	r->reg[IQMOD4] 		= 0x0005;
	r->reg[T_CTRL] 		= 0xC840;
	r->reg[DEV_CTRL] 	= 0x1000;
	r->reg[TEST] 		= 0x0005;
	// r->lf 			= 0xBEFA;
	// r->xo 			= 0x4064;
	// r->cal_time		= 0x9055;
	// r->vco_ctrl		= 0x2D02;
	// r->ct_cal1 		= 0xACBF;
	// r->ct_cal2 		= 0xACBF;
	// r->pll_cal1 	= 0x0028;
	// r->pll_cal2 	= 0x0028;
	// r->vco_auto 	= 0xFF00;
	// r->pll_ctrl 	= 0x8220;
	// r->pll_bias 	= 0x0202;
	// r->mix_count 	= 0x4800;
	// r->p1_freq1 	= 0x1A94;
	// r->p1_freq2 	= 0xD89D;
	// r->p1_freq3 	= 0x8900;
	// r->p2_freq1 	= 0x1E84;
	// r->p2_freq2 	= 0x89D8;
	// r->p2_freq3 	= 0x9D00;
	// r->fn_ctrl 		= 0x2A80;
	// r->ext_mode 	= 0x0000;
	// r->fmod 		= 0x0000;
	// r->sdi_ctrl 	= 0x0000;
	// r->gpo 			= 0x0000;
	// r->t_vco 		= 0x4900;
	// r->iqmod1 		= 0x0281;
	// r->iqmod2 		= 0xF00F;
	// r->iqmod3 		= 0x0000;
	// r->iqmod4 		= 0x0005;
	// r->t_ctrl 		= 0xC840;
	// r->dev_ctrl 	= 0x1000;
	// r->test 		= 0x0005;
}

static void calc_freqs(FREQS *out,int lo_div, int f_PD, int f_LO, int f_VCOMax)
{
	int n_lo = (int)(log2l((LD)f_VCOMax / (LD)f_LO));
	uint16_t lodiv = (int)pow(2,n_lo);
	LD fvco = lo_div * f_LO;
	double fbkdiv;
	if(fvco > 3200000000)
		fbkdiv = 4;
	else
		fbkdiv = 2;

	LD n_div = fvco / fbkdiv / f_PD;
	uint16_t n = (int) n_div;
	uint16_t nummsb = (int) ((2<<16) * (n_div - n));
	uint16_t numlsb = (int) ((2<<8) * (LD)(  ( (LD)(2<<16) * (n_div - n) ) - (LD)nummsb ) );

	out->n = n;
	out->nmsb = nummsb;
	out->nlsb = numlsb;
	out->lodiv = lodiv;
}

static void setup_device_operation(REGS *r, uint8_t _3wire, uint8_t multiSclice, uint8_t full_duplex, uint8_t p1mixidd, uint8_t p2mixidd)
{
	/*
		set-up device operations based on the datasheet

		\param *r register set
		\param _3wire has to 1 if we are using 3-wire, else 0
		\param multiSclice it is possible to control up to 4 devices connected to the same programming bus by enabling multisce mode
		\param p1mixidd path 1 mixer current( from 0b000 to 0x111)
		\param p2mixidd path 2 mixer current( from 0b000 to 0x111)
	*/
  	printf("setup device operations\n");
	uint8_t txd[2];
	uint16_t in;
	uint8_t addr;
	//-----------------
	// // this part doesnt have to be programmed for A devices
	// // P2_FREQ1:p2vcosel = 0
	// addr = P2_FREQ1;
	// in = 0;
	// set_bits(&in, &(r->reg[addr]), 2,1);
	// printf("chgnged register: %x\n",r->reg[addr]);
	// uint16to8(&r->reg[addr], txd);
	// rffc_write(&addr, txd);
	//-----------------
	// // this part doesnt have to be programmed for A devices
	// // VCO_AUTO:ct_min = 0
	// addr = VCO_AUTO;
	// in = 0;
	// set_bits(&in, &(r->reg[addr]), 7,7);
	// printf("chgnged register: %x\n",r->reg[addr]);
	// uint16to8(&r->reg[addr], txd);
	// rffc_write(&addr, txd);
	//-----------------
	// // this part doesnt have to be programmed for A devices
	// // VCO_AUTO:ct_max = 127
	// addr = VCO_AUTO;
	// in = 127;
	// set_bits(&in, &(r->reg[addr]), 7,14);
	// printf("chgnged register: %x\n",r->reg[addr]);
	// uint16to8(&r->reg[addr], txd);
	// rffc_write(&addr, txd);
	//-----------------
	// // this part doesnt have to be programmed for A devices
	// // CT_CAL1:p1ctv = 12
	// addr = CT_CAL1;
	// in = 12;
	// set_bits(&in, &(r->reg[addr]), 5,12);
	// printf("chgnged register: %x\n",r->reg[addr]);
	// uint16to8(&r->reg[addr], txd);
	// rffc_write(&addr, txd);
	//-----------------
	// // this part doesnt have to be programmed for A devices
	// // CT_CAL2:p2ctv = 12
	// addr = CT_CAL2;
	// in = 12;
	// set_bits(&in, &(r->reg[addr]), 5,12);
	// printf("chgnged register: %x\n",r->reg[addr]);
	// uint16to8(&r->reg[addr], txd);
	// rffc_write(&addr, txd);
	//-----------------
	// // this part doesnt have to be programmed for A devices
	// // TEST:rgbyp = 1
	// addr = TEST;
	// in = 1;
	// set_bits(&in, &(r->reg[addr]), 1,2);
	// printf("chgnged register: %x\n",r->reg[addr]);
	// uint16to8(&r->reg[addr], txd);
	// rffc_write(&addr, txd);
	//-----------------
	// according to page 12 of programming datasheet
	// VCO_CTRL:icpup = 0b11 = 0x03
	addr = VCO_CTRL;
	in = 0x03;
	set_bits(&in, &(r->reg[addr]), 2,2);
	// printf("chgnged register: %x\n",r->reg[addr]);
	uint16to8(&r->reg[addr], txd);
	rffc_write(&addr, txd);
	//-----------------
	// according to page 12 of programming datasheet
	// PLL_CTRL:ldlev = 0b11 = 0x03
	addr = PLL_CTRL;
	in = 1;
	set_bits(&in, &(r->reg[addr]), 1,4);
	// printf("chgnged register: %x\n",r->reg[addr]);
	uint16to8(&r->reg[addr], txd);
	rffc_write(&addr, txd);
	//-----------------
	if(_3wire)
	{
		//-----------------
		// SDI_CTRL:sipin = 1
		addr = SDI_CTRL;
		in = 1;
		set_bits(&in, &(r->reg[addr]), 1,15);
		// printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
	}
	else
	{
		//-----------------
		// SDI_CTRL:sipin = 0
		addr = SDI_CTRL;
		in = 0;
		set_bits(&in, &(r->reg[addr]), 1,15);
		// printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
	}
	if(multiSclice)
	{
		//-----------------
		// SDI_CTRL:addr = 1
		addr = SDI_CTRL;
		in = 1;
		set_bits(&in, &(r->reg[addr]), 1,11);
		// printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
	}
	else
	{
		if(multiSclice)
	{
		//-----------------
		// SDI_CTRL:addr = 0
		addr = SDI_CTRL;
		in = 0;
		set_bits(&in, &(r->reg[addr]), 1,11);
		// printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
	}
	}
	if(full_duplex)
	{
		//-----------------
		// MIX_CONT:fulld = 1
		addr = MIX_CONT;
		in = 1;
		set_bits(&in, &(r->reg[addr]), 1,15);
		// printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
	}
	else
	{
		//-----------------
		// MIX_CONT:fulld = 0
		addr = MIX_CONT;
		in = 0;
		set_bits(&in, &(r->reg[addr]), 1,15);
		// printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
	}
	//-----------------
	// MIX_CONT:p1mixidd = input
	// MIX_CONT:p2mixidd = input
	addr = MIX_CONT;
	if(p1mixidd > 0x07)
		in = 0x07;
	else
		in = p1mixidd;
	set_bits(&in, &(r->reg[addr]), 3,14);
	if(p2mixidd > 0x07)
		in = 0x07;
	else
		in = p2mixidd;
	set_bits(&in, &(r->reg[addr]), 3,11);
	// printf("chgnged register: %x\n",r->reg[addr]);
	uint16to8(&r->reg[addr], txd);
	rffc_write(&addr, txd);
	//-----------------
}


static void set_additional_features(REGS *r, uint8_t gpo_p1[6], uint8_t gpo_p2[6], int gate
, int lockOutputSignal, int _4wire, int freqModulator
, int modsetup, int modstep, uint16_t modulation)
{
	/*
	this function sets the additional features

	\param *r register set
	\param gpo_p1 enable or disable gpio path 1: 1=enable, 0=disable
	\param gpo_p2 enable or disable gpio path 2: 1=enable, 0=disable
	\param gate If 0 GPO's are ANDed with enable (forced to zero) when the enable is low, if 1 the GPO's are available when enable is low
	\param lockOutputSignal if 1 sends LOCK signal to GPO4
	\param _4wire enables 4-wire mode
	\param freqModulator It is possible to modulate the frequency of the VCO by programming the device or using one of the GPOs
	\param modsetup set modsetup value (explanation in the code)
	\param modstep set modstep value (value must fit in 4 bit,explanation in the code)
	\param modulation 2's complement of modulation value (16-bit data)
	*/
	printf("set additional features\n");
	uint8_t txd[2];
	uint16_t in;
	uint8_t addr;
	uint8_t flag = 0; // flag for setting the GPO register 
	if(gpo_p1)
	{
		//-----------------
		// GPO:p1gpio = input
		// e.g. input = [0,0,0,1,1,1] -> active GPO1,2,3 and others deactivated
		addr = GPO;
		in = 0;
		if(gpo_p1[0]!=0)
			in |= (0x01);
		if(gpo_p1[1]!=0)
			in |= (1<<1); // 0x02;
		if(gpo_p1[2]!=0)
			in |= (1<<2); // 0x04;
		if(gpo_p1[3]!=0)
			in |= (1<<3); // 0x08;
		if(gpo_p1[4]!=0)
			in |= (1<<4); // 0x10;
		if(gpo_p1[5]!=0)
			in |= (1<<5); // 0x20;
		if(in!=0)
		{
			flag = 1;
			set_bits(&in, &(r->reg[addr]), 7,8);
			// printf("chgnged register(GPO:p1gpio): %x\n",r->reg[addr]);
			printf("chgnged register(GPO:p1gpio): %x\n",in);
		}
		//-----------------
	}
	if(gpo_p2)
	{
		//-----------------
		// GPO:p2gpio = input
		addr = GPO;
		in = 0;
		if(gpo_p2[0]!=0)
			in |= (0x01);
		if(gpo_p2[1]!=0)
			in |= (1<<1); // 0x02;
		if(gpo_p2[2]!=0)
			in |= (1<<2); // 0x04;
		if(gpo_p2[3]!=0)
			in |= (1<<3); // 0x08;
		if(gpo_p2[4]!=0)
			in |= (1<<4); // 0x10;
		if(gpo_p2[5]!=0)
			in |= (1<<5); // 0x20;
		if(in!=0)
		{
			flag = 1;
			set_bits(&in, &(r->reg[addr]), 7,15);
			// printf("chgnged register: %x\n",r->reg[addr]);
			printf("chgnged register(GPO:p2gpio): %x\n",in);			
		}
		//-----------------
	}
	if(gate)
	{
		//-----------------
		// GPO:gate = input
		addr = GPO;
		in = gate!=0?1:0;
		if(in!=0)
		{
			flag = 1;
			set_bits(&in, &(r->reg[addr]), 1,1);
			// printf("chgnged register: %x\n",r->reg[addr]);
			printf("chgnged register(GPO:gate): %x\n",in);
		}
		//-----------------
	}
	if(lockOutputSignal)
	{
		//-----------------
		// GPO:lock = input
		addr = GPO;
		in = lockOutputSignal!=0?1:0;
		if(in!=0)
		{
			flag = 1;
			set_bits(&in, &(r->reg[addr]), 1,0);
			// printf("chgnged register: %x\n",r->reg[addr]);
			printf("chgnged register(GPO:lock): %x\n",in);
		}
		//-----------------
	}
	// finally set tehe GPO register if it changed from zero
	if(flag)
	{
		printf("final output GPO: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
	}

	if(_4wire)
	{
		//-----------------
		// SDI_CTRL:4wire = input
		addr = SDI_CTRL;
		in = _4wire!=0?1:0;
		set_bits(&in, &(r->reg[addr]), 1,12);
		printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
	}
	if(freqModulator)
	{
		if(modsetup)
		{
			//-----------------
			// EXT_MOD:modsetup = input
			// if modsetup == 0 or 2 -> modulation off
			// if modsetup == 1 -> analog modulation
			// if modsetup == 3 -> binary modulation
			addr = EXT_MOD;
			if(modsetup==1)
				in = 0x1;
			else if(modsetup==3)
				in = 0x3;
			set_bits(&in, &(r->reg[addr]), 2,15);
			printf("chgnged register: %x\n",r->reg[addr]);
			//-----------------
		}
		if(modstep)
		{
			//-----------------
			// EXT_MOD:modstep = input
			// Modulation is multiplied by 2^modstep (max is 8)
			addr = EXT_MOD;
			if(modstep >= 8)
				in = 0x08;
			else if(modsetup < 0)
				in = 0;
			else
				in = modstep;
			set_bits(&in, &(r->reg[addr]), 4,13);
			printf("chgnged register: %x\n",r->reg[addr]);
			//-----------------
		}
		// finally set tehe EXT_MOD register if there was a change
		if(modstep || modsetup)
		{
			uint16to8(&r->reg[addr], txd);
			rffc_write(&addr, txd);
		}
		if(modulation)
		{
			//-----------------
			// FMOD:modulation = input
			// value detail in page 8 of datasheet
			// 2's complement value of modulation is stored in this register
			addr = FMOD;
			in = modulation;
			set_bits(&in, &(r->reg[addr]), 16,15);
			printf("chgnged register: %x\n",r->reg[addr]);
			uint16to8(&r->reg[addr], txd);
			rffc_write(&addr, txd);
			//-----------------
		}
	}

}


/////////////////////// must check this ///////////////////////////
static void set_operating_freq(REGS *r,int autoSel, uint16_t autoVCO,
 uint16_t p1vcosel, uint16_t p2vcosel, int autoCTcal, uint16_t p1ct, uint16_t p1ctdef,
 uint16_t p2ct, uint16_t p2ctdef, float VCO_freq, int lo_div1, int f_PD1, int f_LO1
 , int f_VCOMax1, int lo_div2, int f_PD2, int f_LO2, int f_VCOMax2)
{
	/*
	give VCO_freq in MHz
	*/

	printf("details for setting operating frequencies\n");
	uint8_t txd[2];
	uint16_t in;
	uint8_t addr;

	if(!autoSel)
	{
		//-----------------
		// VCO_AUTO:auto = input
		addr = P2_FREQ1;
		in = autoVCO;
		set_bits(&in, &(r->reg[addr]), 1,15);
		printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
		// P1_FREQ1:p1vcosel = input
		// VCO select: 00 -> vco1
		//             01 -> vco2
		//             10 -> vco3	
		addr = P1_FREQ1;
		in = p1vcosel;
		set_bits(&in, &(r->reg[addr]), 2,1);
		printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
		// P2_FREQ1:p2vcosel = input
		// VCO select: 00 -> vco1
		//             01 -> vco2
		//             10 -> vco3
		addr = P2_FREQ1;
		in = p2vcosel;
		set_bits(&in, &(r->reg[addr]), 2,1);
		printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
	}
	if(!autoCTcal)
	{
		//-----------------
		// CT_CAL1:p1ct = input
		addr = CT_CAL1;
		in = p1ct;
		set_bits(&in, &(r->reg[addr]), 5,12);
		printf("chgnged register: %x\n",r->reg[addr]);
		//-----------------
		// CT_CAL1:p1ctdef = input
		addr = CT_CAL1;
		in = p1ctdef;
		set_bits(&in, &(r->reg[addr]), 7,6);
		printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
		// CT_CAL2:p2ct = input
		addr = CT_CAL2;
		in = p2ct;
		set_bits(&in, &(r->reg[addr]), 5,12);
		printf("chgnged register: %x\n",r->reg[addr]);
		//-----------------
		// CT_CAL2:p2ctdef = input
		addr = CT_CAL2;
		in = p2ctdef;
		set_bits(&in, &(r->reg[addr]), 7,6);
		printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
	}
	if(VCO_freq >3200.0)
	{
		//-----------------
		// P1_FREQ1:p1presc = 0b10 which means divide by 4
		addr = P1_FREQ1;
		in = 0x02;
		set_bits(&in, &(r->reg[addr]), 2,3);
		printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
		// P2_FREQ1:p2presc = 0b10 which means divide by 4
		addr = P2_FREQ1;
		in = 0x02;
		set_bits(&in, &(r->reg[addr]), 2,3);
		printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
		// LF:pllcpl = 3
		addr = LF;
		in = 3;
		set_bits(&in, &(r->reg[addr]), 3,2);
		printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
	}
	else
	{
		// P1_FREQ1:p1presc = 0b01 which means divide by 2
		addr = P1_FREQ1;
		in = 1;
		set_bits(&in, &(r->reg[addr]), 2,3);
		printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
		// P2_FREQ1:p2presc = 0b01 which means divide by 2
		addr = P2_FREQ1;
		in = 1;
		set_bits(&in, &(r->reg[addr]), 2,3);
		printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
	}
	FREQS f;
	calc_freqs(&f,lo_div1, f_PD1, f_LO1, f_VCOMax1);
	//-----------------
	// P1_FREQ1:p1n = calculated n
	// P1_FREQ1:p1lodiv = calculated lodiv
	addr = P1_FREQ1;
	in = f.n;
	set_bits(&in, &(r->reg[addr]), 9,15);
	in = f.lodiv;
	set_bits(&in, &(r->reg[addr]), 3,6);
	printf("chgnged register: %x\n",r->reg[addr]);
	uint16to8(&r->reg[addr], txd);
	rffc_write(&addr, txd);
	//-----------------
	// P1_FREQ2:p1n = p1nmsb
	addr = P1_FREQ2;
	in = f.nmsb;
	set_bits(&in, &(r->reg[addr]), 16,15);
	printf("chgnged register: %x\n",r->reg[addr]);
	uint16to8(&r->reg[addr], txd);
	rffc_write(&addr, txd);
	//-----------------
	// P1_FREQ3:p1n = p1nlsb
	addr = P1_FREQ3;
	in = f.nlsb;
	set_bits(&in, &(r->reg[addr]), 8,15);
	printf("chgnged register: %x\n",r->reg[addr]);
	uint16to8(&r->reg[addr], txd);
	rffc_write(&addr, txd);
	//-----------------
	
	calc_freqs(&f, lo_div2, f_PD2, f_LO2, f_VCOMax2);
	//-----------------
	// P2_FREQ1:p1n = calculated n
	// P2_FREQ1:p1lodiv = calculated lodiv
	addr = P2_FREQ1;
	in = f.n;
	set_bits(&in, &(r->reg[addr]), 9,15);
	in = f.lodiv;
	set_bits(&in, &(r->reg[addr]), 3,6);
	printf("chgnged register: %x\n",r->reg[addr]);
	uint16to8(&r->reg[addr], txd);
	rffc_write(&addr, txd);
	//-----------------
	// P2_FREQ2:p1n = p1nmsb
	addr = P2_FREQ2;
	in = f.nmsb;
	set_bits(&in, &(r->reg[addr]), 16,15);
	printf("chgnged register: %x\n",r->reg[addr]);
	uint16to8(&r->reg[addr], txd);
	rffc_write(&addr, txd);
	//-----------------
	// P2_FREQ3:p1n = p1nlsb
	addr = P2_FREQ3;
	in = f.nlsb;
	set_bits(&in, &(r->reg[addr]), 8,15);
	printf("chgnged register: %x\n",r->reg[addr]);
	uint16to8(&r->reg[addr], txd);
	rffc_write(&addr, txd);
	//-----------------

}


static void set_calibration_mode(REGS *r, int en_p1, int en_p2)
{
	/*
	set calibration mode
	\param *r register set
	\param en  enable loop filter calibration mode
	*/
	printf("set calibration mode\n");
	uint8_t txd[2];
	uint16_t in;
	uint8_t addr;

	if(en_p1)
	{
		//-----------------
		// PLL_CAL1:p1kv = 0
		addr = PLL_CAL1;
		in = 0;
		set_bits(&in, &(r->reg[addr]), 1,15);
		printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
	}
	// else
	// {
	// 	//-----------------
	// 	// PLL_CAL1:p1kv = 1
	// 	addr = PLL_CAL1;
	// 	in = 1;
	// 	set_bits(&in, &(r->reg[addr]), 1,15);
	// 	printf("chgnged register: %x\n",r->reg[addr]);
	// 	uint16to8(&r->reg[addr], txd);
	// 	rffc_write(&addr, txd);
	// 	//-----------------
	// }
	if(en_p2)
	{
		//-----------------
		// PLL_CAL2:p2kv = 0
		addr = PLL_CAL2;
		in = 0;
		set_bits(&in, &(r->reg[addr]), 1,15);
		printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
	}
	// else
	// {
	// 	//-----------------
	// 	// PLL_CAL2:p2kv = 1
	// 	addr = PLL_CAL2;
	// 	in = 1;
	// 	set_bits(&in, &(r->reg[addr]), 1,15);
	// 	printf("chgnged register: %x\n",r->reg[addr]);
	// 	uint16to8(&r->reg[addr], txd);
	// 	rffc_write(&addr, txd);
	// 	//-----------------
	// }
}


static void enable_device(REGS *r, int control_method)
{
	/*
	* control method:  programming bus -> 0
					   control pins -> 1
	\param *r register set
	\param control_method select type of control
	*/
	printf("enable device\n");
	uint8_t txd[2];
	uint16_t in;
	uint8_t addr;

	if(!control_method)
	{
		//-----------------
		// SDI_CTRL:sipin = 1
		addr = SDI_CTRL;
		in = 1;
		set_bits(&in, &(r->reg[addr]), 1,15);
		// set_bits(&in, &(r->reg[addr]), 1,14);// this may not work
		// set_bits(&in, &(r->reg[addr]), 1,13);// this may not work
		// // might want to split sipin set and other two registers
		// printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
		// SDI_CTRL:enbl = 1
		// SDI_CTRL:mode = 1
		addr = SDI_CTRL;
		in = 1;
		set_bits(&in, &(r->reg[addr]), 1,14);// this may not work
		set_bits(&in, &(r->reg[addr]), 1,13);// this may not work
		// might want to split sipin set and other two registers
		// printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
	}
	else
	{
		//-----------------
		// SDI_CTRL:sipin = 0 which is default
		addr = SDI_CTRL;
		in = 0;
		set_bits(&in, &(r->reg[addr]), 1,15);
		printf("chgnged register: %x\n",r->reg[addr]);
		uint16to8(&r->reg[addr], txd);
		rffc_write(&addr, txd);
		//-----------------
		// after this ENBL and MODE pins must be set manually
	}
}



