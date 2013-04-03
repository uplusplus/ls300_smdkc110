/* linux/drivers/media/video/s5k4ba.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 * 		http://www.samsung.com/
 *
 * Driver for S5K4BA (UXGA camera) from Samsung Electronics
 * 1/4" 2.0Mp CMOS Image Sensor SoC with an Embedded Image Processor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define S5K4BA_COMPLETE
#ifndef __S5K4BA_H__
#define __S5K4BA_H__

struct s5k4ba_reg {
	unsigned char addr;
	unsigned char val;
};

struct s5k4ba_regset_type {
	unsigned char *regset;
	int len;
};

#undef S5K4BA_COMPLETE

/*
 * Macro
 */
#define REGSET_LENGTH(x)	(sizeof(x)/sizeof(s5k4ba_reg))

/*
 * User defined commands
 */
/* S/W defined features for tune */
#define REG_DELAY	0xFF00	/* in ms */
#define REG_CMD		0xFFFF	/* Followed by command */

/* Following order should not be changed */
enum image_size_s5k4ba {
	/* This SoC supports upto UXGA (1600*1200) */
#if 0
	QQVGA,	/* 160*120*/
	QCIF,	/* 176*144 */
	QVGA,	/* 320*240 */
	CIF,	/* 352*288 */
#endif
	VGA,	/* 640*480 */
#if 0
	SVGA,	/* 800*600 */
	HD720P,	/* 1280*720 */
	SXGA,	/* 1280*1024 */
	UXGA,	/* 1600*1200 */
#endif
};

/*
 * Following values describe controls of camera
 * in user aspect and must be match with index of s5k4ba_regset[]
 * These values indicates each controls and should be used
 * to control each control
 */
enum s5k4ba_control {
	S5K4BA_INIT,
	S5K4BA_EV,
	S5K4BA_AWB,
	S5K4BA_MWB,
	S5K4BA_EFFECT,
	S5K4BA_CONTRAST,
	S5K4BA_SATURATION,
	S5K4BA_SHARPNESS,
};

#define S5K4BA_REGSET(x)	{	\
	.regset = x,			\
	.len = sizeof(x)/sizeof(s5k4ba_reg),}


/*
 * User tuned register setting values
 */
static unsigned char s5ka3dfx_init_reg[][2] = {
{0xEF,0x03},
{0x50,0xD2},	//Mclk
{0x5F,0x03},	//NT Cintr Max
{0x60,0x02},	//PAL Cintr Max
{0x61,0x0C},	//NT shutter Max (FrameAE mode)
{0x62,0x0A},	//PAL shutter Max (FrameAE mode)
{0x6E,0x40},	//Dgain Min
{0x6F,0x5A},	//Dgain Max
{0x63,0x02},	//NT Vblank
{0x64,0xDD},
{0x65,0x02},	//PAL Vblank
{0x66,0xDD},
{0x48,0x00},	//NT Hblank
{0x49,0x9E},
{0x4C,0x00},	//PAL Hblank
{0x4D,0x9E},
{0xEF,0x03},
{0x51,0x10},
{0x52,0x00},
{0x53,0x00},
{0x54,0x00},
{0x56,0x01},
{0x57,0x61},
{0x58,0x25},
{0x67,0xCF},
{0xef,0x02},                                                             
{0x13,0xa0}, //OB sel(R reference)                                       
{0x23,0x53}, //tx_width                                                 
{0x26,0x24}, //clp level                                                 
{0x2c,0x05}, //S1S                                                       
{0x05,0x00}, //S1S end value                                             
{0x03,0x58}, //S1R end value                                                                                                           
{0x24,0x0a}, //cds //1a                                                                                                                    
{0x0b,0x84}, //Analog Offset                                             
{0x1e,0xb7}, //Global gain                                               
{0x56,0x05}, //ADLC                                                      
{0x28,0x96}, //CFPN 
{0x67,0x3f}, //or 3Ch, or 38h//to reduce HN at low lux                                                         
{0xef,0x03},                                                        
{0x0f,0x31}, //ADD vblank value   
{0xef,0x03},
{0x00,0x87}, //07h 60Hz  //87 50Hz //AE, AWB ON
{0xef,0x00},
{0xde,0x00},
{0xdf,0x1F},
{0xe0,0x00},
{0xe1,0x37},
{0xe2,0x08},
{0xe3,0x42},
{0xe4,0x00},
{0xe5,0x12},
{0xe6,0x9E},
{0xe9,0x00},
//////////////////
{0xe7,0x01},
{0xe8,0x1D},
{0xe9,0x03},
///////////////                                                          
{0xef,0x00},                                                             
{0xd1,0x00}, //YC order                                                  
{0xdd,0x03}, //X shade ON                                                                                                                                                                                                                                    
{0xef,0x00},                                                             
{0x23,0x1d}, //GrGb                                                      
{0x24,0x1d},                                                             
{0x25,0x1d},                                                             
{0x27,0x38},                                                             
{0x29,0x60},                                                             
{0x2a,0x22}, //7b                                                                                                                              
{0xef,0x03},                                                             
{0x2b,0x40}, //41 //Auto E-shutter enable                                                                                                      
{0xef,0x00},                                                             
{0x2f,0x01}, //INTP_COEF_Sharpness                                                                                                        
{0xef,0x00},                                                             
{0x36,0x01},//Shading (R, G, B)                                   
{0x37,0x6e},//77
{0x38,0x60},//66
{0x39,0x50},//57
{0x3a,0x01},
{0x3b,0x04},
{0x3c,0x01},
{0x3d,0x5e},                                                            
{0xef,0x00},  //CCM
{0x4c,0x84},
{0x4d,0xb7},
{0x4e,0x8d},
{0x4f,0x9f},
{0x50,0x8c},
{0x51,0xac},
{0x52,0x89},
{0x53,0xb0},
{0x54,0x79},                                                                                                                              
{0xef,0x00},
////////////////////////
{0x48,0x01},
{0x49,0x00},
{0x4A,0x30},
/////////////////////                                                             
{0xef,0x03},                                                                                                       
{0x31,0x00}, //NL
{0x32,0x08}, //00//LL                                                             
{0x33,0x80}, //NL
{0x34,0x80}, //LL                                                               
{0x36,0x40}, //3c//NL
{0x37,0x34}, //30//LL
{0x6a,0x00},
{0x7b,0x05},                                                             
{0x38,0x05},                                                             
{0x39,0x03},                                                                                                                               
{0xef,0x03},                                                             
{0x2d,0x08},//10                                                       
{0x2e,0x2c},//2c                                        
{0x2f,0x60},//4A                                                       
{0x30,0xd8},//4E                                                         
{0x7c,0x04},//10
{0x7d,0x2a},//2c                                             
{0x7e,0x0c},//22                                               
{0x7f,0x2a},//4e                                                                                                                         
{0xef,0x00},                                                       
{0x2c,0x06}, //                                       
{0x2d,0x20},                                                                     
{0xef,0x03},                                                             
{0x28,0x01}, //02 //Sharpness                                                 
{0x29,0x6f}, //9f//org6f //9f                                                             
{0x2a,0x00},                                                                                                                     
{0xef,0x00},                                                             
{0xb9,0x02}, //Sharpness ON                                              
{0xbb,0xa8}, //a0 //HPF                                                       
{0xbc,0x10}, //08                                                             
{0xbd,0x30},                                                             
{0xbf,0x18}, //38 //Core                                                      
{0xc1,0x38}, //88                                                                                                                              
{0xc8,0x11}, //YC delay                                                  
{0xeb,0x81}, //ECS                                                       
{0xed,0x18}, //20                                                                                                                              
{0xef,0x00},                                                             
{0xb1,0x00}, //AWB window                                                
{0xb2,0x62},                                                             
{0xb3,0x00},                                                             
{0xb4,0x00},                                                             
{0xb5,0x01},                                                             
{0xb6,0xa3},                                                             
{0xb7,0x02},                                                             
{0xb8,0x80},                                                                
{0xef,0x00},                                                             
{0x77,0x00}, //Saturation SIN                                            
{0x78,0x00}, //Sat SIN                                                                                                                                                   
{0xef,0x03},                                                             
{0x13,0x00}, //AWB Outdoor Cintr limi                                    
{0x14,0xa0},  
{0x19,0x43},                                                         
{0x1a,0x64},//6b //Outdoor Rgain Max                                         
{0x1b,0x5b},//62 //Outdoor Rgain Min                                         
{0x1c,0x63},//61 //Outdoor Bgain Max                                         
{0x1d,0x52},//4e //Outdoor Bgain Min                                                                                                  
{0x1e,0x64},//6b //Indoor Rgain Max                                          
{0x1f,0x42},//42 //Indoor Rgain Min                                          
{0x20,0x7e},//82 //Indoor Bgain Max                                          
{0x21,0x52},//4e //Indoor Bgain Min                                                                                                           
{0xef,0x03},                                                             
{0x26,0x01},                                                                                                                              
{0xef,0x03},                                                             
{0x3a,0x13}, //14 //[7:4]AWB speed, [3:0] AWB threshold                       
{0x3b,0x3e}, //AWB skip BRT <= same value with AE target<3. 01>          
{0x3c,0x00}, //AWB skip Gain                                             
{0x3d,0x18}, //AWB skip AVG                                                                                                               
{0x35,0x00}, //AWB single CNT                                                                                                               
{0xef,0x00},                                                             
{0x93,0x40}, //AWB Map                                                   
{0x94,0x80},                                                             
{0x95,0xc0},                                                             
{0x96,0xc0},                                                             
{0x97,0x20},                                                             
{0x98,0x20},                                                             
{0x99,0x30},                                                             
{0xA0,0x00},                                                             
{0xA1,0x00},                                                             
{0xA2,0x1c},                                                             
{0xA3,0x16},                                                             
{0xA4,0x03},                                                             
{0xA5,0x07},                                                             
{0xA6,0x00},                                                                                                                               
{0xef,0x00},                                                             
{0xad,0xd0}, //AWB up data                                               
{0xaf,0x26}, //AWB dn data                                                                                                                
{0xef,0x00},                                                             
{0x42,0x5b}, //65 //Rgain (start point of AWB) //6f                           
{0x44,0x60}, //Bgain (start point of AWB) //5a                                                                                             
{0xef,0x03},                                                             
{0x23,0x80}, //AWB window select                                                                                                           
{0x15,0x0b}, //AWB CNT                                                   
{0x16,0xd2},                                                             
{0x17,0x64},                                                             
{0x18,0x78},                                                                                                                               
{0xef,0x03},                                                             
{0x01,0x3e}, //AE target                                                 
{0x02,0x05}, //AE threshold                                              
{0x03,0x20}, //AE step                                                   
{0x04,0x58}, //60 //AGC Max of LowLux                                         
{0x06,0x1c}, //AGC Max of HL                                             
{0x07,0x01}, //AE win_A weight                                           
{0x08,0x01}, //AE win_B weight                                           
{0x0b,0x01}, //cintc max high                                            
{0x4B,0x06},       
{0x55,0x00}, //{0x55,0x22},                                        
{0x69,0x17}, //AE ROW FLAG                                                                                                                 
{0x6e,0x40}, //Dgain Min                                                                                                                   
{0xef,0x00},                                                             
{0x57,0x00}, //AE Min skip                                                                                                                 
{0xef,0x00},                                                             
{0x58,0x00}, //AE window                                                 
{0x59,0x00},                                                             
{0x5a,0x02},                                                             
{0x5b,0x73},                                                             
{0x5c,0x00},                                                             
{0x5d,0x00},                                                             
{0x5e,0x01},                                                             
{0x5f,0xe0},                                                             
{0x60,0x00},                                                             
{0x61,0x80},                                                             
{0x62,0x01},                                                             
{0x63,0xf3},                                                             
{0x64,0x00},                                                             
{0x65,0xd2},                                                             
{0x66,0x01},                                                             
{0x67,0x78},                                                                
{0xef,0x00}, //Flicker setting
{0x6a,0x01}, //080331 Flicker H SIZE High    
{0x6b,0xe0}, //080331 Flicker H SIZE low     
{0x6c,0x05}, //04 //080331 Flicker WINDOW VSIZE 02
{0x6d,0x00}, //080331 Flicker V SIZE START H 
{0x6e,0x0e}, //080331 Flicker V SIZE START L 
{0x6f,0x00}, //080331 Flicker H SIZE START H 
{0x70,0x10}, //080331 Flicker H SIZE START L    
{0xef,0x03},
{0x22,0x24}, //23 //flicker sensitivity H/L
{0x3e,0x23},
{0x3f,0x23},
{0x40,0x00},
{0x41,0x09}, //60hz light - 50hz setting threshold
{0x4a,0x09}, //50hz light - 60hz setting threshold
{0x4b,0x04}, 
{0x5b,0x20}, //10 //detection haunting protection count 
{0x5d,0x35}, //55
{0x5e,0x13},
{0x78,0x0f},  
{0xef,0x03},                                                             
{0x70,0x00},
{0x00,0x07},
{0xef,0x00},                                                             
{0x72,0xe0},
{0x73,0xe0}, 
{0x77,0x00}, 
{0x78,0x40},
{0x79,0xe0},
};

#define S5KA3DFX_INIT_REGS	\
	(sizeof(s5ka3dfx_init_reg) / sizeof(s5ka3dfx_init_reg[0]))

/*
 * EV bias
 */

static const struct s5k4ba_reg s5k4ba_ev_m6[] = {
};

static const struct s5k4ba_reg s5k4ba_ev_m5[] = {
};

static const struct s5k4ba_reg s5k4ba_ev_m4[] = {
};

static const struct s5k4ba_reg s5k4ba_ev_m3[] = {
};

static const struct s5k4ba_reg s5k4ba_ev_m2[] = {
};

static const struct s5k4ba_reg s5k4ba_ev_m1[] = {
};

static const struct s5k4ba_reg s5k4ba_ev_default[] = {
};

static const struct s5k4ba_reg s5k4ba_ev_p1[] = {
};

static const struct s5k4ba_reg s5k4ba_ev_p2[] = {
};

static const struct s5k4ba_reg s5k4ba_ev_p3[] = {
};

static const struct s5k4ba_reg s5k4ba_ev_p4[] = {
};

static const struct s5k4ba_reg s5k4ba_ev_p5[] = {
};

static const struct s5k4ba_reg s5k4ba_ev_p6[] = {
};

#ifdef S5K4BA_COMPLETE
/* Order of this array should be following the querymenu data */
static const unsigned char *s5k4ba_regs_ev_bias[] = {
	(unsigned char *)s5k4ba_ev_m6, (unsigned char *)s5k4ba_ev_m5,
	(unsigned char *)s5k4ba_ev_m4, (unsigned char *)s5k4ba_ev_m3,
	(unsigned char *)s5k4ba_ev_m2, (unsigned char *)s5k4ba_ev_m1,
	(unsigned char *)s5k4ba_ev_default, (unsigned char *)s5k4ba_ev_p1,
	(unsigned char *)s5k4ba_ev_p2, (unsigned char *)s5k4ba_ev_p3,
	(unsigned char *)s5k4ba_ev_p4, (unsigned char *)s5k4ba_ev_p5,
	(unsigned char *)s5k4ba_ev_p6,
};

/*
 * Auto White Balance configure
 */
static const struct s5k4ba_reg s5k4ba_awb_off[] = {
};

static const struct s5k4ba_reg s5k4ba_awb_on[] = {
};

static const unsigned char *s5k4ba_regs_awb_enable[] = {
	(unsigned char *)s5k4ba_awb_off,
	(unsigned char *)s5k4ba_awb_on,
};

/*
 * Manual White Balance (presets)
 */
static const struct s5k4ba_reg s5k4ba_wb_tungsten[] = {

};

static const struct s5k4ba_reg s5k4ba_wb_fluorescent[] = {

};

static const struct s5k4ba_reg s5k4ba_wb_sunny[] = {

};

static const struct s5k4ba_reg s5k4ba_wb_cloudy[] = {

};

/* Order of this array should be following the querymenu data */
static const unsigned char *s5k4ba_regs_wb_preset[] = {
	(unsigned char *)s5k4ba_wb_tungsten,
	(unsigned char *)s5k4ba_wb_fluorescent,
	(unsigned char *)s5k4ba_wb_sunny,
	(unsigned char *)s5k4ba_wb_cloudy,
};

/*
 * Color Effect (COLORFX)
 */
static const struct s5k4ba_reg s5k4ba_color_sepia[] = {
};

static const struct s5k4ba_reg s5k4ba_color_aqua[] = {
};

static const struct s5k4ba_reg s5k4ba_color_monochrome[] = {
};

static const struct s5k4ba_reg s5k4ba_color_negative[] = {
};

static const struct s5k4ba_reg s5k4ba_color_sketch[] = {
};

/* Order of this array should be following the querymenu data */
static const unsigned char *s5k4ba_regs_color_effect[] = {
	(unsigned char *)s5k4ba_color_sepia,
	(unsigned char *)s5k4ba_color_aqua,
	(unsigned char *)s5k4ba_color_monochrome,
	(unsigned char *)s5k4ba_color_negative,
	(unsigned char *)s5k4ba_color_sketch,
};

/*
 * Contrast bias
 */
static const struct s5k4ba_reg s5k4ba_contrast_m2[] = {
};

static const struct s5k4ba_reg s5k4ba_contrast_m1[] = {
};

static const struct s5k4ba_reg s5k4ba_contrast_default[] = {
};

static const struct s5k4ba_reg s5k4ba_contrast_p1[] = {
};

static const struct s5k4ba_reg s5k4ba_contrast_p2[] = {
};

static const unsigned char *s5k4ba_regs_contrast_bias[] = {
	(unsigned char *)s5k4ba_contrast_m2,
	(unsigned char *)s5k4ba_contrast_m1,
	(unsigned char *)s5k4ba_contrast_default,
	(unsigned char *)s5k4ba_contrast_p1,
	(unsigned char *)s5k4ba_contrast_p2,
};

/*
 * Saturation bias
 */
static const struct s5k4ba_reg s5k4ba_saturation_m2[] = {
};

static const struct s5k4ba_reg s5k4ba_saturation_m1[] = {
};

static const struct s5k4ba_reg s5k4ba_saturation_default[] = {
};

static const struct s5k4ba_reg s5k4ba_saturation_p1[] = {
};

static const struct s5k4ba_reg s5k4ba_saturation_p2[] = {
};

static const unsigned char *s5k4ba_regs_saturation_bias[] = {
	(unsigned char *)s5k4ba_saturation_m2,
	(unsigned char *)s5k4ba_saturation_m1,
	(unsigned char *)s5k4ba_saturation_default,
	(unsigned char *)s5k4ba_saturation_p1,
	(unsigned char *)s5k4ba_saturation_p2,
};

/*
 * Sharpness bias
 */
static const struct s5k4ba_reg s5k4ba_sharpness_m2[] = {
};

static const struct s5k4ba_reg s5k4ba_sharpness_m1[] = {
};

static const struct s5k4ba_reg s5k4ba_sharpness_default[] = {
};

static const struct s5k4ba_reg s5k4ba_sharpness_p1[] = {
};

static const struct s5k4ba_reg s5k4ba_sharpness_p2[] = {
};

static const unsigned char *s5k4ba_regs_sharpness_bias[] = {
	(unsigned char *)s5k4ba_sharpness_m2,
	(unsigned char *)s5k4ba_sharpness_m1,
	(unsigned char *)s5k4ba_sharpness_default,
	(unsigned char *)s5k4ba_sharpness_p1,
	(unsigned char *)s5k4ba_sharpness_p2,
};
#endif /* S5K4BA_COMPLETE */

#endif
