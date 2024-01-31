/*******************************************************
   1、LT8911EXB的IIC地址：
   1. IIC address of lt8911exb:

   a)如果LT8911EXB的第31脚(S_ADR）为低，则LT8911EXB的I2C 地址为0x52; // bit0 是读写标志位；如果是Linux系统，IIC address 的 bit7作为读写标志位，则I2C地址 应该是 0x29
   A) if the 31st pin (s_adr) of lt8911exb is low, the I2C address of lt8911exb is 0x52; // bit0 is the read-write flag ; if it is Linux system, the bit7 of IIC address is the read-write flag, then the I2C address should be 0x29

   b)如果LT8911EXB的第31脚(S_ADR）为高，则LT8911EXB的I2C 地址为0x5a; // bit0 是读写标志位；如果是Linux系统，IIC address 的 bit7作为读写标志位，则I2C地址 应该是 0x2d
   b) if the 31st pin (s_adr) of lt8911exb is high, the I2C address of lt8911exb is 0x5a; // bit0 is the read-write flag ; if it is a Linux system, the bit7 of IIC address is the read-write flag, then the I2C address should be 0x2d

   2、IIC速率不要超过100KHz。
   2. IIC speed shall not exceed 100kHz.

   3、要确定MIPI信号给到LT8911EXB之后，再初始化LT8911EXB。
   3. Confirm that Mipi signal is sent to lt8911exb, and then initialize lt8911exb.

   4、必须由前端主控GPIO来复位LT8911EXB；刷寄存器之前，先Reset LT8911EXB ,用GPIO 先拉低LT8911EXB的复位脚 100ms左右，再拉高，保持100ms。
   4. The lt8911exb must be reset by the Master GPIO; before write the register, reset the lt8911exb first, pull down the reset pin by GPIO for about 100ms, and then pull up for 100ms.

   5、LT8911EXB 对MIPI输入信号的要求：
   5. LT8911EXB MIPI input signal requirements:
   a) MIPI DSI
   b) Video mode

   MIPI 输入需要关闭展频(SSC - Spread-Spectrum Clock)，打开EOTP(End Of Transmite Packet)
   Mipi signal needs to turn off SSC(Spread-Spectrum Clock) and turn on eotp(End Of Transmite Packet)
 *********************************************************/
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <ctype.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <memory.h>
#include <stdbool.h>
#include "hi_mipi_tx.h"
#include <sample_comm.h>
#include <linux/i2c-dev.h>
#include "LT8911EXB.h"
#include <linux/i2c-dev.h>
#define  ADDR_BYTE 1
#define  DATA_BYTE 1
#define I2C_ADDR 0x52
//#define _Test_Pattern_
#define _eDP_2G7_
//#define _eDP_1G62_

#define _link_train_enable_

#define u8 unsigned char
#define u16 unsigned short int
#define u32 unsigned int

// #define READ_MIN_CNT 4
// #define WRITE_MIN_CNT 5

int i2c_write(unsigned int addr,unsigned int data) {
    int retval =0;
	int fd = -1;
    int idx = 0;
    unsigned char buf[8];
	unsigned int i2c_num = 2;
    char file_name[0x10];
	sprintf(file_name, "/dev/i2c-%u", i2c_num);
	fd = open(file_name, O_RDWR);
    if (fd < 0) {
        printf("Open %s error!\n",file_name);
		retval=-1;
		goto end0;
		} else {
    }
    
	retval = ioctl(fd,I2C_SLAVE_FORCE,(I2C_ADDR>>1));
    if(retval < 0){
		printf("set i2c device address error!\n");
		retval = -1;
		goto end1;
	}

    if (ADDR_BYTE == 2) {  //2 byte 
        buf[idx] = (addr >> 8) & 0xff;  //shift 8 
        idx++;
        buf[idx] = addr & 0xff;
        idx++;
    } else {
        buf[idx] = addr & 0xff;      
        idx++;    
    }

    if (DATA_BYTE == 2) {  // 2 byte 
        buf[idx] = (data >> 8) & 0xff;  // shift 8 
        idx++;
        buf[idx] = data & 0xff;
        idx++;
    } else {
        buf[idx] = data & 0xff;
        idx++;
    }
    retval = write(fd, buf, (ADDR_BYTE + DATA_BYTE));
    if(retval<0) {
        printf("write error\n");
		retval = -1;
		goto end1;
    }
	retval = 0;
    end1:
	    close(fd);
	end0:
	    return retval;
}

int i2c_read(unsigned int addr)
{
	int fd = -1;
	int retval=0;
    unsigned char buf[8];
	unsigned int data = 0;
	unsigned int i2c_num = 2;
	static struct i2c_rdwr_ioctl_data rdwr;
	static struct i2c_msg msg[2];
	char file_name[0x10];
	memset(buf,0x0,4);

	sprintf(file_name, "/dev/i2c-%u", i2c_num);
	fd = open(file_name, O_RDWR);

    if (fd < 0) {
        printf("Open %s error!\n",file_name);
		retval=-1;
		goto end0;
	} else {
    
    }
    
	retval = ioctl(fd,I2C_SLAVE_FORCE,(I2C_ADDR>>1));
    if(retval < 0){
		printf("set i2c device address error!\n");
		retval = -1;
		goto end1;
	}

	msg[0].addr =(I2C_ADDR>>1);
	msg[0].flags =0;
	msg[0].len =ADDR_BYTE;
	msg[0].buf =buf;

	msg[1].addr =(I2C_ADDR>>1);
	msg[1].flags =0;
	msg[1].flags =I2C_M_RD;
	msg[1].len =ADDR_BYTE;
	msg[1].buf =buf;

	rdwr.msgs = &msg[0];
	rdwr.nmsgs = (u32)2;
    
	if (ADDR_BYTE == 2) {  // 2 byte 
	buf[0] = (addr >> 8) & 0xff;  // shift 8 
	buf[1] = addr & 0xff;   
	} else {
	buf[0] = addr & 0xff;
	}
	retval= ioctl(fd,I2C_RDWR,&rdwr);
	if(retval !=2) {
	
	printf("CMD_I2C_READ error\n");
	return -1;
	goto end1;
	}

	if (DATA_BYTE ==2){
		data = buf[1] | (buf[0] << 8);
	} else{
		data = buf[0];
	}
	retval = 0;
return data;
end1:
    close(fd);
end0:
    return retval;
}

//LT8911EXB_IIC_Write_byte(u8 RegAddr, u8 data);    // IIC write operation, IIC rate do not exceed 100KHz

//extern u8 i2c_read(u8 RegAddr);   // IIC read operation, IIC rate do not exceed 100KHz

enum {
	hfp = 0,
	hs,
	hbp,
	hact,
	htotal,
	vfp,
	vs,
	vbp,
	vact,
	vtotal,
	pclk_10khz
};

//u8		Read_DPCD010A = 0x00;

bool	ScrambleMode = 0;

bool	flag_mipi_on = 0;

#define _read_edid_ //7.13modefy
#define _uart_debug_ //7.13modefy

#ifdef _read_edid_ // read eDP panel EDID

u8		EDID_DATA[128] = { 0 };
u16		EDID_Timing[11] = { 0 };

bool	EDID_Reply = 0;
#endif

//////////////////////LT8911EXB Config////////////////////////////////
#define _1080P_eDP_Panel_

#define _MIPI_Lane_ 4   // 4 /3 /2 / 1

#define _MIPI_data_PN_Swap_En	0xF0
#define _MIPI_data_PN_Swap_Dis	0x00

#define _MIPI_data_PN_ _MIPI_data_PN_Swap_Dis

#define _No_swap_			0x00    // 3210 default
#define _MIPI_data_3210_	0       // default
#define _MIPI_data_0123_	21
#define _MIPI_data_2103_	20

#define _MIPI_data_sequence_ _No_swap_

#define _eDP_data_PN_Swap_En	0xF0 // Please refer to the notes below
#define _eDP_data_PN_Swap_Dis	0x00

#define _eDP_data_PN_ _eDP_data_PN_Swap_Dis // defailt ; disable

#define _Lane0_data_	0
#define _Lane1_data_	1
#define _Lane2_data_	2
#define _Lane3_data_	3

#define _eDP_data3_select_	(_Lane3_data_ << 6) // default; _Lane3_data_select_ is lane3
#define _eDP_data2_select_	(_Lane2_data_ << 4) // default; _Lane2_data_select_ is lane2

#define _eDP_data1_select_	(_Lane1_data_ << 2)	// default; _Lane1_data_select_ is lane1
#define _eDP_data0_select_	(_Lane0_data_ << 0) // default; _Lane0_data_select_ is lane0

// example:lane1 and lane0 swap
//#define _eDP_data1_select_	(_Lane0_data_ << 2)	// default _Lane1_data_select_ is lane0
//#define _eDP_data0_select_	(_Lane1_data_ << 0) // default _Lane0_data_select_ is lane1

#define _eDP_data_No_swap_	 0xe4 // default

#define _eDP_data_sequence_ _eDP_data_No_swap_ // default, no swap
// #define _eDP_data_sequence_ (_eDP_data3_select_ + _eDP_data2_select_ + _eDP_data1_select_ + _eDP_data0_select_) 

//#define _Nvid 0         // 0: 0x0080,default
#define _Nvid 0 //7.24
static int Nvid_Val[] = { 0x0080, 0x0800 };


#ifdef _1080P_eDP_Panel_

#define eDP_lane		2

#define PCR_PLL_PREDIV	0x40

// 根据前端MIPI信号的Timing，修改以下参数：
//According to the timing of the Mipi signal, modify the following parameters:
static int MIPI_Timing[] =
// hfp,	hs,	hbp,	hact,	htotal,	vfp,	vs,	vbp,	vact,	vtotal,	pixel_CLK/10000
//-----|---|------|-------|--------|-----|-----|-----|--------|--------|---------------
{ 88, 	44, 148, 	1920, 	2200, 	4, 		5, 	36, 	1080, 	1125, 		14850 };   // VESA
  //{ 48, 32, 132,   1920,   2132,   3,  6,  66,    1080,   1155,   14770 };    // SL156PP36

/*******************************************************************
   全志平台的屏参 lcd_hbp 是 上面参数 hbp + hs 的和；
   The lcd_hbp of Allwiner platform is the sum of the above parameters hbp + hs;

   同样的，全志平台的屏参 lcd_vbp 是 上面参数 vbp + vs的 和，这点要注意。
   In the same, lcd_vbp are the sum of the above parameters vbp + vs, which should be noted.
   //-------------------------------------------------------------------

   EOTP(End Of Transmite Packet，hs 传完了，会发这样一个包) 要打开，(之前的LT8911B 需要关闭EOTP，这里LT8911EXB 需要打开)
   Eotp (end of transmit packet, HS will send such a packet) must be turn-on (lt8911b needs to turn-off eotp, here lt8911exb needs to turn-on)

   1、MTK平台 的 dis_eotp_en 的值，改成 false;   LK和kernel都需要修改,改成false。
   1. The value of dis_eotp_en of MTK platform should be changed to 'false'; LK and kernel need to be changed to 'false'.

   2、展讯平台的 tx_eotp 的值置 1 。
   2. The value of tx_eotp of Spreadtrum platform is set to 1.

   3、RK平台 EN_EOTP_TX 置1.
   3. The value of EN_EOTP_TX of RK platform is set to 1.

   4、高通平台 找到 dsi_ctrl_hw_cmn.c -->void dsi_ctrl_hw_cmn_host_setup(struct dsi_ctrl_hw *ctrl,struct dsi_host_common_cfg *cfg)-->增加cfg->append_tx_eot = true;
   4、Qualcomm: dsi_ctrl_hw_cmn.c -->void dsi_ctrl_hw_cmn_host_setup(struct dsi_ctrl_hw *ctrl,struct dsi_host_common_cfg *cfg)-->add cfg->append_tx_eot = true;

*******************************************************************/
//#define _6bit_ // eDP panel Color Depth，262K color
#define _8bit_                                              // eDP panel Color Depth，16.7M color

#endif

void LT8911EX_ChipID(void)                                             // read Chip ID
{
	printf("\r\n###################start#####################\n");
	i2c_write(0xff, 0x81);                                 //register bank
	i2c_write(0x08, 0x7f);

#ifdef _uart_debug_ 
	printf("LT8911EXB chip ID:0x%x ", i2c_read(0x00));    // 0x17
	printf("0x%02x ", i2c_read(0x01));                        // 0x05
	printf("0x%x \n", i2c_read(0x02));                        // 0xE0
#endif  
}

/***********************************************************

***********************************************************/
void LT8911EXB_read_edid(void)
{
#ifdef _read_edid_
	u8 reg, i, j;
//	bool	aux_reply, aux_ack, aux_nack, aux_defer;
	i2c_write(0xff, 0xac);
	i2c_write(0x00, 0x20); //Soft Link train
	i2c_write(0xff, 0xa6);
	i2c_write(0x2a, 0x01);

	/*set edid offset addr*/
	i2c_write(0x2b, 0x40); //CMD
	i2c_write(0x2b, 0x00); //addr[15:8]
	i2c_write(0x2b, 0x50); //addr[7:0]
	i2c_write(0x2b, 0x00); //data lenth
	i2c_write(0x2b, 0x00); //data lenth
	i2c_write(0x2c, 0x00); //start Aux read edid

#ifdef _uart_debug_
	printf("\r\n");
	printf("Read eDP EDID......\n");
#endif

	usleep(20000);                         //more than 10ms
	reg = i2c_read(0x25);
	if((reg & 0x0f) == 0x0c)
	{
		for(j = 0; j < 8; j++)
		{
			if(j == 7)
			{
				i2c_write(0x2b, 0x10); //MOT
			}else
			{
				i2c_write(0x2b, 0x50);
			}

			i2c_write(0x2b, 0x00);
			i2c_write(0x2b, 0x50);
			i2c_write(0x2b, 0x0f);
			i2c_write(0x2c, 0x00); //start Aux read edid
			usleep(50000);                         //more than 50ms

			if(i2c_read(0x39) == 0x31)
			{
				i2c_read(0x2b);
				for(i = 0; i < 16; i++)
				{
					EDID_DATA[j * 16 + i] = i2c_read(0x2b);
				}

				EDID_Reply = 1;
			}else
			{
				EDID_Reply = 0;
#ifdef _uart_debug_
				printf("no_reply\r\n");
				//printf("\r\n");
#endif
				return;
			}
		}

#ifdef _uart_debug_

		for(i = 0; i < 128; i++) //print edid data
		{
			if((i % 16) == 0)
			{
				printf("\r\n");
			}
			printf("0x%02x ", EDID_DATA[i]);
		}

		printf("\r\n");
		printf("\r\neDP Timing = { H_FP / H_pluse / H_BP / H_act / H_tol / V_FP / V_pluse / V_BP / V_act / V_tol / D_CLK };" );
		printf("\r\neDP Timing = { " );
		EDID_Timing[hfp] = ((EDID_DATA[0x41] & 0xC0) * 4 + EDID_DATA[0x3e]);
		printf("%d, ",(u32)EDID_Timing[hfp]);        // HFB
		//printf(", ");

		EDID_Timing[hs] = ((EDID_DATA[0x41] & 0x30) * 16 + EDID_DATA[0x3f]);
		printf("%d, ",(u32)EDID_Timing[hs]);         // Hsync Wid
		//printf(", ");

		EDID_Timing[hbp] = (((EDID_DATA[0x3a] & 0x0f) * 0x100 + EDID_DATA[0x39]) - ((EDID_DATA[0x41] & 0x30) * 16 + EDID_DATA[0x3f]) - ((EDID_DATA[0x41] & 0xC0) * 4 + EDID_DATA[0x3e]));
		printf("%d, ",(u32)EDID_Timing[hbp]);        // HBP
		//printf(", ");

		EDID_Timing[hact] = ((EDID_DATA[0x3a] & 0xf0) * 16 + EDID_DATA[0x38]);
		printf("%d, ",(u32)EDID_Timing[hact]);       // H active
		//printf(", ");

		EDID_Timing[htotal] = ((EDID_DATA[0x3a] & 0xf0) * 16 + EDID_DATA[0x38] + ((EDID_DATA[0x3a] & 0x0f) * 0x100 + EDID_DATA[0x39]));
		printf("%d, ",(u32)EDID_Timing[htotal]);     // H total
		//printf(", ");

		EDID_Timing[vfp] = ((EDID_DATA[0x41] & 0x0c) * 4 + (EDID_DATA[0x40] & 0xf0) / 16);
		printf("%d, ",(u32)EDID_Timing[vfp]);        // VFB
		//printf(", ");

		EDID_Timing[vs] = ((EDID_DATA[0x41] & 0x03) * 16 + (EDID_DATA[0x40] & 0x0f));
		printf("%d, ",(u32)EDID_Timing[vs]);         // Vsync Wid
		//printf(", ");

		EDID_Timing[vbp] = (((EDID_DATA[0x3d] & 0x03) * 0x100 + EDID_DATA[0x3c]) - ((EDID_DATA[0x41] & 0x03) * 16 + (EDID_DATA[0x40] & 0x0f)) - ((EDID_DATA[0x41] & 0x0c) * 4 + (EDID_DATA[0x40] & 0xf0) / 16));
		printf("%d, ",(u32)EDID_Timing[vbp]);        // VBP
		//printf(", ");

		EDID_Timing[vact] = ((EDID_DATA[0x3d] & 0xf0) * 16 + EDID_DATA[0x3b]);
		printf("%d, ",(u32)EDID_Timing[vact]);       // V active
		//printf(", ");

		EDID_Timing[vtotal] = ((EDID_DATA[0x3d] & 0xf0) * 16 + EDID_DATA[0x3b] + ((EDID_DATA[0x3d] & 0x03) * 0x100 + EDID_DATA[0x3c]));
		printf("%d, ",(u32)EDID_Timing[vtotal]);     // V total
		//printf(", ");

		EDID_Timing[pclk_10khz] = (EDID_DATA[0x37] * 0x100 + EDID_DATA[0x36]);
		printf("%d, ",(u32)EDID_Timing[pclk_10khz]); // CLK
		printf("};");
		printf("\r\n");
#endif
	}

	return;
#endif
}

/***********************************************************

***********************************************************/
void LT8911EXB_MIPI_Video_Timing(void)                                    // (struct video_timing *video_format)
{
	i2c_write(0xff, 0xd0);
	i2c_write(0x0d, (u8)(MIPI_Timing[vtotal] / 256));  //得到高八位
	i2c_write(0x0e, (u8)(MIPI_Timing[vtotal] % 256));  //得到低八位  //vtotal
	i2c_write(0x0f, (u8)(MIPI_Timing[vact] / 256));
	i2c_write(0x10, (u8)(MIPI_Timing[vact] % 256));      //vactive

	i2c_write(0x11, (u8)(MIPI_Timing[htotal] / 256));
	i2c_write(0x12, (u8)(MIPI_Timing[htotal] % 256));    //htotal
	i2c_write(0x13, (u8)(MIPI_Timing[hact] / 256));
	i2c_write(0x14, (u8)(MIPI_Timing[hact] % 256));      //hactive

	i2c_write(0x15, (u8)(MIPI_Timing[vs] % 256));        //vsa
	i2c_write(0x16, (u8)(MIPI_Timing[hs] % 256));        //hsa
	i2c_write(0x17, (u8)(MIPI_Timing[vfp] / 256));
	i2c_write(0x18, (u8)(MIPI_Timing[vfp] % 256));       //vfp

	i2c_write(0x19, (u8)(MIPI_Timing[hfp] / 256));
	i2c_write(0x1a, (u8)(MIPI_Timing[hfp] % 256));       //hfp
}

void LT8911EXB_eDP_Video_cfg(void)                                        // (struct video_timing *video_format)
{
	i2c_write(0xff, 0xa8);
	i2c_write(0x2d, 0x88);                 // MSA from register

	i2c_write(0x05, (u8)(MIPI_Timing[htotal] / 256));
	i2c_write(0x06, (u8)(MIPI_Timing[htotal] % 256));
	i2c_write(0x07, (u8)((MIPI_Timing[hs] + MIPI_Timing[hbp]) / 256));
	i2c_write(0x08, (u8)((MIPI_Timing[hs] + MIPI_Timing[hbp]) % 256));
	i2c_write(0x09, (u8)(MIPI_Timing[hs] / 256));
	i2c_write(0x0a, (u8)(MIPI_Timing[hs] % 256));
	i2c_write(0x0b, (u8)(MIPI_Timing[hact] / 256));
	i2c_write(0x0c, (u8)(MIPI_Timing[hact] % 256));
	i2c_write(0x0d, (u8)(MIPI_Timing[vtotal] / 256));
	i2c_write(0x0e, (u8)(MIPI_Timing[vtotal] % 256));
	i2c_write(0x11, (u8)((MIPI_Timing[vs] + MIPI_Timing[vbp]) / 256));
	i2c_write(0x12, (u8)((MIPI_Timing[vs] + MIPI_Timing[vbp]) % 256));
	i2c_write(0x14, (u8)(MIPI_Timing[vs] % 256));
	i2c_write(0x15, (u8)(MIPI_Timing[vact] / 256));
	i2c_write(0x16, (u8)(MIPI_Timing[vact] % 256));

}

void LT8911EXB_init(void)
{
	u8	i;
	u8	pcr_pll_postdiv;
	u8	pcr_m;
	u16 Temp16;

	/* init */
	i2c_write(0xff, 0x81); // Change Reg bank
	i2c_write(0x08, 0x7f); // i2c over aux issue
	i2c_write(0x49, 0xff); // enable 0x87xx

	i2c_write(0xff, 0x82); // Change Reg bank
	i2c_write(0x5a, 0x0e); // GPIO test output

	//for power consumption//
	i2c_write(0xff, 0x81);
	i2c_write(0x05, 0x06);
	i2c_write(0x43, 0x00);
	i2c_write(0x44, 0x1f);
	i2c_write(0x45, 0xf7);
	i2c_write(0x46, 0xf6);
	i2c_write(0x49, 0x7f);

	i2c_write(0xff, 0x82);
#if (eDP_lane == 2)
	{
		i2c_write(0x12, 0x33);
	}
#elif (eDP_lane == 1)
	{
		i2c_write(0x12, 0x11);
	}
#elif (eDP_lane == 4)
	{
		i2c_write(0x12, 0xff);
	}

#endif

	/* mipi Rx analog */
	i2c_write(0xff, 0x82); // Change Reg bank
	i2c_write(0x32, 0x51);
	i2c_write(0x35, 0x22); //EQ current 0x22/0x42/0x62/0x82/0xA2/0xC2/0xe2
	i2c_write(0x3a, 0x77); //EQ 12.5db 0x11/0x22/0x33/0x44/0x55/0x66/0x77
	i2c_write(0x3b, 0x77); //EQ 12.5db 0x11/0x22/0x33/0x44/0x55/0x66/0x77

	i2c_write(0x4c, 0x0c);
	i2c_write(0x4d, 0x00);

	/* dessc_pcr  pll analog */
	i2c_write(0xff, 0x82); // Change Reg bank
	i2c_write(0x6a, 0x40);
	i2c_write(0x6b, PCR_PLL_PREDIV);

	Temp16 = MIPI_Timing[pclk_10khz];

	if(MIPI_Timing[pclk_10khz] < 8800)
	{
		i2c_write(0x6e, 0x82); //0x44:pre-div = 2 ,pixel_clk=44~ 88MHz
		pcr_pll_postdiv = 0x08;
	}else
	if(MIPI_Timing[pclk_10khz] < 17600)
	{
		i2c_write(0x6e, 0x81); //0x40:pre-div = 1, pixel_clk =88~176MHz
		pcr_pll_postdiv = 0x04;
	}else
	{
		i2c_write(0x6e, 0x80); //0x40:pre-div = 0, pixel_clk =176~200MHz
		pcr_pll_postdiv = 0x02;
	}

	pcr_m = (u8)(Temp16 * pcr_pll_postdiv / 25 / 100);

	/* dessc pll digital */
	i2c_write(0xff, 0x85);     // Change Reg bank
	i2c_write(0xa9, 0x31);
	i2c_write(0xaa, 0x17);
	i2c_write(0xab, 0xba);
	i2c_write(0xac, 0xe1);
	i2c_write(0xad, 0x47);
	i2c_write(0xae, 0x01);
	i2c_write(0xae, 0x11);

	/* Digital Top */
	i2c_write(0xff, 0x85);                             // Change Reg bank
	i2c_write(0xc0, 0x01);                             //select mipi Rx
#ifdef _6bit_
	i2c_write(0xb0, 0xd0);                             //enable dither
#else
	i2c_write(0xb0, 0x00);                             // disable dither
#endif

	/* mipi Rx Digital */
	i2c_write(0xff, 0xd0);                             // Change Reg bank
	i2c_write(0x00, _MIPI_data_PN_ + _MIPI_Lane_ % 4); // 0: 4 Lane / 1: 1 Lane / 2 : 2 Lane / 3: 3 Lane
	i2c_write(0x02, 0x08);                             //settle
	i2c_write(0x03, _MIPI_data_sequence_);             // default is 0x00
	i2c_write(0x08, 0x00);
	//i2c_write(0x0a, 0x12);               //pcr mode  rock

	i2c_write(0x0c, 0x80);                             //fifo position rock
	i2c_write(0x1c, 0x80);                             //fifo position rock

	//	hs mode:MIPI行采样；vs mode:MIPI帧采样
	i2c_write(0x24, 0x70);                             // 0x30  [3:0]  line limit	  //pcr mode(de hs vs)

	i2c_write(0x31, 0x0a);

	/*stage1 hs mode*/
	i2c_write(0x25, 0x90);                             // 0x80		   // line limit
	i2c_write(0x2a, 0x3a);                             // 0x04		   // step in limit
	i2c_write(0x21, 0x4f);                             // hs_step
	i2c_write(0x22, 0xff);

	/*stage2 de mode*/
	i2c_write(0x0a, 0x02);                             //de adjust pre line
	i2c_write(0x38, 0x02);                             //de_threshold 1
	i2c_write(0x39, 0x04);                             //de_threshold 2
	i2c_write(0x3a, 0x08);                             //de_threshold 3
	i2c_write(0x3b, 0x10);                             //de_threshold 4

	i2c_write(0x3f, 0x04);                             //de_step 1
	i2c_write(0x40, 0x08);                             //de_step 2
	i2c_write(0x41, 0x10);                             //de_step 3
	i2c_write(0x42, 0x60);                             //de_step 4

	/*stage2 hs mode*/
	i2c_write(0x1e, 0x1f);                             // 0x01 changed 2023.08.14
	i2c_write(0x23, 0xf0);                             // 0x80			   //

	i2c_write(0x2b, 0x80);                             // 0xa0

#ifdef _Test_Pattern_
	i2c_write(0x26, (pcr_m | 0x80));
#else

	i2c_write(0x26, pcr_m);

//	i2c_write(0x27, Read_0xD095);
//	i2c_write(0x28, Read_0xD096);
#endif

	LT8911EXB_MIPI_Video_Timing();         //defualt setting is 1080P

	i2c_write(0xff, 0x81); // Change Reg bank
	i2c_write(0x03, 0x7b); //PCR reset
	i2c_write(0x03, 0xff);

#ifdef _eDP_2G7_
	i2c_write(0xff, 0x87);
	i2c_write(0x19, 0x31);
//	i2c_write(0x1a, 0x36); // sync m
	i2c_write(0x1a,0x1b);
	i2c_write(0x1b, 0x00); // sync_k [7:0]
	i2c_write(0x1c, 0x00); // sync_k [13:8]

	// txpll Analog
	i2c_write(0xff, 0x82);
	i2c_write(0x09, 0x00); // div hardware mode, for ssc.

//	i2c_write(0x01, 0x18);// default : 0x18
	i2c_write(0x02, 0x42);
	i2c_write(0x03, 0x00); // txpll en = 0
	i2c_write(0x03, 0x01); // txpll en = 1
//	i2c_write(0x04, 0x3a);// default : 0x3A
	i2c_write(0x0a,0x1b);
	i2c_write(0x04,0x2a);
		
	i2c_write(0xff, 0x87);
	i2c_write(0x0c, 0x10); // cal en = 0

	i2c_write(0xff, 0x81);
	i2c_write(0x09, 0xfc);
	i2c_write(0x09, 0xfd);

	i2c_write(0xff, 0x87);
	i2c_write(0x0c, 0x11); // cal en = 1

	// ssc
	i2c_write(0xff, 0x87);
	i2c_write(0x13, 0x83);
	i2c_write(0x14, 0x41);
	i2c_write(0x16, 0x0a);
	i2c_write(0x18, 0x0a);
	i2c_write(0x19, 0x33);
#endif

#ifdef _eDP_1G62_
	i2c_write(0xff, 0x87);
	i2c_write(0x19, 0x31);
	i2c_write(0x1a, 0x20); // sync m
	i2c_write(0x1b, 0x19); // sync_k [7:0]
	i2c_write(0x1c, 0x99); // sync_k [13:8]

	// txpll Analog
	i2c_write(0xff, 0x82);
	i2c_write(0x09, 0x00); // div hardware mode, for ssc.
	//	i2c_write(0x01, 0x18);// default : 0x18
	i2c_write(0x02, 0x42);
	i2c_write(0x03, 0x00); // txpll en = 0
	i2c_write(0x03, 0x01); // txpll en = 1
	//	i2c_write(0x04, 0x3a);// default : 0x3A

	i2c_write(0xff, 0x87);
	i2c_write(0x0c, 0x10); // cal en = 0

	i2c_write(0xff, 0x81);
	i2c_write(0x09, 0xfc);
	i2c_write(0x09, 0xfd);

	i2c_write(0xff, 0x87);
	i2c_write(0x0c, 0x11); // cal en = 1

	//ssc
	i2c_write(0xff, 0x87);
	i2c_write(0x13, 0x83);
	i2c_write(0x14, 0x41);
	i2c_write(0x16, 0x0a);
	i2c_write(0x18, 0x0a);
	i2c_write(0x19, 0x33);
#endif

	i2c_write(0xff, 0x87);

	for(i = 0; i < 5; i++) //Check Tx PLL
	{
		usleep(5000);
		if(i2c_read(0x37) & 0x02)
		{
			printf("LT8911 tx pll locked\r\n");
			i2c_write(0xff,0x87);
			i2c_write(0x1a,0x36);
			i2c_write(0xff,0x82);
			i2c_write(0x0a,0x36);
			i2c_write(0x04,0x3a);
			break;
		}else
		{       
			printf("LT8911 tx pll unlocked\r\n");
			i2c_write(0xff, 0x81);
			i2c_write(0x09, 0xfc);
			i2c_write(0x09, 0xfd);

			i2c_write(0xff, 0x87);
			i2c_write(0x0c, 0x10);
			i2c_write(0x0c, 0x11);
		}
	}

	i2c_write(0xff, 0xac); // Change Reg bank
	i2c_write(0x15, _eDP_data_sequence_); // eDP data swap
	i2c_write(0x16, _eDP_data_PN_); // eDP P / N swap

	// AUX reset
	i2c_write(0xff, 0x81); // Change Reg bank
	i2c_write(0x07, 0xfe);
	i2c_write(0x07, 0xff);
	i2c_write(0x0a, 0xfc);
	i2c_write(0x0a, 0xfe);

	/* tx phy */
	i2c_write(0xff, 0x82); // Change Reg bank
	i2c_write(0x11, 0x00);
	i2c_write(0x13, 0x10);
	i2c_write(0x14, 0x0c);
	i2c_write(0x14, 0x08);
	i2c_write(0x13, 0x20);

	i2c_write(0xff, 0x82); // Change Reg bank
	i2c_write(0x0e, 0x35);

	/*eDP Tx Digital */
	i2c_write(0xff, 0xa8); // Change Reg bank

#ifdef _Test_Pattern_

	i2c_write(0x24, 0x50); // bit2 ~ bit 0 : test panttern image mode
	i2c_write(0x25, 0x70); // bit6 ~ bit 4 : test Pattern color
	i2c_write(0x27, 0x50); //0x50:Pattern; 0x10:mipi video

//	i2c_write(0x2d, 0x00); //  pure color setting
//	i2c_write(0x2d, 0x84); // black color
	i2c_write(0x2d, 0x88); //  block

#else
	i2c_write(0x27, 0x10); //0x50:Pattern; 0x10:mipi video     
#endif

#ifdef _8bit_
	i2c_write(0x17, 0x10);
	i2c_write(0x18, 0x20);
#else
	//_6bit_
	i2c_write(0x17, 0x00);
	i2c_write(0x18, 0x00);
#endif

	/* nvid */
	i2c_write(0xff, 0xa0);                             // Change Reg bank
	i2c_write(0x00, (u8)(Nvid_Val[_Nvid] / 256));    // 0x08
	i2c_write(0x01, (u8)(Nvid_Val[_Nvid] % 256));    // 0x00
}

//-------------------------------------------------------------------
/* mipi should be ready before configuring below video check setting*/
void LT8911EXB_video_check(void)
{
	//u8	temp;
	u32 reg = 0x00;
	/* mipi byte clk check*/
	i2c_write(0xff, 0x85);     // Change Reg bank
	i2c_write(0x1d, 0x00);     //FM select byte clk
	i2c_write(0x40, 0xf7);
	i2c_write(0x41, 0x30);
    //i2c_write(0xa1, 0x82); //eDP scramble mode; 7.25modefy

	//#ifdef _eDP_scramble_
	if(ScrambleMode)
	{
		i2c_write(0xa1, 0x82); //eDP scramble mode;
	}
	//#else
	else
	{
		i2c_write(0xa1, 0x02); // DP scramble mode;
	}
	//#endif

    //i2c_write(0x17, 0xf0); // 0xf0:Close scramble; 0xD0 : Open scramble

	i2c_write(0xff, 0x81);
	i2c_write(0x09, 0x7d);
	i2c_write(0x09, 0xfd);

	i2c_write( 0xff, 0x85);
	usleep(200000);
	if(i2c_read(0x50) == 0x03)
	{
		reg	   = i2c_read(0x4d);
		reg	   = reg * 256 + i2c_read(0x4e);
		reg	   = reg * 256 + i2c_read(0x4f);

		printf("video check: mipi byteclk = %d\n",reg); // mipi byteclk = reg * 1000
		//printf("%d\r\n",reg);
	}else
	{
		printf("video check: mipi clk unstable\r\n");
	}

	/* mipi vtotal check*/
	reg	   = i2c_read(0x76);
	reg	   = reg * 256 + i2c_read(0x77);

	printf("video check: Vtotal = %d\n",reg);
	//printf("%d\r\n",reg);

	/* mipi word count check*/
	i2c_write(0xff, 0xd0);
	reg	   = i2c_read(0x82);
	reg	   = reg * 256 + i2c_read(0x83);
	reg	   = reg / 3;

	printf("video check: Hact(word counter) = %d\n",reg);
	//printf("%d\r\n",reg);

	/* mipi Vact check*/
	reg	   = i2c_read(0x85);
	reg	   = reg * 256 + i2c_read(0x86);

	printf("video check: Vact = %d\n",reg);
	//printf("%d\r\n",reg);
}

//------------------------------------------------------------------
void DpcdWrite(u32 Address, u8 Data)
{
	/***************************
	   注意大小端的问题!
	   这里默认是大端模式

	   Pay attention to the Big-Endian and Little-Endian!
	   The default mode is Big-Endian here.

	 ****************************/
	u8	AddressH   = 0x0f & (Address >> 16);
	u8	AddressM   = 0xff & (Address >> 8);
	u8	AddressL   = 0xff & Address;

	u8	reg;

	i2c_write(0xff, 0xa6);
	i2c_write(0x2b, (0x80 | AddressH));  //CMD
	i2c_write(0x2b, AddressM);             //addr[15:8]
	i2c_write(0x2b, AddressL);             //addr[7:0]
	i2c_write(0x2b, 0x00);                 //data lenth
	i2c_write(0x2b, Data);                 //data
	i2c_write(0x2c, 0x00);                 //start Aux

	usleep(20000);                                         //more than 10ms
	reg = i2c_read(0x25);

	if((reg & 0x0f) == 0x0c)
	{
		return;
	}
}

u8 DpcdRead(u32 Address)
{
	/***************************
	   注意大小端的问题!
	   这里默认是大端模式

	   Pay attention to the Big-Endian and Little-Endian!
	   The default mode is Big-Endian here.

	 ****************************/

	u8	DpcdValue  = 0x00;
	u8	AddressH   = 0x0f & (Address >> 16);
	u8	AddressM   = 0xff & (Address >> 8);
	u8	AddressL   = 0xff & Address;
	u8	reg;

	i2c_write(0xff, 0xac);
	i2c_write(0x00, 0x20);                 //Soft Link train
	i2c_write(0xff, 0xa6);
	i2c_write(0x2a, 0x01);

	i2c_write(0xff, 0xa6);
	i2c_write(0x2b, (0x90 | AddressH));  //CMD
	i2c_write(0x2b, AddressM);             //addr[15:8]
	i2c_write(0x2b, AddressL);             //addr[7:0]
	i2c_write(0x2b, 0x00);                 //data lenth
	i2c_write(0x2c, 0x00);                 //start Aux read edid

	usleep(50000);                                         //more than 10ms
	reg = i2c_read(0x25);
	if((reg & 0x0f) == 0x0c)
	{
		if(i2c_read(0x39) == 0x22)
		{
			i2c_read(0x2b);
			DpcdValue = i2c_read(0x2b);
		}
		/*
        else
        {
        //	goto no_reply;
        //	DpcdValue = 0xff;
        //return DpcdValue;
        }*/
	}else
	{
		i2c_write(0xff, 0x81); // change bank
		i2c_write(0x07, 0xfe);
		i2c_write(0x07, 0xff);
		i2c_write(0x0a, 0xfc);
		i2c_write(0x0a, 0xfe);
	
    }
	return DpcdValue;;
}

void LT8911EX_link_train(void)
{
	i2c_write(0xff, 0x81);
	i2c_write(0x06, 0xdf); // rset VID TX
	i2c_write(0x06, 0xff);

	i2c_write(0xff, 0x85);
        //i2c_write( 0x17, 0xd0); // turn on scramble

	//i2c_write( 0x17, 0xf0); // turn off scramble

//#ifdef _eDP_scramble_
	if(ScrambleMode)
	{
		i2c_write(0xa1, 0x82); // eDP scramble mode;

		/* Aux operater init */
		i2c_write(0xff, 0xac);
		i2c_write(0x00, 0x20); //Soft Link train
		i2c_write(0xff, 0xa6);
		i2c_write(0x2a, 0x01);

		DpcdWrite(0x010a, 0x01);
		usleep(10000);
		DpcdWrite(0x0102, 0x00);
		usleep(10000);
		DpcdWrite(0x010a, 0x01);
		
		usleep(200000);
	}
//#else
	else
	{
		i2c_write(0xa1, 0x02); // DP scramble mode;
	}
//#endif
	/* Aux setup */
	i2c_write(0xff, 0xac);
	i2c_write(0x00, 0x60);     //Soft Link train
	i2c_write(0xff, 0xa6);
	i2c_write(0x2a, 0x00);

	i2c_write(0xff, 0x81);
	i2c_write(0x07, 0xfe);
	i2c_write(0x07, 0xff);
	i2c_write(0x0a, 0xfc);
	i2c_write(0x0a, 0xfe);

	/* link train */

	i2c_write(0xff, 0x85);
	i2c_write(0x1a, eDP_lane);

#ifdef _link_train_enable_
	i2c_write(0xff, 0xac);
	i2c_write(0x00, 0x64);
	i2c_write(0x01, 0x0a);
	i2c_write(0x0c, 0x85);
	i2c_write(0x0c, 0xc5);
#else
	i2c_write(0xff, 0xac);
	i2c_write(0x00, 0x00);
	i2c_write(0x01, 0x0a);
	i2c_write(0x14, 0x80);
	i2c_write(0x14, 0x81);
	usleep(50000);
	i2c_write(0x14, 0x84);
	usleep(50000);
	i2c_write(0x14, 0xc0);
#endif
}

void LT8911EX_link_train_result(void)
{
	u8 i, reg;
	i2c_write(0xff, 0xac);
	for(i = 0; i < 10; i++)
	{
		reg = i2c_read(0x82);
		//Debug_DispStrNum("0x82 = \r\n", reg);
		if(reg & 0x20)
		{
			if((reg & 0x1f) == 0x1e)
			{
				printf("Link train success, 0x82 = 0x%x\r\n", reg);
			} else
			{
				printf("Link train fail, 0x82 = 0x%x\r\n", reg);
			}

			printf("panel link rate: 0x%02x\r\n", i2c_read(0x83));
			printf("panel link count: 0x%x\r\n", i2c_read(0x84));
			return;
		}else
		{
		    printf("link trian on going...\r\n");
		}
		usleep(100000);
	}
}

void LT8911EXB_MainLoop(void)
{
#ifndef _Test_Pattern_
	u16 reg;

	i2c_write(0xff, 0x85);
	//i2c_write(0x1d,0x00); //FM select byte clk
	//i2c_write(0x40,0xf7);
	//i2c_write(0x41,0x30);

	//if(ScrambleMode)
	//{
		i2c_write(0xa1, 0x82); //
	//}else
	//{
	//	i2c_write(0xa1, 0x02); //
	//}

	i2c_write(0xff, 0x81);     //video check rst
	i2c_write(0x09, 0x7d);
	i2c_write(0x09, 0xfd);
	usleep(50000);

	i2c_write( 0xff, 0x85);
	reg	   = i2c_read(0x76);
	reg	   = reg * 256 + i2c_read(0x77);

	//if(reg == MIPI_Timing[vtotal])
	if((reg > (MIPI_Timing[vtotal] - 5)) && (reg < (MIPI_Timing[vtotal] + 5)))
	{
		if(!flag_mipi_on)
		{
			i2c_write(0xff, 0x81); //PCR reset
			i2c_write(0x03, 0x7b);
			i2c_write(0x03, 0xff);

			i2c_write(0xff, 0xa8);
			i2c_write(0x2d, 0x88);
			flag_mipi_on = 1;
			printf("\r\nPCR reset");
		}
	}else
	{
		i2c_write(0xff, 0xa8);
		i2c_write(0x2d, 0x8c); //edp output idle pattern;
		flag_mipi_on = 0;
	}

#ifdef _uart_debug_
	i2c_write(0xff, 0xd0);
	reg = i2c_read(0x87);
//	reg	   = reg * 256 + HDMI_ReadI2C_Byte(0x77);

	printf("\r\nReg0xD087 =  ");
	printf("%x", reg);
	printf("\r\n ");
	if(reg & 0x10)
	{
		printf("\r\nPCR Clock stable");
	}else
	{
		printf("\r\nPCR Clock unstable");
	}
	printf("\r\n ");
#endif

#endif
}
/*
   reg 0x822/0x8226 : bit1 bit0
   Da_edptx0_tap0_current_tune software output set value:

   2'b00 = None;
   2'b01 = 12.8mA;
   2'b10 = 15mA

   reg 0x8223/ 0x8227 bit7 ~ bit0
   Da_edptx_tap0_current_tune software output set value:

   8'b00000000 = None;
   8'b00000001 = 50uA;
   8'b00000010 = 100uA;
   8'b00000100 = 200uA;
   8'b00001000 = 400uA;
   8'b00010000 = 800uA;
   8'b00100000 = 1.6mA;
   8'b01000000 = 3.2mA;
   8'b10000000 = 6.4mA
 */
enum
{
	_Level0_ = 0,                                               // 27.8 mA  0x83/0x00
	_Level1_,                                                   // 26.2 mA  0x82/0xe0
	_Level2_,                                                   // 24.6 mA  0x82/0xc0
	_Level3_,                                                   // 23 mA    0x82/0xa0
	_Level4_,                                                   // 21.4 mA  0x82/0x80
	_Level5_,                                                   // 18.2 mA  0x82/0x40
	_Level6_,                                                   // 16.6 mA  0x82/0x20
	_Level7_,                                                   // 15mA     0x82/0x00  // level 1
	_Level8_,                                                   // 12.8mA   0x81/0x00  // level 2
	_Level9_,                                                   // 11.2mA   0x80/0xe0  // level 3
	_Level10_,                                                  // 9.6mA    0x80/0xc0  // level 4
	_Level11_,                                                  // 8mA      0x80/0xa0  // level 5
	_Level12_,                                                  // 6mA      0x80/0x80  // level 6
};

u8	Swing_Setting1[] = { 0x83, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x81, 0x80, 0x80, 0x80, 0x80 };
u8	Swing_Setting2[] = { 0x00, 0xe0, 0xc0, 0xa0, 0x80, 0x40, 0x20, 0x00, 0x00, 0xe0, 0xc0, 0xa0, 0x80 };

u8	Level = _Level7_;                                           // normal

void LT8911EX_TxSwingPreSet(void)
{
	i2c_write(0xFF, 0x82);
	i2c_write(0x22, Swing_Setting1[Level]);    //lane 0 tap0
	i2c_write(0x23, Swing_Setting2[Level]);
	i2c_write(0x24, 0x80);                     //lane 0 tap1
	i2c_write(0x25, 0x00);

#if (eDP_lane == 2)
	i2c_write(0x26, Swing_Setting1[Level]);    //lane 1 tap0
	i2c_write(0x27, Swing_Setting2[Level]);
	i2c_write(0x28, 0x80);                     //lane 1 tap1
	i2c_write(0x29, 0x00);
#endif
}

void PCR_Status(void)                                         // for debug
{
#ifdef _uart_debug_
	u8 reg;

	i2c_write( 0xff, 0xd0);
	reg = i2c_read(0x87);

	printf("Reg0xD087 =%x",reg);
	//printf("%x",reg);
	printf("\r\n");
	if(reg & 0x10)
	{
		printf("PCR Clock stable\r\n");
	}else
	{
		printf("PCR Clock unstable\r\n");
	}
	printf("\r\n ");
#endif
}

// void Reset_LT8911EXB(void)
// {
// 	_LT8911_RSTN_LOW;   // GPIO Low
// 	usleep(100000);
// 	_LT8911_RSTN_High;  // GPIO High
// 	usleep(100000);
// }
void Reset_LT8911EXB(void)
{
    gpio_contrl_out(3,4,0); // GPIO Low
    usleep(120000);
    gpio_contrl_out(3,4,1); //GPIO High
    usleep(120000);
	
    //return;
}

void LT8911_MainLoop(void)
{
	u16 reg, H_act, V_act;
//	bool	flag_mipi_on = 0;

	i2c_write(0xff, 0x85);

	//if(ScrambleMode)
	//{
		i2c_write(0xa1, 0x82); //video check from mipi
	//}else
	//{
	//	i2c_write(0xa1, 0x02);
	//}

	i2c_write(0xff, 0x81);     //video check rst
	i2c_write(0x09, 0x7d);
	i2c_write(0x09, 0xfd);
	usleep(50000);

	/* mipi word count check*/
	i2c_write(0xff, 0xd0);
	reg	   = i2c_read(0x82);
	reg	   = reg * 256 + i2c_read(0x83);
	H_act  = reg / 3;
	printf("%d",(int)H_act);

#ifdef _uart_debug_

	printf("\r\nHact(word counter) =  "); // H active = word counter / 3
        printf("%d",(int)H_act);
	//printf(H_act);
	printf("\r\n ");

#endif

	/* mipi Vact check*/
	reg	   = i2c_read(0x85);
	V_act  = reg * 256 + i2c_read(0x86);
    printf("%d",(int)V_act);
#ifdef _uart_debug_

	printf("\r\nVact =  ");
        printf("%d",(int)V_act);
	//printf(V_act);
	printf("\r\n ");
#endif

	i2c_write(0xff, 0x85);
	reg	   = i2c_read(0x76);
	reg	   = reg * 256 + i2c_read(0x77);

#ifdef _uart_debug_
	printf("\r\nvideo check: Vtotal =  ");
	//printf(reg);
        printf("%d",(int)reg);
	printf("\r\n ");
#endif

//	if(reg == MIPI_Timing[vtotal])
	if((reg > (MIPI_Timing[vtotal] - 5)) && (reg < (MIPI_Timing[vtotal] + 5)))
	{
		if(!flag_mipi_on)
		{
			i2c_write(0xff, 0x81); //PCR reset
			i2c_write(0x03, 0x7b);
			i2c_write(0x03, 0xff);

			i2c_write(0xff, 0xa8);
			i2c_write(0x2d, 0x88);
			flag_mipi_on = 1;
#ifdef _uart_debug_
			printf("\r\nPCR reset");
#endif
		}
	}else
	{
		i2c_write(0xff, 0xa8);
		i2c_write(0x2d, 0x8c); //edp output idle pattern;
		flag_mipi_on = 0;
	}

#ifdef _uart_debug_
	i2c_write(0xff, 0xd0);
	reg = i2c_read(0x87);

	printf("\r\nReg0xD087 =  ");
	printf("%x", reg);
	printf("\r\n ");
	if(reg & 0x10)
	{
		printf("\r\nPCR Clock stable");
	}else
	{
		printf("\r\nPCR Clock unstable");
	}
	printf("\r\n ");
#endif
}

void LT8911EXB_LinkTrainResultCheck(void)
{
#ifdef _link_train_enable_
	u8	i;
	u8	val;
	//int ret;

	i2c_write(0xff, 0xac);
	for(i = 0; i < 10; i++)
	{
		val = i2c_read(0x82);
		if(val & 0x20)
		{
			if((val & 0x1f) == 0x1e)
			{
#ifdef _uart_debug_
				//   printf("\r\nLT8911_LinkTrainResultCheck: edp link train successed: 0x%bx", val);
				printf("edp link train successed: 0x%x\r\n", val);
#endif
				return;
			}else
			{
#ifdef _uart_debug_
				//printf("\r\nLT8911_LinkTrainResultCheck: edp link train failed: 0x%bx", val);
				printf("edp link train failed: 0x%x\r\n", val);
#endif
				i2c_write(0xff, 0xac);
				i2c_write(0x00, 0x00);
				i2c_write(0x01, 0x0a);
				i2c_write(0x14, 0x80);
				i2c_write(0x14, 0x81);
				usleep(50000);
				i2c_write(0x14, 0x84);
				usleep(50000);
				i2c_write(0x14, 0xc0);
				//printf("\r\nLT8911_LinkTrainResultCheck: Enable eDP video output while linktrian fail");
			}

#ifdef _uart_debug_

			val = i2c_read(0x83);
			//printf("\r\nLT8911_LinkTrainResultCheck: panel link rate: 0x%bx",val);
			printf("panel link rate: 0x%02x\r\n", val);
			val = i2c_read(0x84);
			//printf("\r\nLT8911_LinkTrainResultCheck: panel link count: 0x%bx",val);
			printf("panel link count: 0x%x\r\n", val);
#endif
			usleep(100000); // return;
		}else
		{
			//printf("\r\nLT8911_LinkTrainResultCheck: link trian on going...");
			usleep(100000);
		}
	}
#endif
}

void                      LT8911EXB_config(void)
//void main(void)
{
/************************************************
    step1:  配置MIPI屏复位
*************************************************/
	Reset_LT8911EXB();     // 先Reset LT8911EXB ,用GPIO 先拉低LT8911EXB的复位脚 100ms左右，再拉高，保持100ms。

	LT8911EX_ChipID();    // read Chip ID

	LT8911EXB_eDP_Video_cfg();

 	LT8911EXB_init();

 	LT8911EXB_read_edid(); // for debug
 	ScrambleMode = 0;

 	LT8911EX_TxSwingPreSet();

 	LT8911EX_link_train();

 	LT8911EXB_LinkTrainResultCheck();

//======================================//
 	LT8911EX_link_train_result();  // for debug

	LT8911EXB_video_check();       // just for Check MIPI Input

 	PCR_Status();                  // just for Check PCR CLK

/* If lt8911exb is controlled by MCU,LT8911_MainLoop() function is added in the main loop.*/
//    while(1)
//    {
//    // 循环检测MIPI信号，如果有断续，需要reset PCR
//    //Loop detection of Mipi signal. If there is interruption, reset PCR will appear
    //LT8911EXB_MainLoop();

    //usleep(1000000);
//    }

}

/************************************** The End Of File **************************************/

