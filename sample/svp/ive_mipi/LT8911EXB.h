#ifndef __LT8911EXB_H__
#define __LT8911EXB_H__

#include <unistd.h>
#include "sample_comm.h"

#define MAX_SENSOR_NUM 3

//int LT8911EXB_i2c_init();

//int gpio_contrl_in(unsigned int gpio_chip_num, unsigned int gpio_offset_num);

int gpio_contrl_out(unsigned int gpio_chip_num, unsigned int gpio_offset_num,unsigned int gpio_out_val);


HI_S32 VOUT_MIPI(HI_U32 CamID,HI_U32 PortID,HI_U32 Sns_fps,VO_INTF_SYNC_E OUT_Type);
//HI_S32 VOUT_MIPI();

void LT8911EXB_config(void);


#endif /* End of #ifndef __SAMPLE_VIO_H__*/

