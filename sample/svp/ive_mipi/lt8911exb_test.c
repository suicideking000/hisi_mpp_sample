#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include "LT8911EXB.h"
#include "sample_comm.h"
#include "hi_mipi_tx.h"

//HI_S32 VOUT_MIPI()
HI_S32 VOUT_MIPI(HI_U32 CamID,HI_U32 PortID,HI_U32 Sns_fps,VO_INTF_SYNC_E OUT_Type)
{
    HI_S32 i;
    HI_S32  s32Ret;

    /* 因(LANE_DIVIDE_MODE_5);必须为0，1，3的设备id号，不能修改 */
    VI_DEV VI_DEV_LIST[MAX_SENSOR_NUM] = {0,1,3};

    /* 因硬件端口必须为下i2c号对应，不能修改 */
    VI_DEV CAM_I2C_BUS_LIST[MAX_SENSOR_NUM] = {1,3,5};

     /* 此表顺序必须和Makefile.param中的顺序一致，不能修改 */
    SAMPLE_SNS_TYPE_E CAM_TYPED_LIST[] =
     {
    
        SENSOR0_TYPE,
        SENSOR1_TYPE,
        SENSOR2_TYPE,
        SENSOR3_TYPE,
        SENSOR4_TYPE,
    };

   //vi——>vpss——>vo——>mipi_dsi
    VI_DEV  ViDev  = VI_DEV_LIST[PortID];//VI的设备号
    //VI_DEV  ViDev  = 0;
    VI_PIPE ViPipe = 0;           //VI的管道号
    VI_CHN  ViChn  = 0;           //VI的通道号 
    SAMPLE_VI_CONFIG_S stViConfig;

    SIZE_S             stSize;
    VB_CONFIG_S        stVbConf;
    PIC_SIZE_E         enPicSize; //图片大小
    HI_U32             u32BlkSize; //缓存块大小，以 Byte 位单位

    ISP_CTRL_PARAM_S stIspCtrlParam;

   // VO_INTF_SYNC_E         enIntfSync     = VO_OUTPUT_USER;

    VPSS_GRP           VpssGrp = 0;
    VPSS_CHN           VpssChn = 0;
    HI_BOOL            abChnEnable[VPSS_MAX_PHY_CHN_NUM] = {0};
    VPSS_CHN_ATTR_S    astVpssChnAttr[VPSS_MAX_PHY_CHN_NUM] = {0};  //VPSS物理通道属性
    VPSS_GRP_ATTR_S    stVpssGrpAttr; //VPSS GROUP 属性

    VO_CHN             VoChn          = 0;
    SAMPLE_VO_CONFIG_S stVoConfig;

/* ----------------------- config vi -----------------------------*/
//获取seneor信息
    for (i = 0; i < VI_MAX_DEV_NUM; i++)  //VI_MAX_DEV_NUM：定义 VI 设备的最大个数
    {
        stViConfig.astViInfo[i].stSnsInfo.s32SnsId = i;
        stViConfig.astViInfo[i].stSnsInfo.s32BusId = i;
        stViConfig.astViInfo[i].stSnsInfo.MipiDev  = i;
        memset_s(&stViConfig.astViInfo[i].stSnapInfo, sizeof(SAMPLE_SNAP_INFO_S), 0, sizeof(SAMPLE_SNAP_INFO_S));
        stViConfig.astViInfo[i].stPipeInfo.bMultiPipe = HI_FALSE;
        stViConfig.astViInfo[i].stPipeInfo.bVcNumCfged = HI_FALSE;
    }

    //指定三个端口所用到的I2C总线号
    stViConfig.astViInfo[0].stSnsInfo.s32BusId  = CAM_I2C_BUS_LIST[PortID];
    //指定sensor的id号就是 = Makefile.param 的SENSORx_TYPE列表
    stViConfig.astViInfo[0].stSnsInfo.s32SnsId  = CamID;
    //指定sensor的名字
    stViConfig.astViInfo[0].stSnsInfo.enSnsType = CAM_TYPED_LIST[CamID];

    stViConfig.s32WorkingViNum                        = 1;
    stViConfig.as32WorkingViId[0]                     = 0;
    stViConfig.astViInfo[0].stDevInfo.ViDev           = ViDev;
    stViConfig.astViInfo[0].stSnsInfo.MipiDev         = ViDev;
    stViConfig.astViInfo[0].stDevInfo.enWDRMode       = WDR_MODE_NONE;
    stViConfig.astViInfo[0].stPipeInfo.enMastPipeMode = VI_ONLINE_VPSS_ONLINE;
    stViConfig.astViInfo[0].stPipeInfo.aPipe[0]       = ViPipe;
    stViConfig.astViInfo[0].stPipeInfo.aPipe[1]       = -1;
    stViConfig.astViInfo[0].stPipeInfo.aPipe[2]       = -1;
    stViConfig.astViInfo[0].stPipeInfo.aPipe[3]       = -1;
    stViConfig.astViInfo[0].stChnInfo.ViChn           = ViChn;
    stViConfig.astViInfo[0].stChnInfo.enPixFormat     = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    stViConfig.astViInfo[0].stChnInfo.enDynamicRange  = DYNAMIC_RANGE_SDR8;
    stViConfig.astViInfo[0].stChnInfo.enVideoFormat   = VIDEO_FORMAT_LINEAR;
    stViConfig.astViInfo[0].stChnInfo.enCompressMode  = COMPRESS_MODE_NONE;

/*-------------------- get picture size ---------------------*/
    s32Ret = SAMPLE_COMM_VI_GetSizeBySensor(stViConfig.astViInfo[0].stSnsInfo.enSnsType, &enPicSize);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("get picture size by sensor failed!\n");
        return s32Ret;
    }

    s32Ret = SAMPLE_COMM_SYS_GetPicSize(enPicSize, &stSize);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("get picture size failed!\n");
        return s32Ret;
    }

/*-------------------- config vb && start sys ---------------------*/
    memset_s(&stVbConf, sizeof(VB_CONFIG_S), 0, sizeof(VB_CONFIG_S));
    stVbConf.u32MaxPoolCnt              = 2; //整个系统中可容纳的缓存池个数

    u32BlkSize = COMMON_GetPicBufferSize(stSize.u32Width, stSize.u32Height, SAMPLE_PIXEL_FORMAT, DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
    stVbConf.astCommPool[0].u64BlkSize  = u32BlkSize;
    stVbConf.astCommPool[0].u32BlkCnt   = 15;

    u32BlkSize = COMMON_GetPicBufferSize(stSize.u32Width, stSize.u32Height, SAMPLE_PIXEL_FORMAT, DATA_BITWIDTH_8, COMPRESS_MODE_NONE, DEFAULT_ALIGN);
    stVbConf.astCommPool[0].u64BlkSize  = u32BlkSize;
    stVbConf.astCommPool[0].u32BlkCnt   = 15;

    s32Ret = SAMPLE_COMM_SYS_Init(&stVbConf);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("system init failed with %d!\n", s32Ret);
        return s32Ret;
    }

/*-------------------- config isp info ---------------------*/
    s32Ret = HI_MPI_ISP_GetCtrlParam(stViConfig.astViInfo[0].stPipeInfo.aPipe[0], &stIspCtrlParam);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("HI_MPI_ISP_GetCtrlParam failed with %d!\n", s32Ret);
        return s32Ret;
    }

    stIspCtrlParam.u32StatIntvl = Sns_fps / 30;

    s32Ret = HI_MPI_ISP_SetCtrlParam(stViConfig.astViInfo[0].stPipeInfo.aPipe[0], &stIspCtrlParam);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("HI_MPI_ISP_SetCtrlParam failed with %d!\n", s32Ret);
        return s32Ret;
    }

/*---------------------- start vi -----------------------*/
    s32Ret = SAMPLE_COMM_VI_StartVi(&stViConfig);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("start vi failed.s32Ret:0x%x !\n", s32Ret);
        goto EXIT;
    }

/*---------------------- config vpss -----------------------*/
    memset_s(&stVpssGrpAttr, sizeof(VPSS_GRP_ATTR_S), 0, sizeof(VPSS_GRP_ATTR_S));
    stVpssGrpAttr.stFrameRate.s32SrcFrameRate    = -1;
    stVpssGrpAttr.stFrameRate.s32DstFrameRate    = -1;
    // stVpssGrpAttr.stFrameRate.s32SrcFrameRate    = 60;
    // stVpssGrpAttr.stFrameRate.s32DstFrameRate    = 60;
    stVpssGrpAttr.enDynamicRange                 = DYNAMIC_RANGE_SDR8;
    stVpssGrpAttr.enPixelFormat                  = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    stVpssGrpAttr.u32MaxW                        = stSize.u32Width;
    stVpssGrpAttr.u32MaxH                        = stSize.u32Height;
    // stVpssGrpAttr.u32MaxW                        = 400;
    // stVpssGrpAttr.u32MaxH                        = 400;
    stVpssGrpAttr.bNrEn                          = HI_TRUE;
    stVpssGrpAttr.stNrAttr.enCompressMode        = COMPRESS_MODE_FRAME;
    stVpssGrpAttr.stNrAttr.enNrMotionMode        = NR_MOTION_MODE_NORMAL;

    astVpssChnAttr[VpssChn].u32Width                    = stSize.u32Width;
    astVpssChnAttr[VpssChn].u32Height                   = stSize.u32Height;
    // astVpssChnAttr[VpssChn].u32Width                    = 400;
    // astVpssChnAttr[VpssChn].u32Height                   = 400;
    astVpssChnAttr[VpssChn].enChnMode                   = VPSS_CHN_MODE_USER;
    astVpssChnAttr[VpssChn].enCompressMode              = COMPRESS_MODE_NONE;
    astVpssChnAttr[VpssChn].enDynamicRange              = DYNAMIC_RANGE_SDR8;
    astVpssChnAttr[VpssChn].enVideoFormat               = VIDEO_FORMAT_LINEAR;
    astVpssChnAttr[VpssChn].enPixelFormat               = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    astVpssChnAttr[VpssChn].stFrameRate.s32SrcFrameRate = Sns_fps;
    astVpssChnAttr[VpssChn].stFrameRate.s32DstFrameRate = Sns_fps;
    astVpssChnAttr[VpssChn].u32Depth                    = 5;
    astVpssChnAttr[VpssChn].bMirror                     = HI_FALSE;
    astVpssChnAttr[VpssChn].bFlip                       = HI_FALSE;
    astVpssChnAttr[VpssChn].stAspectRatio.enMode        = ASPECT_RATIO_NONE;

/*---------------------- start vpss -----------------------*/
    abChnEnable[0] = HI_TRUE;
    s32Ret = SAMPLE_COMM_VPSS_Start(VpssGrp, abChnEnable, &stVpssGrpAttr, astVpssChnAttr);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("start vpss group failed. s32Ret: 0x%x !\n", s32Ret);
        goto EXIT1;
    }
 /*测试*/
    
    s32Ret = SAMPLE_COMM_VI_Bind_VPSS(ViPipe, ViChn, VpssGrp);
   if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("VI Bind VPSS err for %#x!\n", s32Ret);
        goto EXIT2;
    }

/* ----------------------------------------------------- config vo -----------------------------------------------------  */
    SAMPLE_COMM_VO_GetDefConfig(&stVoConfig);
    stVoConfig.enDstDynamicRange = DYNAMIC_RANGE_SDR8;
    stVoConfig.enVoIntfType = VO_INTF_MIPI;

    stVoConfig.enPicSize = enPicSize;
    stVoConfig.enIntfSync = OUT_Type;//VO接口时序类型

/* ----------------------------------------------------- start vo -----------------------------------------------------  */
    s32Ret = SAMPLE_COMM_VO_StartVO(&stVoConfig);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("start vo failed. s32Ret: 0x%x !\n", s32Ret);
        goto EXIT2;
    }
    s32Ret=SAMPLE_COMM_VPSS_GETCHNFRAME_START(0,0);
  /* s32Ret = SAMPLE_COMM_VPSS_Bind_VO(VpssGrp, VpssChn, stVoConfig.VoDev, VoChn);
    if (HI_SUCCESS != s32Ret)
    {  
        goto EXIT3;
    }*/
    
    usleep(800000);
    
    LT8911EXB_config();
    printf("\r\n");
    PAUSE();
    
    //SAMPLE_COMM_VPSS_UnBind_VO(VpssGrp, VpssChn, stVoConfig.VoDev, VoChn);

EXIT3:
    SAMPLE_COMM_VO_StopVO(&stVoConfig);
EXIT2:
    SAMPLE_COMM_VI_UnBind_VPSS(ViPipe, ViChn, VpssGrp);
    SAMPLE_COMM_VPSS_Stop(VpssGrp, abChnEnable);
EXIT1:
    SAMPLE_COMM_VI_StopVi(&stViConfig);
EXIT:
    SAMPLE_COMM_SYS_Exit();
    return s32Ret;
}
