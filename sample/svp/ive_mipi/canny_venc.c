#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <semaphore.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <math.h>

#include "sample_comm_ive.h"

typedef struct hiSAMPLE_IVE_CANNY_INFO_S
{
    IVE_SRC_IMAGE_S stSrc;
    IVE_DST_IMAGE_S stEdge;
    IVE_DST_IMAGE_S stMag;
    IVE_MEM_INFO_S  stStack;
    IVE_CANNY_HYS_EDGE_CTRL_S  stCannyHysEdgeCtrl;
    IVE_MAG_AND_ANG_CTRL_S stMagAndAngCtrl;
    IVE_THRESH_U16_CTRL_S stThrU16Ctrl;
    // FILE* pFpSrc;
    // FILE* pFpDst;
    HI_BOOL bEncode;
    HI_BOOL bVo;
} SAMPLE_IVE_CANNY_INFO_S;

static HI_BOOL s_bStopSignal = HI_FALSE;
static SAMPLE_IVE_CANNY_INFO_S s_stCannyInfo;
static pthread_t s_hIveThread = 0;
static SAMPLE_IVE_SWITCH_S s_stCannySwitch = {HI_FALSE,HI_FALSE};
static SAMPLE_VI_CONFIG_S s_stViConfig = {0};

/******************************************************************************
* function : Canny uninit
******************************************************************************/
static HI_VOID SAMPLE_IVE_Canny_Uninit(SAMPLE_IVE_CANNY_INFO_S* pstCannyInfo)
{
    IVE_MMZ_FREE(pstCannyInfo->stSrc.au64PhyAddr[0], pstCannyInfo->stSrc.au64VirAddr[0]);
    IVE_MMZ_FREE(pstCannyInfo->stEdge.au64PhyAddr[0], pstCannyInfo->stEdge.au64VirAddr[0]);
    IVE_MMZ_FREE(pstCannyInfo->stMag.au64PhyAddr[0], pstCannyInfo->stMag.au64VirAddr[0]);
    IVE_MMZ_FREE(pstCannyInfo->stStack.u64PhyAddr, pstCannyInfo->stStack.u64VirAddr);
    IVE_MMZ_FREE(pstCannyInfo->stCannyHysEdgeCtrl.stMem.u64PhyAddr, \
                 pstCannyInfo->stCannyHysEdgeCtrl.stMem.u64VirAddr);

    // IVE_CLOSE_FILE(pstCannyInfo->pFpSrc);
    // IVE_CLOSE_FILE(pstCannyInfo->pFpDst);
}
/******************************************************************************
* function : Canny init
******************************************************************************/
static HI_S32 SAMPLE_IVE_Canny_Init(SAMPLE_IVE_CANNY_INFO_S* pstCannyInfo,
                                    HI_U32 u32Width, HI_U32 u32Height)
{
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U32 u32Size = 0;
    
    HI_S8 as8Mask[25] = {0, 0, 0, 0, 0,
                         0, -1, 0, 1, 0,
                         0, -2, 0, 2, 0,
                         0, -1, 0, 1, 0,
                         0, 0, 0, 0, 0
                        };

    memset(pstCannyInfo, 0, sizeof(SAMPLE_IVE_CANNY_INFO_S));
    memcpy(pstCannyInfo->stCannyHysEdgeCtrl.as8Mask, as8Mask, 25);
    memcpy(pstCannyInfo->stMagAndAngCtrl.as8Mask, as8Mask, 25);
    pstCannyInfo->stCannyHysEdgeCtrl.u16HighThr = 150;
    pstCannyInfo->stCannyHysEdgeCtrl.u16LowThr = 50;
    pstCannyInfo->stMagAndAngCtrl.enOutCtrl = IVE_MAG_AND_ANG_OUT_CTRL_MAG;
    pstCannyInfo->stMagAndAngCtrl.u16Thr = 0;
    pstCannyInfo->stThrU16Ctrl.enMode = IVE_THRESH_U16_MODE_U16_TO_U8_MIN_MID_MAX;
    pstCannyInfo->stThrU16Ctrl.u16HighThr = 100;
    pstCannyInfo->stThrU16Ctrl.u16LowThr = 100;
    pstCannyInfo->stThrU16Ctrl.u8MaxVal = 255;
    pstCannyInfo->stThrU16Ctrl.u8MidVal = 0;
    pstCannyInfo->stThrU16Ctrl.u8MinVal = 0;

    s32Ret = SAMPLE_COMM_IVE_CreateImage(&pstCannyInfo->stSrc, IVE_IMAGE_TYPE_U8C1, u32Width, u32Height);
    SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, CANNY_INIT_FAIL,
        "Error(%#x),Create Src Image failed!\n", s32Ret);

    s32Ret = SAMPLE_COMM_IVE_CreateImage(&pstCannyInfo->stEdge, IVE_IMAGE_TYPE_U8C1, u32Width, u32Height);
    SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, CANNY_INIT_FAIL,
        "Error(%#x),Create edge Image failed!\n", s32Ret);

    s32Ret = SAMPLE_COMM_IVE_CreateImage(&pstCannyInfo->stMag, IVE_IMAGE_TYPE_U16C1, u32Width, u32Height);
    SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, CANNY_INIT_FAIL,
        "Error(%#x),Create Mag Image failed!\n", s32Ret);

    u32Size = pstCannyInfo->stSrc.au32Stride [0] * pstCannyInfo->stSrc.u32Height * 4 + sizeof(IVE_CANNY_STACK_SIZE_S);
    s32Ret = SAMPLE_COMM_IVE_CreateMemInfo(&pstCannyInfo->stStack, u32Size);
    SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, CANNY_INIT_FAIL,
        "Error(%#x),Create Stack Mem info failed!\n", s32Ret);

    u32Size = pstCannyInfo->stSrc.au32Stride [0] * pstCannyInfo->stSrc.u32Height * 3;
    s32Ret = SAMPLE_COMM_IVE_CreateMemInfo(&pstCannyInfo->stCannyHysEdgeCtrl.stMem, u32Size);
    SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, CANNY_INIT_FAIL,
        "Error(%#x),Create CannyHysEdgeCtrl.stMem Mem info failed!\n", s32Ret);

    
    // pstCannyInfo->pFpSrc = fopen(pchSrcFileName, "rb");
    // SAMPLE_CHECK_EXPR_GOTO(HI_NULL == pstCannyInfo->pFpSrc, CANNY_INIT_FAIL,
    //     "Error,Open file %s failed!\n", pchSrcFileName);

    // pstCannyInfo->pFpDst = fopen(pchDstFileName, "wb");
    // SAMPLE_CHECK_EXPR_GOTO(HI_NULL == pstCannyInfo->pFpDst, CANNY_INIT_FAIL,
    //     "Error,Open file %s failed!\n", pchDstFileName);

    // s32Ret = HI_SUCCESS;

CANNY_INIT_FAIL:

    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_IVE_Canny_Uninit(pstCannyInfo);
    }
    return s32Ret;
}
/******************************************************************************
* function : show complate canny sample
******************************************************************************/
static HI_S32 SAMPLE_IVE_Complate_Canny(SAMPLE_IVE_CANNY_INFO_S* pstCannyInfo, VIDEO_FRAME_INFO_S* pstExtFrmInfo)
{
    HI_S32 s32Ret = HI_SUCCESS;
    HI_BOOL bInstant = HI_TRUE;
    HI_BOOL bBlock = HI_TRUE;
    HI_BOOL bFinish = HI_FALSE;
    IVE_HANDLE IveHandle;
   


    // s32Ret = SAMPLE_COMM_IVE_ReadFile(&(pstCannyInfo->stSrc), pstCannyInfo->pFpSrc);
    // SAMPLE_CHECK_EXPR_RET(HI_SUCCESS != s32Ret,s32Ret,"Error(%#x),Read src file failed!\n",s32Ret);
    bInstant = HI_FALSE;
    s32Ret = SAMPLE_COMM_IVE_DmaImage(pstExtFrmInfo,&pstCannyInfo->stSrc,bInstant);
    SAMPLE_CHECK_EXPR_RET(HI_SUCCESS != s32Ret,s32Ret,"Error(%#x),SAMPLE_COMM_IVE_DmaImage failed!\n",s32Ret);

    s32Ret = HI_MPI_IVE_CannyHysEdge(&IveHandle, &pstCannyInfo->stSrc, \
                                     &pstCannyInfo->stEdge, &pstCannyInfo->stStack, \
                                     &pstCannyInfo->stCannyHysEdgeCtrl, bInstant);
    SAMPLE_CHECK_EXPR_RET(HI_SUCCESS != s32Ret,s32Ret,"Error(%#x),HI_MPI_IVE_CannyHysEdge failed!\n",s32Ret);


    s32Ret = HI_MPI_IVE_Query(IveHandle, &bFinish, bBlock);
    while (HI_ERR_IVE_QUERY_TIMEOUT == s32Ret)
    {
        usleep(100);
        s32Ret = HI_MPI_IVE_Query(IveHandle, &bFinish, bBlock);
    }
    SAMPLE_CHECK_EXPR_RET(HI_SUCCESS != s32Ret,s32Ret,"Error(%#x),HI_MPI_IVE_Query failed!\n",s32Ret);

    s32Ret = HI_MPI_IVE_CannyEdge(&pstCannyInfo->stEdge, &pstCannyInfo->stStack);
    SAMPLE_CHECK_EXPR_RET(HI_SUCCESS != s32Ret,s32Ret,"Error(%#x),HI_MPI_IVE_CannyEdge failed!\n",s32Ret);

    // memcpy(pstExtFrmInfo->stVFrame.u64PhyAddr[0],pstCannyInfo->stEdge.au64VirAddr[0],pstExtFrmInfo->stVFrame.u32Height*pstExtFrmInfo->stVFrame.u32Stride[0]);
    

  
    return s32Ret;
}
static HI_VOID* SAMPLE_IVE_ViToVo(HI_VOID* pArgs)
{
    HI_S32 s32Ret;
    SAMPLE_IVE_CANNY_INFO_S *pstCannyInfo;
    VIDEO_FRAME_INFO_S stBaseFrmInfo;
    VIDEO_FRAME_INFO_S stExtFrmInfo;
    HI_S32 s32MilliSec = 20000;
    VO_LAYER voLayer = 0;
    VO_CHN voChn = 0;
    VENC_CHN vencChn = 0;
    HI_BOOL bEncode;
    HI_BOOL bVo;
    HI_S32 s32VpssGrp = 0;
    HI_S32 as32VpssChn[] = {VPSS_CHN0, VPSS_CHN1};

    HI_VOID * pVirAddr;
    HI_VOID * pVirAddr2;
    HI_U32 u32height;
    HI_U32 u32width;
    HI_U32 u32stride;
    HI_U32 i;
    HI_U32 j;
  

    pstCannyInfo = (SAMPLE_IVE_CANNY_INFO_S*)pArgs;
    VIDEO_FRAME_INFO_S* pstBaseFrmInfo;
    bEncode = pstCannyInfo->bEncode;
    bVo = pstCannyInfo->bVo;

    while (HI_FALSE == s_bStopSignal)
    {
        s32Ret = HI_MPI_VPSS_GetChnFrame(s32VpssGrp, as32VpssChn[1], &stExtFrmInfo, s32MilliSec);
        if(HI_SUCCESS != s32Ret)
        {
            SAMPLE_PRT("Error(%#x),HI_MPI_VPSS_GetChnFrame failed, VPSS_GRP(%d), VPSS_CHN(%d)!\n",
                s32Ret,s32VpssGrp, as32VpssChn[1]);
            continue;
        }

        s32Ret = HI_MPI_VPSS_GetChnFrame(s32VpssGrp, as32VpssChn[0], &stBaseFrmInfo, s32MilliSec);
        SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS!=s32Ret, EXT_RELEASE,
            "Error(%#x),HI_MPI_VPSS_GetChnFrame failed, VPSS_GRP(%d), VPSS_CHN(%d)!\n",
            s32Ret,s32VpssGrp, as32VpssChn[0]);

        s32Ret = SAMPLE_IVE_Complate_Canny(pstCannyInfo, &stExtFrmInfo);
        SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS!=s32Ret, BASE_RELEASE,
            "Error(%#x),SAMPLE_IVE_CannyProc failed!\n", s32Ret);
    
    pstBaseFrmInfo=&stBaseFrmInfo;
    u32height = pstBaseFrmInfo->stVFrame.u32Height;
    u32width  = pstBaseFrmInfo->stVFrame.u32Width;
    u32stride = pstBaseFrmInfo->stVFrame.u32Stride[0];
    
    pVirAddr=(HI_U8*)HI_MPI_SYS_Mmap(pstBaseFrmInfo->stVFrame.u64PhyAddr[0],u32height*u32stride*3/2);

     HI_U8* YUVdata=pVirAddr;
     HI_U8* YUVdata2=(HI_U8*)(HI_UL)pstCannyInfo->stEdge.au64VirAddr[0];
    printf("yuvdate.height=%d\n",u32height); 
    printf("yuvdate.width=%d\n",u32width);
    printf("yuvdate2.height=%d\n",pstCannyInfo->stEdge.u32Height); 
    printf("yuvdate2.width=%d\n",pstCannyInfo->stEdge.u32Width);
    printf("yuvdate.stride=%d\nyuvdata2.stride=%d\n",u32stride,pstCannyInfo->stEdge.au32Stride[0]);

    for(i=0;i<u32height;i++)
    {
       memcpy(YUVdata,YUVdata2,u32stride);
       YUVdata+=u32stride;
       YUVdata2+=pstCannyInfo->stEdge.au32Stride[0];
    }
    // for(i=u32height*u32stride;i<u32height*u32stride*3/2;i++)
    // {
    //     YUVdata[i]=127;
    // }
    HI_MPI_SYS_Munmap((HI_VOID*)pVirAddr,u32height*u32stride*3/2);

        // //Draw rect
        // s32Ret = SAMPLE_COMM_VGS_FillRect(&stBaseFrmInfo, &pstGmm->stRegion, 0x0000FF00);
		// SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS!=s32Ret, BASE_RELEASE,
		// 	"SAMPLE_COMM_VGS_FillRect failed, Error(%#x)!\n", s32Ret);

        //Venc
        if (HI_TRUE == bEncode)
        {
            s32Ret = HI_MPI_VENC_SendFrame(vencChn, &stBaseFrmInfo, s32MilliSec);
			SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS!=s32Ret, BASE_RELEASE,
				"HI_MPI_VENC_SendFrame failed, Error(%#x)!\n", s32Ret);
        }

        //Vo
        if (HI_TRUE == bVo)
        {
            s32Ret = HI_MPI_VO_SendFrame(voLayer, voChn, &stBaseFrmInfo, s32MilliSec);
			SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS!=s32Ret, BASE_RELEASE,
				"HI_MPI_VO_SendFrame failed, Error(%#x)!\n", s32Ret);
        }

        BASE_RELEASE:
            s32Ret = HI_MPI_VPSS_ReleaseChnFrame(s32VpssGrp,as32VpssChn[0], &stBaseFrmInfo);
            if (HI_SUCCESS != s32Ret)
            {
                SAMPLE_PRT("Error(%#x),HI_MPI_VPSS_ReleaseChnFrame failed,Grp(%d) chn(%d)!\n",
                    s32Ret,s32VpssGrp,as32VpssChn[0]);
            }

        EXT_RELEASE:
            s32Ret = HI_MPI_VPSS_ReleaseChnFrame(s32VpssGrp,as32VpssChn[1], &stExtFrmInfo);
            if (HI_SUCCESS != s32Ret)
            {
                SAMPLE_PRT("Error(%#x),HI_MPI_VPSS_ReleaseChnFrame failed,Grp(%d) chn(%d)!\n",
                    s32Ret,s32VpssGrp,as32VpssChn[1]);
            }

    }

    return HI_NULL;
}

HI_VOID SAMPLE_IVE_Canny(HI_CHAR chEncode, HI_CHAR chVo)
{
    // HI_U16 u32Width = 720;
    // HI_U16 u32Height = 576;
    // HI_CHAR* pchSrcFileName = "./data/input/canny/canny.yuv";
    // HI_CHAR achDstFileName[IVE_FILE_NAME_LEN];
    SIZE_S stSize;
    PIC_SIZE_E enSize = PIC_400x400;
    HI_BOOL bEncode = '1' ==chEncode ? HI_TRUE : HI_FALSE;
    HI_BOOL bVo = '1' == chVo ? HI_TRUE : HI_FALSE;
    HI_S32 s32Ret;
    HI_CHAR acThreadName[16] = {0};

    memset(&s_stCannyInfo,0,sizeof(s_stCannyInfo));
    SAMPLE_COMM_IVE_CheckIveMpiInit();

    /******************************************
    step 1: start vi vpss venc vo
    ******************************************/
    s_stCannySwitch.bVenc = bEncode;
    s_stCannySwitch.bVo   = bVo;
    s32Ret = SAMPLE_COMM_IVE_StartViVpssVencVo(&s_stViConfig,&s_stCannySwitch,&enSize);
    SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, CANNY_FAIL,
        "Error(%#x),SAMPLE_COMM_IVE_StartViVpssVencVo failed!\n", s32Ret);

    s32Ret = SAMPLE_COMM_SYS_GetPicSize(enSize, &stSize);
    SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, CANNY_FAIL,
        "Error(%#x),SAMPLE_COMM_SYS_GetPicSize failed!\n", s32Ret);

    printf("enSizeheight=%d\n",stSize.u32Height); 
    printf("enSizewidth=%d\n",stSize.u32Width);

    // snprintf(achDstFileName, sizeof(achDstFileName), "./data/output/canny/cannyout_complete_%c.yuv", chComplete);
    s32Ret = SAMPLE_IVE_Canny_Init(&s_stCannyInfo, stSize.u32Width, stSize.u32Height);
    SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, CANNY_FAIL,
        "Error(%#x),SAMPLE_IVE_Canny_Init failed!\n", s32Ret);


    s_stCannyInfo.bEncode = bEncode;
    s_stCannyInfo.bVo = bVo;
    
    snprintf(acThreadName, 16, "IVE_ViToVo");
    prctl(PR_SET_NAME, (unsigned long)acThreadName, 0,0,0);
    pthread_create(&s_hIveThread, 0, SAMPLE_IVE_ViToVo, (HI_VOID*)&s_stCannyInfo);
    

    SAMPLE_PAUSE();

    s_bStopSignal = HI_TRUE;
    pthread_join(s_hIveThread, HI_NULL);
    s_hIveThread = 0;
    SAMPLE_IVE_Canny_Uninit(&s_stCannyInfo);
    memset(&s_stCannyInfo,0,sizeof(s_stCannyInfo));

CANNY_FAIL:
    SAMPLE_COMM_IVE_StopViVpssVencVo(&s_stViConfig,&s_stCannySwitch);
    SAMPLE_COMM_IVE_IveMpiExit();
    return ;
}
/******************************************************************************
* function :Canny sample signal handle
******************************************************************************/
HI_VOID SAMPLE_IVE_Canny_HandleSig(HI_VOID)
{
    if (0 != s_hIveThread)
    {
        pthread_join(s_hIveThread, HI_NULL);
        s_hIveThread = 0;
    }
    SAMPLE_IVE_Canny_Uninit(&s_stCannyInfo);
    memset(&s_stCannyInfo,0,sizeof(s_stCannyInfo));
    SAMPLE_COMM_IVE_IveMpiExit();
    SAMPLE_COMM_IVE_StopViVpssVencVo(&s_stViConfig,&s_stCannySwitch);
}
