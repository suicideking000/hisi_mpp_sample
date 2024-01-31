#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "sample_comm_ive.h"
#include "sample_ive_main.h"

static char **s_ppChCmdArgv = NULL;

/******************************************************************************
* function : to process abnormal case
******************************************************************************/
#ifndef __HuaweiLite__
HI_VOID SAMPLE_IVE_HandleSig(HI_S32 s32Signo)
{
    signal(SIGINT,SIG_IGN);
    signal(SIGTERM,SIG_IGN);

    if (SIGINT == s32Signo || SIGTERM == s32Signo)
    {
        
                   SAMPLE_IVE_Canny_HandleSig();
        printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
    }
    exit(-1);
}
#endif


/******************************************************************************
* function : ive sample
******************************************************************************/
#ifdef __HuaweiLite__
int app_main(int argc, char *argv[])
#else
int main(int argc, char *argv[])
#endif
{
    s_ppChCmdArgv = argv;
#ifndef __HuaweiLite__
    signal(SIGINT, SAMPLE_IVE_HandleSig);
    signal(SIGTERM, SAMPLE_IVE_HandleSig);
#endif

    
                if ((argc < 4) || (('0' != *argv[2]) && ('1' != *argv[2])) || (('0' != *argv[3]) && ('1' != *argv[3])))
                {
                
                    return HI_FAILURE;
                }
                SAMPLE_IVE_Canny(*argv[2], *argv[3]);

    return 0;

}



