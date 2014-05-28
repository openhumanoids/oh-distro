#include <lcm/lcm.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// input args: srcframe, dstframe , src coordinate
// output args: dst coordinate
int main()
{
    
   lcm_t *lcm;
   BotParam * param;
   BotFrames * frames;
   
   lcm = bot_lcm_get_global (NULL);
   if(!lcm)
     printf("Error getting LCM\n");
   param = bot_param_new_from_server (lcm, 0);
   if(!param)
     printf("Error getting param\n");
   frames = bot_frames_new (lcm, param);
   if(!frames)
     printf("Error getting frames\n");
   /*
   char* src = mxArrayToString(prhs[0]);
   char* dst = mxArrayToString(prhs[1]);
   double* xyz_src = mxGetPr(prhs[3]);
   
   
   //mxCreateNumericMatrix(m, n, classid,   ComplexFlag)
   plhs[0] = mxCreateNumericMatrix(3, 1, mxDOUBLE_CLASS, mxREAL); 
   double* xyz_dst = (double*)mxGetData(plhs[0]);
   bot_frames_transform_vec (frames, src, dst, xyz_src, xyz_dst);

   mxFree(src);
   mxFree(dst);*/
   usleep(1000000);
   lcm_handle(lcm);
   lcm_handle(lcm);
   lcm_handle(lcm);
   double xyz_dst[3];
   double xyz_src[3] = {0,0,0};
   int t = bot_frames_transform_vec (frames, "LHAND_FACE", "local", xyz_src, xyz_dst);
   if(!t) {printf("fail\n");}

   printf("%f %f %f %f %f %f\n", xyz_src[0], xyz_src[1], xyz_src[2], xyz_dst[0], xyz_dst[1], xyz_dst[2]);
   
   double mat[16];
   t = bot_frames_get_trans_mat_4x4(frames, "body", "local", mat);
   if(!t) {printf("fail\n");}
   for(int i=0;i<16;i++)
       printf("%f ", mat[i]);
   
   int n = bot_frames_get_num_frames(frames);
   printf("%d ", n);
   
   printf("\n");
   char **a = bot_frames_get_frame_names(frames);
   for(int i=0;i<n;i++){
     printf("%s\n", a[i]);
   }
   
   
   bot_frames_destroy(frames);
   bot_param_destroy(param);
   lcm_destroy(lcm);
}


