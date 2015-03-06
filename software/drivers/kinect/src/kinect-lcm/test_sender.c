// file: test_sender.c
//
// Test parameters

#include <stdio.h>
#include <lcm/lcm.h>
#include <unistd.h>
#include "timestamp.h"

#include <lcmtypes/kinect_cmd_msg_t.h>


static void
send_message(lcm_t * lcm, int8_t cmd_type, int cmd_property)
{
  int64_t host_utime = timestamp_now();

  kinect_cmd_msg_t k_cmd = {
    .timestamp = host_utime,
    .command_type = cmd_type,
    .tilt_degree = 0,
    .led_status = 0,
    .depth_data_format = 0,
    .image_data_format = 0,
  };

  switch(cmd_type)
  {
    case KINECT_CMD_MSG_T_SET_TILT:
      k_cmd.tilt_degree = cmd_property;
      break;
    case KINECT_CMD_MSG_T_SET_LED:
      k_cmd.led_status = cmd_property;
      break;
    case KINECT_CMD_MSG_T_SET_IMAGE_DATA_FORMAT:
      k_cmd.image_data_format = cmd_property;
      break;
    case KINECT_CMD_MSG_T_SET_DEPTH_DATA_FORMAT:
      k_cmd.depth_data_format = cmd_property;
      break;
    default:
      fprintf(stderr,"Unknown Command\n");
      break;
  }

  kinect_cmd_msg_t_publish(lcm, "KINECT_CMD", &k_cmd);
}

int
main(int argc, char ** argv)
{


  int c;
  int tilt_angle = 0;
  int led_state = 0;//KINECT_CMD_MSG_T_LED_IGNORE;//100;//kinect_cmd_msg_t.LED_IGNORE;  
  int img_format = 0;
  int depth_format = 0;

  int do_tilt = 0;
  int do_led = 0;
  int do_img = 0;
  int do_depth = 0;

  while ((c = getopt (argc, argv, "ht:l:i:d:")) >= 0) {
    switch (c) {
      case 't':
        tilt_angle = atoi(optarg);
        fprintf(stdout,"Angle : %d\n", tilt_angle);
        do_tilt =1;
        break;
      case 'l':
        led_state = atoi(optarg);
        fprintf(stdout,"LED state : %d\n", led_state);
        do_led = 1;
        break;
      case 'i':
        img_format = atoi(optarg);
        fprintf(stdout,"image format : %d\n", img_format);
        do_img = 1;
        break;
      case 'd':
        depth_format = atoi(optarg);
        fprintf(stdout,"depth format : %d\n", depth_format);
        do_depth = 1;
        break;
      case 'h':
      case '?':
        fprintf (stderr, "Usage: %s [-v] [-c] [-d] [-p]\n\
            Options:\n\
            -t     Tilt Angle\n	\
            -l     LED state\n\
            -h     This help message\n", argv[0]);
        return 1;
    }
  }

  lcm_t * lcm;

  lcm = lcm_create(NULL);
  if(!lcm)
    return 1;

  if(do_tilt){
    send_message(lcm,KINECT_CMD_MSG_T_SET_TILT,tilt_angle);
  }
  if(do_led){
    send_message(lcm,KINECT_CMD_MSG_T_SET_LED,led_state);
  }
  if(do_img){
    send_message(lcm,KINECT_CMD_MSG_T_SET_IMAGE_DATA_FORMAT,img_format);
  }
  if(do_depth){
    send_message(lcm,KINECT_CMD_MSG_T_SET_DEPTH_DATA_FORMAT,depth_format);
  }

  lcm_destroy(lcm);
  return 0;
}
