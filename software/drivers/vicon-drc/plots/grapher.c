#include <stdio.h>
#include <lcm/lcm.h>
#include <inttypes.h>
#include "lcmtypes/vicon_drc.h"
#include <netinet/in.h>
#include <sys/socket.h>

  struct sockaddr_in serverAddr_;  // Server's socket address
  int sockAddrSize_;  // Size of socket address structure
  int sFd_;  // Socket file descriptor

/**
  * Accepts a viconstructs_vicon_t* struct and prints out its values
  */
static void handler(const lcm_recv_buf_t *rbuf, const char* channel, const viconstructs_vicon_t* vicon, void* user)
{
    viconstructs_segment_t* segment = vicon->models->segments+1;
  char myRequest[80];
  sprintf(myRequest, "{\"S\":%f, \"V\":%f, \"C\":%f}\0", (float) segment->ba[0], (float) segment->ba[1], (float) segment->ba[2]);
  //sprintf(myRequest, "{\"S\":%f, \"V\":%f, \"C\":%f}\0", (float) segment->A.x, (float) segment->A.y, (float) segment->A.z);
  printf("%s\n",myRequest);
  if (sendto(sFd_, (caddr_t) myRequest, strlen(myRequest), 0,
     (struct sockaddr *) &serverAddr_, sockAddrSize_) == 0) {
    close (sFd_);
    printf("error\n");
  }
}

/**
  * Debugger program to explore broadcast data more easily than bot-spy
  * (aka I didn't know about bot-spy when I wrote this)
  */
int main(int argc, char** argv)
{
  printf("woohoo\n");
  /* build server socket address */
  sFd_ = socket(AF_INET, SOCK_DGRAM, 0);
  sockAddrSize_ = sizeof (struct sockaddr_in);
  bzero ((char *) &serverAddr_, sockAddrSize_);
  //serverAddr_.sin_len = (u_char) sockAddrSize_;
  serverAddr_.sin_family = AF_INET;
  serverAddr_.sin_port = htons (41234);

  if (((serverAddr_.sin_addr.s_addr = inet_addr ("127.0.0.1")) == 0)) {
    close (sFd_);
  }

    lcm_t* lcm = lcm_create(NULL);
    if (!lcm)
        return 1;

    printf("Listening on channel \"drc_vicon\"\n");
    viconstructs_vicon_t_subscribe(lcm,"drc_vicon",handler,NULL);

  printf("starting\n");
    for(;;)
        lcm_handle(lcm);

    lcm_destroy(lcm);
    return 0;
}
