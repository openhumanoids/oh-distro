// file: send_message.c
//
// LCM example program.
//
// compile with:
//  $ gcc -o send_message send_message.c -llcm
//
// On a system with pkg-config, you can also use:
//  $ gcc -o send_message send_message.c `pkg-config --cflags --libs lcm`

#include <stdio.h>
#include <lcm/lcm.h>

#include "exlcm_example_t.h"

int main(int argc, char ** argv) {
  int numToSend=1;
  int numelements=15;
  int j;
  if(argc>1){
    numToSend=atoi(argv[1]);
  }
  if(argc>2){
    numelements=atoi(argv[2]);
  }
  
  lcm_t * lcm = lcm_create(NULL);
  if(!lcm)
    return 1;
  
  
  int16_t* ranges = (int16_t*)malloc(numelements*sizeof(int16_t));
  int i;
  
  
  for(j=0;j<numToSend;j++){
    exlcm_example_t my_data = {
      .timestamp = j,
      .position = { 1, 2, 3 },
      .orientation = { 1, 0, 0, 0 },
    };
    
    for(i = 0; i < numelements; i++)
      ranges[i] = rand()%15;
    my_data.num_ranges = numelements;
    my_data.ranges = ranges;
    my_data.name = "example string";
    my_data.enabled = 1;
    
    
    exlcm_example_t_publish(lcm, "EXAMPLE", &my_data);
    printf("Sent: %d\n",j);
    usleep(1000000);
  }
  
  free(ranges);
  lcm_destroy(lcm);
  return 0;
}
