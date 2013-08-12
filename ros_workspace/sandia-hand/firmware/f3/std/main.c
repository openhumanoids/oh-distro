#include <sys/stat.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"
#include "lowlevel.h"
#include "pins.h"
#include "tactile.h"
#include "comms.h"
#include "imu.h"

/*
#include "params.h"
#include "io.h"
#include "imu.h"
#include "console.h"
#include "lowlevel.h"
#include "adc.h"
*/

void systick_irq()
{
  comms_systick();
  imu_systick();
  tactile_systick();
}

int main(void)
{
  WDT_Disable(WDT);
  PMC_EnablePeripheral(ID_PIOA);
  PMC_EnablePeripheral(ID_PIOB);
  PMC_EnablePeripheral(ID_PIOC);
  PIO_Configure(&pin_led, 1);
  lowlevel_init_clocks();
  tactile_init();
  comms_init();
  imu_init();
  SysTick_Config(F_CPU / 1000); // 1 ms tick clock
  NVIC_SetPriority(SysTick_IRQn, 8); 
  __enable_irq();
  while (1)
  {
    comms_idle();
    imu_idle();
    tactile_idle();
  }
}


///////////////////////////////////////////////////////////////////////////
// libc stubs... we don't have an OS, so just put crap here.
//////////////////////////////////////////////////////////////////////////

extern int _end;
extern caddr_t _sbrk(int incr)
{
  static unsigned char *heap = NULL ;
  unsigned char *prev_heap ;
  if ( heap == NULL )
    heap = (unsigned char *)&_end ;
  prev_heap = heap;
  heap += incr ;
  return (caddr_t) prev_heap ;
}
extern int _kill(int pid, int sig) { return -1; }
extern void _exit(int status) { }
int _getpid() { return 1; }
extern int _write(int fd, const void *buf, size_t count)
{
#if 0
  io_led(true);
  if (g_console_init_complete)
    console_send_block((uint8_t *)buf, count);
  io_led(false);
#endif
  return count;
}
int _close(int fd) { return -1; }
int _fstat(int fd, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}
int _isatty(int fd) { return 1; }
off_t _lseek(int fd, off_t offset, int whence) { return 0; }
ssize_t _read(int fd, void *buf, size_t count) { return 0; }

struct __FILE { int handle; };
FILE __stdout;
FILE __stderr;
int fputc(int ch, FILE *f)
{
  //console_send_string("fputc\r\n");
  return 0;
}
void _ttywrch(int ch)
{
  //console_send_string("ttywrch\r\n");
}

