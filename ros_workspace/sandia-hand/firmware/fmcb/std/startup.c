// originally snarfed from the Atmel sam3s distro, then hacked up a lot
#include "stdint.h"
#include "sam3s/sam3s.h"
#include "sam3s/core_cm3.h"
#include "halls.h"
#include "pb.h"
#include "i2c_sensors.h"
#include "adc.h"

#define STACK_SIZE       0x2000     /** Stack size (in DWords) */
__attribute__ ((aligned(8),section(".stack")))
uint32_t pdwStack[STACK_SIZE] ;     

extern uint32_t _sfixed, _efixed, _etext;
extern uint32_t _srelocate, _erelocate, _szero, _ezero;

extern int main( void ) ;
extern void rs485_irq(void);
extern void systick_irq(void);

void reset_vector( void ) ;
extern void __libc_init_array( void ) ;

void unmapped_error(void)
{
  while (1) { }
}

typedef void (*IntFunc)(void);

__attribute__((section(".vectors")))
IntFunc exception_table[] = {
    (IntFunc)(&pdwStack[STACK_SIZE-1]),
    reset_vector,
    unmapped_error, //NMI_Handler,
    unmapped_error, //HardFault_Handler,
    unmapped_error, //MemManage_Handler,
    unmapped_error, //BusFault_Handler,
    unmapped_error, //UsageFault_Handler,
    0, 0, 0, 0,         /* Reserved */
    unmapped_error, //SVC_Handler,
    unmapped_error, //DebugMon_Handler,
    0,                  /* Reserved  */
    unmapped_error, //PendSV_Handler,
    systick_irq, //SysTick_Handler,

    // add 16 to these numbers to get the ARM ISR number...
    /* Configurable interrupts  */
    unmapped_error, //SUPC_IrqHandler,    /* 0  Supply Controller */
    unmapped_error, //RSTC_IrqHandler,    /* 1  Reset Controller */
    unmapped_error, //RTC_IrqHandler,     /* 2  Real Time Clock */
    unmapped_error, //RTT_IrqHandler,     /* 3  Real Time Timer */
    unmapped_error, //WDT_IrqHandler,     /* 4  Watchdog Timer */
    unmapped_error, //PMC_IrqHandler,     /* 5  PMC */
    unmapped_error, //EEFC_IrqHandler,    /* 6  EEFC */
    unmapped_error, //IrqHandlerNotUsed,  /* 7  Reserved */
    unmapped_error, //UART0_IrqHandler,   /* 8  UART0 */
    unmapped_error, //UART1_IrqHandler,   /* 9  UART1 */
    unmapped_error, //SMC_IrqHandler,     /* 10 SMC */
    halls_pioa_irq, //PIOA_IrqHandler,    /* 11 Parallel IO Controller A */
    halls_piob_irq, //PIOB_IrqHandler,    /* 12 Parallel IO Controller B */
    unmapped_error, //PIOC_IrqHandler,    /* 13 Parallel IO Controller C */
    rs485_irq,      //USART0_IrqHandler,  /* 14 USART 0 */
    pb_usart1_irq,  //USART1_IrqHandler,  /* 15 USART 1 */
    unmapped_error, //IrqHandlerNotUsed,  /* 16 Reserved */
    unmapped_error, //IrqHandlerNotUsed,  /* 17 Reserved */
    unmapped_error, //MCI_IrqHandler,     /* 18 MCI */
    i2c_sensors_twi_irq,   //TWI0_IrqHandler,    /* 19 TWI 0 */
    unmapped_error, //TWI1_IrqHandler,    /* 20 TWI 1 */
    unmapped_error, //SPI_IrqHandler,     /* 21 SPI */
    unmapped_error, //SSC_IrqHandler,     /* 22 SSC */
    pb_tc0_irq,     //TC0_IrqHandler,     /* 23 Timer Counter 0 */
    unmapped_error, //TC1_IrqHandler,     /* 24 Timer Counter 1 */
    unmapped_error, //TC2_IrqHandler,     /* 25 Timer Counter 2 */
    unmapped_error, //TC3_IrqHandler,     /* 26 Timer Counter 3 */
    unmapped_error, //TC4_IrqHandler,     /* 27 Timer Counter 4 */
    unmapped_error, //TC5_IrqHandler,     /* 28 Timer Counter 5 */
    adc_irq, //ADC_IrqHandler,     /* 29 ADC controller */
    unmapped_error, //DAC_IrqHandler,     /* 30 DAC controller */
    unmapped_error, //PWM_IrqHandler,     /* 31 PWM */
    unmapped_error, //CRCCU_IrqHandler,   /* 32 CRC Calculation Unit */
    unmapped_error, //ACC_IrqHandler,     /* 33 Analog Comparator */
    unmapped_error, //USBD_IrqHandler,    /* 34 USB Device Port */
    unmapped_error, //IrqHandlerNotUsed   /* 35 not used */
};

void reset_vector( void )
{
  uint32_t *pSrc, *pDest ;
  EFC->EEFC_FMR = EEFC_FMR_FWS(3);
  pSrc = &_etext;
  pDest = &_srelocate;
  if (pSrc != pDest)
    for (; pDest < &_erelocate;)
      *pDest++ = *pSrc++ ;
  for (pDest = &_szero; pDest < &_ezero;)
    *pDest++ = 0;
  pSrc = (uint32_t *)&_sfixed;
  SCB->VTOR = ((uint32_t)pSrc & SCB_VTOR_TBLOFF_Msk);
  if (((uint32_t)pSrc >= IRAM_ADDR) && ((uint32_t)pSrc < IRAM_ADDR+IRAM_SIZE))
    SCB->VTOR |= 1 << SCB_VTOR_TBLBASE_Pos;
  __libc_init_array();
  main();
  while (1) { }
}

