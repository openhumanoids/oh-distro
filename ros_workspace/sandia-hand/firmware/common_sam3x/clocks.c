#include "sam3x.h"
#include "clocks.h"
#include "led.h"

void clocks_init()
{
  volatile int i, j;
  led_init(); // we are often the first thing called. make sure led is ready.
  EFC0->EEFC_FMR = EEFC_FMR_FWS(4); // set flash wait states so it can handle 
  EFC1->EEFC_FMR = EEFC_FMR_FWS(4); // our blazing speed. otherwise we fail...
  WDT->WDT_MR = WDT_MR_WDDIS; // buh bye watchdog. revisit this?
  __ASM volatile("cpsid i"); // disable all interrupts
  PMC->PMC_PCER0 = (1 << ID_PIOA) | (1 << ID_PIOB) | (1 << ID_PIOC) | 
                   (1 << ID_USART1);
  led_on(0);
  // switch to the slow internal RC oscillator so we can monkey
  // around with the main crystal oscillator and PLL
  PMC->PMC_MCKR = (PMC->PMC_MCKR & ~(uint32_t)PMC_MCKR_CSS_Msk) |
                  PMC_MCKR_CSS_MAIN_CLK;
  PMC->CKGR_MOR = CKGR_MOR_KEY(0x37) |     // "password" hard-wired in logic
                  CKGR_MOR_MOSCXTST(0x10) | // startup time: slowclock*8*this
                  CKGR_MOR_MOSCRCEN | // keep main on-chip RC oscillator on !
                  CKGR_MOR_MOSCXTEN; // crystal oscillator enable (not select)
  while (!(PMC->PMC_SR & PMC_SR_MOSCSELS)) { } // spin until stable
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected
  for (i = 0; i < 50000; i++) { } // can't remember why i did this. maybe
                                  // to waste some time in case JTAG needs
                                  // to connect in case the following code
                                  // is borked somehow.
  for (j = 0; j < 2; j++) // blink the LED a few times just to say we're alive
  {
    for (i = 0; i < 30000; i++) { }
    led_on(0);
    for (i = 0; i < 30000; i++) { }
    led_off(0);
  }
  PMC->CKGR_MOR = CKGR_MOR_KEY(0x37) |     // "password" hard-wired in logic
                  CKGR_MOR_MOSCXTST(0x10) | // startup time: slowclock*8*this
                  CKGR_MOR_MOSCRCEN | // keep main on-chip RC oscillator on !
                  CKGR_MOR_MOSCXTEN; // main crystal oscillator enable
  while (!(PMC->PMC_SR & PMC_SR_MOSCXTS)) { } // busy wait
  // switch to main crystal oscillator
  PMC->CKGR_MOR = CKGR_MOR_KEY(0x37)      |
                  CKGR_MOR_MOSCXTST(0x10) |
                  CKGR_MOR_MOSCRCEN       | // keep on-chip RC oscillator on !
                  CKGR_MOR_MOSCXTEN       |
                  CKGR_MOR_MOSCSEL;
  while (!(PMC->PMC_SR & PMC_SR_MOSCSELS) ||
         !(PMC->PMC_SR & PMC_SR_MCKRDY)       ) { } // spin until stable
  // blink somewhat faster to show crystal oscillator is running
  for (j = 0; j < 4; j++)
  {
    for (i = 0; i < 30000; i++) { }
    led_on(0);
    for (i = 0; i < 30000; i++) { }
    led_off(0);
  }
  PMC->PMC_MCKR = (PMC->PMC_MCKR & ~(uint32_t)PMC_MCKR_CSS_Msk) |
                   PMC_MCKR_CSS_MAIN_CLK; // select main clock (really needed?)
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected

  // now, spin up the PLL. we want PLL output to be 128 MHz
  PMC->CKGR_PLLAR = CKGR_PLLAR_ONE        | // as per datasheet, must set 1<<29
                    CKGR_PLLAR_MULA(0x07) | // pll = crystal * (mul+1)/div
                    CKGR_PLLAR_DIVA(0x01) | // which gives us 128 mhz
                    CKGR_PLLAR_PLLACOUNT(0x01);
  while (!(PMC->PMC_SR & PMC_SR_LOCKA)) { } // spin until lock
  // is this step needed? Atmel's EK does it...
  PMC->PMC_MCKR = PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_MAIN_CLK;
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected
  // finally, switch to PLL output, dividing by 2 to get us 64 MHz PLLACK
  PMC->PMC_MCKR = PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK;
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected

  for (i = 0; i < 200000; i++) { } // why is this here?
  for (j = 0; j < 3; j++) // blink a few times to show we're on 64 MHz clock
  {
    for (i = 0; i < 100000; i++) { }
    led_on(0);
    for (i = 0; i < 100000; i++) { }
    led_off(0);
  }
}
