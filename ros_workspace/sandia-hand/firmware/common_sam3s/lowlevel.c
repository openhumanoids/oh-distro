#include "lowlevel.h"
#include "sam3s/chip.h"

void lowlevel_init_clocks() // clock us up to 64 mhz plz
{
  if (!(PMC->CKGR_MOR & CKGR_MOR_MOSCSEL)) // if not running main oscillator
  {
    PMC->CKGR_MOR = CKGR_MOR_KEY(0x37) |     // "password" hard-wired in logic
                    CKGR_MOR_MOSCXTST(0x20) | // startup time: slowclock*8*this
                    CKGR_MOR_MOSCRCEN | // keep main on-chip RC oscillator on !
                    CKGR_MOR_MOSCXTEN; // main crystal oscillator enable
    while (!(PMC->PMC_SR & PMC_SR_MOSCXTS)) { } // spin...
  }
  // switch to main crystal oscillator
  PMC->CKGR_MOR = CKGR_MOR_KEY(0x37) |
                  CKGR_MOR_MOSCXTST(0x20) |
                  CKGR_MOR_MOSCRCEN | // keep main on-chip RC oscillator on !
                  CKGR_MOR_MOSCXTEN |
                  CKGR_MOR_MOSCSEL;
  while (!(PMC->PMC_SR & PMC_SR_MOSCSELS)) { } // spin until stable
  PMC->PMC_MCKR = (PMC->PMC_MCKR & ~(uint32_t)PMC_MCKR_CSS_Msk) |
                  PMC_MCKR_CSS_MAIN_CLK; // select main clock (really needed?)
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected

  // now, spin up the PLL so we can run at 64 MHz
  PMC->CKGR_PLLAR = CKGR_PLLAR_STUCKTO1 | // as per datasheet, must set 1<<29
                    CKGR_PLLAR_MULA(0x07) | // was 0x03
                    CKGR_PLLAR_DIVA(0x01) |
                    CKGR_PLLAR_PLLACOUNT(0x01);
  while (!(PMC->PMC_SR & PMC_SR_LOCKA)) { } // spin until lock
  // is this step needed? Atmel's EK does it...
  PMC->PMC_MCKR = PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_MAIN_CLK;
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected
  // finally, switch to PLL output
  PMC->PMC_MCKR = PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK;
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected
}


