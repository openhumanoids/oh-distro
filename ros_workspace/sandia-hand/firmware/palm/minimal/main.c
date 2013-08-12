#include <sam3s/sam3s.h>

// this is a power-consumption test. clocks the CPU waaaaaay down to observe
// the static power load.

void main()
{
  PMC->PMC_MCKR = PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_MAIN_CLK; // crystal/2
  while (!(PMC->PMC_SR & PMC_SR_MOSCSELS)) { } // spin until stable
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected

  // turn off PLLA, which was turned on by the bootloader
  PMC->CKGR_PLLAR = CKGR_PLLAR_STUCKTO1;

  PMC->PMC_MCKR = PMC_MCKR_PRES_CLK_64 | PMC_MCKR_CSS_MAIN_CLK; // crystal/2
  while (!(PMC->PMC_SR & PMC_SR_MOSCSELS)) { } // spin until stable
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) { } // spin until selected

  while (1)
  {
    for (volatile int i = 0; i < 20000; i++) { }
    PIOC->PIO_SODR = PIO_PC19;
    for (volatile int i = 0; i < 20000; i++) { }
    PIOC->PIO_CODR = PIO_PC19;
  }
}

