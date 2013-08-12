// this is intended to be included after a few #defines are set

void bl_led(bl_led_state_t state)
{   
  switch (state)
  {
    case BL_LED_OFF: BL_LED_PIO->PIO_CODR = BL_LED_PIN; break;
    case BL_LED_ON : BL_LED_PIO->PIO_SODR = BL_LED_PIN; break;
    case BL_LED_TOGGLE:
      if (BL_LED_PIO->PIO_ODSR & BL_LED_PIN)
        BL_LED_PIO->PIO_CODR = BL_LED_PIN; 
      else
        BL_LED_PIO->PIO_SODR = BL_LED_PIN;
      break;
    default: break;
  }
} 

