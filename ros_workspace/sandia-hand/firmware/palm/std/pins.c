#include "pins.h"
#include "sam3s/sam3s.h"

// default values are for the right hand. 
char g_pins_hand = 'R';

Pin pin_led = { PIO_PC19, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT };

Pin pin_rs485_de = { PIO_PC9, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT };
Pin pin_rs485_di = { PIO_PA6, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };
Pin pin_rs485_ro = { PIO_PA5, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };

Pin pin_spi_mosi = { PIO_PA13, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };
Pin pin_spi_miso = { PIO_PA12, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };
Pin pin_spi_sck =  { PIO_PA14, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };

Pin pin_cs_adc[2] = {{PIO_PA25, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                     {PIO_PC23, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT}};

Pin pin_i2c_scl = { PIO_PA4, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };
Pin pin_i2c_sda = { PIO_PA3, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };

// mux0_a0  PC27
// mux0_a1  PA21
// mux0_a2  PC26

// mux1_a0  PB0
// mux1_a1  PC30
// mux1_a2  PB1

// mux2_a0  PA16
// mux2_a1  PC13
// mux2_a2  PC0

// mux3_a0  PC16
// mux3_a1  PA1
// mux3_a2  PC17

// mux4_a0  PA7
// mux4_a1  PA8
// mux4_a2  PA9

Pin pin_mux[5][3] = {{{PIO_PC27,PIOC,ID_PIOC,PIO_OUTPUT_0,PIO_DEFAULT},
                      {PIO_PA21,PIOA,ID_PIOA,PIO_OUTPUT_0,PIO_DEFAULT},
                      {PIO_PC26,PIOC,ID_PIOC,PIO_OUTPUT_0,PIO_DEFAULT}},
                     {{PIO_PB0 ,PIOB,ID_PIOB,PIO_OUTPUT_0,PIO_DEFAULT},
                      {PIO_PC30,PIOC,ID_PIOC,PIO_OUTPUT_0,PIO_DEFAULT},
                      {PIO_PB1 ,PIOB,ID_PIOB,PIO_OUTPUT_0,PIO_DEFAULT}},
                     {{PIO_PA16,PIOA,ID_PIOA,PIO_OUTPUT_0,PIO_DEFAULT},
                      {PIO_PC13,PIOC,ID_PIOC,PIO_OUTPUT_0,PIO_DEFAULT},
                      {PIO_PC0 ,PIOC,ID_PIOC,PIO_OUTPUT_0,PIO_DEFAULT}},
                     {{PIO_PC16,PIOC,ID_PIOC,PIO_OUTPUT_0,PIO_DEFAULT},
                      {PIO_PA1 ,PIOA,ID_PIOA,PIO_OUTPUT_0,PIO_DEFAULT},
                      {PIO_PC17,PIOC,ID_PIOC,PIO_OUTPUT_0,PIO_DEFAULT}},
                     {{PIO_PA7 ,PIOA,ID_PIOA,PIO_OUTPUT_0,PIO_DEFAULT},
                      {PIO_PA8 ,PIOA,ID_PIOA,PIO_OUTPUT_0,PIO_DEFAULT},
                      {PIO_PA9 ,PIOA,ID_PIOA,PIO_OUTPUT_0,PIO_DEFAULT}}};

// 0  led_0     PC15
// 1  led_1     PA23
// 2  led_2     PA18
// 3  led_3     PC31
// 4  led_4     PA19
// 5  led_5     PA17
// 6  led_6     PA22
// 7  led_7     PA20
// 8  led_8     PC6
// 9  led_9     PC12
// 10 led_10    PA10
// 11 led_11_0  PC3
// 12 led_11_1  PC2
// 13 led_12_0  PA15
// 14 led_12_1  PA31
// 15 led_13_0  PA0
// 16 led_13_1  PC29
// 17 led_14    PC21
// 18 led_15_0  PC18
// 19 led_15_1  PC5

Pin pin_leds[20] = {{PIO_PC15, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PA23, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PA18, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PC31, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PA19, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PA17, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PA22, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PA20, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PC6 , PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PC12, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
        /* 10 */    {PIO_PA10, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PC3 , PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PC2 , PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PA15, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PA31, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
        /* 13_0 */  {PIO_PA0 , PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PC29, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PC21, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PC18, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                    {PIO_PC5 , PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT}};

// helper function to save typing
static void pin_assign(Pin *pin, uint32_t mask, Pio *pio, 
                       uint8_t id, uint8_t type, uint8_t attribute)
{
  pin->mask = mask;
  pin->pio = pio;
  pin->id = id;
  pin->type = type;
  pin->attribute = attribute;
}

void pins_init()
{
  uint32_t bl_hw_version = *((uint32_t *)0x0401ff8); // magic, defined in bootloader
  if (((bl_hw_version >> 16) & 0xffff) != 0xbeef) // check for magic bytes
    bl_hw_version = 0; // undefined
  else
    bl_hw_version &= 0xffff; // keep useful lower 16 bits
  g_pins_hand = (char)((bl_hw_version >> 8) & 0xff);

  // default values are for the right hand. no need for reassignment unless
  // it's for a left hand, or unless future revs of this board tweak the pins
  if (g_pins_hand != 'L')
    return;

  pin_assign(&pin_led,      PIO_PC18, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT);

  pin_assign(&pin_rs485_de, PIO_PA19, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT);
  pin_assign(&pin_rs485_di, PIO_PA22, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT);
  pin_assign(&pin_rs485_ro, PIO_PA21, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT);

  pin_assign(&pin_spi_mosi, PIO_PA13, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT);
  pin_assign(&pin_spi_miso, PIO_PA12, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT);
  pin_assign(&pin_spi_sck,  PIO_PA14, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT);
  pin_assign(&pin_cs_adc[0],PIO_PC31, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_cs_adc[1],PIO_PA30, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT);

  pin_assign(&pin_i2c_scl,  PIO_PA4 , PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT);
  pin_assign(&pin_i2c_sda,  PIO_PA3 , PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT);
  
  pin_assign(&pin_mux[0][0],PIO_PC2 , PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT);
  pin_assign(&pin_mux[0][1],PIO_PC4 , PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT);
  pin_assign(&pin_mux[0][2],PIO_PA25, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT);
  pin_assign(&pin_mux[1][0],PIO_PC10, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT);
  pin_assign(&pin_mux[1][1],PIO_PC9 , PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT);
  pin_assign(&pin_mux[1][2],PIO_PA28, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT);
  pin_assign(&pin_mux[2][0],PIO_PC12, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT);
  pin_assign(&pin_mux[2][1],PIO_PA16, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT);
  pin_assign(&pin_mux[2][2],PIO_PC13, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT);
  pin_assign(&pin_mux[3][0],PIO_PB1 , PIOB, ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT);
  pin_assign(&pin_mux[3][1],PIO_PC29, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT);
  pin_assign(&pin_mux[3][2],PIO_PB14, PIOB, ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT);
  pin_assign(&pin_mux[4][0],PIO_PA23, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT);
  pin_assign(&pin_mux[4][1],PIO_PC26, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT);
  pin_assign(&pin_mux[4][2],PIO_PA18, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT);

  pin_assign(&pin_leds[0] , PIO_PA8 , PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[1] , PIO_PA9 , PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[2] , PIO_PB10, PIOB, ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[3] , PIO_PC11, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[4] , PIO_PA10, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[5] , PIO_PA7 , PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[6] , PIO_PC5 , PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[7] , PIO_PA20, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[8] , PIO_PC6 , PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[9] , PIO_PA24, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[10], PIO_PC15, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[11], PIO_PC30, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[12], PIO_PC3 , PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[13], PIO_PC14, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[14], PIO_PB13, PIOB, ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[15], PIO_PA17, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[16], PIO_PC17, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[17], PIO_PA0 , PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[18], PIO_PB0 , PIOB, ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT);
  pin_assign(&pin_leds[19], PIO_PC0 , PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT);

  // PB10 is used as GPIO here, need to set that in a magic place
  MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO10;
}

