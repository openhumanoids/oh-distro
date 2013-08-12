#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

// hardware connections:
//  PD0 = rxd
//  PD1 = txd

#define PORTC_LED_PIN   0x04
#define PORTC_SSR_PIN   0x08
#define PORTD_TX_PIN    0x02
#define PORTD_FAN_PIN   0x20
#define INA226_I2C_ADDR 0x80
#define I2C_TIMEOUT     1000
#define TIMER1_FREQ   125000

inline void toggle_led() { PORTC ^= PORTC_LED_PIN; }
static volatile uint8_t g_timer_flag = 0, g_raw_temp_valid = 0;
static volatile uint16_t g_raw_temp_adc = 0;
static volatile uint32_t g_timer_count = 0;
#define SWAP_16(x) (((x >> 8) & 0xff) | ((x << 8) & 0xff00))

volatile enum output_state_t
{
  OST_INIT,
  OST_POWER_ON,
  OST_POWER_OVERCURRENT,
  OST_POWER_LOCK_OUT
} output_state = OST_INIT;

ISR(TIMER1_COMPA_vect)
{
  g_timer_flag = 1;
  g_timer_count++;
}

ISR(ADC_vect)
{
  g_raw_temp_valid = 1;
}

int usart0_putc(char c, FILE *f)
{
  while (!(UCSR0A & (_BV(UDRE0)))) { }
  UDR0 = c;
  return 0;
}

inline void i2c_start()
{
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
}

inline void i2c_stop()
{
  TWCR = _BV(TWSTO);
}

inline void i2c_start_tx(uint8_t b)
{
  TWDR = b;
  TWCR = _BV(TWINT) | _BV(TWEN);
}

inline void i2c_start_rx(uint8_t ack)
{
  if (ack)
    TWCR = (TWCR & 0xf) | _BV(TWINT) | _BV(TWEA);
  else
    TWCR = (TWCR & 0xf) | _BV(TWINT);
}

inline uint8_t i2c_get_rx()
{
  return TWDR;
}

inline uint8_t i2c_sync_wait()
{
  for (uint32_t timeouts = 0; 
       !(TWCR & _BV(TWINT)) && timeouts < I2C_TIMEOUT; 
       timeouts++) { }
  return (TWCR & _BV(TWINT)) ? 1 : 0;
}

#define I2C_WAIT() do { if (!i2c_sync_wait()) { i2c_stop(); return 0; } } while (0)

uint8_t i2c_sync_read(const uint8_t dev_addr, const uint8_t dev_reg_addr, 
                      uint8_t *read_data, const uint8_t read_data_len)
{
  i2c_start();
  I2C_WAIT();
  i2c_start_tx(dev_addr);
  I2C_WAIT();
  i2c_start_tx(dev_reg_addr);
  I2C_WAIT();
  i2c_start();
  I2C_WAIT();
  i2c_start_tx(dev_addr | 0x01); // send read addr
  I2C_WAIT();
  for (uint8_t read_idx = 0; read_idx < read_data_len; read_idx++)
  {
    const uint8_t last_byte = (read_idx == read_data_len - 1) ? 1 : 0;
    i2c_start_rx(last_byte ? 0 : 1);
    I2C_WAIT();
    if ((TWSR & 0xf8) != (last_byte ? 0x58 : 0x50)) // check twi status code
    {
      printf("lost i2c arb on byte %d, twsr = 0x%02x\r\n", read_idx, TWSR);
      i2c_stop();
      return 0;
    }
    read_data[read_idx] = i2c_get_rx();
  }
  i2c_stop();
  return 1;
}

uint8_t i2c_sync_write(const uint8_t dev_addr, const uint8_t dev_reg_addr,
                       const uint8_t *write_data, const uint8_t write_data_len)
{
  i2c_start();
  I2C_WAIT();
  i2c_start_tx(dev_addr);
  I2C_WAIT();
  i2c_start_tx(dev_reg_addr);
  I2C_WAIT();
  for (uint8_t write_idx = 0; write_idx < write_data_len; write_idx++)
  {
    i2c_start_tx(write_data[write_idx]);
    I2C_WAIT();
  }
  i2c_stop();
  return 1;
}

uint8_t ina226_write_reg(const uint8_t reg, const uint16_t val)
{
  uint16_t val_rev = SWAP_16(val);
  return i2c_sync_write(INA226_I2C_ADDR, reg, (uint8_t *)&val_rev, 2);
}

uint8_t ina226_read_reg(const uint8_t reg, uint16_t *val)
{
  uint16_t val_rev = 0;
  if (!i2c_sync_read(INA226_I2C_ADDR, reg, (uint8_t *)&val_rev, 2))
  {
    printf("rf\r\n");
    return 0;
  }
  *val = SWAP_16(val_rev);
  return 1;
}

uint16_t ina226_read_current()
{
  uint16_t raw_val = 0;
  if (!ina226_read_reg(4, &raw_val))
    return 0;
  return raw_val;
}

inline void set_output(uint8_t enable)
{
  if (enable)
    PORTC |= PORTC_SSR_PIN;
  else
    PORTC &= ~PORTC_SSR_PIN;
}

inline void set_fan(uint8_t enable)
{
  if (enable)
    PORTD |= PORTD_FAN_PIN;
  else
    PORTD &= ~PORTD_FAN_PIN;
}

inline void set_led(uint8_t enable)
{
  if (enable)
    PORTC |=  PORTC_LED_PIN;
  else
    PORTC &= ~PORTC_LED_PIN;
}

int main()
{
  wdt_enable(0x05); // watchdog countdown of about 0.25 seconds
  PORTC = 0;
  DDRC = PORTC_SSR_PIN | PORTC_LED_PIN;
  DDRD = PORTD_TX_PIN | PORTD_FAN_PIN;
  UCSR0A = 0;
  UCSR0B = _BV(TXEN0);
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8N1 format
  UBRR0H = 0;
  UBRR0L = 12; // 38400 baud
  fdevopen(usart0_putc, NULL);
  printf("hello, world!\r\n");
  uint16_t temp_offset = eeprom_read_word(0);
  printf("eeprom temp offset: %d\r\n", temp_offset);
  TWSR = 0; // twi prescalar = 1
  TWBR = 2; // twi bit rate = 8000000/(16 + 4) = 400 kHz
  /*
  uint16_t die_id = 0;
  if (!ina226_read_reg(0xff, &die_id))
    printf("ina226 die id read fail\r\n");
  else
  {
    printf("die id: 0x%04x\r\n", die_id);
  }
  */
  set_led(1);
  ina226_write_reg(5, 5120); // set ina226 to 0.1 mA resolution
  _delay_ms(10);
  int16_t raw_reading = ina226_read_current();
  printf("ina226 raw = %d\r\n", raw_reading);
  set_led(0);

  // set up analog temperature auto-polling 
  ADCSRA = _BV(ADEN)  |             // adc enabled
           _BV(ADIE)  |             // interrupt enabled
           _BV(ADPS1) | _BV(ADPS0); // adc clock = sysclk/8 = 125 kHz
  ADCSRB = 0; // free running mode
  ADMUX = _BV(REFS1) | _BV(REFS0) | 0x08; // enable 1v1 reference and temp ch
  OCR1A = TIMER1_FREQ / 100; 
  TCCR1A = 0x00; // clear timer on compare match
  TCCR1B = 0x0b; // CTC, timer 1 = sysclk / 64 = 125 kHz
  TCNT1 = 0;
  TIMSK1 = _BV(OCIE1A);
  sei(); // turn on interrupts
  float temp_celsius = 25; // starting value for exponential filter
  const float gain = 0.2;
  uint32_t state_entry_time = 0;
  while (1) 
  { 
    cli();
    volatile uint32_t sys_time = g_timer_count;
    sei();
    /*
    if (sys_time % 100 == 0)
      set_led(1);
    else if (sys_time % 100 == 5)
      set_led(0);
    */
    if (g_timer_flag)
    {
      ADCSRA |= _BV(ADSC); // start on-chip temperature read
      g_timer_flag = 0;
    }
    if (!g_raw_temp_valid)
      continue;
    // first, control system temperature by bang-bang on the fan
    g_raw_temp_valid = 0;
    g_raw_temp_adc = ADC;
    if (temp_offset == 0xffff)
    {
      temp_offset = 334; //g_raw_temp_adc;  // UNCOMMENT THIS in production
      printf("burning temp sensor offset: %d\r\n", temp_offset);
      eeprom_write_word(0, temp_offset); 
    }
    const float cur_temp_celsius = ((float)g_raw_temp_adc-(float)temp_offset) * 
                                   1.06f + 26.0f;
    temp_celsius = gain * cur_temp_celsius + (1.0f - gain) * temp_celsius;
    const int16_t celsius = (int16_t)temp_celsius;
    if (celsius >= 37)
      set_fan(1);
    else if (celsius <= 31)
      set_fan(0);
    /*
    if (sys_time % 100 == 0)
      printf("%d\r\n", celsius);
    */

    // cut the output as needed by watching the current draw
    int16_t milliamps;
    ina226_read_reg(4, (uint16_t *)&milliamps);
    milliamps /= 10;
    /*
    if (sys_time % 100 == 0)
      printf("raw: %d\r\n", milliamps);
    */
    wdt_reset();
    switch (output_state)
    {
      case OST_INIT:
        set_output(0);
        set_led(0);
        if (sys_time - state_entry_time > 100) // one second delay
        {
          //printf("on\r\n");
          output_state = OST_POWER_ON;
        }
        break;
      case OST_POWER_ON:
        set_output(1);
        if (milliamps > 2000)
        {
          //printf("over\r\n");
          output_state = OST_POWER_OVERCURRENT;
          state_entry_time = sys_time;
        }
        break;
      case OST_POWER_OVERCURRENT:
        set_output(1);
        set_led(1);
        if (milliamps < 2000)
        {
          //printf("under\r\n");
          output_state = OST_POWER_ON;
          set_led(0);
        }
        else if (sys_time - state_entry_time > 50) // 500ms in overload
        {
          output_state = OST_POWER_LOCK_OUT;
          state_entry_time = sys_time;
        }
        break;
      case OST_POWER_LOCK_OUT:
        set_output(0);
        if (sys_time % 10 == 0) // blink at 5 hz
          toggle_led();
        if (sys_time - state_entry_time > 1000) // ten second delay
        {
          state_entry_time = sys_time;
          output_state = OST_INIT;
        }
        break;
      default:
        output_state = OST_INIT;
        break;
    }
  }
  return 0;
}

