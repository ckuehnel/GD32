/*
 * Test ADC-DAC-Subsystem of Sipeed Longan Nano a GigaBit RISC-V GD32VF103CBT6 based board.
 * 
 * Connect PA3 and PA4 on Longan Nano board.
 * DAC & ADC values were displayed at on-board display and sent to serial output.
 * I used the logged data for preparation of DAC-ADC-Characteristics via Excel.
 *
 * 2020-07-01 Claus Kühnel (info@ckuehnel.ch)
 *
 * This example code is in the public domain.
 */ 

#include <Arduino.h>

extern "C" {
#include "lcd/lcd.h"
}
#include <stdio.h>
#include <inttypes.h>

const char *s = "Sipeed Logan Nano";
uint32_t value = 0;


static void init_uart0(void)
{ 
   // configure USART0
   usart_deinit(USART0);
   usart_baudrate_set(USART0, 115200U);
   usart_word_length_set(USART0, USART_WL_8BIT);
   usart_stop_bit_set(USART0, USART_STB_1BIT);
   usart_parity_config(USART0, USART_PM_NONE);
   usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
   usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
   usart_receive_config(USART0, USART_RECEIVE_ENABLE);
   usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
   usart_enable(USART0);
}

// retarget the C library printf function to USART0
extern "C" int _put_char(int ch) // used by printf
{
     usart_data_transmit(USART0, (uint8_t) ch );
     while (usart_flag_get(USART0, USART_FLAG_TBE) == RESET){
     }
     return ch;
}

// configure the RCU of peripherals
void rcu_config(void)
{
    /* enable the clock of peripherals */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_USART0);
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_DAC);
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV8);
}

//configure the related GPIO
void gpio_config(void)
{
    /* config the GPIO as analog mode */
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    /* once enabled the DAC, the corresponding GPIO pin is connected to the DAC converter automatically */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
}

void adc_config(void)
{
    /* reset ADC */
    adc_deinit(ADC0);
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE); //ADCs work independently 

    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT); // data right aligned
    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 1);
 
    /* ADC regular channel config */
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_3, ADC_SAMPLETIME_239POINT5);
   
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_NONE); // software trigger
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
    
    /* ADC discontinuous mode */
    adc_discontinuous_mode_config(ADC0, ADC_REGULAR_CHANNEL, 3);
    
    /* enable ADC0 end-of-conversion interrupt */ 
    adc_interrupt_enable(ADC0,ADC_INT_EOC); 

    /* enable ADC interface */
    adc_enable(ADC0);
    delay_1ms(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
}

// configure the DAC
void dac_config(void)
{
    dac_deinit();
    /* configure the DAC0 */
    dac_trigger_source_config(DAC0, DAC_TRIGGER_SOFTWARE);
    dac_trigger_enable(DAC0);
    dac_wave_mode_config(DAC0, DAC_WAVE_DISABLE);
    dac_output_buffer_enable(DAC0);
    /* enable DAC0 for DAC0 */
    dac_enable(DAC0);
}

static void longan_oled_init(void)
{
    Lcd_Init();
    LCD_Clear(BLACK);
    BACK_COLOR = BLACK;
}

void setup()
{
  init_uart0();
  rcu_config();
  gpio_config();
  adc_config();
  dac_config();
  longan_oled_init();
  LCD_Clear(BLACK);
  LCD_ShowString(0, 0, (const u8 *) s, GBLUE);
  printf("\nTest Longan Nano ADC-DAc-Subsystem\n");
  printf(" DAC\tADC");

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop()
{
  for (uint16_t dac = 0; dac < 4096; dac = dac + 0x10)
  {
    char buf[12];
    sprintf(buf, "DAC: %03X", dac);
    LCD_ShowString(0, 32, (u8 const *) buf, GBLUE);
    printf("\n %d", dac);

    // DAC Out - ADC IN
    dac_data_set(DAC0, DAC_ALIGN_12B_R, dac);
    dac_software_trigger_enable(DAC0);
    delay_1ms(100); // wait for settling analog voltage

    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
    delay_1ms(10);
  
    uint16_t adc;
    if(adc_interrupt_flag_get(ADC0, ADC_INT_FLAG_EOC))
    {
      adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOC);
      adc = adc_regular_data_read(ADC0);
    }

    sprintf(buf, "ADC: %03X", adc);
    LCD_ShowString(80, 32, (u8 const *) buf, GBLUE);
    printf("\t %d", adc);

    // turn the LED on by making the voltage LOW    
    digitalWrite(LED_BUILTIN, LOW);
    delay_1ms(100);
    // turn the LED off by making the voltage HIGH
    digitalWrite(LED_BUILTIN, HIGH);
    delay_1ms(900);
  }
}
