/*
 * HelloWorld for Sipeed Longan Nano a GigaBit RISC-V GD32VF103CBT6 based board.
 *
 * Turns an LED on and off for multiple short periods and then off for a longer period
 * echoing the state of the LED to a terminal. This action is continuously repeated.
 * 
 * Using Sipeed init_uart0() procedure to send printf output to the serial port
 * http://longan.sipeed.com/zh/examples/printf.html (thank you to Google for the translation)
 *
 * Based on the example nanoblink2 by Michel Deslierres <https://sigmdel.ca/michel>
 * 2020-06-27 Claus Kühnel (info@ckuehnel.ch)
 *
 * This example code is in the public domain.
 */ 

#include <Arduino.h>       

#define CYCLES         2  // 2 yields a heartbeat effect
#define SHORT_DELAY  100  // 1/10 second
#define LONG_DELAY  1000  // 1 second

#define BOARD "Sipeed Longan Nano (GD32VF103CBT6)"


static void init_uart0(void)
{
   // enable GPIO clock 
   rcu_periph_clock_enable(RCU_GPIOA);
   // enable USART0 clock 
   rcu_periph_clock_enable(RCU_USART0);  
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

// the setup function runs once when you press reset or power the board
void setup() {
   // initialize digital pin LED_BUILTIN as an output.
   pinMode(LED_BUILTIN, OUTPUT);

   init_uart0();
   printf("\nHello World from ");
   printf(BOARD);
   printf("\nPlatform: Arduino");
   printf("\nArduino SW Version ");
   printf("%d", ARDUINO);
   printf("\nClock frequency ");
   printf("%ld", SystemCoreClock/1000000);
   printf(" MHz");
   printf("\nLED_BUILTIN: %d\n", LED_BUILTIN);
   printf("\n");
}

// the loop function runs over and over again forever
void loop() {
   for (int i=0; i<CYCLES; i++) {
     digitalWrite(LED_BUILTIN, LOW);    // turn the red LED on (it is active when the pin is LOW)
     printf("ON ");              // update terminal
     delay(SHORT_DELAY);                // wait for a short on period
     digitalWrite(LED_BUILTIN, HIGH);   // turn the red LED off (by setting the active LOW pin HIGH)
     delay(SHORT_DELAY);                // wait for a short off period
   }    
   digitalWrite(LED_BUILTIN, HIGH);     // turn the LED off
   printf("OFF\n");               // update terminal
   delay(LONG_DELAY);                   // wait for a long off period
}
