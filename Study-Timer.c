#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

//	Name: Jol Singh
// 	Student ID: n10755756
// 	Project: Study Timer

// --== WIRING ==--
// LCD GND  -> GND
// LCD VCC  -> 5V
// LCD V0   -> GND
// LCD RW   -> GND
// LCD LED Anode    -> 220 Ohm -> 5V
// LCD LED Cathode  -> GND

//LED & Buttons
#define SET_BIT(reg, pin)		    (reg) |= (1 << (pin))
#define CLEAR_BIT(reg, pin)		  (reg) &= ~(1 << (pin))
#define WRITE_BIT(reg, pin, value)   (reg) = (((reg) & ~(1 << (pin))) | ((value) << (pin)))
#define BIT_VALUE(reg, pin)		  (((reg) >> (pin)) & 1)
#define BIT_IS_SET(reg, pin)	     (BIT_VALUE((reg),(pin))==1)

//// include the library code:
#include <LiquidCrystal.h>
#define B1 9
#define B0 8
#define D4 4
#define D5 5
#define D6 6
#define D7 7

//Debouncing Mask
#define MASK 0b00111111
#define MASK_2 0b00111111

// initialize the library by associating any needed LCD interface pin
// with the Arduino pin number it is connected to
uint8_t rs = B1, en = B0;
LiquidCrystal lcd(rs, en, D4, D5, D6, D7);

//Timer definitions
#define FREQ (16000000UL)
#define PRESCALE (1024.0) 
//#define DEBOUNCE_THRESHOLD (6)

//Debouncing Variables
volatile uint8_t start__btn_history = 0;
volatile uint8_t debounced_state  = 0;
volatile uint8_t reset_history = 0;
volatile uint8_t reset_state = 0;

// These buffers may be any size from 2 to 256 bytes.
#define  RX_BUFFER_SIZE  64
#define  TX_BUFFER_SIZE  64

//uart definitions
#define MYF_CPU (16000000UL)
#define BAUD (9600)
#define MYUBRR (MYF_CPU/16/BAUD-1)
unsigned char rx_buf;
static volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
static volatile uint8_t tx_buffer_head;
static volatile uint8_t tx_buffer_tail;
static volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint8_t rx_buffer_head;
static volatile uint8_t rx_buffer_tail;

//Functions
int setup_lcd(void);
void timer_stop(void);
void setup_timer1(void);
void initialize_timer(void);
void pause_watch (void);
int start_time(void);
void timer (void);
void process(void);
void r_btn(void);

//UART Functions
void uart_init(unsigned int ubrr);
void uart_putchar(uint8_t c);
uint8_t uart_getchar(void);
uint8_t uart_available(void);
void uart_putstring(unsigned char* s);
void uart_getLine(unsigned char* buf, uint8_t n);

////////////////////////////////////////////////
// Load and Start Application
////////////////////////////////////////////////

int main(){
  setup_lcd();
  setup_timer1();
  
  // Set Duty Cycle to 50%
  OCR0A = 128;
  
  while(1){
    initialize_timer();
	_delay_ms(1);
  }
  return 0;
}

////////////////////////////////////////////////
// Setup UART, Timer Modules, ADC, and PWM
//////////////////////////////////////////////// 
 
void setup_timer1(){
  
	// initialise uart
	uart_init(MYUBRR);
  
  	//Set LEDs 
	DDRD = (1 << 2);
	DDRD = (1 << 3);
	
	//SET BUTTON OUTPUT 
	DDRB = (1 << 3);
	DDRB = (1 << 2);
	
  	//Speaker
  	SET_BIT(DDRB, 4);

  
	// Timer 1 in normal mode, with pre-scaler 1024 ==> ~60 Hz overflow.
    // and overflow interrupt
	TCCR1A = 0;
	TCCR1B = 1; 
    TIMSK1 = 1; 
  
  	// Timer 0 for software-based PWM, with pre-scaler 8 ==> ~0.9 Hz overflow.
    // and overflow interrupt
    //TCCR0A = 0; 
    //TCCR0B = 1; 
    //TIMSK0 = 1; 
	
  // Enable timer overflow, and turn on interrupts.
	sei();

	// initialise adc
	// ADC Enable and pre-scaler of 128: ref table 24-5 in datasheet
    // ADEN  = 1
    // ADPS2 = 1, ADPS1 = 1, ADPS0 = 1
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

   // select channel and ref input voltage
   // channel 0, PC0 (A0 on the uno)
   // MUX0=0, MUX1=0, MUX2=0, MUX3=0
   // REFS0=1
   // REFS1=0
    ADMUX = (1 << REFS0);

    TCCR0A |= (1 << COM0A1);
    // set none-inverting mode
 
    // TinkerCAD Errata: timer clocking must be enabled before WGM
    // set prescaler to 8 and starts PWM  
    TCCR0B = (1 << CS01);
  
    TCCR0A |= (1 << WGM01) | (1 << WGM00);
    // set fast PWM Mode
}

////////////////////////////////////////////////
// Timer Method
//////////////////////////////////////////////// 

volatile int overflow_count = 0;
ISR(TIMER1_OVF_vect)
{
  	// shift history across by one
 	start__btn_history = start__btn_history << 1; 
  	start__btn_history = start__btn_history & MASK;
 
  	reset_history = reset_history << 1; 
  	reset_history = reset_history & MASK_2;
  
  	// add current state to history
  	start__btn_history |= ((PINB & (1 << 3)) >> 3);
  	reset_history |= ((PINB & (1 << 2)) >> 2);
  
  	if (start__btn_history == MASK) {
    	// button is pressed
      	debounced_state  = 1;
    } else if (start__btn_history == 0) {
    	// button is not pressed
      	debounced_state  = 0;
    }
  
  if (reset_history == MASK_2) {
    	// button is pressed
      	reset_state  = 1;
    } else if (reset_history == 0) {
    	// button is not pressed
      	reset_state  = 0;
    }

  
  	// increase counter for next iteration
    overflow_count++;
}

////////////////////////////////////////////////
// LCD Setup
//////////////////////////////////////////////// 

int setup_lcd(){	

	// set up the LCD in 4-pin or 8-pin mode
	lcd.begin(16,2);
  
    // Print messages to the LCD 
	lcd.print("Study Timer");
	lcd.setCursor(0,1);
	_delay_ms(1000);
	lcd.print("Simualtion");
	_delay_ms(1000);
	for (int i = 0; i < 14; i++)
	{
    lcd.scrollDisplayLeft();
    _delay_ms(300);
	}
	_delay_ms(200);

	lcd.clear();
	lcd.print("Right Button");
	lcd.setCursor(0,1);
	lcd.print("to Start/Stop");
	lcd.setCursor(1,1);
	_delay_ms(2500);

	lcd.clear();
	lcd.print("Left Button");
	lcd.setCursor(0,1);
	lcd.print("to Reset");
	lcd.setCursor(1,1);
	_delay_ms(2500);

	lcd.clear();
	lcd.print("Green Light = ");
	lcd.setCursor(0,1);
	lcd.print("Timer Running");
	lcd.setCursor(1,1);
	_delay_ms(2000);

	lcd.clear();
	lcd.print("Red Light = ");
	lcd.setCursor(0,1);
	lcd.print("Timer Resetting");
	lcd.setCursor(1,1);
	_delay_ms(2000);

	lcd.clear();
	lcd.print("Loading...");
	_delay_ms(1000);
	
	return 0;
}

////////////////////////////////////////////////
// Main Application Method
//////////////////////////////////////////////// 

void initialize_timer(){
	lcd.clear();
	lcd.print("Elasped Time: ");
	lcd.setCursor(0,1);
  	CLEAR_BIT(PORTB,PB4);
	timer();
}

void timer (){
	while(1) {
       bool button_pressed = false;
        // check if button was just pressed
        if (debounced_state == 1) {
          // button is pressed, so check if flag has been set
          if (!button_pressed) {
            // flag not set so this is first read of it pressed
            button_pressed = true;
            uart_putstring((unsigned char *) "Timer Recording");
			uart_putchar('\n');
			PORTD |= (1 << 3);
			overflow_count = 0;
			timer_stop();
			break;
          }
        } 
    }
}

void timer_stop (){
	while (1){		
		int status  = start_time();
		while(status != 0){
          process();
          break;
		}
      
	}
}

////////////////////////////////////////////////
// Timer/Stopwatch Method
//////////////////////////////////////////////// 

void process(){	
	while(1){		
	
		char temp_buf[64];
		double elapsed_time = ( overflow_count * 256.0 + TCNT0 ) * PRESCALE / FREQ;
		dtostrf(elapsed_time, 5, 2,temp_buf);
	   
		//Display Elapsed Time	
      	lcd.setCursor(5,1);	
      	lcd.print(temp_buf);
		
	  ////////////////////////////////////////////////
	  // Start/Stop Button Method
	  //////////////////////////////////////////////// 
      bool button_pressed = false;
      for(;;){
        // check if button was just pressed
        if (debounced_state == 1) {
          // button is pressed, so check if flag has been set
          if (!button_pressed) {
            // flag not set so this is first read of it pressed
            button_pressed = true;
			PORTD |= (1 << 3);
            
            //send serial data
            uart_putstring((unsigned char *)temp_buf);
            uart_putstring((unsigned char *)" seconds");
     		uart_putchar('\n');
            
			break;
          }
        }else{
        	button_pressed = false;
         	 PORTD &= ~(1 << 3);
          
			// If start/stop button no longer pressed
			// the reset button can be pressed to update the
			// LDC
			
             r_btn();
        }
      }
      
	}
 	
}

////////////////////////////////////////////////
// Reset Button Method
//////////////////////////////////////////////// 

void r_btn(){
  char temp_buf[64];
  bool button_pressed_2 = false;
		 if(reset_state == 1){
          	if (!button_pressed_2) {
              // flag not set so this is first read of it pressed
              button_pressed_2 = true;
              PORTD |= (1 << 2);
              
              // Start single conversion by setting ADSC bit in ADCSRA
              ADCSRA |= (1 << ADSC);

              // Wait for ADSC bit to clear, signalling conversion complete.
               while ( ADCSRA & (1 << ADSC) ) {}

              // Result now available in ADC
               uint16_t pot = ADC;

              // convert uint16_t to string
              itoa(pot, (char *)temp_buf,10);

             //when converted value is above a threshold, perform an action
             if (pot > 512)
               SET_BIT(PORTB,PB4);
             else
               CLEAR_BIT(PORTB,PB4);
              
              //send serial data
               uart_putstring((unsigned char *) "Timer Reset");
              uart_putchar('\n');
              uart_putstring((unsigned char *) "ADC Output: ");
               uart_putstring((unsigned char *) temp_buf);
				uart_putchar('\n');
              _delay_ms(1000);
              PORTD &= ~(1 << 2);
              initialize_timer();
          	}
        }else{
        	button_pressed_2 = false;      
        }
}

int start_time (){
	unsigned char temp_s = BIT_IS_SET(PORTD, 3);
	if (temp_s != 0){
		return 1;
	}else if (temp_s == 0){
		return 0;
	}
}

//PLEASE NOTE THIS VERSION OF UART USES INTERRUPTS
 
/*  ****** serial uart definitions ************ */
/******************  interrupt based  ********/
 
// Initialize the UART
void uart_init(unsigned int ubrr) {
 
    cli();
 
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)(ubrr);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    tx_buffer_head = tx_buffer_tail = 0;
    rx_buffer_head = rx_buffer_tail = 0;
 
    sei();
 
}
 
// Transmit a byte
void uart_putchar(uint8_t c) {
    uint8_t i;
 
    i = tx_buffer_head + 1;
    if ( i >= TX_BUFFER_SIZE ) i = 0;
    while ( tx_buffer_tail == i ); // wait until space in buffer
    //cli();
    tx_buffer[i] = c;
    tx_buffer_head = i;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << UDRIE0);
    //sei();
}
 
// Receive a byte
uint8_t uart_getchar(void) {
    uint8_t c, i;
 
    while ( rx_buffer_head == rx_buffer_tail ); // wait for character
    i = rx_buffer_tail + 1;
    if ( i >= RX_BUFFER_SIZE ) i = 0;
    c = rx_buffer[i];
    rx_buffer_tail = i;
    return c;
}
 
// Transmit a string
void uart_putstring(unsigned char* s)
{
    // transmit character until NULL is reached
    while(*s > 0) uart_putchar(*s++);
}
 
 
// Receive a string
void uart_getLine(unsigned char* buf, uint8_t n)
{
    uint8_t bufIdx = 0;
    unsigned char c;
 
    // while received character is not carriage return
    // and end of buffer has not been reached
    do
    {
        // receive character
        c = uart_getchar();
 
        // store character in buffer
        buf[bufIdx++] = c;
    }
    while((bufIdx < n) && (c != '\n'));
 
    // ensure buffer is null terminated
    buf[bufIdx] = 0;
}
 
uint8_t uart_available(void) {
    uint8_t head, tail;
 
    head = rx_buffer_head;
    tail = rx_buffer_tail;
    if ( head >= tail ) return head - tail;
    return RX_BUFFER_SIZE + head - tail;
}
 
// Transmit Interrupt
ISR(USART_UDRE_vect) {
    uint8_t i;
 
    if ( tx_buffer_head == tx_buffer_tail ) {
        // buffer is empty, disable transmit interrupt
        UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    }
    else {
        i = tx_buffer_tail + 1;
        if ( i >= TX_BUFFER_SIZE ) i = 0;
        UDR0 = tx_buffer[i];
        tx_buffer_tail = i;
    }
}
 
// Receive Interrupt
ISR(USART_RX_vect) {
    uint8_t c, i;
 
    c = UDR0;
    i = rx_buffer_head + 1;
    if ( i >= RX_BUFFER_SIZE ) i = 0;
    if ( i != rx_buffer_tail ) {
        rx_buffer[i] = c;
        rx_buffer_head = i;
    }
}