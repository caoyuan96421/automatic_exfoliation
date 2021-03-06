#define F_CPU 16000000	// CPU frequency for ATMega328P
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdarg.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <stdlib.h>

/*Constants*/
/*If you want to change any parameters, please do it here*/
#define BAUD_RATE	9600L				/*Baud rate for serial port output (only for debugging)*/
#define BUF_SIZE	512					/*Buffer size for serial port output*/
#define ANTIJITTER	10					/*ms. Anti-jitter test time interval*/
#define LED_BLINK	300 				/*ms. Blinking rate of the indicator LED when in AUTO modes.*/
#define SPEED_UPDATE_INTERVAL	50 		/*ms. Interval of updating speed using readings from the ADC. Note: if this is too small, you will hear funny noises from the motor.*/
#define SPEED_EPS	10					/*Update speed only if current reading is different from previous one by larger than this value. Note: if this is too small, you will hear funny noises from the motor because the speed keeps changing.*/
#define FAST_STEP	128					/*Steps for fast mode*/
#define SLOW_STEP	1					/*Steps for slow mode. Note: these two numbers are fixed by hardware connection.*/
#define FORWARD_STEP	-1				/*Direction settings*/
#define BACKWARD_STEP	1
#define SYSTEM_LENGTH	6600000L		/*Step count for the entire range*/
#define CRITICAL_POS	6300000L		/*Step count for reacheable range. For safety, this should be smaller than SYSTEM_LENGTH*/
#define SPEED_MIN_FAST		400			/*Minimum pulse period for FAST mode. This corresponds to fastest movement*/
#define SPEED_MAX_FAST		3500		/*Maximum pulse period for FAST mode. This corresponds to slowest movement*/
#define SPEED_MIN_SLOW		4			/*The same thing for slow movement*/
#define SPEED_MAX_SLOW		1000

/*Pinout definitions*/
/*Please make sure this definition matches the schematic and hardware*/
/*LIMIT switch*/
#define PIN_LIMIT	PIND
#define DDR_LIMIT	DDRD
#define NUM_LIMIT	2
/*BUTTON switch*/
#define PIN_BUTTON PIND
#define DDR_BUTTON DDRD
#define NUM_BUTTON	3
/*AUTO switch*/
#define PIN_AUTO	PINC
#define DDR_AUTO	DDRC
#define NUM_AUTO	3
/*PULSE output*/
#define PORT_PULSE  PORTB
#define DDR_PULSE  DDRB
#define NUM_PULSE   1
/*MICROSTEP switch*/
#define PIN_MICROSTEP PIND
#define DDR_MICROSTEP DDRD
#define NUM_MICROSTEP  5
/*DIROUT output*/
#define PORT_DIROUT PORTB
#define DDR_DIROUT DDRB
#define NUM_DIROUT  0
/*DIR switch*/
#define PIN_DIRIN  PIND
#define DDR_DIRIN  DDRD
#define NUM_DIRIN   6
/*SPEED potentiometer*/
#define PORT_SPEED  PORTC
#define DDR_SPEED  DDRC
#define NUM_SPEED   0
/*LED positive*/
#define PORT_LEDP   PORTC
#define DDR_LEDP   DDRC
#define NUM_LEDP   1
/*LED negative*/
#define PORT_LEDN   PORTC
#define DDR_LEDN   DDRC
#define NUM_LEDN    2
/*DEBUG switch (jumpers)*/
#define PIN_DEBUG	PIND
#define PORT_DEBUG	PORTD
#define DDR_DEBUG	DDRD
#define NUM_DEBUG	7


/*Utility macros (self-explainary)*/
#define SET(name) PORT_##name |= (1 << (NUM_##name))			/*Set a pin to output high*/
#define CLR(name) PORT_##name &=~(1 << (NUM_##name))			/*Clr a pin to output low*/
#define READ(name) ((PIN_##name & (1<< (NUM_##name))) != 0)		/*Read the value from a pin*/
#define MODE_OUT(name) DDR_##name |= (1<< (NUM_##name))			/*Set a pin to output mode*/
#define MODE_IN(name) DDR_##name &= ~(1<< (NUM_##name))			/*Set a pin to input mode*/

#define LED_OFF CLR(LEDP);CLR(LEDN)								/*Turn off the double-color LED*/
#define LED_ON(state)					\
{										\
	if(state==GREEN){					\
		SET(LEDP);CLR(LEDN);			\
	}									\
	else if(state==RED){				\
		CLR(LEDP);SET(LEDN);			\
	}									\
}																/*Turn on the double-color LED to a specific state. can be GREEN or RED*/
																/*Please change here if the colors are the opposite of what you want*/

#define ISDEBUG		(READ(DEBUG) == 0)							/*If we're in debugging mode?*/

/*State Machine definitions*/
/*State of BUTTON*/
typedef enum{
	RELEASED=0,
	FALL_DETECTED,
	RISE_DETECTED,
	PRESSED,
}BUTTON_State;

/*State of LED*/
typedef enum{
	OFF,
	RED,
	GREEN,
}LED_State;

/*State of the system*/
typedef enum{
	IDLE,
	AUTO_FORWARD,
	AUTO_BACKWARD,
	MANUAL_FORWARD,
	MANUAL_BACKWARD,
}SYSTEM_State;

/*Please change here if the motor is moving in the opposite directions of what you want*/
enum {
	FORWARD = 1,
	BACKWARD = 0,	
};

/*Please change here if the FAST and SLOW modes are not correct*/
enum {
	FAST = 0,
	SLOW = 1,	
};

/*Global variables*/
volatile unsigned long long milliseconds=0;		// Global time-recording variable
volatile unsigned char buffer[BUF_SIZE];		// Buffer for Serial printing
unsigned char buffer2[BUF_SIZE];
volatile unsigned char buffer_head=0, buffer_tail=0;

volatile unsigned int speed=0, oldspeed;				// Direct sample value from the ADC, range from 0-1023. 0=fastest (shortest time), 1023=slowest (longest time). Initialized at fastest for self-check
/*State machines*/
volatile BUTTON_State button_state=RELEASED;
volatile LED_State led_state=OFF;
volatile SYSTEM_State system_state=IDLE, oldstate=IDLE;
/*Global timers for various event processing. Keep as long long for safety.*/
volatile unsigned long long button_timer;
volatile unsigned long long led_timer;
volatile unsigned long long speed_update_timer=0;

volatile unsigned char isfast, oldisfast;		// microstep mode. 1 = fast, 0 = slow

volatile signed long position;						// current position (step count)
volatile signed int step;							// increase of position at every pulse. positive->forward, negative->backward
volatile unsigned char clear_position=0;			// if set as true, position will be set to 0 at next tick update (typically 1ms).
volatile unsigned char self_check=0;				// Self-calibration on every start up

/*Some declarations*/
void timer2CallBack();
void buttonPressedCallBack();
void buttonReleasedCallBack();
void limitTriggeredCallBack();

/*Functions for serial debug printing*/
void _write_serial(unsigned char *s){
	unsigned int count=0;
	for(;*s!='\0';s++,count++){
		buffer[buffer_tail] = *s;
		buffer_tail++;
		if(buffer_tail == BUF_SIZE)
		buffer_tail = 0;
	}
	if(count==0)return;
	UCSR0B |= _BV(TXEN0) | _BV(UDRIE0);	// Start transmitting
}

/*Use as normal printf. Safe even within interrupt environment.*/
void uprintf(char *s, ...){
	va_list args;
	va_start(args,s);
	vsprintf((char*)buffer2,s,args);
	va_end(args);
	_write_serial(buffer2);
}

/*Interrupt Service Routines (ISRs)*/
/* ISR for Timer 2 ticking. Triggers every 1ms, therefore used as global 1ms timing */
ISR(TIMER2_COMPA_vect, ISR_BLOCK){	
	milliseconds++;
	timer2CallBack();	/*Do perioidic work that needs to be done every 1ms*/
}
/* ISR for Timer 0 overflow event. */
/* Some explanations: Timer 0 is hard wired to the output of Timer 1 (see schematic), in order to count how many pulses have been sent out by the controller to the motor. */
/* Timer 0 is a 8-bit timer/counter and increases on each pulse from Timer 1, therefore it overflows when the counter reaches 256 and calls this ISR.*/
/* Whenever this ISR is called, we know that the controller has sent out another 256 pulses, and therefore we know how many steps the motor has moved.*/
ISR(TIMER0_OVF_vect,ISR_BLOCK){ 
	/*Each overflow means 256 steps*/
	position += 256L * step;
	if((position < 0 && system_state == AUTO_FORWARD) || (position > CRITICAL_POS && system_state == AUTO_BACKWARD)){
		/* If position<0 or position>CRITICAL_POS, it means we should have reached the limit*/
		/* For safety, exit AUTO modes immediately */
		system_state = IDLE;
		if(ISDEBUG)
			uprintf("IDLE\r\n");
	}
}

/* ISR for USART (serial port debugging), send next char */
ISR(USART_UDRE_vect,ISR_BLOCK){ 
	if(buffer_head != buffer_tail){
		UDR0 = buffer[buffer_head];
		buffer_head++;
		if(buffer_head == BUF_SIZE)
		buffer_head = 0;
	}
	else{
		UCSR0B &= ~(_BV(TXEN0) | _BV(UDRIE0)) ;	// Transmit finished. Stop it
	}
}

/* ISR for ADC finished event */
/* The ADC starts next sampling immediately after it finishes the last one*/
ISR(ADC_vect,ISR_BLOCK){
	if(milliseconds - speed_update_timer > SPEED_UPDATE_INTERVAL){
		/*We update the speed every SPEED_UPDATE_INTERVAL*/
		
		/*If the relationship of the position on the potentiometer and the speed is opposite to what you expect, use the other line in the following two lines*/
		speed = 1023 - (ADCL | ((unsigned int)ADCH << 8));
		//speed = (ADCL | ((unsigned int)ADCH << 8));
		
		speed_update_timer = milliseconds;
	}
}

/* ISR for LIMIT switch */
ISR(INT0_vect,ISR_BLOCK){
	limitTriggeredCallBack();	/*Go to the callback for details*/
}

/* ISR for BUTTON event */
/* Note: this ISR is called on both pressing and releasing the button */
ISR(INT1_vect,ISR_BLOCK){
	if(ISDEBUG)
		uprintf("BUTTON event at %ld\r\n",milliseconds);
	/*Update the state machine*/
	switch(button_state){
		case RELEASED:
			if(READ(BUTTON) == 0){
				button_timer = milliseconds;				// start timer
				button_state = FALL_DETECTED;
			}
			break;
		case PRESSED:
			if(READ(BUTTON) == 1){
				button_timer = milliseconds;
				button_state = RISE_DETECTED;
			}
			break;
		default:
			break;
	}
	/*After ANTIJITTER milliseconds, the state machine will be updated in timer2CallBack()*/
}

/* System initialization*/
void init(){
	long div;
	
	// Pin mode init
	MODE_OUT(PULSE);
	MODE_OUT(DIROUT);
	MODE_OUT(LEDP);
	MODE_OUT(LEDN);
	MODE_IN(LIMIT);
	MODE_IN(BUTTON);
	MODE_IN(AUTO);
	MODE_IN(SPEED);
	MODE_IN(MICROSTEP);
	MODE_IN(DEBUG);
	
	// Pin state init
	SET(PULSE);
	SET(DIROUT);
	SET(DEBUG);
	LED_OFF;
	
	// Timer 0 init
	TCCR0A |= 0;						// Mode: Normal (overflows at 256)
	TCCR0B |= _BV(CS02) | _BV(CS01);	// Clock source: External T0 pin (hard wired to output of T1 (PULSE), so this timer is synchronized with PULSE output).
	TIMSK0 |= _BV(TOIE0);				// Generate interrupt when overflows
	TCNT0 = 0;
	
	// Timer 1 init
	// This timer is used for generating PWM signal for driving stepper motor driver, ST-M5045
	TCCR1A |= _BV(COM1A0);	// Mode: CTC.  TOP: OCR1A. Toggle PULSE output when counter value and the value in OCR1A matches.
	TCCR1B |= _BV(WGM12); 	// Prescaler: by 8, that is 0.5us per count (will start later)
	TCNT1 = 0;				// Reset timer counter
	
	// Timer 2 init
	// This timer is used for global timing (1ms resolution)
	TCCR2A |= _BV(WGM21);					// Mode: CTC (automatically clears when counter reaches OCR2A)
	TCCR2B |= _BV(CS22);					// Prescaler: by 64
	TIMSK2 |= _BV(OCIE2A);
	OCR2A = 249;							// Frequency = F_CPU / Prescale / (OCR2A + 1) = 1kHz
	TCNT2 = 0;								// Reset timer counter
	
	// External Interrupt (LIMIT and BUTTON) setup
	EICRA |= _BV(ISC10) | _BV(ISC01);	// LIMIT only triggers on FALLING edge, BUTTON triggers on BOTH edges
	EIMSK |= _BV(INT1) | _BV(INT0);		// Enable INT1 and INT0
	
	// Setup a Serial for debugging
	div = (((long)F_CPU) >> 4 ) / BAUD_RATE - 1;
	UBRR0L = div & 0xFF;
	UBRR0H = div >> 8;
	
	// Setup the Analog-Digital Conversion (ADC) 
	ADMUX |= _BV(REFS0);	// AVCC as reference, that is Vcc (5V power supply)
	ADCSRA |= _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);	// Enable ADC, Enable Interrupt and autotrigger. Clock is F_CPU divided by 128 = 125kHz
	ADCSRB |= 0; // Free running ADC
	
	position = 0;
	system_state = IDLE;
	
	self_check = 1;	// ensure that button is not reactive when self-checking
	sei();
	
	_delay_ms(1000);
	uprintf("\r\n\r\nSystem started at %d\r\n",milliseconds);

}

/*Panic state. Wait for Watch dog to recover*/
void panic(){
	cli();
	while(1){	/*Loop forever*/
		LED_ON(RED);
		_delay_ms(100);
		LED_OFF;
		_delay_ms(100);
	}
}

void calibrate(){
	uprintf("Entering self-calibration process.\r\n");
	if(READ(LIMIT) == 0){// If the LIMIT is already triggered at startup
		uprintf("LIMIT triggered at start up.\r\n");
		system_state = MANUAL_BACKWARD; // We move backward to make some space until LIMIT is released
		uprintf("Moving back...");
		while(READ(LIMIT)==0){
			_delay_ms(100);
		}
		_delay_ms(500);
		uprintf("Done.\r\n");
		system_state = IDLE;
	}
	_delay_ms(1000);
	uprintf("Moving forward to calibrate...\r\n");
	unsigned long long timestarted = milliseconds;
	system_state = MANUAL_FORWARD;
	while((milliseconds - timestarted < 40000L) && system_state != IDLE);	// Move forward until LIMIT triggered
	if(milliseconds - timestarted >= 40000L){			// something must has gone wrong after 40s
		TCCR1B &= 0xF8;		// Force stop output
		uprintf("Error when calibrating.\r\nPlease restart system.\r\n");
		uprintf("Error state: %d\n", (int)system_state);
		_delay_ms(100);
		panic();
	}
	clear_position = 1;	// Clear position counter on next tick (at most after 1ms).
	TCNT0 = 0;		// clear timer counter for steps
	_delay_ms(5);
	uprintf("Calibration finished. Position = %ld\r\n",position);
	self_check = 0;	// Finish calibration
}

/*Main loop. NOTE: THIS IS IN AN INTERRUPT ENVIRONMENT. ALL TIMING AND DELAYS WILL *NOT* WORK.*/
/*This main loop is driven by Timer 2 1kHz ticks.*/
void timer2CallBack(){
	//if(ISDEBUG && milliseconds % 1000 == 0){
	//	uprintf("Time: %ld\r\n",milliseconds);
	//}
	/*Update LED status*/
	/*Current state machine: Blink green when AUTO_FORWARD, Blink red when AUTO_BACKWARD, Green-on when MANUAL_FORWARD, Red-on when MANUAL_BACKWARD, OFF otherwise*/
	switch(led_state){
		case OFF:
			switch(system_state){
				case MANUAL_BACKWARD:
					led_state = RED;
					LED_ON(RED);
					break;
				case MANUAL_FORWARD:
					led_state = GREEN;
					LED_ON(GREEN);
					break;
				case AUTO_BACKWARD:
					if(milliseconds - led_timer >= LED_BLINK){
						led_state = RED;
						led_timer = milliseconds;
						LED_ON(RED);
					}
					break;
				case AUTO_FORWARD:
					if(milliseconds - led_timer >= LED_BLINK){
						led_state = GREEN;
						led_timer = milliseconds;
						LED_ON(GREEN);
					}
					break;
				default:
					break;
			}
			break;
		case GREEN:
			if(system_state == AUTO_FORWARD){
				if(milliseconds - led_timer >= LED_BLINK){
					led_state = OFF;
					led_timer = milliseconds;
					LED_OFF;
				}
			}
			else if(system_state != MANUAL_FORWARD){
				led_state = OFF;
				LED_OFF;
			}
			break;
		case RED:
			if(system_state == AUTO_BACKWARD){
				if(milliseconds - led_timer > LED_BLINK){
					led_state = OFF;
					led_timer = milliseconds;
					LED_OFF;
				}
			}
			else if(system_state != MANUAL_BACKWARD){
				led_state = OFF;
				LED_OFF;
			}
			break;
	}
	/*Update BUTTON state machine if ANTIJITTER ms have passed*/
	switch(button_state){
		case FALL_DETECTED:
			if(milliseconds - button_timer >= ANTIJITTER){
				if(READ(BUTTON) == 0){
					button_state = PRESSED;					// Truely Pressed
					buttonPressedCallBack();
				}
				else{
					button_state = RELEASED;				// noise
				}
			}
			break;
		case RISE_DETECTED:
			if(milliseconds - button_timer >= ANTIJITTER){
				if(READ(BUTTON) == 1){
					button_state = RELEASED;
					buttonReleasedCallBack();
				}
				else{
					button_state = PRESSED;
				}
			}
			break;
		default:
			break;
	}
	/*Update microstep status*/
	isfast = (READ(MICROSTEP) == FAST);
	/*Update Pulse output*/
	if(system_state != oldstate || abs(speed - oldspeed) > SPEED_EPS || isfast != oldisfast){// only take action when something changed
		//if(ISDEBUG)
		//	uprintf("Update Output\r\n");
		if(isfast)
			step = FAST_STEP;
		else
			step = SLOW_STEP;
		
		switch(system_state){
			case AUTO_FORWARD:
			case MANUAL_FORWARD:
				CLR(DIROUT);
				step *= FORWARD_STEP;
				goto l1;
			case AUTO_BACKWARD:
			case MANUAL_BACKWARD:
				SET(DIROUT);
				step *= BACKWARD_STEP;
				/*Common for all*/
	l1:			
				if(isfast){
					OCR1A = (long)(SPEED_MAX_FAST - SPEED_MIN_FAST) * speed / 1024 + SPEED_MIN_FAST;
				}
				else{
					OCR1A = (long)(SPEED_MAX_SLOW - SPEED_MIN_SLOW) * speed / 1024 + SPEED_MIN_SLOW;
				}
				TCCR1B |= _BV(CS11);	// Start Timer 1
				break;
			case IDLE:
				TCCR1B &= 0xF8;	// Stop Timer 1, stop output
				position += (long)TCNT0 * step;	// sum up remaining steps
				TCNT0 = 0;
				if(clear_position){ // Clear position if the flag is set
					clear_position = 0;
					position = 0;
					uprintf("Position zeroed\r\n");
				}
				if(ISDEBUG){
					uprintf("Speed at %d\r\n",speed);
					uprintf("Pos at %ld\r\n",position);
				}
		}
		oldstate = system_state;
		oldspeed = speed;
		oldisfast = isfast;
	}
}

/*Called to update the system state machine when BUTTON is pressed. NOTE: THIS IS IN AN INTERRUPT ENVIRONMENT. ALL TIMING AND DELAYS WILL *NOT* WORK. */
void buttonPressedCallBack(){
	if(ISDEBUG)
		uprintf("Button Pressed\r\n");
	if(self_check)
		return;		// BUTTON do not have any effect when self-checking
	switch(system_state){
		case IDLE:
			if(READ(AUTO) == 0){ // AUTO modes
				if(READ(DIRIN) == FORWARD){ //AUTO_FOWARD
					if(READ(LIMIT) == 1 && position >= 0){//Ensure it's valid to move
						system_state = AUTO_FORWARD;
						led_timer = milliseconds;
						led_state = GREEN;
						LED_ON(GREEN);						// Ensure the LED blinks immediately
						if(ISDEBUG)
							uprintf("AUTO_FORWARD\r\n");
					}
					else{
						if(ISDEBUG){
							uprintf("Limit reached. Please use manual mode.\r\n");
						}
					}
				}
				else{ //AUTO_BACKWARD
					if(position <= CRITICAL_POS){//Ensure it's valid to move
						system_state = AUTO_BACKWARD;
						led_timer = milliseconds;
						led_state = RED;
						LED_ON(RED);
						if(ISDEBUG)
							uprintf("AUTO_BACKWARD\r\n");
					}
					else{
						if(ISDEBUG){
							uprintf("Limit reached. Please use manual mode.\r\n");
						}
					}
				}
			}
			else{// MANUAL modes
				if(READ(DIRIN) == FORWARD){ // MANUAL_FORWARD ignore any restrictions on position and LIMIT. Just MOVE!
					system_state = MANUAL_FORWARD;
					if(ISDEBUG)
					uprintf("MANUAL_FORWARD\r\n");
				}
				else{ // MANUAL_BACKWARD
					system_state = MANUAL_BACKWARD;
					if(ISDEBUG)
						uprintf("MANUAL_BACKWARD\r\n");
				}
			}
			break;
		case AUTO_FORWARD:
		case AUTO_BACKWARD:
			system_state = IDLE;			// Press the second time in AUTO modes -> stop
			if(ISDEBUG)
				uprintf("IDLE\r\n");
			break;
		default:
			break;
	}
}

/*Called when BUTTON is released. NOTE: THIS IS IN AN INTERRUPT ENVIRONMENT. ALL TIMING AND DELAYS WILL *NOT* WORK. */
void buttonReleasedCallBack(){
	if(ISDEBUG)
		uprintf("Button Released\r\n");
	if(self_check)
		return;		// Button do not have effect when self-checking
	switch(system_state){
		case MANUAL_BACKWARD:
		case MANUAL_FORWARD:
			system_state = IDLE;			// Release in manual mode -> stop
			if(ISDEBUG)
				uprintf("IDLE\r\n");
			break;
		default:
			break;
	}
}

/*Called when LIMIT is triggered. NOTE: THIS IS IN AN INTERRUPT ENVIRONMENT. ALL TIMING AND DELAYS WILL *NOT* WORK. */
void limitTriggeredCallBack(){
	if(ISDEBUG)
		uprintf("Limit triggered at %d\r\n", milliseconds);
	switch(system_state){
		case AUTO_FORWARD:
		case MANUAL_FORWARD:	// Force stop if moving forward
			system_state = IDLE;
			clear_position = 1;	// clear position at next update
			if(ISDEBUG)
				uprintf("IDLE\r\n");
			break;
		default:			// If we're moving backward, do nothing
			break;
	}
}

/*Program entry point*/
int main()
{
	init();
	calibrate();
	ADCSRA |= _BV(ADSC); // start first sampling
	wdt_enable(WDTO_250MS);	// Enable watch dog timer. The dog gets hungry every 250 ms. If he's not fed, he'll bite and reset the system.
	while(1){
		wdt_reset();	// feed the hungry dog
		_delay_ms(5);
		// do nothing, just wait for interrupts. Everything else is dealt in ISRs
	}
	return 0;
}