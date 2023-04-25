/*
 * MazeSolver.c
 * Created: 4/16/2023 5:58:08 PM
 * Author : Lisa Liu
 */ 

#define SIZE 50 //satic array size
#define F_CPU 20000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/delay.h>
#include <stdbool.h>
#include <stdint.h>

/* Constants as defined by Atmel ATmega328P */
volatile int PWM_Inv = 0b11;
volatile int PWM_NonInv = 0b10;
volatile int T0_CLK_1 = 0b001;
volatile int T0_CLK_8 = 0b010;
volatile int T0_CLK_64 = 0b011;

/* Modes of operation */
volatile bool calibrating = false;
volatile bool testing = true;
volatile bool solving = false;

/* Sensor variables */
volatile uint8_t sensor;
volatile uint8_t prev_sensor;
volatile uint8_t cur_sensor;
volatile uint8_t sensor_read;
volatile uint8_t prev_pinc;
volatile uint8_t cur_pinc;
volatile uint16_t LD_thresh;
volatile uint32_t cur_count;
volatile uint64_t total = 0; //IR sensors calibration result. Used in "calibrating" mode

/* Motor speed parameters */
volatile int speed = 24;
volatile int forward_speed = 30;
volatile uint8_t a = 6;

/* Storage of decisions */
char decision_stack[SIZE];
char solved_stack[SIZE];
int dec_top = 0; 	//Index of next available spot in decision_stack
int sol_top = 0; 	//Index of next available spot in solved_stack
int sol_in = 0; 	//Index of the most recent correct decision made. Used in "solving" mode.

/* Predeclaration of helper functions */
void brake_motors();
void inch_forward();
void slight_left();
void slight_right();
void wiggle_left();
void wiggle_right();
void turn_left();
void turn_right();
void straight();
void light_green_led();
void light_red_led();
void alt_signal();
// ----------------------------- BIT OPERATIONS -------------------------------
/*
 * Function: clear_bit
 * Clear the k bit of n
 * 
 * k: index of bit to clear
 * n: value to clear bit
 *
 * returns: value n after k bit is cleared
 */
uint16_t clear_bit(uint16_t n, int k)
{
	return (n&(~(1<<k)));
}

/*
 * Function: set_bit
 * Set the k bit of n
 * 
 * k: index of bit to clear
 * n: value to set bit
 *
 * returns: value n after k bit is set
 */
uint16_t set_bit(uint16_t n, int k)
{
	return (n|(1<<k));
}

// ----------------------------- TIMER OPERATIONS -------------------------------
/*
 * Function: stop_sensor_timer
 * Stops the sensor timer
 * 
 * returns: none
 */
void stop_sensor_timer()
{
	TCCR1B=0;
}

// ----------------------------- STACK OPERATIONS -------------------------------
/*
 * Function: stack_error
 * Flashes LEDs on the Pololu to indicate an error in the stack has occured
 *
 * returns: none
 */
void stack_error()
{
	light_red_led();
	_delay_ms(200);
	light_red_led();
	_delay_ms(200);
	light_red_led();
	_delay_ms(200);
	light_red_led();
	_delay_ms(200);
	light_red_led();
	_delay_ms(200);
	while(1){};
}

/*
 * Function: push
 * Pushes an element onto the end of the stack
 *
 * c: value to push
 * stack: stack to push to
 * top: index of the next availble spot in stack
 * 
 * returns: none
 */
void push(char c, char stack[SIZE], int* top)
{
	if ((*top)>=SIZE)
	{
		stack_error();
		while(1){}
	}
	else
	{
		stack[(*top)]=c;
		(*top)++;
	}
	
}

/*
 * Function: show_stack
 * Makes the robot physically show the contents of the stack through wiggles
 *
 * stack: stack to show
 * top: index of the next availble spot in stack
 * 
 * returns: none
 */
void show_stack(char stack[SIZE], int top)
{
	if (top>=SIZE||top<0)
	{
		stack_error();
	}
	int i=0;
	while(i<top)
	{
		uint8_t d = stack[i];
		if(d=='L')
		{
			wiggle_left();
		}
		else if(d=='S')
		{
			inch_forward();
		}
		else if(d=='R')
		{
			wiggle_right();
		}
		else if(d=='U')
		{
			turn_right();
			turn_right();
		}
		_delay_ms(1000);
		i++;
	}
}

/*
 * Function: consolidate_dec_stack
 * Populates the solved_stack with correct decisions based on turns taken stored in decision_stack
 *
 * returns: none
 */
void consolidate_dec_stack()
{
	if (dec_top>=SIZE||dec_top==0||sol_top!=0)
	{
		stack_error();
		while(1){}
	}
	//1 or 2 decisions, cannot reduce
	if(dec_top==1)
	{
		push(decision_stack[0],solved_stack,&sol_top);
	}
	else if(dec_top==2)
	{
		push(decision_stack[0],solved_stack,&sol_top);
		push(decision_stack[1],solved_stack,&sol_top);
	}
	else
	{
		char d1;
		char d2;
		char d3;
		int i=0;
		while(i<dec_top-2)
		{
			d1=decision_stack[i];
			d2=decision_stack[i+1];
			d3=decision_stack[i+2];

			//LUL -> S
			if((d1=='L')&&(d2=='U')&&(d3=='L'))
			{
				solved_stack[sol_top]='S';
				i+=3;
			}
			//LUS -> R
			else if((d1=='L')&&(d2=='U')&&(d3=='S'))
			{
				solved_stack[sol_top]='R';
				i+=3;
			}
			//SUL -> R
			else if((d1=='S')&&(d2=='U')&&(d3=='L'))
			{
				solved_stack[sol_top]='R';
				i+=3;
			}
			else
			{
				solved_stack[sol_top]=d1;
				i++;
			}
			sol_top++;
		}

		//flush
		if(i==dec_top-1)
		{
			solved_stack[sol_top]=decision_stack[i];
		}
		else if(i==dec_top-2)
		{
			solved_stack[sol_top]=decision_stack[i];
			sol_top++;
			i++;
			solved_stack[sol_top]=decision_stack[i];
		}
		sol_top++;
	}
}

/*
 * Function: make_decision
 * Turn the robot based on the decision stored in solved_stack
 *
 * returns: none
 */
void make_decision()
{
	alt_signal();
	if (sol_top<=0||sol_in>=sol_top)
	{
		stack_error();
		while(1){}
	}
	char dec;
	dec=solved_stack[sol_in];
	//LUL -> S
	if(dec=='L')
	{
		turn_left();
	}
	//LUS -> R
	else if(dec=='S')
	{
		straight();
	}
	//SUL -> R
	else if(dec=='R')
	{
		turn_right();
	}
	else
	{
		stack_error();
	}
	sol_in++;
}

// ----------------------------- SENSING -------------------------------
/*
 * Function: ready_to_calibrate
 * Called by ISR when the sensor has discharged. Used in calibration
 *
 * bit_read: bit corresponding to the sensor that has discharged
 * count: time it took the sensor to discharge
 * 
 * returns: none
 */
void ready_to_calibrate(int bit_read, int count)
{
	total+=count;
	sensor_read=set_bit(sensor_read, bit_read);
	return;
}

/*
 * Function: calibrate
 * Calibrates the robot's 5 sensors by averaging the time sensors took to discharge
 * 
 * returns: the average time of sensor discharge
 */
double calibrate()
{
	DDRC = 0x3f;
		
	//Start of sensing
	PCMSK1 = 0; //mask all pc1 interrupts
	PORTC=clear_bit(PORTC, 5);
		
	sensor_read = 0;
	sensor = 0;
		
	//Charge sensors
	PORTC=0x1f;
	_delay_us(10);
	prev_pinc = 0x1f;
	cur_pinc = 0x1f;
		
	PORTC=set_bit(PORTC,5);
		
	DDRC&=0b11100000; //start discharge
	PORTC&=0b11100000; //disable pullup resistors
		
	PCICR = (1<<PCIE1);
	PCMSK1|=(1<<PCINT8)|(1<<PCINT9)|(1<<PCINT10)|(1<<PCINT11)|(1<<PCINT12);
	sei();
		
	while(sensor_read<0x1f){}
	return total/5.0;	
}

/*
 * Function: sense
 * Senses from the 5 sensors and stores in "sensor"
 * 
 * returns: none
 */
void sense()
{
	//Define thresholds
	DDRC = 0x3f;
	uint32_t timeout_th = 0x1000; //0x0800
	LD_thresh = 0x02c0; //0x0350;
	
	//Start of sensing
	PCMSK1 = 0; //mask all pc1 interrupts
	PORTC=clear_bit(PORTC, 5);
	
	sensor_read = 0;
	sensor = 0;
	
	TCCR1A = (0<<COM1A1) | (1<<COM1A0); //toggle OC1A on compare match
	OCR1A=timeout_th;
	TCCR1B = 0;
	
	//Charge sensors
	PORTC=0x1f;
	_delay_us(10);
	prev_pinc = 0x1f;
	cur_pinc = 0x1f;
	
	PORTC=set_bit(PORTC,5);
	
	TCCR1B = (0<<CS12) | (1<<CS11) | (0<<CS10); //starts timer with prescale 8
	DDRC&=0b11100000; //start discharge
	PORTC&=0b11100000; //disable pullup resistors
	
	PCICR = (1<<PCIE1);
	PCMSK1|=(1<<PCINT8)|(1<<PCINT9)|(1<<PCINT10)|(1<<PCINT11)|(1<<PCINT12);
	if(!calibrating)
	{
		TIMSK1 = 1<<OCIE1A;
	}
	sei();
	
	while(sensor_read<0x1f){}
	TCNT1=0;
	stop_sensor_timer();
	if(calibrating)
	{
		volatile double avg = total/5.0; //put breakpoiint here when calibrating
	}
	return;
}

/*
 * Function: sense_further
 * Inches forward and senses again to determine type of intersection
 * 
 * returns: none
 */
void sense_further()
{
	brake_motors();
	prev_sensor = sensor;
	//_delay_ms(300);
	inch_forward();
	sense();
	cur_sensor = sensor;
	//_delay_ms(300);
}


// ----------------------------- DEBUGGING -------------------------------
/*
 * Function: light_red_led
 * Flashes red LED
 * 
 * returns: none
 */
void light_red_led()
{
	DDRD=set_bit(DDRD, 1);
	PORTD=set_bit(PORTD, 1);
	_delay_ms(500);
	PORTD=clear_bit(PORTD, 1);
}

/*
 * Function: light_green_led
 * Flashes green LED
 * 
 * returns: none
 */
void light_green_led()
{
	DDRD=set_bit(DDRD, 7);
	PORTD=set_bit(PORTD, 7);
	_delay_ms(500);
	PORTD=clear_bit(PORTD, 7);
}

/*
 * Function: light_green_led
 * Flashes red LED twice
 * 
 * returns: none
 */
void red_signal()
{
	light_red_led();
	_delay_ms(100);
	light_red_led();
	_delay_ms(100);
}

/*
 * Function: light_green_led
 * Flashes green LED twice
 * 
 * returns: none
 */
void green_signal()
{
	light_green_led();
	_delay_ms(100);
	light_green_led();
	_delay_ms(100);
}

/*
 * Function: light_green_led
 * Flashes green and red LED alternately
 * 
 * returns: none
 */
void alt_signal()
{
	light_green_led();
	_delay_ms(100);
	light_red_led();
	_delay_ms(100);
	light_green_led();
	_delay_ms(100);
	light_red_led();
	_delay_ms(100);
}

/*
 * Function: wiggle_left
 * Wiggles the robot to the left
 * 
 * returns: none
 */
void wiggle_left()
{
	OCR0A = speed+20;
	OCR2B = speed+20;
	_delay_ms(300);
	brake_motors();
	OCR0B = speed+20;
	OCR2A = speed+20;
	_delay_ms(300);
	brake_motors();
}

/*
 * Function: wiggle_right
 * Wiggles the robot to the right
 * 
 * returns: none
 */
void wiggle_right()
{
	OCR0B = speed+20;
	OCR2A = speed+20;
	_delay_ms(300);
	brake_motors();
	OCR0A = speed+20;
	OCR2B = speed+20;
	_delay_ms(300);
	brake_motors();
}

/*
 * Function: spin
 * Spins the robot
 * 
 * returns: none
 */
void spin()
{
	OCR0B = speed+20;
	OCR2A = speed+20;
	_delay_ms(1000);
}

// ---------------------------------- 3PI MOTORS  -----------------------------------
/*
 * Function: set_up_motors
 * Configures PWM mode for the ports that control the motors on the Pololu
 * 
 * returns: none
 */
void set_up_motors()
{
	DDRD=set_bit(DDRD, 5);
	DDRD=set_bit(DDRD, 6);
	DDRD=set_bit(DDRD, 3);
	DDRB=set_bit(DDRB, 3);
	
	TCCR0A= PWM_Inv << COM0B0 | PWM_Inv << COM0A0 | 1 << WGM00 | 1 << WGM01 ; // toggle pd5 and pd6
	TCCR0B= T0_CLK_8;
	OCR0B= 0;
	OCR0A= 0;
	
	TCCR2A= PWM_Inv << COM2B0 | PWM_Inv << COM2A0 | 1 << WGM20 | 1 << WGM21; //toggle pd3 and pb3
	TCCR2B= T0_CLK_8;
	OCR2B= 0;
	OCR2A= 0;
}

void straight()
{
	OCR0B= forward_speed;
	OCR2B= forward_speed;
}

void slight_left()
{
	OCR0B= speed+a; //left
	OCR2B= speed-a; //right
}

void slight_right()
{
	OCR0B= speed-a; //left
	OCR2B= speed+a; //right
}

void left()
{
	OCR0B= speed+b; //left
	OCR2B= speed-b; //right
}

void right()
{
	OCR0B= speed-b; //left
	OCR2B= speed+b; //right
}

void inch_forward()
{
	straight();
	_delay_ms(380);
	brake_motors();
}

void brake_motors()
{
	//100% inverting
	OCR0B= 0;
	OCR2B= 0;
	OCR0A= 0;
	OCR2A= 0;
}

/*
 * Function: turn_left
 * Turns the Pololu 90 degrees left
 * 
 * returns: none
 */
void turn_left()
{
	OCR0A = speed;
	OCR2B = speed;
	_delay_ms(760);
	brake_motors();
}

/*
 * Function: turn_right
 * Turns the Pololu 90 degrees right
 * 
 * returns: none
 */
void turn_right()
{
	OCR0B = speed;
	OCR2A = speed;
	_delay_ms(770);
	brake_motors();
}

/*
 * Function: turn_back
 * Makes a U-turn
 * 
 * returns: none
 */
void turn_back()
{
	OCR0B = speed;
	OCR2A = speed;
	_delay_ms(1480);
	brake_motors();
}

// ----------------------------- INTERRUPTS -----------------------------
/*
 * ISR: Timer1 Output Compare A
 * Sensors have timed out; set sensors as reading dark
 * 
 * returns: none
 */
ISR(TIMER1_COMPA_vect)
{
	sensor_read=0x1f;
}

/*
 * ISR: Pin Change Interrupt
 * One or more sensors have discharged; determine if light or dark
 * 
 * returns: none
 */
ISR(PCINT1_vect)
{
	prev_pinc = cur_pinc;
	cur_pinc = PINC;
	cur_count = TCNT1;
	if (cur_pinc!=prev_pinc){
		if((cur_pinc&(1<<0)) != (prev_pinc&(1<<0))){
			if (cur_count<(LD_thresh+0x0070)){
				sensor|=(1<<0); //set to light
			}
			sensor_read|=(1<<0);
		}
		if((cur_pinc&(1<<1)) != (prev_pinc&(1<<1))){
			if (cur_count<LD_thresh){
				sensor|=(1<<1); //set to light
			}
			sensor_read|=(1<<1);
		}
		if((cur_pinc&(1<<2)) != (prev_pinc&(1<<2))){
			if (cur_count<LD_thresh){
				sensor|=(1<<2); //set to light
			}
			sensor_read|=(1<<2);
		}
		if((cur_pinc&(1<<3)) != (prev_pinc&(1<<3))){
			if (cur_count<LD_thresh){
				sensor|=(1<<3); //set to light
			}
			sensor_read|=(1<<3);
		}
		if((cur_pinc&(1<<4)) != (prev_pinc&(1<<4))){
			if (cur_count<(LD_thresh+0x0070)){
				sensor|=(1<<4); //set to light
			}
			sensor_read|=(1<<4);
		}
	}
}


int main(void)
{		
	if(calibrating)
	{
		set_up_motors();
		sense();
	}
	else if(testing)
	{
		set_up_motors();
		_delay_ms(1000);
		while(1)
		{
			sense(); //sensor should be changed
			if(sensor==0b00011||sensor==0b00111)
			{
				
				sense_further(); //cur_sensor should be changed
				if(cur_sensor==0b11011||cur_sensor==0b10001||cur_sensor==0b10011||cur_sensor==0b11001||cur_sensor==0b10111)
				{
					if(solving)
					{
						make_decision();
					}
					else
					{
						//decision: forward, could go right
						green_signal();
						push('S', decision_stack, &dec_top);
						straight();
					}
				}
				else if(cur_sensor==0b11111)
				{
					turn_right();
				}
				else
				{
					red_signal();
				}
			}
			else if(sensor==0b11000||sensor==0b11100)
			{
				
				sense_further(); //cur_sensor should be changed
				if(cur_sensor==0b11011||cur_sensor==0b10001||cur_sensor==0b10011||cur_sensor==0b11001||cur_sensor==0b11101||cur_sensor==0b10111)
				{
					if(solving)
					{
						make_decision();
					}
					else
					{
						//decision: left, could go forward
						green_signal();
						push('L', decision_stack, &dec_top);
						turn_left();
					}
				}
				else if(cur_sensor==0b11111)
				{
					turn_left();
				}
				else
				{
					alt_signal();
				}
			}
			else if(sensor==0b00000)
			{
				sense_further(); //cur_sensor should be changed
				if(cur_sensor==0b11111||cur_sensor==0b11011||cur_sensor==0b10001)
				{
					if(solving)
					{
						make_decision();
					}
					else
					{
						green_signal();
						//decision: turn left, could go right
						push('L', decision_stack, &dec_top);
						turn_left();
					}
			
				}
				else if(cur_sensor==0b00000)
				{
					if(solving)
					{
						while(1);
					}
					else
					{
						//DONE
						brake_motors();
						green_signal();
						green_signal();

						_delay_ms(3000);

						consolidate_dec_stack();
					
						show_stack(solved_stack, sol_top);
						alt_signal();
						_delay_ms(3000);

						alt_signal();
						solving=true;
						main(); //rerun program under "solving" mode
					}
				}
				else
				{
					red_signal();
				}
			}
			else if(sensor==0b11111)
			{
				sense_further(); //cur_sensor should be changed
				if(cur_sensor==0b11111)
				{
					green_signal();
					turn_back();
					push('U', decision_stack, &dec_top);
				}
				else
				{
					red_signal();
				}
			}
			else if(sensor==0b10111)
			{
				slight_left();
			}
			else if(sensor==0b11101)
			{
				slight_right();
			}
			else
			{
				straight();
			}
		}
	}	
}