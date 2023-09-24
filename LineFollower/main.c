/*
 * Lab10.c
 *
 * Created: 4/4/2023 9:22:03 AM
 * Author : Lisa
 */

#define F_CPU 20000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/delay.h>
#include <stdint.h> //for u_int32

volatile int PWM_Inv = 0b11;
volatile int T0_CLK_1 = 0b001;
volatile int T0_CLK_64 = 0b011;
volatile int speed = 44; // 192

volatile uint8_t sensor;
volatile uint8_t sensor_read;
volatile uint8_t prev_pinc;
volatile uint8_t cur_pinc;
volatile uint16_t LD_thresh;
volatile uint32_t cur_count;

volatile uint8_t a;
volatile uint8_t b;

// Clear the k bit of n
uint16_t clear_bit(uint16_t n, int k)
{
   return (n & (~(1 << k)));
}

// Set the k bit of n
uint16_t set_bit(uint16_t n, int k)
{
   return (n | (1 << k));
}

// Toggle the k bit of n
uint16_t toggle_bit(uint16_t n, int k)
{
   return (n ^ (1 << k));
}

// Reading sensor
// bit_read: (0-4) the bit of the sensor array that is ready to be read
// count: the count value for the sensor bit
void ready_to_read(int bit_read, int count)
{
   if (count < LD_thresh)
   {
      sensor = set_bit(sensor, bit_read); // set to light
   }
   sensor_read = set_bit(sensor_read, bit_read);
   return;
}

void stop_timers()
{
   TCCR1B = 0;
}

// Stop motors
void brake_motors()
{
   // 100% inverting
   OCR0B = 0;
   OCR2B = 0;
}

void set_up_motors()
{
   DDRD = set_bit(DDRD, 5);
   DDRD = set_bit(DDRD, 6);
   DDRD = set_bit(DDRD, 3);
   DDRB = set_bit(DDRB, 3);

   PORTD |= (1 << 6);
   PORTB |= (1 << 3);

   TCCR0A = PWM_Inv << COM0B0 | 1 << WGM00 | 1 << WGM01; // toggle pd5
   TCCR0B = T0_CLK_1;
   OCR0B = 0;

   TCCR2A = PWM_Inv << COM2B0 | 1 << WGM20 | 1 << WGM21; // toggle pd3
   TCCR2B = T0_CLK_1;
   OCR2B = 0;
   // brake_motors();
}

void straight()
{
   // 25% forward
   OCR0B = speed; // 192
   OCR2B = speed;
}

void slight_left()
{
   OCR0B = speed + a; // left
   OCR2B = speed - a; // right
}

void slight_right()
{
   OCR0B = speed - a; // left
   OCR2B = speed + a; // right
}

void left()
{
   OCR0B = speed + b; // left
   OCR2B = speed - b; // right
}

void right()
{
   OCR0B = speed - b; // left
   OCR2B = speed + b; // right
}

// Make it stop
void terminate()
{
   brake_motors();
   while (1)
   {
   }
}

// Timer interrupt: sensor timed out. Set dark.
ISR(TIMER1_COMPA_vect)
{
   sensor_read = 0x1f;
}

// Pin change interrupt on PORTC: sensor sensed. Ready to be read.
ISR(PCINT1_vect)
{
   prev_pinc = cur_pinc;
   cur_pinc = PINC;
   cur_count = TCNT1; // input capture register that reads from tcnt1
   if (cur_pinc != prev_pinc)
   {
      if ((cur_pinc & (1 << 0)) != (prev_pinc & (1 << 0)))
      {
         ready_to_read(0, cur_count);
      }
      if ((cur_pinc & (1 << 1)) != (prev_pinc & (1 << 1)))
      {
         ready_to_read(1, cur_count);
      }
      if ((cur_pinc & (1 << 2)) != (prev_pinc & (1 << 2)))
      {
         ready_to_read(2, cur_count);
      }
      if ((cur_pinc & (1 << 3)) != (prev_pinc & (1 << 3)))
      {
         ready_to_read(3, cur_count);
      }
      if ((cur_pinc & (1 << 4)) != (prev_pinc & (1 << 4)))
      {
         ready_to_read(4, cur_count);
      }
   }
}

// Pin change interrupt on PORTB: button pressed
ISR(PCINT0_vect)
{
   terminate();
}

void senseprev()
{
   // Define thresholds
   DDRC = 0x3f;
   // uint32_t timeout_th = 100;
   uint32_t timeout_th = 0x0730;
   LD_thresh = 0x0350;

   // Start of sensing
   PCMSK1 = 0; // mask all pc1 interrupts
   // TIMSK1=0;
   PORTC = clear_bit(PORTC, 5);

   sensor_read = 0;
   sensor = 0;

   TCCR1A = (0 << COM1A1) | (1 << COM1A0); // toggle OC1A on compare match
   OCR1A = timeout_th;
   TCCR1B = 0;

   // Charge sensors
   PORTC = 0x1f;
   _delay_us(10);
   prev_pinc = 0x1f; // should be all 1s
   cur_pinc = 0x1f;  // should be all 1s

   PORTC = set_bit(PORTC, 5);

   TCCR1B = (0 << CS12) | (1 << CS11) | (0 << CS10); // starts timer with prescale 8
   DDRC &= 0b11100000;                               // start discharge

   PCICR = (1 << PCIE1);
   PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11) | (1 << PCINT12);

   TIMSK1 = 1 << OCIE1A;
   sei();

   while (sensor_read < 0x1f)
   {
   }
   TCNT1 = 0;
   stop_timers();
   return;
}

// Read from all 5 sensors once. Stores in sensor
void sense()
{
   // Define thresholds
   DDRC = 0x3f;
   uint32_t timeout_th = 0x0800;
   LD_thresh = 0x0300;

   // Start of sensing
   cli();
   PORTC = clear_bit(PORTC, 5);

   sensor_read = 0;
   sensor = 0;

   TCCR1A = (0 << COM1A1) | (1 << COM1A0); // toggle OC1A on compare match
   OCR1A = timeout_th;
   TCCR1B = 0;

   // Charge sensors
   PORTC = 0x1f;
   _delay_us(10);
   prev_pinc = 0x1f; // should be all 1s
   cur_pinc = 0x1f;  // should be all 1s

   PORTC = set_bit(PORTC, 5);

   TCCR1B = (0 << CS12) | (1 << CS11) | (0 << CS10); // starts timer with prescale 8
   DDRC &= 0b11100000;                               // start discharge

   PCICR = (1 << PCIE1);
   PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11) | (1 << PCINT12);

   TIMSK1 = 1 << OCIE1A;
   sei();

   while (sensor_read < 0x1f)
   {
   }
   TCNT1 = 0;
   stop_timers();
   return;
}

int main(void)
{
   // Set up motors pins as output
   set_up_motors();
   a = 2;
   b = 10;

   _delay_ms(3000);

   while (1)
   {
      senseprev(); // sensor should be changed
      volatile int test = 0;
      if (sensor == 0b00010)
      {
         right();
      }
      else if (sensor == 0b11001)
      {
         slight_right();
      }
      else if (sensor == 0b10111)
      {
         left();
      }
      else if (sensor == 0b10011)
      {
         slight_left();
      }
      else if (sensor == 0b11011)
      {
         straight();
      }
      else
      {
         brake_motors();
      }
   }
}
