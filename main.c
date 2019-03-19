/*
Stepper motor position control
Components: 
* ATMEGA328P (Arduino UNO)
* 28BYJ-48 STEPPER MOTOR
* ULN2003A DRIVER
* 10K POTENTIOMETER
* 4 DIGIT, 7 SEGMENT DISPLAY
* 4 x 1K RESISTORS
* CONNECTING WIRES	
Iacob Casian George		
*/

#define F_CPU 16000000UL
#define TCCR0A _SFR_IO8(0x24)

#define Stepper_IN1 0 //PB0 - D8
#define Stepper_IN2 1 //PB1 - D9
#define Stepper_IN3 2 //PB2 - D10
#define Stepper_IN4 3 //PB3 - D11

#include <avr/io.h>
#include <avr/interrupt.h>

//variables used for adc reading
int adc_l = 0; // low byte
int adc_h = 0; // high byte
int adc = 0; // final value
int adc_mean = 0; //mean used for more stable analog to digital conversions

const int t = 3; //the motor performs 1 step/t ms
int ms = 0; //milliseconds counter variable

int digit = 0; //display digit selection
int stepCounter = 0; 

int direction = 0;
int x = 0;
int dx = 0;
const int errorHigh = 1;
const int errorLow = -1;
int degrees = 0;

void display(short position, int number)
{
	PORTC = 0b11111100;
	PORTD = 0;
	
	switch(position)
	{
		case 1:
		PORTC &= ~(1<<2); //digit 1 -> PC2 - A2
		break;
		
		case 2:
		PORTC &= ~(1<<3); // digit 2 -> PC3 - A3
		break;
		
		case 3:
		PORTC &= ~(1<<4); //digit 3 -> PC4 - A4
		break;
		
		case 4:
		PORTC &= ~(1<<5); //digit 4 -> PC5 - A5
		break;
	}
	
	switch(number)
	{
		case 0:
		PORTD = 0b00111111; //0
		break;
		
		case 1:
		PORTD = 0b00000110; //1
		break;
		
		case 2:
		PORTD = 0b10011011; //2
		break;
		
		case 3:
		PORTD = 0b10001111; //3
		break;
		
		case 4:
		PORTD = 0b10100110; //4
		break;
		
		case 5:
		PORTD = 0b10101101; //5
		break;
		
		case 6:
		PORTD = 0b10111101; //6
		break;
		
		case 7:
		PORTD = 0b00000111; //7
		break;
		
		case 8:
		PORTD = 0b10111111; //8
		break;
		
		case 9:
		PORTD = 0b10101111; //9
		break;
		
		default:
		PORTD = 0b10001001;
		break;
	}
}



void oneStepClockWise()
{
	PORTB = 0;
	switch (stepCounter)
	{   
		case 0:
		PORTB |= (1 << Stepper_IN1);
		break;

		case 1:
		PORTB |= (1 << Stepper_IN2);
		break;

		case 2:
		PORTB |= (1 << Stepper_IN3);
		break;

		case 3:
		PORTB |= (1 << Stepper_IN4);
		break;
	}
	stepCounter++;
	if(stepCounter > 3)
	stepCounter = 0;
	x++;
}

void oneStepTrigonometric()
{
	PORTB = 0;
	switch (stepCounter)
	{
		case 0:
		PORTB |= (1 << Stepper_IN1);
		break;

		case 1:
		PORTB |= (1 << Stepper_IN2);
		break;

		case 2:
		PORTB |= (1 << Stepper_IN3);
		break;

		case 3:
		PORTB |= (1 << Stepper_IN4);
		break;
	}
	stepCounter--;
	if(stepCounter <0)
	stepCounter = 3;
	x--;
}


void init_timer0()
{
	TCCR0A = 0b10000010;  //Clear OC0A on Compare Match, CTC
	TCCR0B = 0b00000011;  //
	TCNT0 = 0;
	OCR0A = 125;
	TIMSK0 |= 0b00000010; //Timer/Counter0, Output Compare A Match Interrupt Enable
}

void init_adc()
{
	ADMUX = 0b01000000;  //Vcc reference
	ADCSRA = 0b10000111; //bit 7 -> activate adc, bit 2:0 ->prescaler 128
	ADCSRA |= (1<<3);    //ADIE
}


void get_adc()
{
	ADMUX &= 0b11100000; //refresh channel
	ADMUX |= 0; //analog pin selection -> A0
	ADCSRA |= (1<<6); //start conversion
}


ISR(ADC_vect)
{
	adc_l = ADCL;
	adc_h = ADCH;
	adc = (adc_h<<8)|adc_l;
	adc*=2;
	/*
	the motor position (steps) can be between [0,2048]
	adc takes values [0, 1023]
	adc will also be the new position so it is multiplied by 2
	*/
	adc_mean = (float)adc_mean*0.95+(float)adc*0.05; // adc_mean is stabilizing the value read from the potentiometer
	/*
	adc_mean holds value for future position
	x holds value for current position
	dx is the displacement
	*/
	dx = adc_mean - x;
	degrees =((float)adc_mean*180)/1023;
	if(dx >=errorHigh)
	{
		direction = 1;
	}
	else if(dx <= errorLow)
	{
		direction = 0;		
	}
	else
	{
		direction = 2;
	}
}

ISR(TIMER0_COMPA_vect)
{
	//
	digit++;
	switch(digit)
	{
		case 1:
		display(1, (degrees/1000)%10);
		break;
		
		case 2:
		display(2, (degrees/100)%10);
		break;
		
		case 3:
		display(3, (degrees/10)%10);
		break;
		
		case 4:
		display(4, degrees%10);
		digit = 0;
		break;
	}
	
		if(ms == t)
		{
			get_adc();			
			switch(direction)
			{
				case 0 :
				oneStepTrigonometric();
				break;
				
				case 1:
				oneStepClockWise();
				break;
				
				default:
				break;
			}
			ms = 0;
		}
		else
		ms++;	
}

int main(void)
{
	DDRC |= 0b00111100; //selection pins for 4 digit display
	DDRD = 0b11111111; //7 segments pins
	/*
	a - PD0 - D0
	b - PD1 - D1
	c - PD2 - D2
	d - PD3 - D3
	e - PD4 - D4
	f - PD5 - D5
	g - PD7 - D7
	*/
	DDRB |= 0b00001111; // stepper pins OUTPUT
	SREG |= (1<<7); //global interrupt enable
	DDRB |= (1<<4); //decimal point
	init_timer0();
	init_adc();
	while (1)
	{
	}
}

