/*
 * pir_control.c
 * RC intrrnal Clock 9,6Mhz
 * Created: 23.08.2016 15:53:47
 * Author : Mazinov
 * PORTB3 Bulb out, log 1;
 * PORTB0 PIR-sensor, zero active level;
 * PORTB1 Light switch, zero active level;
 */ 
#ifndef F_CPU
	#define F_CPU 9600000L
#endif	
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>	
#include <avr/sfr_defs.h>	
#define DEBOUNCE 2000
#define PIRDELAY 3000
#define PHOTOOFFSET 5000
#define CONSTANT_FOR_HOURS 7
#define AC_SWITCHOFF_DELAY 5
#define MILLISECONDS_IN_SECONDS 999
#define SECONDS_IN_MINUTES 59
#define MINUTES_IN_HOURS 59
#define BULB 3
#define PIR PINB0
#define AC_SWITCH PINB1
#define FIRST_ADC_INPUT 1
#define LAST_ADC_INPUT 2
//Bandgap Voltage Reference: Off;
#define ADC_VREF_TYPE ((0<<REFS0) | (0<<ADLAR))

//Global varibles;

volatile unsigned int adcScanningData[LAST_ADC_INPUT-FIRST_ADC_INPUT+1];
//Counters;
volatile unsigned int on=0, timeCounter=0, timeCounterDown=0;
volatile unsigned int photoSeensor;
volatile unsigned int sensivity;
//Switch flags;
volatile unsigned char keyFlag = 0;
volatile unsigned int milliSeconds = 0, seconds = 0, minutes=0, hours=0;
		
//Prototype of functions;


//Peripherial initialisation;
void MCU_init (void);
// ADC data read function;
void adcInputsread (void);
//Main logic function;
void mainLogic (void);


//Free running ADC interrupt;

ISR(ADC_vect)
{
	static unsigned char input_index=0;
	//Read the AD conversion result;
	adcScanningData[input_index]=ADCW;
	//Select next ADC input;
	if (++input_index > (LAST_ADC_INPUT-FIRST_ADC_INPUT))
	input_index=0;
	ADMUX=(FIRST_ADC_INPUT | ADC_VREF_TYPE)+input_index;
	//Delay needed for the stabilization of the ADC input voltage;
	_delay_us(10);
	//Start the AD conversion;
	ADCSRA|=(1<<ADSC); //ADCSRA|=0x40
}

//Timer0 compare interrupt;

ISR (TIM0_COMPA_vect)
{
	//Counter for light switch - is 25 milliseconds;
	const short INTERRUPT_DELAY = 25;
	static unsigned char acNetCounter = INTERRUPT_DELAY;
	
	if(PINB & (1<<AC_SWITCH))
	{
		//Read 1 state - switch is active;
		//Time checking;
		if(acNetCounter)
		{
			//Time is not run;
			acNetCounter --;
		}
		else
		{
			//15 mseconds switch is on;
			keyFlag = 0;
			//Set the counter again;
			acNetCounter = INTERRUPT_DELAY;
		}
	}
	else
	{
		//Read 0 - Switch is on;
		keyFlag = 1;
		//Set the counter again;
		acNetCounter = INTERRUPT_DELAY;
	}
	//Start with switch / debounnce;
	if (keyFlag)
	{
		timeCounter++;

		if (timeCounter>=DEBOUNCE)
		{
			timeCounter = DEBOUNCE;
			milliSeconds++;

			if (milliSeconds==MILLISECONDS_IN_SECONDS)
			{
				milliSeconds=0;
				seconds++;

				if (seconds==SECONDS_IN_MINUTES)
				{
					seconds=0;
					minutes++;

					if (minutes==MINUTES_IN_HOURS)
					{
						minutes=0;
						hours++;
					
						if (hours > AC_SWITCHOFF_DELAY)
						{
							hours=CONSTANT_FOR_HOURS;
						}
					}
				}
			}
		}
	}
	else
	{
		milliSeconds=0; seconds=0, minutes=0, hours=0; timeCounter=0;  
	}

	//Switch on delay and Pir sensor debounce;
	if ((!(PINB & (1<<PIR))) && (keyFlag==0)&&(photoSeensor>=sensivity))
			on++;
	else	on=0;
	
	if ((!(PINB & (1<<PIR))) && (photoSeensor<sensivity) && ((PINB & (1<<BULB))))
			timeCounterDown++;
	else    timeCounterDown = 0;
	//Reset the counter;
	TCNT0=0x00;
}


int main(void)
{
	MCU_init();
	//PORTB &= ~(1<<3);

	//Main program;

	while (1)
	{

		//Main logic function;
		mainLogic();
		
	}
	
}

void MCU_init (void)
{
	//Crystal Oscillator division factor: 1;
	#pragma optsize-
	CLKPR=(1<<CLKPCE);
	CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
	#ifdef _OPTIMIZE_SIZE_
	#pragma optsize+
	#endif

	//Input/Output Ports initialization;
	//Port B initialization;
	//Function: Bit5=In Bit4=In Bit3=Out Bit2=In Bit1=In Bit0=In;
	DDRB=(0<<DDB5) | (0<<DDB4) | (1<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
	// State: Bit5=T Bit4=P Bit3=1 Bit2=T Bit1=P Bit0=T;
	PORTB=(0<<PORTB5) | (1<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (1<<PORTB1) | (1<<PORTB0);

	//ADC initialization;
	//ADC Clock frequency: 150,000 kHz;
	//ADC Bandgap Voltage Reference: Off;
	//ADC Auto Trigger Source: Free Running;
	//Digital input buffers on ADC0: On, ADC1: On, ADC2: On, ADC3: On;
	DIDR0=(0<<ADC0D) | (0<<ADC2D) | (0<<ADC3D) | (0<<ADC1D);
	ADMUX=FIRST_ADC_INPUT | ADC_VREF_TYPE;
	ADCSRA=(1<<ADEN) | (1<<ADSC) | (1<<ADATE) | (0<<ADIF) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (0<<ADPS0);
	ADCSRB=(0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);

	//Timer/Counter 0 initialization;
	//Clock source: System Clock;
	//Clock value: 150,000 kHz;
	//Mode: CTC top=OCR0A;
	//Timer Period: 1 ms;
	TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (0<<WGM00);
	TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (1<<CS00);
	TCNT0=0x00;
	OCR0A=0x95;
	OCR0B=0x00;

	//Timer/Counter 0 Interrupt(s) initialization;
	TIMSK0=(0<<OCIE0B) | (1<<OCIE0A) | (0<<TOIE0);

	sei ();
}

void adcInputsread (void)
{
	//First channel value, PORTB2;
	photoSeensor=adcScanningData[0];
	//Second channel value, PORTB4;
	sensivity=adcScanningData[1];
}

void mainLogic (void)
{
	//Switch state
	unsigned char switchOn;
	//Forbid global interrupts;
	cli ();
	switchOn = keyFlag;
	//Allow global interrupts;
	sei ();
	// ADC read function(10bit);
	adcInputsread();
	//AC switch priority;
	while (switchOn && (timeCounter>=DEBOUNCE) && (hours<AC_SWITCHOFF_DELAY))
	{
		PORTB |=(1<<BULB);
	}
	
		//PIR=0,Switch=1;
		if ((photoSeensor>=sensivity) && (!(PINB & (1<<PIR)))  && (on>=PIRDELAY))
		{
			//Out is log 1;
			PORTB |=(1<<BULB);
		}

			else if ((!(PINB & (1<<PIR))) && (photoSeensor<sensivity) &&
					(PINB & (1<<BULB)) && (timeCounterDown>PHOTOOFFSET))

			{
						PORTB &=~(1<<BULB);
			}
	
				else if ((!(PINB & (1<<PIR))) && (!switchOn||switchOn) &&
						(photoSeensor>=sensivity) && ((PINB & (1<<BULB))))

				{
							PORTB |=(1<<PINB3);
				}

					else if (PINB & (1<<PIR))
						{
							PORTB &=~(1<<BULB);
						}
	
	
}
