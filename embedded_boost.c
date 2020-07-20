/* embedded_boost.c
 *
 *  Author: Steve Gunn & Klaus-Peter Zauner & Przemyslaw Forys
 * Licence: This work is licensed under the Creative Commons Attribution License.
 *          View this license at http://creativecommons.org/about/licenses/
 *   Notes:
 *          - Use with a terminal program
 *
 *          - F_CPU must be defined to match the clock frequency
 *
 *          - Compile with the options to enable floating point
 *            numbers in printf():
 *               -Wl,-u,vfprintf -lprintf_flt -lm
 *
 *          - Pin assignment:
 *            | Port | Pin | Use                         |
 *            |------+-----+-----------------------------|
 *            | A    | PA0 | Voltage at load             |
 *            | D    | PD0 | Host connection TX (orange) |
 *            | D    | PD1 | Host connection RX (yellow) |
 *            | D    | PD7 | PWM out to drive MOSFET     |
 *	      | B    | PB0 | Button to control V_target  |
 *            | B    | PB1 | Button to control V_target  |
 *	      | B    | PB2 | Output to diode 1  		 |
 *            | B    | PB3 | Output to diode 2  		 |
 
 */

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

#define DELAY_MS      500
#define BDRATE_BAUD  9600
#define MULTIPLIER 50

#define ADCREF_V     3.3
#define ADCMAXREAD   1023   /* 10 bit ADC */
#define Vin 1.5

#define PWM_DUTY_MAX 240    /* 94% duty cycle */
#define R1 22000
#define R2 4700


typedef struct
{
	double dState;
	double iState;
	double iMax, iMin;
	double iGain, pGain, dGain;

}SPid;



void init_stdio2uart0(void);
int uputchar0(char c, FILE *stream);
int ugetchar0(FILE *stream);

void init_adc(void);
double v_load(void);

void init_pwm(void);
void pwm_duty(uint8_t x);
double updatePID(SPid *pid, double error, double v_out);

double v_target;
double v_out;
double error;
double k;
uint8_t d = PWM_DUTY_MAX/2;



int main(void)
{

	SPid pid={0};
	DDRB = 0x00;
	v_target = 5;
	int f=1;
	DDRB |=  _BV(2);
	DDRB |=  _BV(3);
	DDRA &=  ~_BV(1);
	
	init_stdio2uart0();
	init_pwm();
	init_adc();

	pid.iGain = 0.1;
	pid.pGain = 0.1;
	pid.dGain = 0.1; //to be determined more precisely during testing phase
	pid.iMax = 1;
	pid.iMin = 0.01;


	for(;;) {
		 
		v_out = v_load()*(R2+R1)/R2;// calculating v_out based on voltage divider formula
		if(v_out>14 ||v_out/R2>1 )
			d=0;
		error = v_out-v_target;
		if(f==1)
		{
		
			if(!(fabs(error)<(v_out-Vin)*0.05 )) // checking if the result is within tolerance, if not, adjusting it
			{
				k = updatePID(&pid, error, v_out);
				d -= MULTIPLIER*k;
				pwm_duty(d);
			}
		}
		
		if((fabs(error)>0.4)) // diode singalling the output is close to desired one
		{
			PORTB |= _BV(2);
			PORTB &= ~_BV(3);
		}
		else
		{
			PORTB |= _BV(3);
			PORTB &= ~_BV(2);
		}
		

		printf("%f/%f\n",v_out,v_target);
		if(((PINB & _BV(PB1))!=0) && v_target<10) // increasing/decreasing v_target using a button
		{
			v_target+=0.1;
			_delay_ms(40);
		}
		if(((PINB & _BV(PB0))!=0) && v_target>2)
		{
			v_target-=0.1;
			_delay_ms(40);
		}
	   
	}
}

int uputchar0(char c, FILE *stream)
{
	if (c == '\n') uputchar0('\r', stream);
	while (!(UCSR0A & _BV(UDRE0)));
	UDR0 = c;
	return c;
}

int ugetchar0(FILE *stream)
{
	while(!(UCSR0A & _BV(RXC0)));
	return UDR0;
}

void init_stdio2uart0(void)
{
	/* Configure UART0 baud rate, one start bit, 8-bit, no parity and one stop bit */
	UBRR0H = (F_CPU/(BDRATE_BAUD*16L)-1) >> 8;
	UBRR0L = (F_CPU/(BDRATE_BAUD*16L)-1);
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);
	UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);

	/* Setup new streams for input and output */
	static FILE uout = FDEV_SETUP_STREAM(uputchar0, NULL, _FDEV_SETUP_WRITE);
	static FILE uin = FDEV_SETUP_STREAM(NULL, ugetchar0, _FDEV_SETUP_READ);

	/* Redirect all standard streams to UART0 */
	stdout = &uout;
	stderr = &uout;
	stdin = &uin;
}


void init_adc (void)
{
    /* REFSx = 0 : Select AREF as reference
     * ADLAR = 0 : Right shift result
     *  MUXx = 0 : Default to channel 0
     */
    ADMUX = 0x00;
    /*  ADEN = 1 : Enable the ADC
     * ADPS2 = 1 : Configure ADC prescaler
     * ADPS1 = 1 : F_ADC = F_CPU / 64
     * ADPS0 = 0 :       = 187.5 kHz
     */
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1);
}


double v_load(void)
{
     uint16_t adcread;

     /* Start single conversion */
     ADCSRA |= _BV ( ADSC );
     /* Wait for conversion to complete */
     while ( ADCSRA & _BV ( ADSC ) );
     adcread = ADC;

     //printf("ADC=%4d", adcread);

     return (double) (adcread * ADCREF_V/ADCMAXREAD);
}



void init_pwm(void)
{
    /* TIMER 2 */
    DDRD |= _BV(PD6); /* PWM out */
    DDRD |= _BV(PD7); /* inv. PWM out */


    TCCR2A = _BV(WGM20) | /* fast PWM/MAX */
	     _BV(WGM21) |
	     _BV(COM2A1); /* A output */
    TCCR2B = _BV(CS20);   /* no prescaling */
}


/* Adjust PWM duty cycle
   Keep in mind this is not monotonic
   a 100% duty cycle has no switching
   and consequently will not boost.
*/
void pwm_duty(uint8_t x)
{
    x = x > PWM_DUTY_MAX ? PWM_DUTY_MAX : x;

    //printf("PWM=%3u  ==>  ", x);

    OCR2A = x;
}

double updatePID(SPid * pid, double error, double v_out)
{
	
	//code adapted from "PID without a PhD" by Tim Wescott
	double pTerm, dTerm, iTerm;

	pTerm = pid->pGain*error;

	pid->iState += error;
	if(pid->iState > pid->iMax )
		pid->iState = pid->iMax;
	else if(pid->iState < pid->iMin)
		pid->iState = pid->iMin;

	iTerm = pid->iGain * pid->iState;

	dTerm = pid->dGain * (pid->dState - v_out);
	pid->dState = v_out;

	return pTerm + dTerm + iTerm;


}

