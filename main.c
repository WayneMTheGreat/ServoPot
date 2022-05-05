#include "TM4C123.h"

void initPLL(uint32_t divisor);
void initPWM(void);

void initADC(void);

uint16_t map(uint16_t input, uint16_t inputMin, uint16_t inputMax, uint16_t desiredMin, uint16_t desiredMax);

/*This program is designed to generate a 50Hz PWM signal to drive a servo motor. Right now
this program is very specific in regards to which frequencies of bus clock and PWM signal
are allowed. Future updates will make it more generic.*/


int main()
	{
		volatile uint16_t ADCOutput = 0;
		volatile uint16_t convertedValue = 0;
		
		initPLL((uint32_t)0x1F800000);
		initPWM();
		initADC();
		
		while(1)
			{
				
				ADC0->PSSI |= 0x08;
				while((ADC0->RIS&0x08)==0){}
				ADCOutput = ADC0->SSFIFO3;
				ADC0->ISC |= 0x01;
				//convertedValue = map(ADCOutput,0,4095,12500,14900);
				convertedValue = map(ADCOutput,0,4095,15080,12200);
				PWM0->_0_CMPA = convertedValue;
		
		
		   } //allow the signal to continually generate.
}

void initPLL(uint32_t divisor){
	//CODE BASED ON VALVNO EMBEDDED BOOK.
		SYSCTL->RCC2 |= 0x80000000;
		SYSCTL->RCC2 |= 0x00000800;
		SYSCTL->RCC &= ~0x000007C0;
		SYSCTL->RCC |= 0x00000540;
		SYSCTL->RCC2 &= ~(0x00000070);
		//SYSCTL->RCC2 |= 0x40000000; //SET SYSDIVLSB TO BE USED.
		SYSCTL->RCC2 &= ~(0x1FC00000);
		SYSCTL->RCC2 |= divisor; //3125MHz? Yes.
	while((SYSCTL->RIS&0X00000040)== 0){} //WAIT FOR PLLRIS BIT.
		SYSCTL->RCC2 &= ~(0x00000800); //LET THE PLL RUN.
}

void initPWM(void){
	SYSCTL->RCGCGPIO |= 0x02;
		while(!(SYSCTL->PRGPIO)){}
		SYSCTL->RCGCPWM |= 0x11; //START THE PWM CLOCK
		while(!(SYSCTL->PRPWM)){}
		GPIOB->DIR |= 0xC1;
		GPIOB->DEN |= 0xC1;
		GPIOB->AFSEL = 0xC0;
		GPIOB->PCTL = (4<<28) | (4<<24); //PMC bits begin at 24 and 28 for PB6 & PB7 respectively                                                              
				

		SYSCTL->RCC |= 1<<20; //Set the bit that says the PWM Divisor is to be used.
		//SYSCTL->RCC &= ~(1<<17); //Change clock divisor to 2, but it is currently not working. 0x00100542 THIS IS WHAT RCC SHOULD BE.
		SYSCTL->RCC = 0x01900542; //why do i have to force the address to get the divisor to work?
		PWM0->CTL = 0;
		PWM0->_0_GENA |= 0x008C;
		PWM0->_0_GENB	|= 0x080C;
		PWM0->_0_LOAD |= 0x3D08; //3.125MHz BUS CLOCK NEEDS 15624 SAMPLES.
		PWM0->_0_CMPA |= 0x30D4; //RETURN TO RIGHT 2mS PULSE. 12500
		//PWM0->_0_CMPA |= 0x347C; //RETURN TO CENTER 1.5mS PULSE.
		PWM0->_0_CMPB |= 0x3A98; //RETURN TO LEFT 1mS PULSE.14061;14100;14200;14300;14400;14500;14600;14900
		//PWM0->_0_CMPB |= 0x52;
		PWM0->CTL |= 0x01;
		PWM0->_0_CTL = 0x03;
		PWM0->ENABLE = 0x03;
}

void initADC(void){
	
	  SYSCTL->RCGCADC |= 0x01;
		SYSCTL->RCGCGPIO |= 0x10;
		GPIOE->AFSEL |= 0x08;
		GPIOE->DEN &= ~(0x08);
		GPIOE->AMSEL |= 0x08;
		//GPIOE->ADCCTL |= 0x08;
		ADC0->ACTSS &= ~(0x08);
		ADC0->EMUX &= ~(1<<0);
		ADC0->SSMUX3 |= 0x01;
		ADC0->SSCTL3 |= 0x04;
		ADC0->ACTSS |= 0x08;
	
}

uint16_t map(uint16_t input, uint16_t inputMin, uint16_t inputMax, uint16_t desiredMin, uint16_t desiredMax)
{
	
	return (input - inputMin) * (desiredMax - desiredMin) / (inputMax - inputMin) + desiredMin;
	
}
