#include "stm32f407xx.h"

//define pin and port for lcd pins
#define LCDD0pin 0
#define LCDD0Port GPIOD
#define LCDD1pin 1
#define LCDD1Port GPIOD
#define LCDD2pin 2
#define LCDD2Port GPIOD
#define LCDD3pin 3
#define LCDD3Port GPIOD
#define LCDD4pin 10
#define LCDD4Port GPIOC
#define LCDD5pin 5
#define LCDD5Port GPIOB
#define LCDD6pin 6
#define LCDD6Port GPIOB
#define LCDD7pin 7
#define LCDD7Port GPIOB
volatile int j;
void wait(){
	volatile int i = 200000;
	while(i--);
}
void SendBitToPortAndPin(GPIO_TypeDef *port, int pinNumber, uint8_t bitState){
	if(bitState){
			port->BSRR |= (1 << pinNumber);
	}else{
			port->BSRR |= (1 << (pinNumber+16));
	}
}
void SendACharacterToTheLCDDataPins(char character){
	SendBitToPortAndPin(LCDD0Port, LCDD0pin, character & (1 << 0));
	SendBitToPortAndPin(LCDD1Port, LCDD1pin, character & (1 << 1));
	SendBitToPortAndPin(LCDD2Port, LCDD2pin, character & (1 << 2));
	SendBitToPortAndPin(LCDD3Port, LCDD3pin, character & (1 << 3));
	SendBitToPortAndPin(LCDD4Port, LCDD4pin, character & (1 << 4));
	SendBitToPortAndPin(LCDD5Port, LCDD5pin, character & (1 << 5));
	SendBitToPortAndPin(LCDD6Port, LCDD6pin, character & (1 << 6));
	SendBitToPortAndPin(LCDD7Port, LCDD7pin, character & (1 << 7));
}

void  setPortAndPinForOutput(GPIO_TypeDef *port,int pin){
	
	// Enable clock for the ports in use using the RCC struct
	if( port == GPIOA){
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	}
	if( port == GPIOB){
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	}
	if( port == GPIOC){
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	}
	if( port == GPIOD){
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	}
	if( port == GPIOE){
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	}
	if( port == GPIOF){
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	}
	if( port == GPIOG){
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
	}	
	if( port == GPIOH){
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
	}
	if( port == GPIOI){
			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
	}
	//configure pin for output using the required registers
	//Moder Register
	port->MODER |= (1 << pin*2);
	port->MODER &= ~(1 << ((pin*2)+1) );
	//Otype Register
	port->OTYPER &= ~(1 << pin);
	//0Speed Register
	port->OSPEEDR |= (1 << ((pin*2)+1) );
	//PUPDR Register
	port->PUPDR &= ~(0x3UL << pin*2);
}
void initializePortsForLCD(){
	setPortAndPinForOutput(LCDD0Port,LCDD0pin);
	setPortAndPinForOutput(LCDD1Port,LCDD1pin);
	setPortAndPinForOutput(LCDD2Port,LCDD2pin);
	setPortAndPinForOutput(LCDD3Port,LCDD3pin);
	setPortAndPinForOutput(LCDD4Port,LCDD4pin);
	setPortAndPinForOutput(LCDD5Port,LCDD5pin);
	setPortAndPinForOutput(LCDD6Port,LCDD6pin);
	setPortAndPinForOutput(LCDD7Port,LCDD7pin);
}
int main (void){
	initializePortsForLCD();
//	// Enable clock for the ports in use using the RCC struct
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
//	
	// configure each pin for output using the required registers
//	GPIOB->MODER |= (0x55UL << 8);
//	GPIOB->OTYPER &= ~(0xFUL << 4);
//	GPIOB->OSPEEDR |= (0xAAUL << 8);
//	GPIOB->OSPEEDR &= ~(0x55UL << 8);
//	GPIOB->PUPDR = 0;
//	
//	GPIOD->MODER |= 0x55UL;
//	GPIOD->OTYPER &= ~(0xFUL);
//	GPIOD->OSPEEDR |= (0xAAUL << 4);
//	GPIOD->OSPEEDR &= ~(0x55UL << 4);	
//	GPIOD->PUPDR = 0;

SendACharacterToTheLCDDataPins('A');
	while(1){
		for(int i = 0; i <= 255; i++){
				SendACharacterToTheLCDDataPins(i);
				wait();
				j= i;
		}
	}
}