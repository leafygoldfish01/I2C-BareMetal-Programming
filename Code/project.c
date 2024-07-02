#include <stm32f4xx.h>


//Register Definitions
#define LCD_ADDRESS (0x4E)

// For clearing ADDR bit
uint8_t clc;

void I2C_Start(void);
void I2C_Write(uint8_t var);
void Cmd_WRITE(uint8_t Device_Addr, uint8_t data);
void Write_Data_Function(uint8_t Device_Addr, uint8_t data);
void TIM4_Delay(void);


/*For the I2C Master to LCD Slave the connections are as follows:
P0 - Register Select -- distinguishes between instruction and data being sent, 0 indicates the data
P1 - Read/Write -- direction of data transfer, 0 indicates data is transfered to LED
P2 - Enable
P3 - Back Light
P4 - Data Pin1
P5 - Data Pin2
P6 - Data Pin3
P7 - Data Pin4*/

void I2C_Write(uint8_t var){
	
	// CHECKING TXE BIT FOR DATA REGISTER EMMPTY OR NOT
	while(!(I2C1->SR1 & (1<<7))){}

	// DATA TO BE WRITTEN
	I2C1->DR = var;

	// CHECK BTF BIT TRANSFER FINISHED
	while(!(I2C1->SR1 & (1<<2))){}
}

void I2C_Start(void){
	// ENALBE ACK BIT
	I2C1->CR1 |= 1<<10;

	// SET START BIT TO START COMMUNICATION
	I2C1->CR1 |= 1UL<<8;

	// TO MAKE SURE COMMUNICATION HAS STARTED
	while(!(I2C1->SR1 & 0x1)){}
}


void Cmd_WRITE(uint8_t Device_Addr, uint8_t data){
	//IN D1 AND D2 WE ARE STORING THE UPPER 4 BITS OF DATA
	//WE ARE SETTING EN=1,BACKLED IS ON
	uint8_t d1 = (data & 0xF0) | 0x0C;
	//HERE EN = 0
	uint8_t d2 = (data & 0xF0) | 0x08; 

	//IN D3 AND D4 WE ARE STORING THE UPPER 4 BITS OF DATA
	//WE ARE SETTING EN=1,BACKLED IS ON
	uint8_t d3 = ((data << 4) & 0xF0)| 0x0C;
	//HERE EN = 0
	uint8_t d4 = ((data << 4) & 0xF0)| 0x08; 

	I2C_Start();
	//ADDRESS TO BE SENT
	I2C1->DR = Device_Addr;

	// ADDRESS TRANSMITTED ADDR BIT
	while(!(I2C1->SR1 & 0x2)){}
	
	// CLEARING THE ADDR BIT
	clc = (I2C1->SR1 | I2C1->SR2);
		
	I2C_Write(d1);
	I2C_Write(d2);
	I2C_Write(d3);
	I2C_Write(d4);
	
	// STOP COMMUNCATION
	I2C1->CR1 |= 1<<9;
}

void Write_Data_Function(uint8_t Device_Addr, uint8_t data){

	//IN D1 AND D2 WE ARE STORING THE UPPER 4 BITS OF DATA
	//WE ARE SETTING EN=1,BACKLED IS ON
	uint8_t d1 = (data & 0xF0) | 0x0D;
	//HERE EN = 0
	uint8_t d2 = (data & 0xF0) | 0x09; 

	//IN D3 AND D4 WE ARE STORING THE UPPER 4 BITS OF DATA
	//WE ARE SETTING EN=1,BACKLED IS ON
	uint8_t d3 = ((data << 4) & 0xF0)| 0x0D;
	//HERE EN = 0
	uint8_t d4 = ((data << 4) & 0xF0)| 0x09; 

	// ENALBE ACK BIT
	I2C1->CR1 |= 1<<10;

	// SET START BIT TO START COMMUNICATION
	I2C1->CR1 |= 1UL<<8;

	// TO MAKE SURE COMMUNICATION HAS STARTED
	while(!(I2C1->SR1 & 0x1)){}
	
	//ADDRESS TO BE SENT
	I2C1->DR = Device_Addr;

	// ADDRESS TRANSMITTED ADDR BIT
	while(!(I2C1->SR1 & 0x2)){}
	
	// CLEARING THE ADDR BIT
	clc = (I2C1->SR1 | I2C1->SR2);

	I2C_Write(d1);
	I2C_Write(d2);
	I2C_Write(d3);
	I2C_Write(d4);
	
	// STOP COMMUNCATION
	I2C1->CR1 |= 1<<9;
}




void TIM4_Delay(void){
	RCC->APB1ENR |= 0x4;
	TIM4->PSC = 15999;
	TIM4->ARR = 100;
	TIM4->CNT = 0;
	TIM4->CR1 |= 1;

	
	while(!(TIM4->SR & 0x1)){} 
	TIM4->SR &= ~(0x1); 
}

int main(void){
	// ON CLOCK FOR PORT B
	RCC->AHB1ENR |= 0x2;
	
	// SET MODER FOR PIN PB6 - SCL AND PB7 - SDA AS AF MODE
	GPIOB->MODER |= (2<<(6*2));
	GPIOB->MODER |= (2<<(7*2));
	
	// SELECT ALTERNATE FUNCTION (AF4) AS I2C
	GPIOB->AFR[0] |= 4<<(6*4);
	GPIOB->AFR[0] |= 4<<(7*4);
	
	// SETTING AT HIGH SPEED
	GPIOB->OSPEEDR |= (2<<(6*2));
	GPIOB->OSPEEDR |= (2<<(7*2));
	
	// SETTING AS PULL UP
	GPIOB->PUPDR |= (1<<(6*2));
	GPIOB->PUPDR |= (1<<(7*2));

	// SETTING AS OPEN DRAIN
	GPIOB->OTYPER |=(1<<6) ;
	GPIOB->OTYPER |=(1<<7);

	// Enable I2C1 clock
	RCC->APB1ENR |= 1UL<<21;

	// RESETTING I2C BY SWSRT AT 15
	I2C1->CR1 |= 1<<15;
	I2C1->CR1 &= ~(1<<15);

	// SET I2C CLK AT 16MHZ BY FREQ[5:0] VALUE
	I2C1->CR2 |= 16<<0;

	// Needs to be set high by software for I2C
	I2C1->OAR1 |= 1<<14;

	// SET SC1 AT 100KHZ BY CCR[11:0] VALUE
	I2C1->CCR |= 0x50;

	// SET RISE TIME AS 1000NS BY TRISE[5:0] VALUE
	I2C1->TRISE |= 17;

	// ENABLE I2C1
	I2C1->CR1 |= 1;
	
	TIM4_Delay();
	Cmd_WRITE(LCD_ADDRESS,0x30);
	TIM4_Delay();
	//SETTING OUR LED TO 4 BIT MODE
	Cmd_WRITE(LCD_ADDRESS,0x20); 
	TIM4_Delay();
	
	Write_Data_Function(LCD_ADDRESS, 0x50); // MAPPING VALUES IN LCD TO SHOW 'P' USING ROM PATTERN
	Write_Data_Function(LCD_ADDRESS, 0x52); // MAPPING VALUES IN LCD TO SHOW 'R' USING ROM PATTERN
	Write_Data_Function(LCD_ADDRESS, 0x49); // MAPPING VALUES IN LCD TO SHOW 'I' USING ROM PATTERN
	Write_Data_Function(LCD_ADDRESS, 0x59); // MAPPING VALUES IN LCD TO SHOW 'Y' USING ROM PATTERN
	Write_Data_Function(LCD_ADDRESS, 0x41); // MAPPING VALUES IN LCD TO SHOW 'A' USING ROM PATTERN
	Write_Data_Function(LCD_ADDRESS, 0x4D); // MAPPING VALUES IN LCD TO SHOW 'M' USING ROM PATTERN
	TIM4_Delay();
	while(1){}
}
