#include "stm32f4xx_discovery.h"

#include "stdio.h"
#include "stdlib.h"

#define LOCAL_SERVER 0
#define OPENWEATHER_SERVER  1

struct __FILE {int handle;};
FILE __stdout;

int fputc(int c, FILE *f) {
	return ITM_SendChar(c);
}

uint32_t tick = 0;
void SysTick_Handler(void)
{
	tick++; //10ms
}

void delay_ms(uint32_t d)
{
	tick = 0;
	while(d > tick);
}

void led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = BLUE_LED;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_SetBits(LED_PORT, BLUE_LED);
}

//#define MAX_STRLEN 12 // this is the maximum string length of our string in characters
//volatile char received_string[MAX_STRLEN+1]; // this will hold the recieved string

#define RINGFIFO_SIZE (2048)              /* serial buffer in bytes (power 2)   */
#define RINGFIFO_MASK (RINGFIFO_SIZE-1ul) /* buffer size mask                   */

/* Buffer read / write macros                                                 */
#define RINGFIFO_RESET(ringFifo)      {ringFifo.rdIdx = ringFifo.wrIdx = 0;}
#define RINGFIFO_WR(ringFifo, dataIn) {ringFifo.data[RINGFIFO_MASK & ringFifo.wrIdx++] = (dataIn);}
#define RINGFIFO_RD(ringFifo, dataOut){ringFifo.rdIdx++; dataOut = ringFifo.data[RINGFIFO_MASK & (ringFifo.rdIdx-1)];}
#define RINGFIFO_EMPTY(ringFifo)      (ringFifo.rdIdx == ringFifo.wrIdx)
#define RINGFIFO_FULL(ringFifo)       ((RINGFIFO_MASK & ringFifo.rdIdx) == (RINGFIFO_MASK & (ringFifo.wrIdx+1)))
#define RINGFIFO_COUNT(ringFifo)      (RINGFIFO_MASK & (ringFifo.wrIdx - ringFifo.rdIdx))

/* buffer type                                                                */
typedef struct{
    uint32_t size;
    uint32_t wrIdx;
    uint32_t rdIdx;
    uint8_t data[RINGFIFO_SIZE];
} RingFifo_t;
RingFifo_t gUartFifo;

void uart_init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
	
	/* Enable clock for USATR1
	*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	/* enable the peripheral clock for the pins used by 
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
	
	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	
	/* Now the USART_InitStruct is used to define the 
	 * properties of USART1 
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	
	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured 
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff	

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}

void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) ); 
		USART_SendData(USARTx, *s);
		*s++;
	}
}

void USART_reads()
{
	static char data;
	while(!RINGFIFO_EMPTY(gUartFifo)) {
      /* consume fifo using RINGFIFO_RD */
			RINGFIFO_RD(gUartFifo, data)
			printf("%c",data);
    }
}

void USART1_IRQHandler(void){
	
	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART1->DR; // the character from the USART1 data register is saved in t
		
		RINGFIFO_WR(gUartFifo, t);
	}
}

int main()
{
	printf("New program started!\r\n");
	led_init();
	uart_init(115200);
	SysTick_Config(SystemCoreClock / 100);
	USART_puts(USART1, "AT\r\n");
	delay_ms(100);
	USART_reads();
	//Join Access Point
	USART_puts(USART1, "AT+GMR\r\n");
	delay_ms(100);
	USART_reads();
	USART_puts(USART1, "AT+CWJAP=\"Phome\",\"26Bex067\"\r\n");
	delay_ms(500);
	USART_reads();
	USART_puts(USART1, "AT+CIFSR\r\n");
	delay_ms(500);
	USART_reads();
	USART_puts(USART1, "AT+CWMODE?\r\n");
	delay_ms(100);
	USART_reads();
	USART_puts(USART1, "AT+CIPMUX=1\r\n");
	delay_ms(100);
	USART_reads();
	
	#if LOCAL_SERVER==1
		USART_puts(USART1, "AT+CIPSTART=1,\"TCP\",\"192.168.0.106\",80\r\n");
	#elif OPENWEATHER_SERVER==1
		USART_puts(USART1, "AT+CIPSTART=2,\"TCP\",\"api.openweathermap.org\",80\r\n");
	#endif
	delay_ms(500);
	USART_reads();
	while(1)
	{
		GPIO_ToggleBits(LED_PORT, BLUE_LED);
		#if OPENWEATHER_SERVER==1
			/*
			* OpenWeatherMap
			* Request: GET /data/2.5/weather?q=gwarko&APPID=d33ccc4fe677629315a697ffa4ae097e HTTP/1.1\r\nHost: api.openweathermap.org\r\n\r\n
			* Length : 112
			*/
			USART_puts(USART1, "AT+CIPSEND=2,112\r\n");
			delay_ms(100);
			USART_reads();
			USART_puts(USART1, "GET /data/2.5/weather?q=gwarko&APPID=d33ccc4fe677629315a697ffa4ae097e HTTP/1.1\r\nHost: api.openweathermap.org\r\n\r\n");
			delay_ms(100);
			USART_reads();
		#elif LOCAL_SERVER==1
			/*
			* My server using xamp
			* Request: GET /page/ HTTP/1.1\r\nHost: 192.168.0.106\r\n\r\n
			* Length : 44
			*/
			USART_puts(USART1, "AT+CIPSEND=1,44\r\n");
			delay_ms(500);
			USART_reads();
			USART_puts(USART1, "GET /page/ HTTP/1.1\r\nHost: 192.168.0.106\r\n\r\n");
			delay_ms(500);
			USART_reads();
		#endif	
		delay_ms(500);
	}
}