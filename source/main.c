/*
 * PIC_programmer.c
 *
 * Created: 2021. 07. 15. 10:01:59
 * Author : Skriba
 */ 

#define F_CPU 14745600UL
#define USRT_BAUDRATE 115200
#define UBRR_VALUE ((F_CPU / (USRT_BAUDRATE * 16UL)) - 1)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile uint8_t code_buffer[512];

extern void my_lpm();


//CLK gen. and data
volatile int clk_cntr = 0;
volatile int bit_cntr = 0;
volatile int finish = 0;
volatile int send = 0;
uint16_t read_data_bit = 0;
uint16_t data_bit = 0;
uint16_t mask = 1;

uint16_t read_value = 0;
uint16_t add = 0;


void send_uart(char data)
{
	while(LINSIR & (1 << LBUSY))
	{
		asm("nop");
	}
	LINDAT=data;
}

char receive_uart(void)
{
	while(LINSIR & (1 << LBUSY))
	{
		asm("nop");
	}
	return LINDAT;
}

void enter_prog(void) {
	//VDD high
	PORTC &= ~(1<<PORTC0);
	_delay_us(150);
	//VPP high
	PORTC |= (1<<PORTC1);
	_delay_us(5);
}

void exit_prog(void) {
	//VDD low
	PORTC |= (1<<PORTC0);
	_delay_us(150);
	//VPP low
	PORTC &= ~(1<<PORTC1);
}

void send_bits (uint16_t data, int bits, int is_send) {
	send = is_send;
	finish = 0;
	data_bit = data;
	bit_cntr = bits;
	TIMSK0 = 0b00000010;
	while (finish == 0) {
		asm("nop");
	}
}

void inc_addr (void) {
	send_bits (0b00000110,6,1);
}

void set_addr (uint16_t address) {
	for (uint16_t i = 0; i <= address; i++) {
		send_bits (0b00000110,6,1);
	}
}

uint16_t read_data (void) {
	read_data_bit = 0;
	send_bits (0b00000100,6,1);
	DDRC &= ~(1<<DDC2);
	_delay_us(1);
	send_bits (0b0000000000000000,16,0);
	DDRC |= (1<<DDC2);
	return read_data_bit;
}


void load_data (uint16_t data) {
	send_bits (0b00000010,6,1);
	send_bits (data,16,1);
	send_bits (0b00001000,6,1);
	_delay_ms(1);
	send_bits (0b00001110,6,1);
	_delay_us(100);
}




int main(void)
{
	//Write 0xFF to code_buffer
	uint16_t j = 0;
	for(j = 0; j < 512; j++)
	{
		code_buffer[j] = 0xFF;
	}
	
	//Write hex data to code_buffer
	my_lpm();
	
	
	//Programming ports
	DDRC = (1<<DDC0) | (1<<DDC1) | (1<<DDC2) | (1<<DDC3);
	PORTC &= ~(1<<PORTC3);
	PORTC &= ~(1<<PORTC1);
	PORTC &= ~(1<<PORTC2);
	//Timer 0 | CTC | 64 prescale
	TCCR0A = (1<<WGM01) ;
	TCCR0B = (1<<CS01) | (1<<CS00);
	TIMSK0 = 0b00000010;
	TCNT0 = 0;
	OCR0A = 10;
	//Enable interrupt
	sei();
	
	
	LINBRRL = UBRR_VALUE;
	LINBRRH = (UBRR_VALUE>>8);
	LINBTR = 0b00010000;
	
	//Enable UART
	// Asynchronous mod, 8 Data Bit, No parity Bit, 1 Stop Bit
	LINCR = 0b00001000;
	
	
	uint16_t read_value = 0;
	uint16_t add = 0;
	
	
    while (1) 
    {
		if (add == 0) {
			enter_prog();
			inc_addr();
			//Bulk erease
			send_bits (0b00001001,6,1);
			_delay_ms(7);
		}
		
		
		if(add < 512) {
			load_data(((((code_buffer[add+1])<<8)|(code_buffer[add]))<<1)&0b0001111111111110);
			read_value = read_data();
			send_uart(read_value & 0b11111111);
			send_uart((read_value>>8) & 0b11111111);
			
			if (add != 510)
				inc_addr();
	
			add=add+2;
		}
		if (add == 512)
		{
			_delay_ms(500);
			exit_prog();
			_delay_ms(1000);
			add = 0;
			enter_prog();
			load_data(0b0001111111001110);
			_delay_ms(5000);	
		}
	}
}

ISR (TIMER0_COMPA_vect) {
	
	//Clock generation and data read
	if (PORTC & 0b00001000)
	{
		if ((send == 0) && (clk_cntr > 1))
		{
			read_data_bit |= (((PINC & (1 << PINC2)) >> PINC2)<< (clk_cntr-2));
		}
		PORTC &= ~(1<<PORTC3);
	}
	
	else
	{
		if ((clk_cntr) != bit_cntr)
		PORTC |= (1<<PORTC3);
		clk_cntr++;
	}
	
	//Data shift-out
	if ((clk_cntr-1) < bit_cntr) {
		if (send == 1)
		{
			if ((mask << (clk_cntr-1)) & data_bit)
			PORTC |= (1<<PORTC2);
			else
			PORTC &= ~(1<<PORTC2);
		}
	}
	else {
		clk_cntr = 0;
		finish = 1;
		if (send == 1)
		PORTC &= ~(1<<PORTC3);
		TIMSK0 = 0b00000000;
	}
}
