#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL
#include <util/delay.h> // Retardos por software
#define BR9600 (0x67)	// 0x67=103 configura BAUDRATE=9600@16MHz
#include "SerialPort.h"


void Iniciar_Sistema(void);
void chequeoEntrada(void);
uint8_t leerADC(void);

volatile char RX_Buffer=0;
uint8_t intRojo;

typedef enum{rojo, verde, azul, inicial}state;
state estado;

int main(void)
{
	//Seteo timer1
	TCCR1A |= 1<<WGM10;	//Configuración timer 1 phase correct 8bits
	TCCR1B |= (1<<CS02);	//Prescaler a 256
	TCCR1A |= (1<<COM1A0) | (1<<COM1A1);	//Set OC1A on compare match
	TCCR1A |= (1<<COM1B0) | (1<<COM1B1);

	OCR1A = 0;	//Azul
	OCR1B = 0;	//Verde
	intRojo = 0;

	DDRB |= (1<<PINB1);
	DDRB |= (1<<PINB2);
	DDRB |= (1<<PINB5);
	
	DDRC &= ~(1<<PINC3);
	
	ADMUX &= ~(1<<REFS1);
	ADMUX |= (1<<REFS0);
	ADMUX |= (1<<ADLAR);
	ADMUX &= ~(1<<MUX3);
	ADMUX &= ~(1<<MUX2);
	ADMUX |= (1<<MUX1);
	ADMUX |= (1<<MUX0);
	
	ADCSRA = 0X87;
	
	uint8_t lecturaADC;
	
	Iniciar_Sistema();
	estado = inicial;
	while (1)
	{
		chequeoEntrada();
		lecturaADC = leerADC();
		//En el registro OCR1A y OCR1B se controlan las intensidades del led verde y azul respectivamente
		
		switch(estado){
			case(rojo):
				intRojo = lecturaADC;
				break;
			case(verde):
				OCR1B = lecturaADC;
				break;
			case(azul):
				OCR1A = lecturaADC;
				break;
			case (inicial):
				break;
		}
		if(TCNT1 < intRojo){
			//Se enciende
			PORTB &= ~(1<<PINB5);
			} else {
			//Se apaga
			PORTB |= (1<<PINB5);
		}
	}
}

void Iniciar_Sistema(){
	SerialPort_Init(BR9600); 		// Se inicializa en formato 8N1 y BAUDRATE = 9600bps
	SerialPort_TX_Enable();			// Activo el Transmisor del Puerto Serie
	SerialPort_RX_Enable();			// Activo el Receptor del Puerto Serie
	SerialPort_Send_String("Presionando R/G/B para controlar la intensidad de cada color\n\r");   // Envío el mensaje
	SerialPort_RX_Interrupt_Enable();	// Activo Interrupción de recepcion.
	sei();								// Activo la mascara global de interrupciones (Bit I del SREG en 1)
}

uint8_t leerADC(){
	ADCSRA |= (1<<ADSC);
	while((ADCSRA&(1<<ADIF)) == 0);
	ADCSRA |= (1<<ADIF);
	return ADCH;
}

void chequeoEntrada(){
	if(RX_Buffer){ // recepción NO Bloqueante
		// Si presiona 'r'/'g'/'b', se setea el rojo/verde/azul según corresponda
		
		if(RX_Buffer == 'r'){
			estado = rojo;
			} else if(RX_Buffer == 'g'){
			estado = verde;
			} else if(RX_Buffer == 'b'){
			estado = azul;
		}
		RX_Buffer=0;
	}
}

ISR(USART_RX_vect){
	RX_Buffer = UDR0; //la lectura del UDR borra flag RXC
}
