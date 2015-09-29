#define F_CPU 1000000L
#include <avr/io.h>
#include <util/delay.h>
#include <inttypes.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include "manchester.h"
#include "1wire.h"
#include "tinudht.h"

#define TINUDHT_RCV_TIMEOUT 255
#define TINUDHT_RCV_DELAY 5
#define TINUDHT_RCV_LENGTH 3

//#define DS18B20_FAMILY_ID                0x28
#define DS18B20_ID                       22
//#define DS18B20_START_CONVERSION         0x44
//#define DS18B20_READ_SCRATCHPAD          0xbe
//#define DS18B20_ERROR                    1000
#define TINUDHT_PIN PB4
//#define BUS 							ONEWIRE_PIN_4
//#define MAX_DEVICES 					1

#define TxPin 2;
uint8_t moo = 0; //last led status
uint8_t data[7] = {DS18B20_ID, 0, 0, 0, 0,0,0};
Manchester man;

uint16_t readVccVoltage(void) {
	
	ADMUX = _BV(MUX3) | _BV(MUX2);
	ADCSRA |= _BV(ADEN) | _BV( ADSC) | _BV(ADPS1) | _BV(ADPS0);
	_delay_ms(1);
    ADCSRA |= _BV(ADSC);				// Start 1st conversion
	while( ADCSRA & _BV( ADSC) ) ;		// Wait for 1st conversion to be ready...	
	ADCSRA |= _BV(ADSC);				// Start a conversion
	while( ADCSRA & _BV( ADSC) ) ;		// Wait for 1st conversion to be ready...
										//..and ignore the result
	uint8_t low  = ADCL;
	uint8_t high = ADCH;

	uint16_t adc = (high << 8) | low;		// 0<= result <=1023
	uint8_t vccx10 = (uint8_t) ( (11 * 1024) / adc); 
	ADCSRA &= ~_BV( ADEN );			// Disable ADC to save power
	return( vccx10 );
}


void system_sleep(void);

void setup_watchdog(int ii)
{
	cli();
	uint8_t tout;
	if (ii > 9)
		ii = 9;
	tout = ii & 7;
	if (ii > 7)
		tout |= _BV(WDP3);

	tout |= _BV(WDCE);

	MCUSR &= ~_BV(WDRF);
	cli();
#if defined(WDTCR)
	WDTCR |= _BV(WDCE) | _BV(WDE);
	WDTCR = tout;
	WDTCR |= _BV(WDIE);
#elif defined (WDTSCR)
	WDTCR |= _BV(WDCE) | _BV(WDE);
	WDTCR = tout;
	WDTCR |= _BV(WDIE);
#endif
sei();
}

volatile uint8_t f_wdt = 1;
uint8_t loop_count = 0;

void system_sleep()
{
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
	sleep_enable();
	sleep_mode();                        // System sleeps here
	sleep_disable();                     // System continues execution here when watchdog timed out 
}


int main(void)
{
  wdt_disable();
  setup_watchdog(WDTO_8S);
  init(&man, 100);
  workAround1MhzTinyCore(&man, 1);
  setupTransmit(&man, (1<<PB3),MAN_1200);
  
  
//unsigned int temp = 0;
	//cli();
//	     DDRB |= (1<<4);
		//DDRB |= SENDER_PIN; // PB4 for the sender or Control LED
//		DDRB &= ~(1 << PB4); //PB3 as input
		//static oneWireDevice devices[MAX_DEVICES];
		//oneWireDevice *ds18b20;
//init_Timer_OOK();
_delay_us(550);
		//oneWireInit(BUS);
		//while (oneWireSearchBuses(devices, MAX_DEVICES, BUS) != ONEWIRE_SEARCH_COMPLETE);
		//ds18b20 = oneWireFindFamily(DS18B20_FAMILY_ID, devices, MAX_DEVICES);
	//sei();
 // PORTB is output, all pins
   //PORTB = 0x00; // Make pins low to start

for ( ; ; ) {
  if (loop_count % 20 == 0) {
  
      for (int j = 0; j < 5; j++ ) {
      uint8_t vccx10 = readVccVoltage();
     data [1] = vccx10;
			TinuDHT tinudht;
		uint8_t tinudht_result = tinudht_read(&tinudht, TINUDHT_PIN);
		if (tinudht_result == TINUDHT_OK) {
			data [2] = tinudht.temperature_int; 
			data [3] = tinudht.temperature_dec;
			data [4] = tinudht.humidity_int;
			data [5] = tinudht.humidity_dec;

		} 

			data [6] = oneWireDataCrc(data);
			//_delay_ms(200);
			loop_count = 0;
			transmitArray(&man, 7, data);
			transmitArray(&man, 7, data);
		//}
		
    
      }
	  }
system_sleep();
_delay_ms(1000); // wait some time
loop_count++;
}
return 0;
}
 
uint8_t tinudht_read(TinuDHT *ptinudht, uint8_t dht_pin) {
	
	// Buffer to received data
	uint8_t data[5];
	// Empty the buffer
	data[0] = data[1] = data[2] = data[3] = data[4] = 0;
//	for (uint8_t i=0; i< 5; i++) data[i] = 0;	// Another way to empty the data buffer.

	// Send request
	DDRB |= (1 << dht_pin);	// Set port as output
	PORTB &= ~(1 << dht_pin);	// Set to 0
	_delay_ms(18);	// Wait 18 ms
	PORTB |= (1 << dht_pin);	// Set to 1
	_delay_us(40);	// Wait 40 us

	// Receive response
	DDRB &= ~(1 << dht_pin);	// Set port as input
		PORTB |= (1 << dht_pin);	// Set to 1, optional pull-up.

	uint8_t timeout;

	// Acknowledge
	timeout = TINUDHT_RCV_TIMEOUT;
	while(bit_is_clear(PINB, dht_pin))	// Wait for 1
		if (timeout-- == 0)
			return TINUDHT_ERROR_TIMEOUT;
	
	timeout = TINUDHT_RCV_TIMEOUT;
	while(bit_is_set(PINB, dht_pin))	// Wait for 0
		if (timeout-- == 0)
			return TINUDHT_ERROR_TIMEOUT;

	uint8_t bit_index = 7;
	uint8_t byte_index = 0;
	// READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
	for (uint8_t i = 0; i < 40; i++)
	{
		// Wait for start
		timeout = TINUDHT_RCV_TIMEOUT;
		while(bit_is_clear(PINB, dht_pin))	// Wait for 1
			if (timeout-- == 0)
				return TINUDHT_ERROR_TIMEOUT;

		// Determine the bit value
		uint8_t len = 0;
		while(bit_is_set(PINB, dht_pin)) {	// Wait for 0
			_delay_us(TINUDHT_RCV_DELAY);
			if (len++ == TINUDHT_RCV_TIMEOUT)
				return TINUDHT_ERROR_TIMEOUT;
			}

		if (len >= TINUDHT_RCV_LENGTH) data[byte_index] |= (1 << bit_index);

		// Experiments:
		// delay =    0,     len = 3..6 - 7:unstable
		// delay =  1us,     len = 3..5
		// delay =  2us,     len = 3..5
		// delay =  5us,     len = 3..5
		// delay = 10us,     len = 2..4
		// delay = 15us,     len = 2..3
		// delay = 20us,     len = 2    - 3:unstable
		// delay =  9..29us, len = 2
		
		// NOTE: The values of TINUDHT_RCV_DELAY and TINUDHT_RCV_LENGTH
		//       may need to be adjusted if the F_CPU changes.
		
		if (bit_index == 0)	// next byte?
		{
			bit_index = 7;	// restart at MSB
			byte_index++;	// next byte!
		}
		else bit_index--;
	}

	uint8_t checksum = ((data[0] + data[1]+ data[2] + data[3]) & 0xFF);	// TODO: Fix this, does not work in border cases.
	if (data[4] != checksum) return TINUDHT_ERROR_CHECKSUM;

	// On DHT11 data[1],data[3] are always zero so not used.
	ptinudht->humidity_int= data[0]; 
	ptinudht->humidity_dec = data[1]; 
	ptinudht->temperature_int = data[2]; 
	ptinudht->temperature_dec = data[3]; 
    //int8_t temperature;
	//temperature = data[2]; 
	//return temperature;
	return TINUDHT_OK;
}

ISR(WDT_vect)
{
 sleep_disable();          // Disable Sleep on Wakeup
 f_wdt = 1;  // set global flag
  // Your code goes here...
  // Whatever needs to happen every 1 second
  sleep_enable();           // Enable Sleep Mode
	
}