/*  This implements the interface with the Nasa Targer depth sounder issue 3 (1992) hardware.

	Values are obtained by monitoring and decoding the serial communication between the Nasa propriety microcontroller and
	the LCD driver IC (32-Segment CMOS LCD Driver, alike AY0438) using the ESP32 SPI Slave interface. Control is provided by GPIO's
	connected to the physical switches (SET, UP, DOWN) on the board.

	FIXME: Determine the nessesety of power on/off control of the Nasa Target logic in order to set the 'keel offset' by the ESP32.

	The Nasa microcontroller interfaces with the LCD Driver trough 3 lines: DATA, CLOCK and LOAD (see the AY0438 datasheet). DATA is
	used as MOSI for ESP32 SPI, CLOCK for SPI CLK and LOAD for SPI CS. A message of 32 bits is captured by the SPI slave interface
	and decoded into a numeric value (the three rightmost sements) and a function (the leftmost segment).

	Values inside the Nasa Target depth sounder:
	- Depth                     E in left segment, depth below sounder - keel offset
	- Keel offset               Displayed on power on, - in left segment, range 0.0 - 2.5
	- Sensitivity threshold     Displayed after pressing SET, - and _ in left segment, range = 0.0 - 5.0, increment pressing UP, decrement pressing DOWN
	- Shallow Alarm             Displayed after pressing UP, ¯ in left segment, range = 1.0 - 25.0, increment pressing UP, decrement pressing DOWN
	- Deep Alarm                Displayed after pressing DOWN, _ in left segment, range = 2.5 - 99.5, increment pressing UP, decrement pressing DOWN

	Logic inside Nasa Target depth sounder:
	- Press SET, then power on: display keel offset, adjust with UP and DOWN, store with SET
	- Power on: display keel offset, after 2 seconds display depth
	- Press SET: display sensitivity threshold, adjust with UP and DOWN, store with SET
	- Press UP: display shallow alarm, adjust with UP and DOWN, store with SET
	- Press DOWN: display deep alarm, adjust with UP and DOWN, store with SET
	- Press UP and DOWN: toggle alarm, display A if on, - if off




*/







#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "sounder.h"

#define RX_QUEUE_LENGTH 16
#define RX_QUEUE_ITEM_SIZE 4

static const char *TAG = "sounder";

//#define DEBUG_LCD_DECODER


//class Sounder {
	//private:



#ifdef DEBUG_LCD_DECODER
	const char *bit_rep[16] = {
		[ 0] = "0000", [ 1] = "0001", [ 2] = "0010", [ 3] = "0011",
		[ 4] = "0100", [ 5] = "0101", [ 6] = "0110", [ 7] = "0111",
		[ 8] = "1000", [ 9] = "1001", [10] = "1010", [11] = "1011",
		[12] = "1100", [13] = "1101", [14] = "1110", [15] = "1111",
	};

	void print_byte(uint8_t byte)
	{
		printf("%s %s ", bit_rep[byte >> 4], bit_rep[byte & 0x0F]);
	}
#endif




	int segment_digit_decoder(char segment) {
		switch (segment)
		{
		case 0b00000000:    // blank
			return(0);
			break;

		case 0b11101110:
			return(0);
			break;

		case 0b10000010:
			return(1);
			break;

		case 0b11011100:
			return(2);
			break;

		case 0b11010110:
			return(3);
			break;

		case 0b10110010:
			return(4);
			break;

		case 0b01110110:
			return(5);
			break;

		case 0b01111110:
			return(6);
			break;

		case 0b11000010:
			return(7);
			break;

		case 0b11111110:
			return(8);
			break;

		case 0b11110110:
			return(9);
			break;

		default:
			return(-1);
			break;
		}
	}

	// most significant number

	int segment_digit1_decoder(char *buffer) {
		int i;
		char segment = ( (buffer[1] & 0xF0) | ((buffer[2] & 0x07) << 1) );

		i = segment_digit_decoder( segment );

	#ifdef DEBUG_LCD_DECODER
			printf( "segment_digit1_decoder(): error decoding bits: %s %s\n", bit_rep[segment >> 4], bit_rep[segment & 0x0F] );
	#endif

		return( i );

	}

	int segment_digit2_decoder(char *buffer) {
		int i;
		char segment = ( (buffer[0] & 0x1E) << 3) | ((buffer[3] & 0xE0) >> 4 );

		i = segment_digit_decoder( segment );

	#ifdef DEBUG_LCD_DECODER
			printf( "segment_digit2_decoder(): error decoding bits: %s %s\n", bit_rep[segment >> 4], bit_rep[segment & 0x0F] );
	#endif

		return( i );

	}

	// least significant number

	int segment_digit3_decoder(char *buffer) {
		int i;
		char segment = ( (buffer[0] & 0xE0) | (buffer[3] & 0x0F) );

		i = segment_digit_decoder( segment );

	#ifdef DEBUG_LCD_DECODER
			printf( "segment_digit3_decoder(): error decoding bits: %s %s\n", bit_rep[segment >> 4], bit_rep[segment & 0x0F] );
	#endif

		return( i );

	}

	int segment_0_decoder(char *buffer) {
		char segment = (((buffer[1] & 0x07) << 4 ) | ((buffer[2] & 0xF0) >> 4));

		switch (segment)
		{
		case 0b00000000:    // blank
			return(0);
			break;

		case 0b00100000:    // Upper or Shallow Alarm
			return('S');
			break;

		case 0b00000001:    // Lower or Deep Alarm
			return('D');
			break;

		case 0b01111010:    // A = Alarm on
			return('A');
			break;

		case 0b00001001:    // sensitivity threshold
			return('T');
			break;

		case 0b00111011:    // Depth
			return('E');
			break;

		case 0b00101000:
			return('K');     // Set keel offset
			break;

		case 0b00001000:
			return('P');     // Display keel offset on power on
			break;

	default:
	#ifdef DEBUG_LCD_DECODER
			printf("segment0decode(): error decoding bits: %s %s\n", bit_rep[segment >> 4], bit_rep[segment & 0x0F]);
	#endif
			return(-1);
			break;
		}
	}


	struct data {
		struct
		{
			float measured_depth, keel_offset, sensitivity_threshold, shallow_alarm, deep_alarm;
		} value;
		struct s {
			unsigned char measured_depth : 1;
			unsigned char keel_offset : 1;
			unsigned char sensitivity_threshold : 1;
			unsigned char shallow_alarm : 1;
			unsigned char deep_alarm : 1;
			unsigned char alarm_on : 1;
		} status;
	} sounderdata;

/*	float measured_depth, keel_offset, sensitivity_threshold, shallow_alarm, deep_alarm;
	struct s {
		unsigned char measured_depth : 1;
		unsigned char keel_offset : 1;
		unsigned char sensitivity_threshold : 1;
		unsigned char shallow_alarm : 1;
		unsigned char deep_alarm : 1;
		unsigned char alarm_on : 1;
	} value_status;
*/

	void segment_decoder(char *buffer) {
		int i;
		float number;

#ifdef DEBUG_LCD_DECODER
		print_byte(buffer[0]);
		print_byte(buffer[1]);
		print_byte(buffer[2]);
		print_byte(buffer[3]);
		printf("\n");
#endif

		i = segment_digit1_decoder(buffer);
		if(i != -1){
			number = i * 10;
			i = segment_digit2_decoder(buffer);
			if(i != -1){
				number += i;
				i = segment_digit3_decoder(buffer);
				if(i != -1){
					number += (i / 10.0);
#ifdef DEBUG_LCD_DECODER
					printf("  %02.1f\n", number );
#endif
					i = segment_0_decoder(buffer);
					switch(i)
					{
					case 'E':
						sounderdata.value.measured_depth = number;
						sounderdata.status.measured_depth = 1;
						break;

					case 'S':
#ifdef DEBUG_LCD_DECODER
						printf("Set shallow alarm\n");
#endif
						sounderdata.value.shallow_alarm = number;
						sounderdata.status.shallow_alarm = 1;
						break;

					case 'D':
#ifdef DEBUG_LCD_DECODER
						printf("Set deep alarm\n");
#endif
						sounderdata.value.deep_alarm = number;
						sounderdata.status.deep_alarm = 1;
						break;

					case 'A':
#ifdef DEBUG_LCD_DECODER
						printf("Alarm on\n");
#endif
						sounderdata.status.alarm_on = 1;
						break;

					case 'T':
#ifdef DEBUG_LCD_DECODER
						printf("Set sensitivity threshold\n");
#endif
						sounderdata.value.sensitivity_threshold = number;
						sounderdata.status.sensitivity_threshold = 1;
						break;

					case 'K':
#ifdef DEBUG_LCD_DECODER
						printf("Set keel offset\n");
#endif
						sounderdata.value.keel_offset = number;
						sounderdata.status.keel_offset = 1;
						break;

					case 'P':
#ifdef DEBUG_LCD_DECODER
						printf("Display keel offset\n");
#endif
						sounderdata.value.keel_offset = number;
						sounderdata.status.keel_offset = 1;
						break;

					default:
						break;
					}
				}
			}
		}
	}


	QueueHandle_t xQueue;

	//public:

			Sounder::Sounder() {
			sounderdata.status.measured_depth = 0;
			sounderdata.status.keel_offset = 0;
			sounderdata.status.sensitivity_threshold = 0;
			sounderdata.status.shallow_alarm = 0;
			sounderdata.status.deep_alarm = 0;
			sounderdata.status.alarm_on =0;

			xQueue = xQueueCreate( RX_QUEUE_LENGTH, RX_QUEUE_ITEM_SIZE );

		}

		void Sounder::InQueue( unsigned char *buffer ) {
			if( xQueue != 0 )
 			{
     			// Send an uint32_t.  Wait for 10 ticks for space to become
     			// available if necessary.
     			if( xQueueSendToBack( xQueue, ( void * ) buffer, ( TickType_t ) 10 ) != pdPASS )
     			{
         		// Failed to post the message, even after 10 ticks.
     			}
			}

		}

extern "C" void SounderInQueue( unsigned char +buffer ) {
    Sounder::InQueue( buffer );
}

//};