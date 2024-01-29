
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


    LCD Segment Display analysis:

      -------------              -------------              -------------              -------------        
    |      15       |          |      10       |          |       5       |          |       1       |
    |  | ------- |  |          |  | ------- |  |          |  | ------- |  |          |  | ------- |  |
    |  |         |  |          |  |         |  |          |  |         |  |          |  |         |  |
    |16|         |14|          |11|         | 9|   ----   |6 |         | 4|          |2 |         |32|
    |  |         |  |          |  |         |  |  | 8  |  |  |         |  |          |  |         |  |
    |  | ------- |  |          |  | ------- |  |   ----   |  | ------- |  |          |  | ------- |  |
           17                         12                          7                          3
    |  | ------- |  |          |  | ------- |  |   ----   |  | ------- |  |          |  | ------- |  |
    |  |         |  |          |  |         |  |  | 8  |  |  |         |  |          |  |         |  |
    |19|         |21|          |22|         |24|   ----   |25|         |27|          |29|         |31|
    |  |         |  |          |  |         |  |          |  |         |  |          |  |         |  |
    |  | ------- |  |   ----   |  | ------- |  |   ----   |  | ------- |  |   ----   |  | ------- |  |
    |      20       |  | 13 |  |      23       |  | 24 |  |      26       |  | 28 |  |      30       |
      -------------     ----     -------------     ----     -------------     ----     -------------        

Segment code:
  b
c   a
  d
e   g
  f

1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32     bit number
b  c  d  a  b  c  d  .  a  b  c  d  .  a  b  c  d  .  e  f  g  e  f  g  e  f  g  .  e  f  g  a     segment code

3  3  3  2  2  2  2     1  1  1  1     0  0  0  0     0  0  0  1  1  1  2  2  2     3  3  3  3     segment count
x  x  x                                                                             x  x  x  x     segment 3
         x  x  x  x                                                     x  x  x                    segment 2
                        x  x  x  x                             x  x  x                             segment 1
                                       x  x  x  x     x  x  x                                      segment 0

0  0  0  0  0  0  0  0  1  1  1  1  1  1  1  1  2  2  2  2  2  2  2  2  3  3  3  3  3  3  3  3     byte count



A = 00000000 00000111 10101000 00000000
E = 11111011 00000011 10110000 01110111
                  abc d.efg
*/






#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#include "sounder.h"


#define CONFIG_FREERTOS_HZ 100      // FIXME: temp hack
//#define DEBUG_DECODER

#ifdef DEBUG_DECODER
const char *bit_rep[16] = {
    [ 0] = "0000", [ 1] = "0001", [ 2] = "0010", [ 3] = "0011",
    [ 4] = "0100", [ 5] = "0101", [ 6] = "0110", [ 7] = "0111",
    [ 8] = "1000", [ 9] = "1001", [10] = "1010", [11] = "1011",
    [12] = "1100", [13] = "1101", [14] = "1110", [15] = "1111",
};

void print_byte(uint8_t byte)
{
    printf("%s%s ", bit_rep[byte >> 4], bit_rep[byte & 0x0F]);
}

void print_bits(uint8_t byte, int8_t count)
{
    uint8_t string[9];

    string[count] = 0;
    for(int8_t i=1; i<=count; i++) {
        string[count-i] = (byte & 0x01) ? '1' : '0';
        byte >>= 1;
    }
    printf("%s ", string);
}

void string_bits( uint8_t byte, int8_t count, uint8_t *string )
{
    string[count] = 0;
    for(int8_t i=1; i<=count; i++) {
        string[count-i] = (byte & 0x01) ? '1' : '0';
        byte >>= 1;
    }
    printf("%s ", string);
}

void print_segments(uint8_t *buffer)
{
    print_byte(buffer[0]);
    print_byte(buffer[1]);
    print_byte(buffer[2]);
    print_byte(buffer[3]);

    // printf("  %02.1f\n", number );
}
#endif

struct v sounder_control::value;


uint8_t sounder_control::get_segment_0( uint8_t *buffer ) {
    uint8_t result;

    result  = ( buffer[1] & 0x07 ) << 4;    // segment 13, 14, 15
    result |= ( buffer[2] & 0x80 ) >> 4;    // segment 16
    result |= ( buffer[2] & 0x38 ) >> 3;    // segment 18, 19, 20
/*
A = 00000000 00000111 10101000 00000000
E = 11111011 00000011 10110000 01110111
                  abc d.efg
*/
    return( result );
}

uint8_t sounder_control::get_segment_1( uint8_t *buffer ) {
    uint8_t result;

    result  = ( buffer[1] & 0xF0 ) >> 1;    // segment 9, 10, 11, 12
    result |= ( buffer[2] & 0x07 ) >> 0;    // segment 21, 22, 23

    return( result );
}

uint8_t sounder_control::get_segment_2( uint8_t *buffer ) {
    uint8_t result;

    result  = ( buffer[0] & 0x1E ) << 2;    // segment 4, 5, 6, 7
    result |= ( buffer[3] & 0xE0 ) >> 5;    // segment 25, 26, 27

    return( result );
}

uint8_t sounder_control::get_segment_3( uint8_t *buffer ) {
    uint8_t result;

    result =  ( buffer[3] & 0x01 ) << 6;    // segment 32
    result |= ( buffer[0] & 0xE0 ) >> 2;    // segment 1, 2, 3
    result |= ( buffer[3] & 0x0E ) >> 1;    // segment 29, 30, 31

    return( result );
}

int sounder_control::segment_digit_decoder( uint8_t segment )
{
    switch ( segment )
    {
    case 0b0000000:    // blank
        return(0);
        break;
    
    case 0b1110111:
        return(0);
        break;
    
    case 0b1000001:
        return(1);
        break;
    
    case 0b1101110:
        return(2);
        break;
    
    case 0b1101011:
        return(3);
        break;
    
    case 0b1011001:
        return(4);
        break;
    
    case 0b0111011:
        return(5);
        break;
    
    case 0b0111111:
        return(6);
        break;
    
    case 0b1100001:
        return(7);
        break;
    
    case 0b1111111:
        return(8);
        break;
    
    case 0b1111011:
        return(9);
        break;
    
   default:
#ifdef DEBUG_DECODER
        uint8_t string[9];

        string_bits( segment, 7, string );
        printf("segment_digit_decoder(): error decoding bits: %s\n", string );
#endif
        return(-1);
        break;
    }
}

int sounder_control::segment_0_decoder( uint8_t segment )
{
    switch (segment)
    {
    case 0b0000000:    // blank
        return(0);
        break;
    
    case 0b0100000:    // Upper or Shallow Alarm
        return('S');
        break;
    
    case 0b0000010:    // Lower or Deep Alarm
        return('D');
        break;
    case 0b1111101:    // A = Alarm on
        return('A');
        break;
    
    case 0b00001010:    // sensitivity threshold
        return('T');
        break;
    
    case 0b0111110:
        return('E');
        break;

    case 0b0101000:
        return('K');     // Set keel offset
        break;
    
    case 0b0001000:
        return('P');     // Display keel offset on power on
        break;
/*
A = 00000000 00000111 10101000 00000000
E = 11111011 00000011 10110000 01110111
                  abc d.efg
*/
    
   default:
#ifdef DEBUG_DECODER
        uint8_t string[9];

        string_bits( segment, 7, string );
        printf("segment_0_decoder(): error decoding bits: %s\n", string );
#endif
        return(-1);
        break;
    }
}

int sounder_control::segment_decoder( uint8_t *buffer ) {
	int i;
	float number;
#ifdef DEBUG_DECODER
    print_segments(buffer);
    printf("    (32 bit spi message in 4 bytes)\n");
    print_byte( ( ( buffer[1] & 0x0F ) << 4 ) | ( ( buffer[2] & 0xF0 ) >> 4 ) );
    printf("    (mid 8 bits)\n");
#endif

	i = segment_digit_decoder( get_segment_1( buffer ) );    // most significant digit
	if( i != -1 ) {
		number = i * 10;
		i = segment_digit_decoder( get_segment_2( buffer ) );
		if(i != -1){
			number += i;
			i = segment_digit_decoder( get_segment_3( buffer ) );
			if(i != -1){
				number += (i / 10.0);

                i = segment_0_decoder( get_segment_0( buffer ) );
#ifdef DEBUG_DECODER
                printf("  %02.1f   ", number );
                //printf("%i\n", i);
#endif

                switch(i)
                {
                case 'E':
                    value.depth = number;
                    status.depth = true;
                    break;

                case 'S':
#ifdef DEBUG_DECODER
                    printf("Set shallow alarm\n");
#endif
                    value.shallow = number;
                    status.shallow = true;
                    break;

                case 'D':
#ifdef DEBUG_DECODER
                    printf("Set deep alarm\n");
#endif
                    value.deep = number;
                    status.deep = true;
                    break;

                case 'A':
#ifdef DEBUG_DECODER
                    printf("Alarm on\n");
#endif
                    status.alarm = true;
                    break;

                case 'T':
#ifdef DEBUG_DECODER
                    printf("Set sensitivity threshold\n");
#endif
                    value.sensit = number;
                    status.sensit = true;
                    break;

                case 'K':
#ifdef DEBUG_DECODER
                    printf("Set keel offset\n");
#endif
                    value.keel = number;
                    status.keel = true;
                    break;

                case 'P':
#ifdef DEBUG_DECODER
                    printf("Display keel offset\n");
#endif
                    value.keel = number;
                    status.keel = true;
                    break;

                default:
                    break;
                }
            }
        }
    }
    return(-1);
}

QueueHandle_t sounder_control::xQueue;

QueueHandle_t sounder_control::queue_handle(){
    return( xQueue );
};

void sounder_control::report() {
    printf("\n measured depth:         " );
    if( status.depth ) {
        printf("%02.1f   ", value.depth );
    } else {
        printf("---" );
    }
    printf("\n keel offset:            " );
    if( status.keel ) {
        printf("%02.1f   ", value.keel );
    } else {
        printf("---" );
    }
    printf("\n shallow alarm:          " );
    if( status.shallow ) {
        printf("%02.1f   ", value.shallow );
    } else {
        printf("---" );
    }
    printf("\n deep alarm:             " );
    if( status.deep ) {
        printf("%02.1f   ", value.deep );
    } else {
        printf("---" );
    }
    printf("\n sensitivity threshold:  " );
    if( status.sensit ) {
        printf("%02.1f   ", value.sensit );
    } else {
        printf("---" );
    }
    printf("\n alarm:                  " );
    if( status.alarm ) {
        printf("on" );
    } else {
        printf("off" );
    }
    printf("\n" );
}


void sounder_control::queue_receive(){
	uint8_t buffer[4];


    if( xQueueReceive( xQueue, buffer, 2 * portTICK_PERIOD_MS ) == pdTRUE ) {
        segment_decoder( buffer );
    }
};

sounder_control::sounder_control( QueueHandle_t handle )
{
    xQueue = handle;
    status.depth = false;
    status.keel = false;
    status.shallow = false;
    status.deep = false;
    status.sensit = false;
    status.alarm = false;
}

sounder_control::~sounder_control()
{
}

