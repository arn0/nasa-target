/* SPI Slave example, receiver (uses SPI Slave driver to communicate with sender)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"


/*
SPI receiver (slave) example.

This example is supposed to work together with the SPI sender. It uses the standard SPI pins (MISO, MOSI, SCLK, CS) to
transmit data over in a full-duplex fashion, that is, while the master puts data on the MOSI pin, the slave puts its own
data on the MISO pin.
*/

/*
Pins in use. The SPI Master can use the GPIO mux, so feel free to change these if needed.
*/
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define GPIO_MOSI 12
#define GPIO_MISO 13
#define GPIO_SCLK 15
#define GPIO_CS 14

#elif CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2
#define GPIO_MOSI 7
#define GPIO_MISO 2
#define GPIO_SCLK 6
#define GPIO_CS 10

#elif CONFIG_IDF_TARGET_ESP32C6
#define GPIO_MOSI 19
#define GPIO_MISO 20
#define GPIO_SCLK 18
#define GPIO_CS 9

#elif CONFIG_IDF_TARGET_ESP32H2
#define GPIO_MOSI 5
#define GPIO_MISO 0
#define GPIO_SCLK 4
#define GPIO_CS 1

#elif CONFIG_IDF_TARGET_ESP32S3
#define GPIO_MOSI 11
#define GPIO_MISO 13
#define GPIO_SCLK 12
#define GPIO_CS 10

#endif //CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2


#ifdef CONFIG_IDF_TARGET_ESP32
#define RCV_HOST    HSPI_HOST

#else
#define RCV_HOST    SPI2_HOST

#endif

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;

#ifdef CONFIG_BLINK_LED_RMT

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#endif

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

int segment0decode(char *buffer)
{
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
    
    case 0b00111011:
        return('E');
        break;
    
    case 0b00101000:
        return('K');     // Keel offset
        break;
    
   default:
        printf("segment0decode(): error decoding bits: %s %s ", bit_rep[segment >> 4], bit_rep[segment & 0x0F]);
        return(-1);
        break;
    }
}

int segment1number(char *buffer)
{
    char segment = ((buffer[1] & 0xF0) | ((buffer[2] & 0x07) << 1));

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
        printf("segment1number(): error decoding bits: %s %s ", bit_rep[segment >> 4], bit_rep[segment & 0x0F]);
        return(-1);
        break;
    }
}

int segment2number(char *buffer)
{
    char segment = ((buffer[0] & 0x1E) << 3) | ((buffer[3] & 0xE0) >> 4);

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
        printf("segment2number(): error decoding bits: %s %s ", bit_rep[segment >> 4], bit_rep[segment & 0x0F]);
        return(-1);
        break;
    }
}
int segment3number(char *buffer)
{
    char segment = ((buffer[0] & 0xE0) | (buffer[3] & 0x0F));

    switch (segment)
    {
    case 0b00000000:    // blank
        return(0);
        break;
    
    case 0b11001111:
        return(0);
        break;
    
    case 0b00000011:
        return(1);
        break;
    
    case 0b10101101:
        return(2);
        break;
    
    case 0b10100111:
        return(3);
        break;
    
    case 0b01100011:
        return(4);
        break;
    
    case 0b11100110:
        return(5);
        break;
    
    case 0b11101110:
        return(6);
        break;
    
    case 0b10000011:
        return(7);
        break;
    
    case 0b11101111:
        return(8);
        break;
    
    case 0b11100111:
        return(9);
        break;
    
   default:
        printf("segment3number(): error decoding bits: %s %s ", bit_rep[segment >> 4], bit_rep[segment & 0x0F]);
        return(-1);
        break;
    }
}

void app_decode_segments(char *buffer)
{
    int i;
    float number;

//    print_byte(buffer[0]);
//    print_byte(buffer[1]);
//    print_byte(buffer[2]);
//    print_byte(buffer[3]);

    i = segment1number(buffer);
    if(i != -1){
        number = i * 10;
        i = segment2number(buffer);
        if(i != -1){
            number += i;
            i = segment3number(buffer);
            if(i != -1){
                number += (i / 10.0);
                printf("  %02.1f\n", number );
            }
        }
    }
    i = segment0decode(buffer);
    switch(i)
    {
    case 'S':
        printf("Set shallow alarm\n");
        break;

    case 'D':
        printf("Set deep alarm\n");
        break;

    case 'A':
        printf("Alarm on\n");
        break;

    case 'T':
        printf("Set sensitivity threshold\n");
        break;

    case 'K':
        printf("Set keel offset\n");
        break;

    default:
        break;
    }
}


//Main application
void app_main(void)
{
    esp_err_t ret;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg={
        .mode=3,
        .spics_io_num=GPIO_CS,
        .queue_size=8,
        .flags=0,
    };

    //Initialize SPI slave interface
    ret=spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_DISABLED);
    assert(ret==ESP_OK);

    //WORD_ALIGNED_ATTR char sendbuf[129]="";
    WORD_ALIGNED_ATTR char recvbuf[8];
    memset(recvbuf, 0, sizeof(recvbuf));
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    /* Configure the peripheral according to the LED type */
    configure_led();

    while(1) {
        //Clear receive buffer
        memset(recvbuf, 0, sizeof(recvbuf));
//        memset(recvbuf, 0xA5, 129);

        s_led_state = 0;
        blink_led();

        //Set up a transaction of 128 bytes to send/receive
        t.length=4*8;
        t.tx_buffer=NULL;
        t.rx_buffer=recvbuf;
        /* This call enables the SPI slave interface to send/receive to the sendbuf and recvbuf. The transaction is
        initialized by the SPI master, however, so it will not actually happen until the master starts a hardware transaction
        by pulling CS low and pulsing the clock etc.
        */
        ret=spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        s_led_state = 1;
        blink_led();

        //spi_slave_transmit does not return until the master has done a transmission, so by here we have sent our data and
        //received data from the master. Print it.

        app_decode_segments(recvbuf);

        //printf("\n                         ^\n");
    }

}
