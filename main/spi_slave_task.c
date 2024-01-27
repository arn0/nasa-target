#include <string.h>
#include "esp_err.h"
#include "driver/spi_slave.h"

#include "sounder.h"
#include "spi_slave_task.h"

/*
From SPI receiver (slave) example.

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

void spi_slave_loop() {
    esp_err_t ret;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg= {
        .mode = 1,
        .spics_io_num = GPIO_CS,
        .queue_size = 8,
        .flags = 0,
    };

    //Initialize SPI slave interface
    ret = spi_slave_initialize( RCV_HOST, &buscfg, &slvcfg, SPI_DMA_DISABLED );
    assert( ret == ESP_OK );

    char recvbuf[8];
    spi_slave_transaction_t t;
    memset( &t, 0, sizeof(t) );

    while(1) {
        //Clear receive buffer
        memset( recvbuf, 0, sizeof(recvbuf) );

        //Set up a transaction of 4 bytes to send/receive
        t.length=4*8;
        t.tx_buffer=NULL;
        t.rx_buffer=recvbuf;
        
        /* This call enables the SPI slave interface to send/receive to the sendbuf and recvbuf. The transaction is
        initialized by the SPI master, however, so it will not actually happen until the master starts a hardware transaction
        by pulling CS low and pulsing the clock etc.
        */
        ret=spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        //spi_slave_transmit does not return until the master has done a transmission, so by here we have sent our data and
        //received data from the master. Print it.

    }
}

void spi_slave_start(){
    xTaskCreate( spi_slave_loop, "spi_slave_loop", 2048, NULL, 5, NULL );
}