#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_slave.h"


#include "spi_receiver.h"


#define RX_QUEUE_LENGTH 16
#define RX_QUEUE_ITEM_SIZE 4


void spi_receiver_task( void *handle ) {
    esp_err_t ret;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 16,
        .flags = 0,
        .isr_cpu_id = INTR_CPU_ID_AUTO,
        .intr_flags = 0,
   };
    
    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg={
        .spics_io_num=GPIO_CS,
        .flags=0,
        .queue_size=8,
        .mode=1,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL,
    };

    //Initialize SPI slave interface
    ret=spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_DISABLED);
    assert(ret==ESP_OK);

    char recvbuf[8];
    memset(recvbuf, 0, sizeof(recvbuf));
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    while(1) {
        //Clear receive buffer
        memset(recvbuf, 0, sizeof(recvbuf));

        //Set up a transaction of 4 bytes to send/receive
        t.length=4*8;
        t.tx_buffer=NULL;
        t.rx_buffer=recvbuf;

        /* This call enables the SPI slave interface to send/receive to the sendbuf and recvbuf. The transaction is
        initialized by the SPI master, however, so it will not actually happen until the master starts a hardware transaction
        by pulling CS low and pulsing the clock etc.
        */
        ret=spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        //spi_slave_transmit does not return until the master has done a transmission, so by here we have
        //received data from the master.

		if( xQueueSendToBack( ( QueueHandle_t ) handle, ( void * ) recvbuf, ( TickType_t ) 0 ) == pdTRUE ) {
        }

    }
    vTaskDelete( NULL );
}

QueueHandle_t spi_xQueue;

spi_receiver::spi_receiver() {
    spi_xQueue = xQueueCreate( RX_QUEUE_LENGTH, RX_QUEUE_ITEM_SIZE );
    if( spi_xQueue != NULL ) {
        xTaskCreate( spi_receiver_task, "spi_receiver_task", 2048, (void *) spi_xQueue, 5, NULL );
    }
}

spi_receiver::~spi_receiver() {
}

QueueHandle_t spi_receiver::get_xQueue() {
    return( spi_xQueue );
}