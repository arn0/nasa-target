#ifndef spi_receiver_h
#define spi_receiver_h

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

class spi_receiver
{
private:
    /* data */
public:
    spi_receiver();
    ~spi_receiver();


    QueueHandle_t spi_xQueue;
    QueueHandle_t get_xQueue();
};



#endif // spi_receiver_h
