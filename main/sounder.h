#ifndef sounder_h
#define sounder_h

#include <stdint.h>

 
struct v {
    float depth, shallow, deep, sensit, keel;
};


class sounder_control
{
private:
    static struct v value;
    
    struct s {
        unsigned int depth, shallow, deep, sensit, keel, alarm : 1;
    } status;
    

    int segment_digit_decoder( uint8_t segment );
    uint8_t get_segment_0( uint8_t *buffer );
    uint8_t get_segment_1( uint8_t *buffer );
    uint8_t get_segment_2( uint8_t *buffer );
    uint8_t get_segment_3( uint8_t *buffer );
    int segment_0_decoder( uint8_t segment );
    int segment_decoder( uint8_t *buffer );

public:
    static QueueHandle_t xQueue;
    QueueHandle_t queue_handle();

    void report();
    void queue_receive();

    sounder_control( QueueHandle_t handle );
    ~sounder_control();
};


#endif // sounder_h