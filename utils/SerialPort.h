#ifndef _SerialPort_h
#define _SerialPort_h

#include <stdint.h>

class SerialPort {
public:

    virtual void write(uint8_t) = 0;
    virtual uint8_t read() = 0;
    virtual bool isReadPending() = 0;
};

#endif
