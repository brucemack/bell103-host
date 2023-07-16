#ifndef _CicrularByteBuffer_h
#define _CircularByteBuffer_h

#include <stdlib.h>

class CircularByteBuffer {
public:

    CircularByteBuffer(uint8_t* workArea, unsigned int workAreaSize);

    void clear();
    bool canWrite() const;
    bool canRead() const;

    void write(uint8_t c);
    uint8_t read();

private:

    unsigned int _next(unsigned int current) const;

    uint8_t* _base;
    unsigned int _size;
    unsigned int _writePtr;
    unsigned int _readPtr;
};

#endif
