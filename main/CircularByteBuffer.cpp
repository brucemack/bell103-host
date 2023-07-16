#include "CircularByteBuffer.h"

CircularByteBuffer::CircularByteBuffer(uint8_t* workArea, unsigned int size)
:   _base(workArea),
    _size(size),
    _writePtr(0),
    _readPtr(0) {
}

void CircularByteBuffer::clear() {
    _writePtr = 0;
    _readPtr = 0;
}

bool CircularByteBuffer::canWrite() const {
    if (_next(_writePtr) == _readPtr) {
        return false;
    } else {
        return true;
    }
}

bool CircularByteBuffer::canRead() const {
    return _readPtr != _writePtr;
}

void CircularByteBuffer::write(uint8_t c) {
    if (canWrite()) {
        _base[_writePtr] = c;
        _writePtr = _next(_writePtr);
    }
}

uint8_t CircularByteBuffer::read() {
    if (!canRead()) {
        return 0;
    }
    uint8_t r = _base[_readPtr];
    _readPtr = _next(_readPtr);
    return r;
}

unsigned int CircularByteBuffer::_next(unsigned int current) const {
    return (current + 1) % _size;
}
