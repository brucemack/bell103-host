#ifndef _ShellProcessor_h
#define _ShellProcessor_h

#include "SerialPort.h"

/**
 * Abstract interface used to report events from the ShellProcessor
 */
class ShellProcessorEvent {
public:

    virtual void handleCommand(const uint8_t* cmd) = 0;    
    virtual void handleOverflow() { }
};

class ShellProcessor {
public:

    ShellProcessor(SerialPort* port, ShellProcessorEvent* event) 
    :   _port(port),
        _event(event),
        _bufLen(0),
        _echo(true)
    {
    }

    void setEcho(bool on) {
        _echo = on;
    }

    void processInput(uint8_t c) {        
        // Look for backspace
        if (c == 8) {
            if (_bufLen > 0) {
                // Clear the character from the screen
                _port->write(8);
                _bufLen--;
                _buf[_bufLen] = 0;
            }
        }
        // Look for clear line 
        else if (c == 18) {
            _clearLine();
        }
        // Look for end-of-line 
        else if (c == 10) {
            // Optional echo 
            if (_echo) {
                _port->write(10);
                _port->write(13);
            }

            _buf[_bufLen] = 0;
            
            // Fire event
            if (_bufLen > 0) {
                _event->handleCommand(_buf);
            }

            // Reset
            _bufLen = 0;
        } 
        // Normal accumulaton
        else {
            // Look for overflow of buffer
            if (_bufLen == sizeof(_buf) - 1) {                
                _event->handleOverflow();
                _clearLine();
            } else {
                // Optional echo 
                if (_echo)
                    _port->write(c);
                _buf[_bufLen++] = c;
            }
        }
    }

    /**
     * Takes a null-terminated string and processes each character indpendently.
     */
    void processInput(const char* cs) {
        for (const char* p = cs; *p != 0; p++)
            processInput(*p);
    }

    void pullInputFromPort() {        
    }

private:

    void _clearLine() {
        // Clear line
        _port->write(27);
        _port->write('[');
        _port->write('2');
        _port->write('K');
        // Move to beginning of line
        _port->write(13);
        _buf[0] = 0;
        _bufLen = 0;
    }

    SerialPort* _port;
    ShellProcessorEvent* _event;
    unsigned int _bufLen;
    uint8_t _buf[80];
    bool _echo;
};

#endif
