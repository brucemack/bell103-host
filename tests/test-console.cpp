#include <iostream>
#include "SerialPort.h"
#include "ShellProcessor.h"

class TestPort : public SerialPort {
public:

    TestPort() {        
    }

    void write(uint8_t c) {
        std::cout << c;
    }

    uint8_t read() {
    }

    bool isReadPending() {
    }
};

class TestEvent : public ShellProcessorEvent {
public:

    void handleCommand(const uint8_t* cmd) {       
        std::cout << "GOT COMMAND: [" << cmd << "]" << std::endl; 
    }
};

void test_1() {   

    TestPort port;
    TestEvent event;
    ShellProcessor<80> proc(&port, &event);

    // Backspace test
    proc.processInput("b");
    proc.processInput(8);
    proc.processInput("l");
    proc.processInput("s\n");

    // Clear line test
    proc.processInput("b");
    proc.processInput(18);
    proc.processInput("ls\n");
}

int main(int argc, const char** argv) {
    test_1();
}
