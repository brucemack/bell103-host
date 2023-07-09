INCDIRS = ../utils

tests:	test-console

test-console: test-console.o
	g++ -g -o test-console test-console.o 
	

test-console.o: test-console.cpp ../utils/ShellProcessor.h ../utils/SerialPort.h
	g++ -g -I$(INCDIRS) -c $<

%.o: %.cpp
	g++ -g -I$(INCDIRS) -c $<

clean:
	rm -f *.o
	rm -f test-console.exe

test: tests
	test-console

