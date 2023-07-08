#include <stdio.h>
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <thread>

// In order on DEVKIT V1 board
#define PIN_DATAOUT GPIO_NUM_13
#define PIN_CLK GPIO_NUM_12
#define PIN_DATAIN GPIO_NUM_14
#define PIN_CS GPIO_NUM_27
#define PIN_HOOKSWITCH GPIO_NUM_26
#define PIN_LED GPIO_NUM_2

class cmx865a {
public:

  cmx865a();

  void write0(uint8_t addr);
  void write8(uint8_t addr, uint8_t data);
  void write16(uint8_t addr, uint16_t data);

  uint8_t read8(uint8_t addr);
  uint16_t read16(uint8_t addr);

  // #### TODO: BOOLEAN
  int rxReady();
  int rxEnergy();
  int txReady();
  int programReady();

private:

  void _write8(uint8_t b);
  uint8_t _read8();
  void _setCS(int level);
  void _setCLK(int level);
  void _strobeCLK();
  void _setDATAIN(int level);
  int _getDATAOUT();
};

cmx865a::cmx865a() {
  gpio_reset_pin(PIN_CS);
  gpio_set_direction(PIN_CS, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_CS, 1);
  gpio_reset_pin(PIN_CLK);
  gpio_set_direction(PIN_CLK, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_CLK, 0);
  gpio_reset_pin(PIN_DATAIN);
  gpio_set_direction(PIN_DATAIN, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_DATAIN, 0);
  gpio_reset_pin(PIN_DATAOUT);
  gpio_set_direction(PIN_DATAOUT, GPIO_MODE_INPUT);
}

void cmx865a::_setCLK(int level) {
    gpio_set_level(PIN_CLK, level);
}

void cmx865a::_strobeCLK() {
  _setCLK(1);
  _setCLK(0);
}

void cmx865a::_setCS(int level) {
    gpio_set_level(PIN_CS, level);
}

void cmx865a::_setDATAIN(int level) {
    gpio_set_level(PIN_DATAIN, level);
}

int cmx865a::_getDATAOUT() {
    return gpio_get_level(PIN_DATAOUT);
}

void cmx865a::_write8(uint8_t b) {
  // MBS first! 
  for (unsigned int i = 0; i < 8; i++) {
    if (b & 0x80) {
      _setDATAIN(1);
    } else {
      _setDATAIN(0);
    }
    _strobeCLK();
    b = b << 1;
  }
}

uint8_t cmx865a::_read8() {
  uint8_t result = 0;
  for (unsigned int i = 0; i < 8; i++) {
    result = result << 1;
    // Per 5.11 data is valid when serial clock is high
    _setCLK(1);    
    if (_getDATAOUT() == 1) {
      result |= 1;
    }    
    _setCLK(0);    
  }
  return result;
}

void cmx865a::write0(uint8_t addr) {
  _setCS(0);
  _write8(addr);
  // ????
  _setDATAIN(0);        
  _setCS(1);    
}

void cmx865a::write8(uint8_t addr, uint8_t data) {
  _setCS(0);
  _write8(addr);
  _write8(data);
  // ????
  _setDATAIN(0);        
  _setCS(1);    
}

void cmx865a::write16(uint8_t addr, uint16_t data) {
  _setCS(0);
  _write8(addr);
  // MSByte
  _write8((data & 0xff00) >> 8);
  // LSByte
  _write8((data & 0x00ff));
  // ????
  _setDATAIN(0);        
  _setCS(1);    
}

uint8_t cmx865a::read8(uint8_t addr) {
  _setCS(0);
  _write8(addr);
  uint8_t d = _read8();
  _setCS(1);    
  return d;
}

uint16_t cmx865a::read16(uint8_t addr) {
  _setCS(0);
  _write8(addr);
  // MSByte
  uint8_t msb = _read8();
  // LSByte
  uint8_t lsb = _read8();
  _setCS(1);    
  return (msb << 8) | lsb;
}

int cmx865a::rxReady() {
  uint16_t a = read16(0xe6);
  return (a & 0b0000000001000000) != 0;
}

int cmx865a::rxEnergy() {
  uint16_t a = read16(0xe6);
  //            5432109876543210
  return (a & 0b0000010000000000) != 0;
}

int cmx865a::txReady() {
  uint16_t a = read16(0xe6);
  //            5432109876543210
  return (a & 0b0000100000000000) != 0;
}

int cmx865a::programReady() {
  uint16_t a = read16(0xe6);
  //            5432109876543210
  return (a & 0b0010000000000000) != 0;
}

/**
 * Enables modem (data) mode
 */
static void setupModem(cmx865a& modem) {
  // TX control
  // We are the answering modem (high band)
  //                    5432109876543210    
  modem.write16(0xe1, 0b0111011000010110);
  // TX OFF
  //modem.write16(0xe1, 0b0000111000010110);

  // RX control
  // This is the calling modem setting used for echo test:
  //              5432109876543210    
  //modem.write16(0xe2, 0b0111111000110110); 
  // We are the answering modem (low band)
  // NORMAL:
  //                    5432109876543210    
  modem.write16(0xe2, 0b0110000000110110); 
}

static void sendSilence(cmx865a& modem) {
  // TX control: Tone pair TA
  modem.write16(0xe1, 0b0001000000000000);
}

static void sendDialTone(cmx865a& modem) {
  // TX control: Tone pair TA
  modem.write16(0xe1, 0b0001000000001100);
}

static void sendRingTone(cmx865a& modem) {
  // TX control: Tone pair TB
  modem.write16(0xe1, 0b0001000000001101);
}

static void sendText(cmx865a& modem,const char* s) {
  for (unsigned int i = 0; s[i] != 0; i++) {
    // Wait util we can send
    while (!modem.txReady()) {
    }
    modem.write8(0xe3, (uint8_t)s[i]);
  }
}

/**
 * Simulation of the Arduino delay(ms) function.  Just sleeps.
 */
static void delay(unsigned long ms) {
  const TickType_t delay_ticks = ms / portTICK_PERIOD_MS;
  vTaskDelay(delay_ticks);
}

static unsigned long millis() {
  int64_t us_since_boot = esp_timer_get_time();
  return us_since_boot / 1000;
}

static void loop(cmx865a& modem);

static void run() {

  gpio_reset_pin(PIN_HOOKSWITCH);
  gpio_set_direction(PIN_HOOKSWITCH, GPIO_MODE_INPUT);
  gpio_reset_pin(PIN_LED);
  gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);

  cmx865a modem;

  // General reset of the CMX865A
  modem.write0(0x01);

  delay(20);

  // Write general control to power on
  //write16(0xe0, 0x0180); 
  //                    5432109876543210
  modem.write16(0xe0, 0b0000000110000000);
  
  // Per datasheet, start clock
  delay(20);

  // Configure analog loopback, power on
  //write16(0xe0, 0x0900);
  //                    5432109876543210
  modem.write16(0xe0, 0b0000100100000000);
    
  // Program the tone pairs
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x8000);

  // TA is used for US dial tone
  // TA1 frequency is 440, 440 * 3.414 = 1502, hex = 05DE
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x05DE);
  // TA1 amplitude is 0.5Vrms * 93780 / 3.3 = 14,209, hex = 3781
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x3981);
  // TA2 frequency is 350, 350 * 3.414 = 1194, hex = 04AA
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x04AA);
  // TA2 amplitude is 0.5Vrms * 93780 / 3.3 = 14,209, hex = 3781
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x3981);

  // TB is used for US ring tone
  // TB1 frequency is 440, 440 * 3.414 = 1502, hex = 05DE
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x05DE);
  // TB1 amplitude is 0.5Vrms * 93780 / 3.3 = 14,209, hex = 3781
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x3981);
  // TB2 frequency is 480, 480 * 3.414 = 1638 , hex = 0666 
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x0666);
  // TB2 amplitude is 0.5Vrms * 93780 / 3.3 = 14,209, hex = 3781
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x3981);

  printf("Programmed\n");

  // Show good startup
  gpio_set_level(PIN_LED, 1);
  delay(250);
  gpio_set_level(PIN_LED, 0);
  delay(250);
  gpio_set_level(PIN_LED, 1);
  delay(250);
  gpio_set_level(PIN_LED, 0);

  loop(modem);
}

#define ON_HOOK 1
#define OFF_HOOK 0

static void loop(cmx865a& modem) {

  int lastHs = 1;
  long lastHsTransition = 0;
  int hsState = ON_HOOK;
  int state = 0;
  long stateChangeStamp = 0;
  int clickCount = 0;
  int digitCount = 0;
  int dialDigits[10];

  while (true) {
       
    if (state == 11) {
      
      // Check for inbound on the modem
      if (modem.rxReady()) {
        uint8_t d = modem.read8(0xe5);
        //Serial.print((char)d);
        // Echo typed characters
        while (!modem.txReady()) { }
        modem.write8(0xe3, (uint8_t)d);
      }
    
      /*
      if (Serial.available()) {
        int c = Serial.read();
        // Wait util we can send
        while (!modem.txReady()) {
        }
        modem.write8(0xe3, (uint8_t)c);
      }
      */

      // Show RX energy indication on the LED
      if (modem.rxEnergy()) {
        gpio_set_level(PIN_LED, 1);
      } else {
        gpio_set_level(PIN_LED, 0);
      }
    }
    
    // Look for a hookswitch transition  
    const int hs = gpio_get_level(PIN_HOOKSWITCH);
    if (hs != lastHs) {
      lastHs = hs;
      lastHsTransition = millis();
    }  

    // Look for debounced hookswitch transition
    if (millis() - lastHsTransition > 50 && hsState != lastHs) {
      hsState = lastHs;
    }

    // State machine

    // Hung up
    if (state == 0) {
      if (hsState == OFF_HOOK) {
        // Short delay before the tone starts
        delay(200);      
        sendDialTone(modem);
        
        clickCount = 0;
        digitCount = 0;
        state = 1;
        stateChangeStamp = millis();
      }
    }
    // Sending dial tone
    else if (state == 1) {
      // Look for timeout
      if (millis() - stateChangeStamp > 15000) {
        sendSilence(modem);
        state = 99;
        stateChangeStamp = millis();
      }
      // Look for break.  Could be hang up or dialing.
      if (hsState == ON_HOOK) {
        // Turn off dial tone
        sendSilence(modem);
        state = 2;
        stateChangeStamp = millis();
      }
    }
    // A first break was detected, look for dialing
    else if (state == 2) {
      if (hsState == ON_HOOK) {
        // Look for hang up
        if (millis() - stateChangeStamp > 500) {
          state = 0;
          stateChangeStamp = millis();
        }
      }
      // Look for transition back off hook.  This indicates
      // dialing
      else if (hsState == OFF_HOOK) {
        state = 4;
        stateChangeStamp = millis();
        clickCount++;
      }
    }
    // In this state a click (rising edge) was just recorded
    else if (state == 4) {
      if (hsState == ON_HOOK) {
        state = 2;
        stateChangeStamp = millis();
      } else {
        // Look for digit timeout
        if (millis() - stateChangeStamp > 250) {
          dialDigits[digitCount] = clickCount;
          clickCount = 0;        
          digitCount++;
          // Check to see if we have a full number
          if (digitCount == 7) {
            state = 10;
          } else {
            state = 5;   
          }
        }
      }
    }
    // In this state we are waiting for the next digit to be dialed,
    // or timeout
    else if (state == 5) {
      // Look for break.  Could be hang up or more dialing.
      if (hsState == ON_HOOK) {
        state = 2;
        stateChangeStamp = millis();
      } else {
        // Look for inter-digit timeout
        if (millis() - stateChangeStamp > 10000) {
          // Send error
          state = 99;
          stateChangeStamp = millis();
        }
      }
    }
    else if (state == 10) {

      //Serial.print("Connecting to ");
      //for (int i = 0; i < digitCount; i++) 
      //  Serial.print(dialDigits[i]);
      //Serial.println();

      delay(1000);
      sendRingTone(modem);
      delay(2000);
      sendSilence(modem);
      delay(4000);
      sendRingTone(modem);
      delay(2000);
      sendSilence(modem);
      delay(4000);    

      setupModem(modem);
      state = 11;
      stateChangeStamp = millis();

      // Notify that we have a connection
      //Serial.write(10);   
      //Serial.print("AT+CONN=1.1.1.1");   
      //Serial.write(10);   

      // Send a message to the remote station
      delay(1000);
      sendText(modem, "You are now connected.\r\nWelcome to the 1980's\r\n\r\n");
    }
    else if (state == 11) {
      
      // Wait for hangup
      if (hsState == ON_HOOK) {

        //Serial.write(26);
        //Serial.write(26);
        //Serial.print("AT+DISC");
        //Serial.write(10);
        
        sendSilence(modem);
        
        state = 0;
        stateChangeStamp = millis();
      }
    }
    else if (state == 99) {
      if (hsState == ON_HOOK) {
        state = 0;
        stateChangeStamp = millis();
      }
    }

    std::this_thread::yield();
  }
}

extern "C" {
  void app_main() {
    run();
  }
}
