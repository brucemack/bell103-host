#include "driver/gpio.h"
#include "cmx865a.h"

// In order on DEVKIT V1 board
#define PIN_DATAOUT GPIO_NUM_13
#define PIN_CLK GPIO_NUM_12
#define PIN_DATAIN GPIO_NUM_14
#define PIN_CS GPIO_NUM_27

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

void cmx865a::sendByte(uint8_t b) {
    // Wait util we can send
    while (!txReady()) { }
    write8(0xe3, b);
}

void cmx865a::send(const char* s) {
  for (unsigned int i = 0; s[i] != 0; i++) 
    sendByte(s[i]);
}
