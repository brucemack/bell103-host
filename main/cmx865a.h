#ifndef _cmx865a_h
#define _cmx865a_h

#include <stdint.h>

/**
 * A simple driver for the CMX865A modem/DTMF IC.
 */
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

  /**
   * Waits until txRead and then sends a byte on the UART
   */
  void sendByte(uint8_t b);

  /**
   * Sends multiple characters of text using the UART
   */
  void send(const char* s);

private:

  void _write8(uint8_t b);
  uint8_t _read8();
  void _setCS(int level);
  void _setCLK(int level);
  void _strobeCLK();
  void _setDATAIN(int level);
  int _getDATAOUT();
};

#endif
