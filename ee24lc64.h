#ifndef EE24LC64_H_INCLUDED
#define EE24LC64_H_INCLUDED

#include "Arduino.h"

class EE24LC64
{
  public:
    byte init ();
    byte init (int addr);
    byte init (int addr, bool set_16bit_addr);
    void write_byte(unsigned int eeaddress, byte data );
    void write_byte( int deviceaddress, unsigned int eeaddress, byte data );
    void write_page(unsigned int eeaddresspage, byte* data, byte length );
    void write_page( int deviceaddress, unsigned int eeaddresspage, byte* data, byte length );
    void write_buffer(unsigned int eeaddress, byte *data, byte length );
    void write_buffer(int deviceaddress, unsigned int eeaddress, byte *data, byte length );
    byte read_byte( unsigned int eeaddress );
    byte read_byte( int deviceaddress, unsigned int eeaddress );
    void read_buffer( unsigned int eeaddress, byte *buffer, int length );
    void read_buffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length );

    void set_eeprom_addr(int addr) { eeprom_addr = addr; }
  
  private:
    int eeprom_addr = 0x50;
    bool use_16bit_addr = true;
};

#endif // EE24LC64_H_INCLUDED

