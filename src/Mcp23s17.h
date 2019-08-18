/*
 *   Mcp23s17.h - Library to handle Microchip MCP23S17 integrated circuits.
 *
 *   Copyright (C) 2019 Óscar Laborda
 *
 *   This file is part of Mcp23s17 library.
 *
 *   Mcp23s17 is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *   Mcp23s17 is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *  along with Mcp23s17.  If not, see <https://www.gnu.org/licenses/>.
 */


#ifndef _MCP23S17_H_
#define _MCP23S17_H_

#include <Arduino.h>
#include <SPI.h>

// Arduino has no endian.h header, but fortunately it is Little Endian
// so we can just define these macros to identity in that case.
#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#  include <endian.h>
#else
#  define le16toh(x) (x)
#  define htole16(x) (x)
#endif

/*
 *   If more than 1 chip is managed by an object instance, hardware addressing
 *  will be enabled (IOCON.HAEN = 1).
 *   If only 1 chip is used and hardware addressing is not forced by the user,
 *  it will not be activated; in that circumstance, the MCP23S17 Rev A chip has
 *  a bug that makes addressing chips with A2 address pin set to high only read
 *  opcodes where that address bit is also set to high.
 *   This is why when hardware addressing is disabled, the chip should always
 *  be initialized with that address bit set correctly in the addChip() method.
 *   Define macro DISABLE_MCP23S17_REV_A_BUGFIX in compile time if your chips
 *  are free of this bug:
 *   -DDISABLE_MCP23S17_REV_A_BUGFIX
 *  In this case, the address in the call to addChip() for a asingle chip
 *  does not matter, as it will be overwritten to 0b000.
 */

class Mcp23s17
{
public:
  // Chip pin identifier
  enum GpioPin: uint8_t
  {
    // Automatically assigned values 0 through 15
    GPA0, GPA1, GPA2, GPA3, GPA4, GPA5, GPA6, GPA7,
    GPB0, GPB1, GPB2, GPB3, GPB4, GPB5, GPB6, GPB7
  };

  // Chip pin identifier for masks
  enum GpioPinMask: word
  {
    GPA0_BIT = bit(GPA0),
    GPA1_BIT = bit(GPA1),
    GPA2_BIT = bit(GPA2),
    GPA3_BIT = bit(GPA3),
    GPA4_BIT = bit(GPA4),
    GPA5_BIT = bit(GPA5),
    GPA6_BIT = bit(GPA6),
    GPA7_BIT = bit(GPA7),
    GPB0_BIT = bit(GPB0),
    GPB1_BIT = bit(GPB1),
    GPB2_BIT = bit(GPB2),
    GPB3_BIT = bit(GPB3),
    GPB4_BIT = bit(GPB4),
    GPB5_BIT = bit(GPB5),
    GPB6_BIT = bit(GPB6),
    GPB7_BIT = bit(GPB7)
  };

  // We need to redefine interrupt modes because both HIGH and CHANGE
  // are defined to 1 in Arduino
  enum InterruptMode: uint8_t
  {
    INT_OFF, INT_LOW, INT_HIGH, INT_CHANGE
  };

  // Class public constants
  static const uint8_t GPIO_NUM_PINS = 16;  // Total I/O pins per chip
  static const uint8_t GPIO_NUM_PORTS = 2;  // Each chip has 2 GPIO ports
  static const uint8_t GPIO_NUM_PINS_PER_PORT = GPIO_NUM_PINS / GPIO_NUM_PORTS;
  static const uint8_t MAX_CHIPS = 8;       // Max number of chips per CS line
  static const byte ADDR_NOHW = 0b000;

  // Public methods
  Mcp23s17(uint8_t Cs);
  bool addChip(byte Addr=ADDR_NOHW, uint8_t *pChipId=nullptr);
  bool begin(bool HwAddressing=false) const;
  bool configInterrupts(uint8_t ChipId, uint8_t Polarity, bool Mirror=false,
    bool OpenDrain=false) const;
  bool configGpio(uint8_t ChipId, GpioPinMask PinMask, uint8_t Mode,
    bool PolInverse=false, InterruptMode IntMode=INT_OFF) const;

  uint8_t readPin(uint8_t ChipId, GpioPin Pin) const;
  void writePin(uint8_t ChipId, GpioPin Pin, uint8_t Value) const;

  inline word readPort(uint8_t ChipId) const;
  inline byte readPortA(uint8_t ChipId) const;
  inline byte readPortB(uint8_t ChipId) const;

  inline void writePort(uint8_t ChipId, word Value) const;
  inline void writePortA(uint8_t ChipId, byte Value) const;
  inline void writePortB(uint8_t ChipId, byte Value) const;

  inline word readIntFlags(uint8_t ChipId) const;
  inline byte readIntFlagsA(uint8_t ChipId) const;
  inline byte readIntFlagsB(uint8_t ChipId) const;

  inline word readIntCaptured(uint8_t ChipId) const;
  inline byte readIntCapturedA(uint8_t ChipId) const;
  inline byte readIntCapturedB(uint8_t ChipId) const;

  void end() const;

protected:
  // Register addressed for default IOCON.BANK = 0
  enum RegAddr: byte  
  {
    _IODIR =   0x00, _IODIRA =   0x00, _IODIRB =   0x01,
    _IPOL =    0x02, _IPOLA =    0x02, _IPOLB =    0x03,
    _GPINTEN = 0x04, _GPINTENA = 0x04, _GPINTENB = 0x05,
    _DEFVAL =  0x06, _DEFVALA =  0x06, _DEFVALB =  0x07,
    _INTCON =  0x08, _INTCONA =  0x08, _INTCONB =  0x09,
    _IOCON =   0x0A, _IOCONA =   0x0A, _IOCONB =   0x0B,
    _GPPU =    0x0C, _GPPUA =    0x0C, _GPPUB =    0x0D,
    _INTF =    0x0E, _INTFA =    0x0E, _INTFB =    0x0F,
    _INTCAP =  0x10, _INTCAPA =  0x10, _INTCAPB =  0x11,
    _GPIO =    0x12, _GPIOA =    0x12, _GPIOB =    0x13,
    _OLAT =    0x14, _OLATA =    0x14, _OLATB =    0x15
  };

  // Bit values for registers
  enum: byte
  {
    // I/O direction register
    _IODIR_OUTPUT = 0, _IODIR_INPUT = 1, _IODIR_DEFAULT = _IODIR_INPUT,
    // Input polarity register
    _IPOL_NORMAL = 0, _IPOL_INVERTED = 1, _IPOL_DEFAULT = _IPOL_NORMAL,
    // Interrupt-on-change control register
    _GPINTEN_DISABLED = 0, _GPINTEN_ENABLED = 1,
      _GPINTEN_DEFAULT = _GPINTEN_DISABLED,
    // Default compare for interrupt-on-change register
    // DEFVAL: interrupt on opposite value to DEFVAL value -> HIGH/LOW inverted
    _DEFVAL_HIGH = 0, _DEFVAL_LOW = 1, _DEFVAL_DEFAULT = _DEFVAL_HIGH,
    // Interrupt control register: with what to compare interrupt on change
    _INTCON_PREVVAL = 0, _INTCON_DEFVAL = 1, _INTCON_DEFAULT = _INTCON_PREVVAL,
    // Pullup register
    _GPPU_DISABLED = 0, _GPPU_ENABLED = 1, _GPPU_DEFAULT = _GPPU_DISABLED,
    // Configuration register
    _IOCON_DISABLED = 0, _IOCON_ENABLED = 1, _IOCON_DEFAULT = _IOCON_DISABLED
  };

  // IOCON function position
  enum: byte
  {
    // _IOCON_U =      0,  // Unused
    _IOCON_INTPOL = 1,
    _IOCON_ODR =    2,
    _IOCON_HAEN =   3,
    // _IOCON_DISSLW = 4,  // I2C stuff
    _IOCON_SEQOP =  5,
    _IOCON_MIRROR = 6,
    _IOCON_BANK =   7
  };

  // IOCON bit mask
  enum: byte
  {
    // _IOCON_U_BIT =      bit(_IOCON_U),  // Unused
    _IOCON_INTPOL_BIT = bit(_IOCON_INTPOL),
    _IOCON_ODR_BIT =    bit(_IOCON_ODR),
    _IOCON_HAEN_BIT =   bit(_IOCON_HAEN),
    // _IOCON_DISSLW_BIT = bit(_IOCON_DISSLW),  // I2C stuff
    _IOCON_SEQOP_BIT =  bit(_IOCON_SEQOP),
    _IOCON_MIRROR_BIT = bit(_IOCON_MIRROR),
    _IOCON_BANK_BIT =   bit(_IOCON_BANK)
  };

  // Protected class constants
  static const byte _OPCODE_R = 0b01000001;
  static const byte _OPCODE_W = 0b01000000;
  static const byte _OPCODE_NOOP = 0x00;
  static const byte _ADDR_NOHW_BUG = 0b100;
  static const uint8_t _OPCODE_ADDR_SHIFT = 1;
  static const uint32_t _CLOCK_MAX_FREQ = 10E6;  // Max freq of chip is 10MHz
  static const SPISettings _SpiSettings;

  // Object protected data
  const uint8_t _Cs;  // Chip Select Arduino pin
  uint8_t _NumChips;  // Number of MCP chips in this CS line (max 8)
  byte _ChipOcAddr[MAX_CHIPS];  // Address in opcode format (already shifted)

  // Protected methods
  void _enableHwAddressing() const;
  bool _gpioIoMode(uint8_t PinMode, uint8_t *pIodir, uint8_t *pGppu) const;
  void _gpioPolMode(bool PolInverse, uint8_t *pIpol) const;
  bool _gpioIntMode(InterruptMode IntMode, uint8_t *pGpinten, uint8_t *pDefval,
    uint8_t *pIntcon) const;
  byte _read(byte OcAddr, RegAddr Reg) const;
  void _read(byte OcAddr, RegAddr StartReg, byte *pData, uint8_t Count) const;
  void _write(byte OcAddr, RegAddr Reg, byte Value) const;
  void _write(byte OcAddr, RegAddr StartReg, const byte *pData, uint8_t Count)
    const;

  inline static byte _opcodeAddr(byte Addr);
  inline static byte _opcodeRead(byte OcAddr);
  inline static byte _opcodeWrite(byte OcAddr);
};


/*
 *   Reads both general purpose I/O ports (A & B) from the GPIO registers and
 *  returns them as a 16bit word. This read will clear both ports interrupt
 *  flags and allow changes to INTCAP registers.
 *   When ChipId is not valid, returns with undefined value and no action.
 *  Parameters:
 *  * ChipId: which of the managed MCP23S17 chips we are reading.
 *  Returns: a 16 bit word with the GPIO value, containing
 *           port A in the low byte and port B in the high byte.
 */
inline word Mcp23s17::readPort(uint8_t ChipId) const
{
  word Value;

  // Check chip id and return when invalid
  if (ChipId >= _NumChips)
    return 0U;

  // Read GPIOA/B. Convention: A:low B:high => got it stored as Little Endian
  _read(_ChipOcAddr[ChipId], _GPIO, (byte *) &Value, sizeof Value);

  // Convert Little Endian to host CPU endian
  return le16toh(Value);
}


/*
 *   Reads general purpose I/O port A from the GPIOA register and returns it
 *  as a 8bit byte. This read will clear ports A interrupt flags and allow
 *  changes to INTCAPA register.
 *   When ChipId is not valid, returns with undefined value and no action.
 *  Parameters:
 *  * ChipId: which of the managed MCP23S17 chips we are reading.
 *  Returns: 8 bit byte with the GPIO A port value
 */
inline byte Mcp23s17::readPortA(uint8_t ChipId) const
{
  // Check chip id and return when invalid
  if (ChipId >= _NumChips)
    return 0U;

  // Read GPIOA
  return _read(_ChipOcAddr[ChipId], _GPIOA);
}


/*
 *   Reads general purpose I/O port B from the GPIOB register and returns it
 *  as a 8bit byte. This read will clear ports B interrupt flags and allow
 *  changes to INTCAPB register.
 *   When ChipId is not valid, returns with undefined value and no action.
 *  Parameters:
 *  * ChipId: which of the managed MCP23S17 chips we are reading.
 *  Returns: 8 bit byte with the GPIO B port value
 */
inline byte Mcp23s17::readPortB(uint8_t ChipId) const
{
  // Check chip id and return when invalid
  if (ChipId >= _NumChips)
    return 0U;

  // Read GPIOB
  return _read(_ChipOcAddr[ChipId], _GPIOB);
}


/*
 *   Writes both general purpose I/O ports (A & B), passed as the Value 16 bit
 *  word. The write is performed in the OLAT output registers.
 *   When ChipId is not valid, returns without writing anything.
 *  Parameters:
 *  * ChipId: which of the managed MCP23S17 chips we are writing.
 *  * Value: 16 bit word where the desired otput GPIO values are stored,
 *    containing A port in the low byte and B port in the high byte.
 */
inline void Mcp23s17::writePort(uint8_t ChipId, word Value) const
{
  // Check chip id and return when invalid
  if (ChipId >= _NumChips)
    return;

  // Need to send A first, then B => we need Value in Little Endian
  Value = htole16(Value);
  // Write OLATA/B. Convention: A first, then B
  _write(_ChipOcAddr[ChipId], _OLAT, (byte *) &Value, sizeof Value);
}


/*
 *   Writes port A general purpose I/O port, passed as the Value 8 bit
 *  byte. The write is performed in the OLATA output register.
 *   When ChipId is not valid, returns without writing anything.
 *  Parameters:
 *  * ChipId: which of the managed MCP23S17 chips we are writing.
 *  * Value: 8 bit byte where the desired otput GPIO A values are stored.
 */
inline void Mcp23s17::writePortA(uint8_t ChipId, byte Value) const
{
  // Check chip id and return when invalid
  if (ChipId >= _NumChips)
    return;

  // Write OLATA
  _write(_ChipOcAddr[ChipId], _OLATA, Value);
}


/*
 *   Writes port B general purpose I/O port, passed as the Value 8 bit
 *  byte. The write is performed in the OLATB output register.
 *   When ChipId is not valid, returns without writing anything.
 *  Parameters:
 *  * ChipId: which of the managed MCP23S17 chips we are writing.
 *  * Value: 8 bit byte where the desired otput GPIO B values are stored.
 */
inline void Mcp23s17::writePortB(uint8_t ChipId, byte Value) const
{
  // Check chip id and return when invalid
  if (ChipId >= _NumChips)
    return;

  // Write OLATB
  _write(_ChipOcAddr[ChipId], _OLATB, Value);
}


/*
 *   Reads both interrupt flag ports (A & B) from the INTF registers and
 *  and returns them as a 16bit word.
 *   When ChipId is not valid, returns with undefined value with no action.
 *  Parameters:
 *  * ChipId: which of the managed MCP23S17 chips we are using.
 *  Returns: 16 bit word with the INTF registers, containing
 *           port A flags in the low byte and port B flags in the high byte.
 */
inline word Mcp23s17::readIntFlags(uint8_t ChipId) const
{
  word Flags;

  // Check chip id and return when invalid
  if (ChipId >= _NumChips)
    return 0U;

  // Read INTFA/B. Convention: A:low B:high => got it stored as Little Endian
  _read(_ChipOcAddr[ChipId], _INTF, (byte *) &Flags, sizeof Flags);

  // Convert Little Endian to host CPU endian
  return le16toh(Flags);
}


/*
 *   Reads interrupt flag port A from the INTFA register and returns it
 *  as a 8bit byte.
 *   When ChipId is not valid, returns with undefined value with no action.
 *  Parameters:
 *  * ChipId: which of the managed MCP23S17 chips we are using.
 *  Returns: 8 bit byte with the INTF A port values
 */
inline byte Mcp23s17::readIntFlagsA(uint8_t ChipId) const
{
  // Check chip id and return when invalid
  if (ChipId >= _NumChips)
    return 0U;

  // Read INTFA
  return _read(_ChipOcAddr[ChipId], _INTFA);
}


/*
 *   Reads interrupt flag port B from the INTFB register and returns it
 *  as a 8bit byte.
 *   When ChipId is not valid, returns with undefined value with no action.
 *  Parameters:
 *  * ChipId: which of the managed MCP23S17 chips we are using.
 *  Returns: 8 bit byte with the INTF B port values
 */
inline byte Mcp23s17::readIntFlagsB(uint8_t ChipId) const
{
  // Check chip id and return error when invalid
  if (ChipId >= _NumChips)
    return 0U;

  // Read INTFB
  return _read(_ChipOcAddr[ChipId], _INTFB);
}


/*
 *   Reads both interrupt capture (A & B) from the INTCAP registers and
 *  and returns them as a 16bit word. This read will clear both ports interrupt
 *  flags and allow changes to INTCAP registers.
 *   When ChipId is not valid, returns with undefined value with no action.
 *  Parameters:
 *  * ChipId: which of the managed MCP23S17 chips we are using.
 *  Returns: 16 bit word with the INTCAP registers, containing
 *           port A flags in the low byte and port B flags in the high byte.
 */
inline word Mcp23s17::readIntCaptured(uint8_t ChipId) const
{
  word Cap;

  // Check chip id and return when invalid
  if (ChipId >= _NumChips)
    return 0U;

  // Read INTCAPA/B. Convention: A:low B:high => got it stored as Little Endian
  _read(_ChipOcAddr[ChipId], _INTCAP, (byte *) &Cap, sizeof Cap);

  // Convert Little Endian to host CPU endian
  return le16toh(Cap);
}


/*
 *   Reads interrupt capture A from the INTCAPA register and returns it
 *  as a 8bit byte. This read will clear ports A interrupt flags and allow
 *  changes to INTCAPA register.
 *   When ChipId is not valid, returns with undefined value with no action.
 *  Parameters:
 *  * ChipId: which of the managed MCP23S17 chips we are using.
 *  Returns: 8 bit byte with the INTCAP A port values
 */
inline byte Mcp23s17::readIntCapturedA(uint8_t ChipId) const
{
  // Check chip id and return when invalid
  if (ChipId >= _NumChips)
    return 0U;

  // Read INTCAPA
  return _read(_ChipOcAddr[ChipId], _INTCAPA);
}


/*
 *   Reads interrupt capture A from the INTCAPB register and returns it
 *  as a 8bit byte. This read will clear ports B interrupt flags and allow
 *  changes to INTCAPB register.
 *   When ChipId is not valid, returns with undefined value with no action.
 *  Parameters:
 *  * ChipId: which of the managed MCP23S17 chips we are using.
 *  Returns: 8 bit byte with the INTCAP B port values
 */
inline byte Mcp23s17::readIntCapturedB(uint8_t ChipId) const
{
  // Check chip id and return when invalid
  if (ChipId >= _NumChips)
    return 0U;

  // Read INTCAPB
  return _read(_ChipOcAddr[ChipId], _INTCAPB);
}


/*
 *   Converts and address into opcode format.
 *  Parameters:
 *  * Addr: a chip address with the 3 lower bits determining the address.
 *  Returns: an Opcode addrres where the address bits take their corresponding
 *  places in the Opcode.
 */
inline byte Mcp23s17::_opcodeAddr(byte Addr)
{
  return Addr << _OPCODE_ADDR_SHIFT;
}


/*
 *   Composes a read opcode with the given address, already in opcode format.
 *  Parameters:
 *  * OcAddr: opcode address to merge into the opcode.
 *  Returns: the opcode to read from the chip with the desired address.
 */
inline byte Mcp23s17::_opcodeRead(byte OcAddr)
{
  return _OPCODE_R | OcAddr;
}


/*
 *   Composes a write opcode with the given address, already in opcode format.
 *  Parameters:
 *  * OcAddr: opcode address to merge into the opcode.
 *  Returns: the opcode to write to the chip with the desired address.
 */
inline byte Mcp23s17::_opcodeWrite(byte OcAddr)
{
  return _OPCODE_W | OcAddr;
}


#endif  // _MCP23S17_H_
