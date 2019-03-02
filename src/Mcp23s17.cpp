#include "Mcp23s17.h"
#include <string.h>

// When IOCON.HAEN = 0 (hardware addressing disabled): If the A2 pin is high,
// then the device must be addressed as A2, A1, A0 = 1xx
// (i.e. OPCODE = b0100 1XX)
// See document 80311a.pdf MCP23S13 Rev. A Silicon Errdata for more info.
// Define macro DISABLE_MCP23S17_REV_A_BUGFIX if your chips are free of this
// bug:
//  #define DISABLE_MCP23S17_REV_A_BUGFIX
// or in compile time with parameter:
//  -DDISABLE_MCP23S17_REV_A_BUGFIX


// Initialize static member data

const SPISettings Mcp23s17::_SpiSettings =
{
  _CLOCK_MAX_FREQ,
  MSBFIRST,
  SPI_MODE0
};


/*
 *   Constructor. Initialized the class with the Chip Select pin number that
 *  will be used to manage all chips in this object.
 *   Parameters:
 *   * Cs: Chip Select (CS, aka Slave Select, SS) is the pin in the Arduino
 *     where the CS pin for all the MCP23S17 chips managed by this class will
 *     be connected.
 */
Mcp23s17::Mcp23s17(uint8_t Cs):
  _Cs(Cs)
{
  _NumChips = 0U;
}


/*
 *   Registers a new chip to be managed by the class in the same CS channel.
 *  When there is more than one chip, they will be accessed by their address.
 *  The address should be unique inside the CS channel.
 *  Parameters:
 *  * Addr: address of the chip (bits representing the A2,A1,A0 address
 *    selection pins).
 *  * pChipId: when not nullptr, it will be filled with the Id that will be used
 *    from now on to reference this chip inside the class. Ids are returned
 *    sequentially starting with 0.
 *  Returns:
 *  * false when success.
 *  * true when Addr is not valid or is already in use (includin all chip slots
 *    are filled).
 */
bool Mcp23s17::addChip(byte Addr /* =DEFAULT_ADDR */,
  uint8_t *pChipId /* = nullptr */)
{
  byte OcAddr;
  uint8_t ChipIdx;

  // Check address and available chip slots
  if (Addr >= MAX_CHIPS || _NumChips >= MAX_CHIPS)
    return true;

  // Check that chip address is not already in use
  OcAddr = _opcodeAddr(Addr);
  for (ChipIdx = 0; ChipIdx < _NumChips; ChipIdx++)
    if (OcAddr == _ChipOcAddr[ChipIdx])
      return true;

  // Register chip inside the address array and update the count of chips
  if (pChipId != nullptr)
    *pChipId = _NumChips;
  _ChipOcAddr[_NumChips++] = OcAddr;

  return false;
}


/*
 *   Configures CS pin in the Arduino, enables SPI communications and
 *  configures hardware addressing. Hardware addressing will be enabled
 *  when there is more than one chip registered or
 *  when the corresponding parameter ForceHwAddressing is true.
 *  Parameters:
 *  * ForceHwAddressing: when true, enables hardware addressing of chips even
 *    when only one chip is registered. When false, hardware addressing will
 *    only be activated when only one chip is registered.
 *  Returns:
 *  * true when there are no registered chips to manage; initialization
 *    is not performed in that case.
 *  * false when successful.
 */
bool Mcp23s17::begin(bool ForceHwAddressing) const
{
  if (_NumChips == 0)
    return true;

  // Initialize SPI library
  SPI.begin();

  // Init CS pin and disable communication
  pinMode(_Cs, OUTPUT);
  digitalWrite(_Cs, HIGH);

  if (ForceHwAddressing || _NumChips > 1)
    _enableHwAddressing();

  return false;
}


/*
 *   Configures a managed chip ChipId on how the interrupts will be generated
 *  when configured in the corresponding registers.
 *  Parameters:
 *  * ChipId: id of the managed chip in which to enable interrupts.
 *  * Polarity (LOW, HIGH): configures the INT pins as either active
 *    LOW or active HIGH according to this parameter.
 *  * Mirror: enable (true) or disable (false) the interrupt pin mirror feature.
 *    - when true, both INTA and INTB pins will always be synced and both will
 *    rise simultaneously when an interrupt is generated on either of the two
 *    ports A or B.
 *    - when false, INTA and INTB pins are kept appart and each will only rise
 *    when an interrupt is generated in its own port.
 *  * OpenDrain: configures INT pins as open drain output. Useful when INT pins
 *    from several chips are attached to the same Arduino pin. Configure as
 *    open drain and pull it up or down in the circuit.
 *  Returns:
 *  * false when success.
 *  * true when ChipId or Polarity is invalid.
 */
bool Mcp23s17::configInterrupts(uint8_t ChipId, uint8_t Polarity,
  bool Mirror /* =false */, bool OpenDrain /* =false */) const
{
  byte Iocon;

  // Check parameters and return error when invalid
  if (ChipId >= _NumChips || (Polarity != LOW && Polarity != HIGH))
    return true;

  // Read IOCON register
  Iocon = _read(_ChipOcAddr[ChipId], _IOCON);

  // Configure interrupt bits in IOCON
  bitWrite(Iocon, _IOCON_INTPOL, Polarity);
  bitWrite(Iocon, _IOCON_MIRROR, (uint8_t) Mirror);
  bitWrite(Iocon, _IOCON_ODR, (uint8_t) OpenDrain);

  // Write IOCON back to the chip
  _write(_ChipOcAddr[ChipId], _IOCON, Iocon);

  return false;
}


/*
 *   Configures GPIO pins for I/O and interrupts as required by the caller.
 *  Parameters:
 *  * ChipId: which of the managed MCP23S17 chips we are configuring.
 *  * PinMask: mask with a bit enabled for each pin we are configuring.
 *  * IoMode (INPUT, OUPUT, OUTPUT_PULLUP): desired I/O function for the pins
 *    selected in PinMask.
 *  * PolInverse: whether to activate inverse polarity or not for the selected
 *    pins in PinMask. When enabled, the signal will be treated as inverted
 *    by the chip.
 *  * IntMode: desired interrupt function for the selected pins in PinMask.
 *  Returns:
 *  * false when success.
 *  * true when a paramter has invalid value.
 */
bool Mcp23s17::configGpio(uint8_t ChipId, GpioPinMask PinMask, uint8_t IoMode,
  bool PolInverse, InterruptMode IntMode) const
{
  // Buffer for GPIO control registers
  byte Registers[_INTCAPB+1];
  byte PortPinMask;
  uint8_t Port, Pin;
  uint8_t Iodir, Ipol, Gpinten, Defval, Intcon, Gppu;

  // Check parameters and return error
  if (ChipId >= _NumChips)
    return true;

  // Read all GPIO related registers: IODIRA through INTCAPB
  _read(_ChipOcAddr[ChipId], _IODIR, Registers, sizeof Registers);

  // Get values for registers
      _gpioPolMode(PolInverse, &Ipol);
  if (_gpioIoMode(IoMode, &Iodir, &Gppu) ||
      _gpioIntMode(IntMode, &Gpinten, &Defval, &Intcon))
    return true;

  // Modify Registers according to request

  // Loop over the chip ports (A, B)
  for (Port = 0; Port < GPIO_NUM_PORTS; Port++)
  {
    // Get a byte of the PinMask in each iteration corresponding to
    // the port, from Low to High (0 is port A; 1 is port B)
    PortPinMask = (byte) (PinMask >> (Port*8));

    // Check each pin in the port
    for (Pin = 0; Pin < GPIO_NUM_PINS_PER_PORT; Pin++, PortPinMask>>=1)
    {
      // Check if the mask for the current pin is set
      if (bitRead(PortPinMask, 0))
      {
        // For set pins in mask, activate/disable requested features
        bitWrite(Registers[_IODIR + Port], Pin, Iodir);
        bitWrite(Registers[_IPOL + Port], Pin, Ipol);
        bitWrite(Registers[_GPINTEN + Port], Pin, Gpinten);
        bitWrite(Registers[_DEFVAL + Port], Pin, Defval);
        bitWrite(Registers[_INTCON + Port], Pin, Intcon);
        bitWrite(Registers[_GPPU + Port], Pin, Gppu);
     }
    }
  }

  // Write back GPIO related registers
  _write(_ChipOcAddr[ChipId], _IODIR, Registers, sizeof Registers);

  return false;
}

/*
 *   Reads general purpose I/O port corresponding to the requested Pin
 *  and returns the value of that Pin. It is not checked that the Pin
 *  is configured as input.
 *   As readPort* methods this method also resets interrupts in the read port
 *  and enables changes in the corresponging INTCAP register.
 *   Note how this is an inefficient method to check several pins as each
 *  call will trigger a full MCP chip port read through SPI.
 *   Its purpose is mainly for test or debug or when only one input pin is used.
 *   When ChipId is not valid, returns with undefined value and no action.
 *  Parameters:
 *  * ChipId: which of the managed MCP23S17 chips we are using.
 *  * Pin: GPIO pin which value will be returned.
 *  Returns: value of the requested pin
 *  * LOW
 *  * HIGH
 *  * On invalid argument, or when the pin is not configured as input,
 *    the return value is undetermined
 */
uint8_t Mcp23s17::readPin(uint8_t ChipId, GpioPin Pin) const
{
  RegAddr Register;
  uint8_t PortPin;
  byte Values;

  // Check chip id and return when invalid
  if (ChipId >= _NumChips)
    return 0U;

  // Select port and normalize pin
  if (Pin < GPIO_NUM_PINS_PER_PORT)
  {
    Register = _GPIOA;
    PortPin = Pin;
  }
  else
  {
    Register = _GPIOB;
    PortPin = Pin - GPIO_NUM_PINS_PER_PORT;
  }
  
  // Read register and return normalized bit value
  Values = _read(_ChipOcAddr[ChipId], Register);
  return bitRead(Values, PortPin);
}

/*
 *   Writes a single general purpose I/O Pin value by reading the OLAT register
 *  and writing it back with only the selected Pin modified. It is not checked
 *  that the Pin is configured as output.
 *   As readPort* methods this method also resets interrupts in the read
 *  port and enables changes in the corresponging INTCAP register.
 *   Note how this is an inefficient method to write several pins as each
 *  call will trigger a full MCP chip OLAT register read and write through SPI.
 *   Its purpose is mainly for test or debug or when only one output pin
 *  is used.
 *   When ChipId is not valid, returns without writing anything.
 *  Parameters:
 *  * ChipId: which of the managed MCP23S17 chips we are using.
 *  * Pin: GPIO pin which value will be modified.
 *  * Value (LOW, HIGH): state at which the Pin should be set. Anything
 *    different from LOW is considered HIGH.
 */
void Mcp23s17::writePin(uint8_t ChipId, GpioPin Pin, uint8_t Value) const
{
  RegAddr Register;
  uint8_t PortPin;
  byte OlatValues;

  // Check chip id and return when invalid
  if (ChipId >= _NumChips)
    return;

  // Select port and normalize pin
  if (Pin < GPIO_NUM_PINS_PER_PORT)
  {
    Register = _OLATA;
    PortPin = Pin;
  }
  else
  {
    Register = _OLATB;
    PortPin = Pin - GPIO_NUM_PINS_PER_PORT;
  }
  
  // Read OLAT register, set the Pin value and write the register back
  OlatValues = _read(_ChipOcAddr[ChipId], Register);
  bitWrite(OlatValues, PortPin, Value);
  _write(_ChipOcAddr[ChipId], Register, OlatValues);

}


/*
 *   Disable further communications through SPI.
 */
void Mcp23s17::end() const
{
  SPI.end();
}


/*
 *   Initializes all chips in the _Cs line to hardware addressing mode. This is
 *  done twice, for addresses 0b000 and 0b100 due to a bug in the chip design.
 *  Define macro DISABLE_MCP23S17_REV_A_BUGFIX if your chips are free of this
 *  bug.
 *  When IOCON.HAEN = 0 (hardware addressing disabled): If the A2 pin is high,
 *  then the device must be addressed as A2, A1, A0 = 1xx
 *  (i.e. OPCODE = b0100 1XX)
 *  See document 80311a.pdf MCP23S13 Rev. A Silicon Errdata for more info.
 */
void Mcp23s17::_enableHwAddressing() const
{
#ifndef DISABLE_MCP23S17_REV_A_BUGFIX
  _write(_ADDR_NOHW_BUG, _IOCON, _IOCON_HAEN_BIT);
#endif
  _write(_ADDR_NOHW, _IOCON, _IOCON_HAEN_BIT);
}


/*
 *   Returns register values for desired GPIO pin I/O configuration.
 *  Parameters:
 *  * IoMode (INPUT, OUPUT, OUTPUT_PULLUP): desired I/O function for the pin
 *  * pIodir: a bit value indicating whether the corresponding IODIR register
 *    bit should be enabled or disabled.
 *  * pGppu: a bit value indicating whether the corresponding GPPU register
 *    bit should be enabled or disabled.
 *  Returns:
 *  * false when no error.
 *  * true when IoMode has an invalid value.
 */
bool Mcp23s17::_gpioIoMode(uint8_t IoMode, uint8_t *pIodir, uint8_t *pGppu)
  const
{
  switch (IoMode)
  {
  case INPUT:
    *pIodir = _IODIR_INPUT;
    *pGppu = _GPPU_DISABLED;
    break;
  case OUTPUT:
    *pIodir = _IODIR_OUTPUT;
    *pGppu = _IODIR_DEFAULT;  // Does not apply to output
    break;
  case INPUT_PULLUP:
    *pIodir = _IODIR_INPUT;
    *pGppu = _GPPU_ENABLED;
    break;
  default:
    // Invalid parameter: return error
    return true;
  }
  return false;
}


/*
 *   Returns register values for desired GPIO pin polarity configuration.
 *  Parameters:
 *  * PolInverse: whether to activate inverse polarity or not
 *  * pIpol: a bit value indicating whether the corresponding IPOL register
 *    bit should be enabled or disabled.
*/
void Mcp23s17::_gpioPolMode(bool PolInverse, uint8_t *pIpol) const
{
  *pIpol = PolInverse? _IPOL_INVERTED: _IPOL_NORMAL;
}


/*
 *   Returns register values for desired interrupt configuration.
 *  Parameters:
 *  * IntMode: desired interrupt function for the pin
 *  * pGpinten: a bit value indicating whether the corresponding GPINTEN
 *    register bit should be enabled or disabled.
 *  * pDefval: a bit value indicating whether the corresponding DEFVAL register
 *    bit should be enabled or disabled.
 *  * pIntcon: a bit value indicating whether the corresponding INTCON register
 *    bit should be enabled or disabled.
 *  Returns:
 *  * false when no error.
 *  * true when IntMode has an invalid value.
 */
bool Mcp23s17::_gpioIntMode(InterruptMode IntMode, uint8_t *pGpinten,
  uint8_t *pDefval, uint8_t *pIntcon) const
{
  switch (IntMode)
  {
  case INT_OFF:
    *pGpinten = _GPINTEN_DISABLED;
    *pDefval = _DEFVAL_DEFAULT;  // Not used
    *pIntcon = _INTCON_DEFAULT;  // Not used
    break;
  case INT_LOW:
    *pGpinten = _GPINTEN_ENABLED;
    *pDefval = _DEFVAL_LOW;
    *pIntcon = _INTCON_DEFVAL;
    break;
  case INT_HIGH:
    *pGpinten = _GPINTEN_ENABLED;
    *pDefval = _DEFVAL_HIGH;
    *pIntcon = _INTCON_DEFVAL;
    break;
  case INT_CHANGE:
    *pGpinten = _GPINTEN_ENABLED;
    *pDefval = _DEFVAL_DEFAULT;  // Not used
    *pIntcon = _INTCON_PREVVAL;
    break;
  default:
    // Invalid parameter: return error
    return true;
  }
  return false;
}

/*
 *   Reads one register in the selected chip.
 *  Parameters:
 *  * OcAddr: address of chip to read from, already in opcode format.
 *  * Reg: register to read
 *  Returns: data read from the register.
 */
byte Mcp23s17::_read(byte OcAddr, RegAddr Reg) const
{
  uint8_t OpCode;
  uint8_t Value;

  // Calculate chip opcode
  OpCode = _opcodeRead(OcAddr);

  // Perform SPI communications
  SPI.beginTransaction(_SpiSettings);
  
  digitalWrite(_Cs, LOW);              // Start transmission with CS pin
  
  SPI.transfer(OpCode);
  SPI.transfer(Reg);
  Value = SPI.transfer(_OPCODE_NOOP);  // Transfer anything to read data
  
  digitalWrite(_Cs, HIGH);             // End transmission with CS pin
  
  SPI.endTransaction();

  return Value;
}


/*
 *   Reads a sequence of registers in the selected chip. Assumes ICON.SEQOP=0
 *  (automatic sequential pointer increment) and IOCON.BANK=0 (address mapping).
 *  Parameters:
 *  * OcAddr: address of chip to read from already in opcode format.
 *  * StartReg: first register to read
 *  * pData: pointer to a buffer where the read data will be stored.
 *  * Count: number of bytes to read.
 */
void Mcp23s17::_read(byte OcAddr, RegAddr StartReg, byte *pData, uint8_t Count)
  const
{
  uint8_t OpCode;

  // Calculate chip opcode
  OpCode = _opcodeRead(OcAddr);
  // Prepare data to send while receiving
  memset(pData, _OPCODE_NOOP, Count);

  // Perform SPI communications
  SPI.beginTransaction(_SpiSettings);
  
  digitalWrite(_Cs, LOW);   // Start transmission with CS pin
  
  SPI.transfer(OpCode);
  SPI.transfer(StartReg);
  // Chip cursor advances automatically because IOCON.SEQOP = 0
  SPI.transfer(pData, (size_t) Count);
  
  digitalWrite(_Cs, HIGH);  // End transmission with CS pin
  
  SPI.endTransaction();
}


/*
 *   Writes one register in the selected chip.
 *  Parameters:
 *  * OcAddr: address of chip to write to already in opcode format.
 *  * Reg: register to write
 *  * Value: data to write into the register.
 */
void Mcp23s17::_write(byte OcAddr, RegAddr Reg, byte Value) const
{
  uint8_t OpCode;

  // Calculate chip opcode
  OpCode = _opcodeWrite(OcAddr);

  // Perform SPI communications
  SPI.beginTransaction(_SpiSettings);
  
  digitalWrite(_Cs, LOW);   // Start transmission with CS pin
  
  SPI.transfer(OpCode);
  SPI.transfer(Reg);
  SPI.transfer(Value);
  
  digitalWrite(_Cs, HIGH);  // End transmission with CS pin
  
  SPI.endTransaction();
}


/*
 *   Writes a sequence of registers in the selected chip. Assumes ICON.SEQOP=0
 *  (automatic sequential pointer increment) and IOCON.BANK=0 (address mapping).
 *  Parameters:
 *  * OcAddr: address of chip to write to already in opcode format.
 *  * StartReg: first register to write
 *  * pData: pointer to a buffer with the data to write.
 *  * Count: number of bytes to write.
 */
void Mcp23s17::_write(byte OcAddr, RegAddr StartReg, const byte *pData,
  uint8_t Count) const
{
  uint8_t OpCode;

  // Calculate chip opcode
  OpCode = _opcodeWrite(OcAddr);

  // Perform SPI communications
  SPI.beginTransaction(_SpiSettings);
  
  digitalWrite(_Cs, LOW);   // Start transmission with CS pin
  
  SPI.transfer(OpCode);
  SPI.transfer(StartReg);
  // Loop writing data bufer
  // Chip cursor advances automatically because IOCON.SEQOP = 0
  while (Count--)
    SPI.transfer(*pData++);
  
  digitalWrite(_Cs, HIGH);  // End transmission with CS pin
  
  SPI.endTransaction();
}
