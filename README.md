https://github.com/escaner/Mcp23s17

Mcp23s17 is an Arduino library to manage Microchip Mcp23s17 general purpose
multi I/O integrated circuits.

Each object instance of the class Mcp23s17 can manage up to 8 chips in the
same CS line. The library should be used like this:

// E.g.:
// Object for up to 8 chips using Arduino pin 10 as CS
Mcp23s17 Mcp(10);
// Set ChipAddress A3 to actual value to override MCP bug even when
// hardware addressing will not be enabled.
// Set ChipAddress to actual address when hardware addressing will be enabled
// or more than one chip is added
Mcp.addChip(0b000);
Mcp.addChip(0b001);
Mcp.addChip(0b010);
// Initialize chips to hardware addressing or do not enable it for single chip
// and initialize SPI library
Mcp.begin(false);
// Advertise SPI if we are reading/writing via SPI to the MCP chip from inside
// an interrupt service routine
SPI.usingInterrupt(digitalPinToInterrupt(MY_INT_PIN));
// Configure chip 1 interrupt as HIGH, with both INTA and INTB synced
Mcp.configInterrupts(1, HIGH, true, false);
// Configure chip 1 input pins with pullup resistor and enable interrupts on CHANGE
GpioPinMask Input = GPA0_BIT | GPA1_BIT | GPB0_BIT | GPB1_BIT;
Mcp.configGpio(1, Input, INPUT_PULLUP, false, INT_CHANGE);
// Configure chip 0 output pins
GpioPinMask Output = GPA2_BIT | GPA3_BIT | GPB6_BIT | GPB7_BIT;
Mcp.configGpio(0, Output, OUTPUT, false);
// Read and write some values
word Values = Mcp.readPort(1);
Mcp.writePortA(0, 0b00001100);
Mcp.writePin(GPB6, 0);
// Free SPI resources
SPI.notUsingInterrupt(digitalPinToInterrupt(MY_INT_PIN));
Mcp.end();


Copyright (C) 2019, Óscar Laborda

This file is part of Mcp23s17 library.

Mcp23s17 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Mcp23s17 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Mcp23s17.  If not, see <https://www.gnu.org/licenses/>.
