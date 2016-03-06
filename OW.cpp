/*
  OW (1-WIRE) library for Arduino
  Adapted to Particle Photon
  2009 raynham engineering

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

//#include "WConstants.h"
#include "OW.h"

// read status block (8 bytes + crc) into buf
// device = ds2406
// SENSOR:0:12D8046000E9
// Family:12
// 2406
// deviceReadStatus:10
// Status=FF:FF:FF:FF:FF:0:0:7F:ED:C1:
// ok!!

uint8_t OW::readStatus(uint8_t *ROM, uint8_t *buf)
{
//    unsigned char status;
    int i;
    wireReset();
    wireSelect(ROM);
    busyWait(true);
    wireWriteByte(0xAA);
    wireWriteByte(0x00);
    wireWriteByte(0x00);
    for(i=0; i<10; i++)
        *buf++ = wireReadByte();
    return 1;
}

// write status byte to DS2406
// SENSOR:0:12D8046000E9
// Family:12
// 2406 Before (power-on default=0X7F)
// Status=FF:FF:FF:FF:FF:0:0:7F:ED:C1:
// Channel Info:33
// deviceWriteStatus: crc=1FF6
// SENSOR:0:12D8046000E9
// Family:12
// 2406 After Status Byte Write (0x0F)
// Status=FF:FF:FF:FF:FF:0:0:F:EC:25:
// Channel Info:30
// deviceWriteStatus: crc=1FF6

// status byte (7)
// |7        |6     |5     |4      ||3      |2      |1      |0        |
// |Supply   |PIO-B |PIO-A |CSS4   ||CSS3   |CSS2   |CSS1   |CSS0     |
// |Ind.     |Chan  |Chan  |Chan   ||Chan   |Source |Source |Polarity |
// |read-only| F-F  | F-F  |Select ||Select |Select |Select |         |

uint8_t OW::writeStatus(uint8_t *ROM, uint8_t status)
{
    uint8_t crc1, crc2;
    bool ok;
    // should verify family here ??
    wireReset();
    wireSelect(ROM);
    busyWait(true);
    wireWriteByte(0x55);
    wireWriteByte(0x07);	// write just the status byte
    wireWriteByte(0x00);
    wireWriteByte(status);
    crc1=wireReadByte();
    crc2=wireReadByte();
//    serial->print("writeStatus: crc=");
//    serial->print(crc1,HEX);
//    serial->println(crc2,HEX);
    ok = true;			// calc crc and compare to set ok
    if(ok) wireWriteByte(0xFF);
    else wireReset();
}

// This should be called channelAccess
uint8_t OW::channelAccess(uint8_t *ROM, uint8_t channel_control, uint8_t *buf)
{

}

// read channel info byte into buf
// device = ds2406
// Family:12
// 2406
// Channel Info:33

// Need to pass channel_control as a parameter??
// No,  Need to develope a general channelInfo routine,
// call it from readChannel and writeChannel ??
// This version reads Ch B Only
uint8_t OW::readChannel(uint8_t *ROM, uint8_t *buf)
{
    uint8_t channel_control = 0x48;	// 0 1 0 0 1 0 0 0
    int i;
    wireReset();
  #ifdef SERIAL_DEBUGXX
    Serial.print("readChannel: ROM:");
    for(int k=0;k<7;k++) {
      Serial.print(ROM[k],HEX);
      Serial.print(":");
    }
  #endif
    wireSelect(ROM);
    busyWait(true);
    wireWriteByte(0xF5);
    wireWriteByte(channel_control);
  #ifdef SERIAL_DEBUGXX
    Serial.print("  channel_control: ");
    Serial.print(channel_control, BIN);
  #endif
    wireWriteByte(0xFF);
    *buf = wireReadByte();
  #ifdef SERIAL_DEBUGXX
    Serial.print("    ChannelInfo: ");
    Serial.println(*buf, BIN);
  #endif
    return *buf;
}

//ds1820 class thermometer
bool OW::readThermometer(uint8_t *ROM, double *rTempF) {
  //COPY FROM MY BOOK..
  uint8_t present = 0;
  uint8_t data[12];
  int raw;
  double temp;
  #ifdef SERIAL_DEBUGXX
  Serial.print("readThermometer-ROM:");
  Serial.println(ROM[1], HEX);
  #endif
  wireReset();
  wireSelect(ROM);
  wireWriteByte(0x44);     //start conversion
#ifdef DS1820_POWERED
// if NOT parasetic powered, read bits until
// 1 is returned (reads 0 until conversion complete)
#else
  delay(400);  //maybe longer ??
#endif
  present = wireReset();  //reset ok because scratcpad written
  if(present) {
    wireSelect(ROM);
    wireWriteByte(0xbe);   //read scratchpad
    #ifdef SERIAL_DEBUGXX
    Serial.print("Scratchpad: ");
    for(int i=0; i<9; i++) {
      data[i] = wireReadByte();
      Serial.print(data[i],HEX);
      Serial.print(":");
    }
    Serial.println();
    #endif
    if(DS2482::crc8(data,8) != data[8]) {
      #ifdef SERIAL_DEBUG
      Serial.println("CRC is not valid");
      #endif
      return FALSE;
    }
    raw = (data[1]<<8)+data[0];   //put two bytes of temp into raw
    #ifdef SERIAL_DEBUGXX
    Serial.println("convert tempC=");
    Serial.println(raw);
    #endif
    temp = (double)raw * 0.0625;  //convert to celcius
    *rTempF = temp * 1.8 + 32;    //convert to Fahrenheit
    return TRUE;
  }
  return FALSE;
}

// write PIO-A and/or PIO-B to ds2406 class device
uint8_t OW::writePIO(uint8_t *ROM, uint8_t port, uint8_t val)
{
//  using write to status memory
    status_memory_7 = 0x1f;		// shut off bits 5 & 6   (0001 1111)
    status_memory_7 |= val << 5;	// set bits 5 & 6        (0xx0 0000)
    wireReset();
    wireSelect(ROM);
    busyWait(true);
    wireWriteByte(0x55);
    wireWriteByte(0x07);
    wireWriteByte(0x00);
    wireWriteByte(status_memory_7);
    wireReadByte();

    return 1;
}

// read PIO state (switch family)
uint8_t OW::readPIO(uint8_t *ROM)
{
    uint8_t pio;
    // for now, read only ds2405
    switch(ROM[0])
    {
#ifdef INCL_FMLY_2405
  	case FMLY_2405:
#ifdef OLD-WAY
    		uint8_t r1, r2;	// first and second read
    		uint8_t sbuf1[2];
    		uint8_t sbuf2[2];
    		readChannel(ROM, sbuf1);
    		readChannel(ROM, sbuf2);
    		pio = sbuf2[0] == sbuf1[0];
#endif
		uint8_t r1, r2;
		readChannel(ROM, &r1);
		readChannel(ROM, &r2);
		return r1 == r2;
		break;
#endif
#ifdef INCL_FMLY_2406
  	case FMLY_2406:
		readChannel(ROM, &pio);
		return pio;
#endif
	default:
#ifdef SERIAL_DEBUG
		Serial->println("UNSUPPORTED");
#endif
		return 0xFF;
     }
}

uint8_t OW::condSearch(uint8_t *newAddr)
{
    uint8_t search_status;
    // set devices for conditional search
    // this MUST be done external to this and prior to calling this
    // set for conditional search
    setCondSearch();
    search_status = wireSearch(newAddr);
    // restore normal search
    setStdSearch();
    return search_status;
}
