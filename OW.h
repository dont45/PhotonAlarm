/*
  OW (1-WIRE) library for Arduino
  Copyright (C) 2009 raynham engineering

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

#ifndef __OW_H__
#define __OW_H__

//#include <inttypes.h>
#include "DS2482.h"
// move these to an include ??
#define FMLY_2405 0x05
#define FMLY_2406 0x12
#define INCL_FMLY_2405
#define INCL_FMLY_2406

#define DS2406_PIOA_SENSE 	(1<<2)
#define DS2406_PIOB_SENSE 	(1<<3)


class OW : public DS2482
{
public:
    OW(uint8_t address): DS2482(address){ status_memory_7 = 0x1F; };

    uint8_t readStatus(uint8_t *ROM, uint8_t *);		// read status bytes of ds2406
    uint8_t writeStatus(uint8_t *ROM, uint8_t);
    uint8_t readChannel(uint8_t *ROM, uint8_t *);		// read status bytes of ds2406
    uint8_t channelAccess(uint8_t *ROM, uint8_t channel_control, uint8_t *buf);
    bool readThermometer(uint8_t *ROM, double *rTempF);
    uint8_t readPIO(uint8_t *ROM);		// read PIO state (switch family)
    uint8_t readPIOA(uint8_t *ROM) { return readPIO(ROM) & DS2406_PIOA_SENSE ? 1 : 0; };
    uint8_t readPIOB(uint8_t *ROM) { return readPIO(ROM) & DS2406_PIOB_SENSE ? 1 : 0; };
    uint8_t writePIO(uint8_t *ROM, uint8_t, uint8_t );
    uint8_t condSearch(uint8_t *);
private:
    uint8_t status_memory_7;
};


#endif
