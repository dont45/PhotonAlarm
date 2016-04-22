/*
  util.h
  utilities for Photon Alarm
  Don Thompson
  Raynham MA

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

#ifndef __UTIL_H__
#define __UTIL_H__
#include "parms.h"

#define HALT while(1)
typedef enum {UNDEFINED=0, SWITCH=1, OW_SENSOR=2, OW_INDICATOR=3, OW_RELAY=4, OW_THERMOMETER=5,
              MCP9808_THERMOMETER=6, USER_KEY=7, MASTER_KEY=8} SENSOR_TYPE;
typedef enum {DEV_MISSING=0, DEV_PRESENT=1, DEV_UNKNOWN=2};
typedef enum {SENSOR_DISABLED=0, SENSOR_ACTIVE=1};
typedef enum {SENSOR_CLEAR, SENSOR_TRIPPED};
typedef enum {SENSE_NORMAL_CLOSED=0, SENSE_NORMAL_OPEN=1};
const char* sensor_status_def[3] = {"missing", "present", "unknown"};
const char* sensor_state_def[2] = {"deactivated", "active"};
const char* sensor_use_def[9] = {"undefined","switch","ow switch","ow indicator","ow relay","ow thermometer",
          "mcp9808 therm","user key", "master key"};
const char* sensor_sense_def[2] = {"normal closed", "normal open"};
const char* sensor_use_descr[] = {"undefined","switch", "ow-sensor","ow-indicator","ow-relay","ow-thermometer",
              "mcp9808-thermometer","user-key","master-key" };

struct state_t
{
  uint8_t magic;
  uint8_t current_state;
  //uint8_t tripped;
  uint8_t alert_hours;
}
alarm_saved_state;
#define ALARM_STATE_ADDRESS 0
#define CONFIGURATION_ADDRESS sizeof(state_t)

//device config data in eeprom array (Saved)
struct config_t
{
  uint8_t magic;
  uint8_t dev_addr[MAXDEVICE][8];
  uint8_t port[MAXDEVICE];
  uint8_t use[MAXDEVICE];
  uint8_t sense[MAXDEVICE];
  int alert_min[MAXDEVICE];  // for thermometers
  char name[MAXDEVICE][SENSOR_NAME_SIZE+1];
};  //configuration - move to class, not global

//device config in runnint std::list (running)
//ADD alert-level to trip e.g. alert for thermometer
typedef struct device_t
{
    uint8_t idx;          // index: v0.1.6
    uint8_t status;       // missing, present, unknown
    uint8_t state;        //disabled, active
		uint8_t dev_rom[8];
    uint8_t port;         //PIO-A==0, PIO-B==1
    uint8_t dev_use;
    uint8_t sense;        //sensor NC or NO
    int alert_min;        // 0 ==> no alert_point, < this ==> do alert
    // and max ?
    float dev_reading;    //last PIO, temp, etc.
    String name;
    uint8_t tripped;      //checkSensor says tripped
    uint8_t reported;     //reported by alert
};

//just use put(address, object)
template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
  return i;
}

//just use EEPROM.get(addr, object)
template <class T> int EEPROM_readAnything(int ee, T& value)
{
  byte* p = (byte*)(void*)&value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return i;
}

// Log message to cloud, message is a printf-formatted string
// e.g. debug("ledPin = %d", ledPin);
void debug(String message, int value) {
    char msg [50];
    sprintf(msg, message.c_str(), value);
    Spark.publish("DEBUG", msg);
}

#endif
