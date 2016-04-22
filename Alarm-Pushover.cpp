/*
  pushover alarm system for Particle Photon
  @file     Alarm-Pushover.cpp
  @author   device_list. Thompson
  @license  GNU General Public License (see license.txt)
  @version  0.1.6

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

/* Added in v 1.6
 * Sensor Id (sensor#) to device_t (array index in config_t)
 * Find by Id (in sensor LIST)
 * update to config_t at idx and write to EEPROM
 * command NAME to rename sensor (by sensor#)
 * command SEN to display all data of a sensor (by sensor#)
 * Removed short_name
 */

/* Particle Cloud published events:
 * alarmState with name of new state any time state changes
 */
// To Do:
// a) fix sysState clearing to shut off blinking Status
//    USE a timer in sysState
//    Timer ext and in class cause build to fail
// b) fix multiple notifications upon alarm condition
// d) add i-button support as key and as config management
// e) add HTU21 humidity sensor support
// f) second 1-wire buss for i-button support (clear readint)
//    or can we skip-rom to just read class 01 == i-button ??
// g) add ds thermometer support

#define SYSTEM_VERSION 0.1.6
#define SYSTEM_VERSION_MAJOR 0
#define SYSTEM_VERSION_MINOR 1
#define SYSTEM_VERSION_SUB 6

//#define SERIAL_DEBUG
//#define SERIAL_DEBUGXX
//#define SERIAL_WAIT
#include "application.h"
#include <queue>
#include <list>
#include "Adafruit_MCP9808.h"
#include "OW.h"
#include "parms.h"
#include "util.h"
#include "sensor.h"
#include "math.h"

//STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));
//retained uint8_t testcurState;

int ledStatus = SYSTEM_STATUS_PIN;            //blink this LED at each loop pass
int ledNotice = SYSTEM_NOTIFICATION_PIN;      //blink this LED at each notification
int trigger3 = TRIGGER_PIN;
int ledStatusState = 0;
int ledStatusBlink = 0;                       // 0=solid,1=slow blink,2=fast blink
int ledStatusBlinkCount = 0;
int message_countdown = 2;

Adafruit_MCP9808 mcp9808;
OW ow(0);       //default 0 is 0x18 !!
// OW ow1(1);   //NO! testing second ds2483 for iButtons
// iButton ib(1);
// we SHOULD make a specific iButton class, similar to OW

#include "state.h"
State sys;
#include "notifier.h"

// set sensor indicator (on PIO_A)
void Sensor::setSensorIndicator(device_t d, uint8_t indval) {
  #ifdef SERIAL_DEBUGXX
  Serial.print("setting indicator:");
  Serial.println(indval);
  #endif
  //#define PIO_A 0
  //#define PIO_B 1
  //d.port is the sensor read port
  //indicator port is the other one ??
  uint8_t indicator_port = d.port ? 0 : 1;
  #ifdef SERIAL_DEBUGXX
    Serial.print("setSensorIndicator:prot=");
    Serial.print(d.port);
    Serial.print(" indicator port=");
    Serial.println(indicator_port);
  #endif
  p_ow->writePIOtest(d.dev_rom, indicator_port, indval);  //WRITE TO PIO-x
}

// return TRUE if tripped
bool Sensor::readSensor(device_t &d) {
    uint8_t senval;
    double tempF;
    bool senret;
    bool rok;
    bool toSetSensor;
    if(d.status != DEV_PRESENT) {
  #ifdef SERIAL_DEBUG
      Serial.print("Device Missing - ROM:");
      Serial.println(d.dev_rom[7],HEX);
  #endif
      return FALSE;
    }
    if(d.state != SENSOR_ACTIVE) {
  #ifdef SERIAL_DEBUG
      Serial.print("Device deactivated - ROM:");
      Serial.println(d.dev_rom[7],HEX);
  #endif
      return FALSE;
    }
    switch(d.dev_use) {
      case SWITCH:    //simple switch on PHOTON pin
  #ifdef SERIAL_DEBUGXX
          Serial.print("readSensor rom=");
          Serial.print(d.dev_rom[7],HEX);
          Serial.print(" use=");
          Serial.print(d.dev_use);
  #endif
         senval = digitalRead(d.dev_rom[0]);
         d.dev_reading = (float)senval;
  #ifdef SERIAL_DEBUGXX
          Serial.print(" readvalue=");
          Serial.println(senval);
  #endif
          return senval==d.sense;
          break;
      case OW_SENSOR:
      case OW_INDICATOR:  //one-wire device on DS2482-100
          senval = p_ow->readPIOX(d.dev_rom, d.port);    //??just readPIO and use d.port
          toSetSensor = (uint8_t)d.dev_reading != senval;
          d.dev_reading = (float)senval;
          senret = senval==d.sense;
          #ifdef SERIAL_DEBUGXX
                  Serial.print("readSensor OW_SENSOR: dev_use=");
                  Serial.print(d.dev_use);
                  Serial.print(" ow value=");
                  Serial.print(senval);
                  Serial.print(" return=");
                  Serial.println(senret);
          #endif
          if(d.dev_use==OW_INDICATOR)
            //if(toSetSensor) setSensorIndicator(d, senret);
            setSensorIndicator(d, senret);
          return senret;
          break;
      case MCP9808_THERMOMETER:  //one-wire device on DS2482-100
              // to be implemented
              // readTempF should use config parameters to get device info
              // i.e. i2c address etc.  This allows multiple devices on bus
              last_temp = mcp9808.readTempF();
              d.dev_reading = last_temp;
              // save to sensor structure
              // return value should  be tripped if outside temp range ??
              if((int)last_temp <= d.alert_min) return TRUE;
              // readTemp should return an error condition ??
              return FALSE;
        break;
        case OW_THERMOMETER:    //ds1820 thermometer
            #ifdef SERIAL_DEBUG
              Serial.print("ds1820 testing-rom:");
              for(int i=0;i<8;i++) {
                Serial.print(d.dev_rom[i],HEX);
                Serial.print("-");
              }
              Serial.println();
            #endif
              d.dev_reading = -1.0; //debug marker ??
              rok = p_ow->readThermometer(d.dev_rom, tempF);
              if(rok) {
                d.dev_reading = tempF;
                Serial.print("temp Fahrenheit x100:");
                Serial.println((int)tempF*100);
                if((int)tempF <= d.alert_min) return TRUE;
              }
              return FALSE;  //not tripped
        break;
      default:
        break;
  }
  return FALSE;
}

//REMOVE THIS...use readSensor()
float Sensor::readTemperature() {
  // last_temp should be stored by sensor,
  // part of sensor data ??
  last_temp = mcp9808.readTempF();
  return last_temp;
}

float Sensor::getLastTemperature() {
  return last_temp;
}

char* Sensor::romFormat(char *buf, uint8_t rom[]) {
  char *bp = buf;
  int n = 0;
  for(int i=0;i<8;i++) {
    bp = &buf[n];
    int rb = rom[i];
    if(rb<16) *bp++='0';
    itoa((int)rb, bp, 16);
    n+=2;
    buf[n++]='-';
  }
  buf[n-1]=0;
  return buf;
}

Sensor sensor(&ow);

// initialize notify with alarm1 too ??
Notifier notify("shed_alarm", &sensor);
// Alarm states xxx  FIX:???
// need two states: alarm_state; DISARMED, armed
// and sensor_state: clear, tripped, notified
typedef enum {alarm_disarmed=0,alarm_armed=1,alarm_tripped=2,alarm_notifying=3,alarm_clearing=4};
const char* alarm_state_name_def[5] = {"disarmed", "armed", "tripped", "notify", "clear"};

#include "alarm.h"
/*
class Alarm {
public:
  Alarm() {
    prevTripState = FALSE;
    firstTrippedSensor=0;  //?? use tripedList
    clearing_cnt = 0;
    p_config = new config_t;
  }
  ~Alarm();
  bool setState(uint8_t new_state);
  uint8_t getState();
  bool readSavedState();
  bool writeSavedState();
  bool generateConfiguration();
  bool readConfiguration();
  bool writeTestConfiguration();
  bool updateConfiguration(device_t* dev, int n);
  uint8_t buildDeviceList();
  void dump_device_list();
  bool validate_device_list();
  char* deviceListing(char *buf);
  uint8_t wiredeviceAquire();
  bool wiredeviceValidate(uint8_t *ROM);
  bool isTripped();
  bool prevTripped();
  device_t *firstTripped();
  void tripListString();
  bool isArmed();
  void doStatusBlink();
  void clearing_countup();
  void blinkTimeout();
  bool alarmNotifying();
  String getPendingMessage();
  bool checkSensors(void);
  String getLastTemperature();  //testing
  bool clearSensorReported();
  bool allClear();
  int clearingCount();
  device_t* getDevice(uint8_t);
private:
  config_t configuration;
  config_t *p_config;
  uint8_t curState;
  device_t device;
  std::list<device_t> device_list; //??name
  std::list<device_t> trippedList;
  bool prevTripState;
  uint8_t firstTrippedSensor;
  int clearing_cnt;
  String message;  //??name
  String trippedString;
};
*/
// lets make a special config class to handle EEPROM config
// allocate configuration with new and dealoc in destructor
// use instance of this class in Alarm routines which handle
// configuration. ???
Alarm::~Alarm(void) {
  delete p_config;
}

//scan 1-wire bus and build configuration of devices found
//preserve data (use, name, etc. ) from existing configuration
//if device was previously present
bool Alarm::generateConfiguration() {

}

//update a specific device based upon user input changes (NAME)
bool Alarm::updateConfiguration(device_t* dev, int n) {
  strncpy(configuration.name[n], dev->name.c_str(), SENSOR_NAME_SIZE);
  EEPROM_writeAnything(CONFIGURATION_ADDRESS, configuration);
}

int Alarm::clearingCount() {
  return clearing_cnt;
}

//return FALSE if some sensor still tripped
//return TRUE if all now clear
bool Alarm::clearSensorReported() {
  // run thru sensor list and clear reporting if not tripped
  // send error message if still tripped and leave reported
  std::list<device_t>::iterator k;
  bool all_clear = TRUE;
  for(k=device_list.begin(); k != device_list.end(); ++k) {
     //device_t sp = *k;
     if(k->reported) {
       if (k->tripped) {
         all_clear = FALSE;
       }
       else
         k->reported = FALSE;
     }
   }
   return all_clear;
}

String Alarm::getLastTemperature() {
  //String t = String("100.00");  //will get from sensor.....???
  return String(sensor.getLastTemperature());
  //return t;
}
//typedef enum {alarm_disarmed=0,alarm_armed=1,alarm_tripped=2,alarm_notifying=3,alarm_clearing=4};
uint8_t Alarm::getState() {
  return curState;
}

// review all this (states)
bool Alarm::setState(uint8_t new_state) {
  bool result = TRUE;
  Serial.print("setState to ");
  Serial.print(alarm_state_name_def[new_state]);
  Serial.print(" :curState=");
  Serial.println(alarm_state_name_def[curState]);
  if(curState == new_state)
    return FALSE;
  // Send a publish of new alarm sate to your devices...
  Particle.publish("alarmState",alarm_state_name_def[new_state],60);
  Serial.print("Published alarmState event:");
  Serial.println(alarm_state_name_def[new_state]);
  switch(new_state) {
      case alarm_disarmed:
        curState = alarm_disarmed;
        //write to EEPROM
    #ifdef SERIAL_DEBUG
        Serial.println("alarm disarmed");
    #endif
        message = "DISARMED";
        ledStatusState = 0;
        ledStatusBlink = 0;
        break;
    case alarm_armed:
        //write to EEPROM
    #ifdef SERIAL_DEBUG
        Serial.println("alarm armed");
    #endif
        // add concept of prevTripped
        if(isTripped()) {
            //fast blink status
            ledStatusBlink = 2;
            message = "NOT CLEAR";
            result = FALSE;
            break;
        }
        ledStatusState = 1;
        ledStatusBlink = 0;
        prevTripState = FALSE;
        clearing_cnt=0;
        if(isArmed())
          message = "ALARM RESET";
        else
          message = "ALARM SET";
        curState = new_state;
        break;
    case alarm_tripped:
        curState = alarm_tripped;
    #ifdef SERIAL_DEBUG
        Serial.println("alarm tripped");
    #endif
        ledStatusBlink = 1;
        prevTripState = TRUE;
        tripListString();
        message = trippedString; //??FIX
        break;
    case alarm_notifying:
        curState = new_state;
        prevTripState = FALSE;
    #ifdef SERIAL_DEBUG
        Serial.print("alarm notifying:");
        Serial.println(clearing_cnt);
    #endif
        char buf[20];
        sprintf(buf,"NOTIFYING:%d", clearing_cnt);
        message = String(buf);
        ledStatusState = 1;
        ledStatusBlink = 2;
        /*
        if(prevTripped()) {
          if(clearing_cnt > CLEARING_COUNT_MAX) { //?? count
            debug("clearing_cnt=%d\\n",clearing_cnt);
            prevTripState = FALSE;
            clearing_cnt = 0;
            setState(alarm_clearing);
          }
        }
        */
        break;
    case alarm_clearing:
        //only if sensors clear??
        message = "CLEARED";
        Serial.println("set state alarm_clearing...");
        //let user ack before setting to armed
        //setState(alarm_armed);  //recursive maybe not still armed??
        break;
    default:
    #ifdef SERIAL_DEBUG
        Serial.println("invalid status");
    #endif
        message = "INVALID ALARM STATUS";
  }
  digitalWrite(ledStatus, ledStatusState);
  return result;
}

uint8_t Alarm::buildDeviceList() {
  uint8_t dev_cnt = 0;
  char nbuf[30];  //at least SENSOR_NAME_SIZE   and SIZE for ROMHEX ??
  if(configuration.magic != EE_MAGIC_CONFIG) return 0;
#ifdef SERIAL_DEBUGXX
  Serial.println("buildDeviceList:");
#endif
  for(int j = 0; j< MAXDEVICE; j++) {
    if(!configuration.use[j]) continue;
    #ifdef SERIAL_DEBUGXX
      sensor.romFormat(&nbuf[0], configuration.dev_addr[j]);
      Serial.print(" config rom=");
      Serial.print(nbuf);
    #endif
    device.idx=j;
    for(int i=0;i<8;i++)
      device.dev_rom[i] = configuration.dev_addr[j][i];
    device.port = configuration.port[j];
    device.dev_use = configuration.use[j];
    device.state = SENSOR_ACTIVE;
    if(device.dev_use == SWITCH || device.dev_use == MCP9808_THERMOMETER )
      device.status = DEV_PRESENT;
    else
      device.status = DEV_MISSING;
    device.sense = configuration.sense[j];
    device.alert_min = configuration.alert_min[j];
    strncpy(nbuf,configuration.name[j],SENSOR_NAME_SIZE);
    nbuf[SENSOR_NAME_SIZE]=0;
    device.name = String(nbuf);
#ifdef SERIAL_DEBUGXX
    Serial.print("adding device: name=");
    Serial.println(device.name);
#endif
    device.dev_reading = 0.0;
    device.tripped = 0;
    device.reported = 0;
   device_list.push_back(device);
    dev_cnt++;
  }
#ifdef SERIAL_DEBUGXX
  Serial.print("added to device_list:");
  Serial.println(dev_cnt);
#endif
  return dev_cnt;
}

char* Alarm::deviceListing(char *buf) {
  std::list<device_t>::iterator k;
  char temp[10];
  buf[0]=0;
  //int i=0;
  strcat(buf,"\\n"); //json escape line feed
  for(k=device_list.begin(); k !=device_list.end(); ++k) {
     device_t sp = *k;
     sprintf(temp,"%02d ", sp.idx);
     strcat(buf,temp);
     //strncat(buf,sp.short_name,SHORT_NAME_SIZE);
     //strcat(buf," ");
     strncat(buf,sp.name,SENSOR_NAME_SIZE);
     strcat(buf," state:");
     if(k->status==0) strcat(buf,"M");      //missing
     else if(k->status==1) strcat(buf,"P"); //present
     else strcat(buf,"U");                  //unknown
     if(k->state == SENSOR_ACTIVE) {
       strcat(buf,"A");                     //active
       if(k->tripped) strcat(buf,"T");      //tripped
       else strcat(buf,"C");                //clear
       if(k->reported) strcat(buf,"R");     //reported
     } else
        strcat(buf,"I");                    //inactive
     strcat(buf,"\\n");
     //if(++i>9) break;  //<<<===debug, just show ONE !!!
   }
   Serial.println("deviceListing");
   Serial.println(buf);
   return buf;
}

// trippedList to String
void Alarm::tripListString() {
  std::list<device_t>::iterator k;
  int i = 0;
  bool has_tripped = FALSE;
  char trippedbuf[60];
  trippedbuf[0]=0;
  strcpy(trippedbuf,"TRIPPED SENSORS:\\n");
  for(k=trippedList.begin(); k != trippedList.end(); ++k) {
     device_t sp = *k;
     //strncat(trippedbuf,sp.short_name,SHORT_NAME_SIZE);
     strncat(trippedbuf,sp.name,SENSOR_NAME_SIZE);
     strcat(trippedbuf,"\\n");
     i++;
     has_tripped=TRUE;
   }
   if(!has_tripped) strcat(trippedbuf,"none");
   Serial.print("trippedListString: no=");
   Serial.print(i);
   Serial.print(" List=");
   Serial.print(trippedbuf);
   trippedString = String(trippedbuf);
}
//debug dump of sensor list
void Alarm::dump_device_list() {
  std::list<device_t>::iterator k;
  char romhex[30];

for(k=device_list.begin(); k !=device_list.end(); ++k) {
   device_t sp = *k;
   sensor.romFormat(romhex, sp.dev_rom);
   Serial.print(sp.idx);
   Serial.print("  Status:");
   Serial.print(sp.status);
   Serial.print(" Port:");
   Serial.print(sp.port);
   Serial.print(" Use:");
   Serial.print(sp.dev_use);
   Serial.print(" [");
   Serial.print(sensor_use_descr[sp.dev_use]);
   Serial.print("] ROM:");
   Serial.print(romhex);
   Serial.print(" Name:");
   Serial.print(sp.name);
   Serial.println();
  }
Serial.println();
}

bool Alarm::validate_device_list() {
  //run thru device list and check rom present
  std::list<device_t>::iterator k;
  bool all_present = TRUE;
  int dev_present_cnt = 0;
  for(k=device_list.begin(); k !=device_list.end(); ++k) {
    device_t sp = *k;
    //OW_SENSOR=2, OW_RELAY=3, OW_THERMOMETER=4,
    if(sp.dev_use >= OW_SENSOR && sp.dev_use <= OW_THERMOMETER)
      if(sp.status != 1) {
        all_present = FALSE;
        Serial.print("DEV not present:");
        Serial.println(sp.name);
      }
      else dev_present_cnt++;
  }
  //if ow devices configured, must have at least one to run
  if(!dev_present_cnt && !all_present) {
    // no ow dev found
    sys.addStatus(fail_owdevice);
    notify.sendMessage("NO OWDEV");   //send this message now
    sys.sysState(sys_fail); //this will do HANG
  }
  if(all_present)
    Serial.println("all dev present");
  return all_present;
}

void Alarm::setDeviceActive(device_t *d, bool toActive) {
    if(toActive)
      d->state = SENSOR_ACTIVE;
    else
      d->state = SENSOR_DISABLED;
}

void Alarm::setDeviceAlertMin(device_t *d, int p3) {
    d->alert_min = p3;
}

bool Alarm::alarmNotifying() {
  return curState == alarm_notifying;
}

bool Alarm::isTripped() {
  return curState==alarm_tripped;  // || curState==alarm_notifying;
}

bool Alarm::prevTripped() {
  return prevTripState;  //??
}
device_t* Alarm::firstTripped() {
  if(trippedList.empty()) return NULL;
  return &trippedList.front();
}
bool Alarm::isArmed() {
  return curState != alarm_disarmed;
}
String Alarm::getPendingMessage() {
  return message;
}
void Alarm::doStatusBlink() {
  ledStatusBlinkCount++;
  switch(ledStatusBlink) {
    case 0: //solid
            break;
    case 1: //slow
            if(ledStatusBlinkCount > 4) {
              ledStatusBlinkCount = 0;
              //switch led state
              if(++ledStatusState > 1) ledStatusState = 0;
            }
            break;
    case 2: //fast
            if(ledStatusBlinkCount > 1) {
              ledStatusBlinkCount = 0;
              if(++ledStatusState > 1) ledStatusState = 0;
            }
            break;
  }
  digitalWrite(ledStatus, ledStatusState);
}

void Alarm::blinkTimeout() {
#ifdef SERIAL_DEBUG
  Serial.println("Alarm blinkTimeout");
#endif
  ledStatusBlink = 0;
  ledStatusBlinkCount = 0;
}

// IS this ROM in device_list?
bool Alarm::wiredeviceValidate(uint8_t *ROM) {
  std::list<device_t>::iterator k;
  for(k=device_list.begin(); k !=device_list.end(); ++k) {
    device_t sp = *k;
    bool matching = TRUE;
    for(int i=0;i<8;i++)
      if(ROM[i] != sp.dev_rom[i]) matching = FALSE;
    if(matching) {
  #ifdef SERIAL_DEBUG
      Serial.print("ROM CD:");
      Serial.print(ROM[7],HEX);
      Serial.print(" dev_rom:");
      Serial.print(k->dev_rom[7],HEX);
      Serial.println(" seting status PRESENT");
  #endif
      k->status = DEV_PRESENT;  //why does this NOT set status ????
      return TRUE;
   }
  }
  return FALSE;
}

// Scan wire for all DEVICES
// MOVE TO SENSOR
// DEBUG, for now just validate them as present
uint8_t Alarm::wiredeviceAquire() {
  uint8_t ROM[8];
  uint8_t i,j,k;
  ow.wireResetSearch();
  for(i=0;i<MAX_ALLOW_OWDEV;i++) {
    j=ow.wireSearch(ROM); //CRASHES HERE !!!
    if(j==0) break;
    //how to handle iButton keys ??
    /*
     * if iButton is present =>
     * we are generating a NEW configuration
     * if not present, validate what is found
     * NOTE: must do iButton family search
     * to get ONLY ibuttons, then so
     * wiredeviceAquire to validate or generate ??
     */
    //validate that this sensor is in device_list
    uint8_t valid = wiredeviceValidate(ROM);
#ifdef SERIAL_DEBUG
    Serial.print("sensor add:");
    for(k=0;k<8;k++) {
      Serial.print( ROM[k],HEX);
      Serial.print(":");
    }
    Serial.print(" valid:");
    Serial.println(valid);
#endif
  }
  ow.wireResetSearch();
  return i;
}

// Write simple test configuration to EEPROM
bool Alarm::writeTestConfiguration() {
  int i = 0;
  configuration.magic = EE_MAGIC_CONFIG;
//test config for neshed
#define TEST_SENSOR_1
//#define TEST_SENSOR_2
//#define TEST_SENSOR_3
//#define TEST_SENSOR_4
//#define TEST_SENSOR_5
#define TEST_SENSOR_6
//#define TEST_SENSOR_7
//#define TEST_SENSOR_8
#define TEST_SENSOR_9
#define TEST_SENSOR_10
  //Sensor 1
#ifdef TEST_SENSOR_1
  configuration.dev_addr[i][0] = MCP9808_I2CADDR;
  for(int k=1;k<8;k++)
    configuration.dev_addr[i][k] = 0xff;
  configuration.port[i] = 0;
  configuration.use[i] = MCP9808_THERMOMETER;
  configuration.sense[i] = 0;
  configuration.alert_min[i] = 0;
  strncpy(configuration.name[i], "INSIDE TEMP", SENSOR_NAME_SIZE);
  i++;
#endif
  //Sensor 2
#ifdef TEST_SENSOR_2
const uint8_t testrom1[8] = { 0x12,0x3a,0x84,0x72,0x00,0x00,0x00,0xd8 };
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom1[k];
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  configuration.alert_min[i] = 0;
  strncpy(configuration.name[i], "NE SHED DOOR", SENSOR_NAME_SIZE);
  i++;
#endif
  //Sensor 3
#ifdef TEST_SENSOR_3
const uint8_t testrom2[8] = { 0x05,0xe9,0xef,0x05,0x00,0x00,0x00,0x42 };
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom2[k];
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  configuration.alert_min[i] = 0;
  strncpy(configuration.name[i], "OWS SWITCH3", SENSOR_NAME_SIZE);
  i++;
#endif
  //Sensor 4
#ifdef TEST_SENSOR_4
const uint8_t testrom3[8] = { 0x10,0xc8,0xb6,0x1e,0x00,0x00,0x00,0x3b };
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom3[k];
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_THERMOMETER;
  configuration.sense[i] = 0;
  configuration.alert_min[i] = 0;
  strncpy(configuration.name[i], "OUTSIDE TEMP4", SENSOR_NAME_SIZE);
  i++;
#endif
  //Sensor 5
#ifdef TEST_SENSOR_5
const uint8_t testrom4[8] = { 0x10,0x5d,0xab,0x4c,0x01,0x08,0x00,0xf3 };
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom4[k];
    configuration.port[i] = PIO_B;
  configuration.use[i] = OW_THERMOMETER;
  configuration.sense[i] = 0;
  configuration.alert_min[i] = 70;
  strncpy(configuration.name[i], "OUTSIDE TEMP", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef TEST_SENSOR_6 //ds2407 green star
  const uint8_t testrom5[8] = { 0x12,0x6d,0x25,0x0a,0x00,0x00,0x00,0x39 };
    for(int k=0;k<8;k++)
      configuration.dev_addr[i][k] = testrom5[k];
    configuration.port[i] = PIO_B;
    configuration.use[i] = OW_SENSOR;
    configuration.sense[i] = SENSE_NORMAL_OPEN;
    configuration.alert_min[i] = 0;
    strncpy(configuration.name[i], "MOTION", SENSOR_NAME_SIZE);
    i++;
#endif
#ifdef TEST_SENSOR_7
//add:12:73:25:A:0:0:0:71:  ds2407 red star
  const uint8_t testrom6[8] = { 0x12,0x73,0x25,0x0a,0x00,0x00,0x00,0x71 };
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom6[k];
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  configuration.alert_min[i] = 0;
  strncpy(configuration.name[i], "O7 DS2407", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef TEST_SENSOR_8
//add:5:67:F8:5:0:0:0:96:
  const uint8_t testrom7[8] = { 0x05,0x67,0xf8,0x05,0x00,0x00,0x00,0x96 };
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom7[k];
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  configuration.alert_min[i] = 0;
  strncpy(configuration.name[i], "O8 DS2407", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef TEST_SENSOR_9
//add:3A:7:30:18:0:0:0:BA:  ds2413
  const uint8_t testrom8[8] = { 0x3A,0X07,0X30,0X18,0x00,0x00,0x00,0xBA };
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom8[k];
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_INDICATOR;
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  configuration.alert_min[i] = 0;
  strncpy(configuration.name[i], "O9 DS2413", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef TEST_SENSOR_10
//sensor add:12:D5:7F:46:0:0:0:45: WatchDog Water Alarm
  const uint8_t testrom9[9] = { 0x12,0xd5,0x7f,0x46,0x00,0x00,0x00,0x45 };
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom9[k];
  configuration.port[i] = PIO_A;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_min[i] = 0;
  strncpy(configuration.name[i], "WATER DETECTOR", SENSOR_NAME_SIZE);
  i++;
#endif
    // and Write it
  EEPROM_writeAnything(CONFIGURATION_ADDRESS, configuration);
#ifdef SERIAL_DEBUG
  Serial.print("WriteTestConfiguration: DEV Count=");
  Serial.println(i);
#endif
  return TRUE;
}

bool Alarm::readConfiguration() {
  // READ configuration for updates
  // How to read eeprom on photon ??
  //EEPROM_readAnything(CONFIGURATION_ADDRESS, configuration);    //we will read unless button pressed to init network??
  EEPROM.get(CONFIGURATION_ADDRESS, configuration);
  if(configuration.magic==EE_MAGIC_CONFIG)
  {
  #ifdef SERIAL_DEBUG
    Serial.println("VALID CONFIG MAGIC");
  #endif
    return TRUE;
  }
  // MUST setup actual sensors, i.e. pinmodes etc.
  // Where?
#ifdef SERIAL_DEBUG
  Serial.println("INVALID CONFIG MAGIC");
#endif
  return FALSE;
}

bool Alarm::readSavedState() {
  EEPROM.get(ALARM_STATE_ADDRESS, alarm_saved_state);
  if(alarm_saved_state.magic==EE_MAGIC_STATE)
  {
#ifdef SERIAL_DEBUG
    Serial.println("VALID MAGIC STATE");
    Serial.print("sysState=");
    Serial.println(alarm_saved_state.current_state);
#endif
    curState = alarm_saved_state.current_state;
    ledStatusState = alarm_saved_state.current_state != alarm_disarmed;
    prevTripState = FALSE;
    return TRUE;
  }
#ifdef SERIAL_DEBUG
  Serial.println("INVALID MAGIC STATE");
#endif
  return FALSE;
}
bool Alarm::writeSavedState() {
  alarm_saved_state.magic = EE_MAGIC_STATE;
  alarm_saved_state.current_state = curState;
  alarm_saved_state.alert_hours = notify.getAlertHours();
#ifdef SERIAL_DEBUG
  Serial.print("writing SavedState: current_state=");
  Serial.println(alarm_saved_state.current_state);
#endif
  EEPROM.put(ALARM_STATE_ADDRESS, alarm_saved_state);
  return TRUE;
}

bool Alarm::allClear() {
  return trippedList.empty();
}
// returns TRUE if any sensor unreported tripped
bool Alarm::checkSensors(void) {
  //bool temptrip = FALSE;
  bool reporting = FALSE;
  //?? Must check all sensors here for alarm condition and notify
  // if not already notified and handle countdown to too many notifications
  // and reset conditions once alarm condition is cleared
  //here we pass thru configuration struct
  firstTrippedSensor = 0;
  trippedList.clear();
  //here we use will device_list
  //??? Add device scan here rather than
  // will require change to readSensor also
  std::list<device_t>::iterator k;
  for(k=device_list.begin(); k !=device_list.end(); ++k) {
    device_t sp = *k;
  #ifdef SERIAL_DEBUGXX
    Serial.print("checkSensor: rom=");
    Serial.print(k->dev_rom[7],HEX);
    Serial.print(" use=");
    Serial.println(k->dev_use);
  #endif
    //if(sp.status == DEV_PRESENT) {  <=== add this ???
    if(sensor.readSensor(*k)) {
      k->tripped = TRUE;
      if(!k->reported) reporting = TRUE;
      //temptrip = TRUE;
      trippedList.push_back(sp);
      #ifdef SERIAL_DEBUG
            Serial.println();
            Serial.print("sensor tripped:name=");
            Serial.println(sp.name);
      #endif
    }
    else { // ??????
      k->tripped = FALSE;
    }
  }
  return reporting;
}
void Alarm::clearing_countup() {
  if(allClear()) {
    if(clearing_cnt++ > CLEARING_COUNT_MAX) { //?? count
      //debug("clearing_countup:cnt=%d\\n",clearing_cnt);
      prevTripState = FALSE;
      clearing_cnt = 0;
      setState(alarm_clearing);
    }
  }
  else clearing_cnt = 0;
}

//get device by index
device_t* Alarm::getDevice(uint8_t n) {
  std::list<device_t>::iterator k;
  for(k=device_list.begin(); k != device_list.end(); ++k)
    if(k->idx == n) {
      device_t* sp=&(*k);
      return sp;
    }
  return 0;
}

// initialize with sensor ???
Alarm alarm1;
//timer use CAUSES build to FAIL ???
//Timer blinkTimer(1000, &Alarm::blinkTimeout, alarm1, TRUE);
//Since timer in class does not build
//void clear_blinking() {
//  alarm1.blinkTimeout();
//}

//Timer blink2Timer(5000, clear_blinking);

// set time at initialization
void Notifier::setStartTime() {
  hours_between_alert = alarm_saved_state.alert_hours;
  hour=Time.hour();
  lasthour = hour - (hour % hours_between_alert);
  if(lasthour <0) lasthour += 24;
  #ifdef SERIAL_DEBUG
    Serial.print("hour=");
    Serial.println(hour);
    Serial.print("lasthour=");
    Serial.println(lasthour);
    #endif
    lastminute=Time.minute();
}

void Notifier::checkTime() {
  String worry_message;
  int tempHour = hour;
  int min_hours_between;
  hour=Time.hour();
  minute=Time.minute();

  // 10  18  18-10 = 8
  elapsedminutes = minute - lastminute;
  //50 10 10 - 50 + 60 = 20
  if(elapsedminutes < 0) elapsedminutes += 60;
  if(elapsedminutes >= WORRY_MINUTES) {
    elapsedminutes = minute;
  }
  min_hours_between = max(1,hours_between_alert);
  tempHour = hour;
  if(tempHour < lasthour) tempHour += 24;
  //if hours_between_alert = 4  {4,8,12}
  //FIRST TIME EARLY: LAST_HOUR 3
  //if hours_between_alert is zero, don't alert but do log temp hourly
  if(tempHour - lasthour >= min_hours_between) { //NEEDS TO BE ON BOUNDARY
    if(lasthour!=-1){
      if(hours_between_alert != 0) {
         worry_message = updData();
         queueMessage(worry_message);
      }
      char tempF[10];
      sprintf(tempF,"%4.1fF",p_sensor->readTemperature());
      Particle.publish("temperature2", tempF);
      hourlyReset();
      #ifdef SERIAL_DEBUG
        Serial.print("hour changed! ");
        Serial.print("HOUR:");
        Serial.println(hour);
      #endif
    }
    lasthour = hour;
  }
}

// push status message on demand
int Notifier::upd(String command) {
  String msg;
  msg = updData();  //nameing ??
  msg_limit = 0;    //force clear with status
  queueMessage(msg);
  return 1;
}
// format common update message
String Notifier::updData() {
  device_t* ts;
  char uptime[40];
  char buf[40];
  //make this a method in Notifier
  //to share with worry
  sys.upTime(uptime);
  String msg = String("\\nsys up time: ");
  msg.concat(uptime);
  sprintf(buf,"\\nworry %d hours",hours_between_alert);
  msg.concat(buf);
  ts = alarm1.firstTripped();
  if(ts != NULL) {
    //SHOULD do all tripped, not just first
    sprintf(buf,"\\ntripped sensors:\\n");
    strcat(buf,ts->name);
  } else
    sprintf(buf,"\\nall sensor clear");
  msg.concat(buf);
  //sprintf(buf,"\\nclearing: %d",alarm1.clearingCount());
  //msg.concat(buf);
  return msg;
}

// set alarm state remotely
int Notifier::set(String cmd_line) {
  int cidx;
  int p1,p2,p3;
  int i,h,n;
  char msg[20];
  char args[20];
  char args2[20];
  char args3[20];
  char device_listing[600];   //size ????
  device_t *d;
  String delim = String(".");
  String commands = String("HLP.ABT.RAN.SET.DIS.ACK.HOU.LIS.NAM.SEN.ACT.DEA.MIN.");
  //                        0   1   2   3   4   5   6   7   8   9   10  11  12
  if(bad_secret_cnt >= BAD_SECRET_MAX) {
    queueMessage("TOO MANY BAD SECRETS");  //debug message
    return 0;
  }
  sscanf (cmd_line,"%s %s %s %s",msg,args,args2,args3);
  Serial.printf("set: cmd=%s args=%s args2=%s\n",msg,args,args2);
  String parsed_cmd = String(msg);
  parsed_cmd.toUpperCase();
  parsed_cmd = parsed_cmd.substring(0,3);
  parsed_cmd.concat(delim);
  Serial.print("parsed cmd:");
  Serial.println(parsed_cmd);
  cidx=commands.indexOf(parsed_cmd);
  if(cidx==-1) {
    //debug w/ cmd
    sprintf(msg, "INVALID COMMAND:%s", parsed_cmd.c_str());
    queueMessage(msg);
    //queueMessage("INVALID COMMAND");
    return 0;
  }
  else cidx /=4;
  // now parse args ...
  //kludge: if contains ' '
  p1 = 0;
  String arg_list = String(args);
  String arg_list2 = String(args2);
  String arg_list3 = String(args3);
  Serial.print("args:");
  Serial.println(args);
  p1 = arg_list.toInt();
  n = arg_list.indexOf(' ');
  Serial.print("index of blank:");
  Serial.println(n);
  p2 = arg_list2.toInt();
  p3 = arg_list3.toInt();
  Serial.print("p1=");
  Serial.print(p1);
  Serial.print(" p2=");
  Serial.println(p2);
  Serial.print("args3:");
  Serial.println(arg_list3);

  Serial.printf("cmd index=%d\n",cidx);
  Serial.printf("secret=%d\n",p1);
  Serial.printf("arg=%d\n",p2);
  //now we can switch i 1..n!!!
  //ABT,HEP.RAN are no-secret
  if(cidx > 2) {
    if(!secret) return 0;
    if(p1 == secret)
      secret = 0;
    else {
      bad_secret_cnt++;
      queueMessage("INVALID SECRET");
      return 0;
    }
  }
  switch(cidx) {
    case 0 : // HELP
      //device_listing[0] = 0;
      strcpy(device_listing,"\\nABT - About");
      strcat(device_listing,"\\nHLP - Help");
      strcat(device_listing,"\\nRAN - New Secret");

      strcat(device_listing,"\\nSET Alarm On");
      strcat(device_listing,"\\nDISable Alarm");
      strcat(device_listing,"\\nACKnowledge Alert");
      strcat(device_listing,"\\nHOU btw Worry to n");
      strcat(device_listing,"\\nLISt All Sensors");
      strcat(device_listing,"\\nNAMe Sensor n");
      strcat(device_listing,"\\nSENsor Detail");
      strcat(device_listing,"\\nACTivate sensor n");
      strcat(device_listing,"\\nDEActivate sensor n");
      strcat(device_listing,"\\nMINimum Alert Temp to x");

      queueMessage(device_listing);
      break;
    case 1 : // ABT
      //device_listing[0] = 0;
      strcpy(device_listing,"\\nphotonAlarm\\n");
      strcat(device_listing,"\\ncopyright re:Engineering 2016");
      strcat(device_listing,"\\nDonald Thompson");
      sprintf(msg,"\\nVersion %d.%d.%d", SYSTEM_VERSION_MAJOR, SYSTEM_VERSION_MINOR, SYSTEM_VERSION_SUB);
      strcat(device_listing, msg);
      queueMessage(device_listing);
      break;
    case 2 : // RANDOM Secret
        secret = random(RANDOM_MIN,RANDOM_MAX); //small range for DEBUG
        secret_timeout = 120; // one minutes @ 500ms loops
        sprintf(msg, " Secret %d", secret);
        queueMessage(msg);
        break;
    case 3 : // SET Alarm if Secret matches
        //command = 'SET <secret>'
          // set unless tripped
        if(alarm1.setState(alarm_armed)) {
            alarm_saved_state.current_state = alarm1.getState();
            alarm1.writeSavedState();
            #ifdef SERIAL_DEBUG
              Serial.print('set armed');
            #endif
        }
        queueMessage(alarm1.getPendingMessage());
        return 1;
        break;
    case 4 : // Disarm Alarm if Secret matches
        //command = 'DISARM <secret>'
        // DISABLE alarm here!!
        alarm1.setState(alarm_disarmed);
        alarm_saved_state.current_state = alarm_disarmed;
        alarm1.writeSavedState();
        queueMessage(alarm1.getPendingMessage());
        return 1;
        break;
    case 5 : // Acknowledge tripped Alarm
        // clear blink state
        alarm1.setState(alarm_armed);
        if(alarm1.clearSensorReported())
          queueMessage("ALARM ACKNOWLEDGED");
        else
          queueMessage("SENSOR STILL TRIPPED");
        queueMessage(alarm1.getPendingMessage());
        return 1;
        break;
    case 6 : // SET alert Hours if Secret matches
          //command = 'HOUR <hours> <secret>'
          //sscanf (cmd_line,"%s %d %d",msg,&i, &h);
          // set hours_between_alert if valid
          if(p2 >= 0 and p2 <= 12) {
              setAlertHours(p2);
              if(p2)
                sprintf(msg, "HOURS BTW WORRY SET TO %d", p2);
              else
                sprintf(msg, "WORRY ALERTS DISABLED");
              queueMessage(msg);
              #ifdef SERIAL_DEBUG
                Serial.print('seting hours_between_alert');
              #endif
          } else
            queueMessage("INVALID ALERT HOURS");
            //queueMessage(alarm1.getPendingMessage());
          return 1;
          break;
    case 7 : // List all Devices
          //command = 'LIST <secret>'
          // LIST ALL Sensor Devices
          alarm1.deviceListing(device_listing);
          queueMessage(device_listing);
          return 1;
          break;

    case 8 : // Name a Sensor
          //command = 'NAM <secret> <sen#> <short name> <long name>'
          msg[0]=0;
          // NAME a Sensor Devices
          d = alarm1.getDevice(p2);
          if(d) {
              d->name = args3;
              //now update configuration in EEPROM
              alarm1.updateConfiguration(d, p2);
              sprintf(msg, "Rename Sen# %d to %s", p2, args3);
              //queueMessage(msg);
          }
          else
              sprintf(msg, "NO SENSOR# %d", p2);
          queueMessage(msg);
          return 1;
          break;
    case 9 : // List a Devices in Full
          //command = 'SEN <secret> <sen#>'
          device_listing[0]=0;
          d = alarm1.getDevice(p2);
          if(d) {
              sprintf(device_listing,"\\nIndex: %d",d->idx);
              strcat(device_listing,"\\nrom: ");
              char buf[26];
              p_sensor->romFormat(&buf[0], d->dev_rom);
              strcat(device_listing,buf);
              strcat(device_listing,"\\nStatus: ");
              strcat(device_listing,sensor_status_def[d->status]);
              strcat(device_listing,"\\nState: ");
              strcat(device_listing,sensor_state_def[d->state]);
              strcat(device_listing,"\\nUse: ");
              strcat(device_listing,sensor_use_def[d->dev_use]);
              strcat(device_listing,"\\nSense: ");
              strcat(device_listing,sensor_sense_def[d->sense]);
              if(d->alert_min) {
                sprintf(buf,"\\nAlert Min: %d", d->alert_min);
                strcat(device_listing, buf);
              }
              strcat(device_listing,"\\nName: ");
              strncat(device_listing,d->name,SENSOR_NAME_SIZE);
              strcat(device_listing,"\\nTripped: ");
              if(d->tripped)
                  strcat(device_listing,"Y");
              else
                  strcat(device_listing,"N");
              strcat(device_listing,"\\nReported: ");
              if(d->reported)
                  strcat(device_listing,"Y");
              else
                  strcat(device_listing,"N");
              sprintf(buf,"\\nReading: %5.2f",d->dev_reading);
              strcat(device_listing,buf);
          }
          else
              sprintf(device_listing, "NO SENSOR# %d", p2);
          queueMessage(device_listing);
          return 1;
          break;
      case 10 : // Activate a sensor
          //command = 'ACT <secret> <sen#>'
          d = alarm1.getDevice(p2);
          if(d) {
              alarm1.setDeviceActive(d, TRUE);
              sprintf(msg, "SENSOR ACTIVATED: %d", p2);
          }
          else
              sprintf(msg, "NO SENSOR# %d", p2);
          queueMessage(msg);
          return 1;
          break;
      case 11 : // Deactivate a sensor
          //command = 'DEA <secret> <sen#>'
          d = alarm1.getDevice(p2);
          if(d) {
              alarm1.setDeviceActive(d, FALSE);
              sprintf(msg, "SENSOR DEACTIVATED: %d", p2);
          }
          else
              sprintf(msg, "NO SENSOR# %d", p2);
          queueMessage(msg);
          return 1;
          break;
      case 12 : // Set Sensor Minimum Alarm Temperature
          //command = 'MIN <secret> <sen#> <min>'
          d = alarm1.getDevice(p2);
          if(d) {
              if(d->dev_use !=  OW_THERMOMETER && d->dev_use != MCP9808_THERMOMETER) {
                  sprintf(msg, "SENSOR# %d NOT THERM", p2);
                  queueMessage(msg);
                  return 0;
              }
              alarm1.setDeviceAlertMin(d, p3);
              sprintf(msg, "SENSOR# %d ALERT MIN : %d", p2, p3);
          }
          else
              sprintf(device_listing, "NO SENSOR# %d", p2);
          queueMessage(msg);
          return 1;  //ret values ??
          break;

      default:   // Error
          queueMessage("UNRECOGNIZED COMMAND");
          return 0;
  }
  return 1;
}

//function to count down secret_timeout, and clear secret
void Notifier::secret_countdown() {
  if(!secret_timeout--) secret = 0;
}
void Notifier::setAlertHours(int hours) {
  lasthour = Time.hour() - 1;
  if(lasthour < 0) lasthour = 23;
  hours_between_alert = hours;
  //xxxxyy
  //write ??
  alarm1.writeSavedState();
}
uint8_t Notifier::getAlertHours() {
  return hours_between_alert;
}

bool Notifier::msgqueueEmpty() {
    return msg_queue.empty();
}
void Notifier::queueMessage(String msg) {
  msg_queue.push(msg);
}
void Notifier::hourlyReset() {
  bad_secret_cnt = 0;
  msg_limit = 0;
  digitalWrite(MESSAGE_PIN, LOW);
  if (millis() - lastSync > ONE_DAY_MILLIS) {
    // Request time synchronization from the Particle Cloud
    Particle.syncTime();
    lastSync = millis();
  }
}
void Notifier::dequeMessage() {
  String next_message;
  if(!msg_queue.empty()){
    next_message = msg_queue.front();
    //don't flood messages, but if they are not sent\
    //here we need to try again when hour changes...
    //just call dequeMessage..it checks if empty
    //Serial.print(" not empty");
    if(msg_limit++ < MAX_MESSAGE_PER_HOUR) {
      msg_queue.pop();
      sendMessage(next_message);
    }
    else
      digitalWrite(MESSAGE_PIN, HIGH);
  }
#ifdef SERIAL_DEBUGXX
  else {
    Serial.print("queue empty!");
  }
  Serial.println(" ");
#endif
}

void Notifier::sendMessage(String msg) {
  char tmessage[600];
  strncpy(tmessage, msg.c_str(), 598);
  tmessage[599]=0;
  sendMessage(tmessage);
}
//change queueMessage to simply insert message into pending queue
//add method to throttle calls to dequeMessage (x minutes between)
//Rename this method to actuall push next message
//loop will call dequeMessage every pass
// send message to pushover
void Notifier::sendMessage(char* msg) {
  Serial.print("sendMessage:");
  String time = Time.format("%m/%d/%y %H:%M:%S");
  sprintf(event_message,"%s %4.1fF\\n%s",time.c_str(),
      p_sensor->readTemperature(),
      alarm_state_name_def[alarm1.getState()]);
  strcat(event_message," ");

  if(alarm1.isTripped()) {
    strcat(event_message," sensor tripped:\\n");
    device_t* ts;
    //NULL if trippedList empty
    //iterate  thru trippedList
    ts = alarm1.firstTripped();
    if(ts != NULL) {
      strncat(event_message,ts->name,6);
      strcat(event_message," ");
    }
  }
  strcat(event_message, msg);
#ifdef PUSHOVER_SEND_MESSAGES
  Serial.print(" publish ");
  Spark.publish(webhook_id, event_message, 60, PRIVATE);
//  Spark.publish(webhook_id, event_message, 60, PUBLIC);
#endif
#ifdef SERIAL_DEBUG
  Serial.print("msg:");
  Serial.println(event_message);
#endif
}
void Notifier::pushAlarm(char* msg) {
  //this can include global info about
  //what is tripped ??
  if(alarm_times_notified++ < MAX_ALARM_NOTIFIED) {
    event_message[0] = 0;
    strcat(event_message,Time.timeStr().c_str());

    //  strcat(event_message, name of alarm in trip); ??

    //strcat(event_message, msg);
    //publish should be to another webhook, ie webhook_alarm
    //and set quite times different for this hook
    //Spark.publish now returns a bool to indicate success!!!
    Spark.publish(webhook_id, event_message, 60, PRIVATE);
  }
  else
    digitalWrite(MESSAGE_PIN, HIGH);
}

void reset_handler()
{
    // tell the world what we are doing
    Particle.publish("reset", "going down for reboot NOW!");
    notify.queueMessage("SYSTEM REBOOTING");
}

void setup() {
  // register the reset handler
  System.on(reset, reset_handler);
  int newseed = 0;
  random_seed_from_cloud(newseed); //?? usage ??
  pinMode(ledStatus, OUTPUT);
  pinMode(ledNotice, OUTPUT);
  pinMode(trigger3, OUTPUT);
  sys.sysState(sys_undefined);
#ifdef SERIAL_DEBUG
    Serial.begin(9600);
  #ifdef SERIAL_WAIT
    while(!Serial.available())  // Wait here until the user presses ENTER
      Spark.process();          // in the Serial Terminal. Call the BG Tasks
    Serial.read();
  #endif
#endif
    //delay(1000);
    Serial.printf("\\nPhotoAlarm initializing...Version %d.%d.%d",
          SYSTEM_VERSION_MAJOR, SYSTEM_VERSION_MINOR, SYSTEM_VERSION_SUB);
    sys.sysState(sys_starting);
//    digitalWrite(trigger3, HIGH); //DEBUG, trigger pin3 for logic 8 analyzer
    Time.zone(EDT_OFFSET);
    uint8_t stat = mcp9808.begin(MCP9808_I2CADDR);  //bus master
    //non-zero = error, not mcp9808
    if(stat) {
      sys.addStatus(fail_hardware);
      notify.sendMessage("MCP9808 FAIL");
      sys.sysState(sys_fail); //this will do HANG
    }
    ow.reset();
    if(!alarm1.readSavedState()) {
      alarm_saved_state.current_state = alarm_disarmed;
      alarm_saved_state.alert_hours = ALERT_HOURS;
      alarm1.writeSavedState();
    }
    else {
      notify.setAlertHours(alarm_saved_state.alert_hours);
    }
    if(!alarm1.readConfiguration()) {
      notify.queueMessage("FAILED");
      #ifdef SERIAL_DEBUG
          Serial.println("EEPROM Config Invalid");
      #endif
      if(alarm1.writeTestConfiguration()) {
        notify.queueMessage("WRITECONFIG");
        #ifdef SERIAL_DEBUG
            Serial.println("EEPROM WriteConfig");
        #endif
      }
      sys.addStatus(fail_eeprom);
      notify.sendMessage("CFG FAIL");   //send this message now
      sys.sysState(sys_fail); //this will do HANG
    }
    //build device list from read configuration
    if(alarm1.buildDeviceList()) {
      alarm1.dump_device_list();
    }
    else notify.queueMessage("NODEVICE");

    // check DS2482 status and config to confirm presence
    ow.reset();
    stat = ow.wireReadStatus(FALSE);
    #ifdef SERIAL_DEBUG
    Serial.print("DS2482 Reset - Status Reg:");
    Serial.println(stat, HEX);  //sb 0x18 (RST=1 AND 1-WIRE LL HIGH?)
    #endif
    if(stat != 0x18) {
      notify.sendMessage("HDW FAIL");   //send this message now
      sys.addStatus(fail_ds2482);
      sys.sysState(sys_fail);           //and halt
    }
    stat = ow.wireReadConfig();
    #ifdef SERIAL_WAIT
    Serial.print("Config Reg:");
    Serial.println(stat, HEX);
      while(!Serial.available())  // Wait here until the user presses ENTER
        Spark.process();          // in the Serial Terminal. Call the BG Tasks
      Serial.read();
    #endif
    // CHECK / COMPARE devices from config to
    // actual devices discovered
    // and mark PRESENT OR notify errors
    //CHECK FOR OW PRESENCE IF OW DEVICES IN CONFIGURE ???
    if(sys.sysStatus()==0) {
      if(ow.wireReset()) {
        #ifdef SERIAL_DEBUGXX
          Serial.println("1-wire device pulse detected");
          #endif
      }
      else {
        #ifdef SERIAL_DEBUG
          Serial.println("NO DS2482 device found");
          #endif
        }
#define  DEBUG_AQUIRE
      #ifdef DEBUG_AQUIRE
        Serial.println("scanning ow...");
        uint8_t rwa;
        rwa = alarm1.wiredeviceAquire();
        Serial.print("wiredeviceAquire return:");
        Serial.println(rwa);
        #endif
    }
#ifdef SERIAL_DEBUGXX
    Serial.println("Final Dump device_list");
#endif
    alarm1.dump_device_list();
    // iterate over device_list and do search rom if type in xx
    if(alarm1.validate_device_list()) {
      notify.queueMessage("Starting");
  #ifdef SERIAL_DEBUG
      Serial.println("Device List Valid");
  #endif
    }
    else {
      //we allow system to run with some missing sensors
      //should halt if count is zero ??
      notify.queueMessage("DEVFAILVAL");
#ifdef SERIAL_DEBUG
    Serial.println("Device List NOT Valid");
#endif
    }
    if(sys.sysStatus()==0) {
      sys.sysState(sys_running);
      notify.queueMessage("SYSTEM RUNNING");
    }
    else {
      char buf[18];
      sprintf(buf,"SYSTEM FAIL:%x\n",sys.sysStatus());
      notify.queueMessage(buf);

    }

    // set lasthour to  be on 'hours_between_alert' multiple
    notify.setStartTime();
    unsigned long timeSeed = micros();
    randomSeed(timeSeed);
    sensor.readTemperature();
    Serial.println("running...");
    //sync alarm state with eeprom status  ?? necessary or already done ??
    alarm1.setState(alarm_saved_state.current_state);
}

void loop() {
    notify.checkTime();
  #ifdef SERIAL_DEBUG
    Serial.print(".");
  #endif
    // MAKE this all a notify function
    if(alarm1.isArmed()) {
      if(alarm1.checkSensors()) {
        // an unreported sensor is tripped
        // already know about this if alarm_tripped or alarm_notifying
        // else set state to alarm_tripped
        //typedef enum {alarm_disarmed=0,alarm_armed=1,alarm_tripped=2,alarm_notifying=3,alarm_clearing=4};
        //const char* alarm_state_name_def[5] = {"disarmed", "armed", "tripped", "notify", "clear"};
        if(!alarm1.isTripped()) {    //???? map the state moves out ??
          notify.queueMessage("sensor tripped\\n");
          alarm1.setState(alarm_tripped);
        }
      }
      else {
        // all sensors are clear OR reported
        if(alarm1.prevTripped()) {
          //goes back to armed state by ack message or timer timeout
          //or by loop countdown ???
          alarm1.setState(alarm_notifying);
          notify.queueMessage(alarm1.getPendingMessage());
        }
      }
    }
    alarm1.doStatusBlink();
    //blink2Timer.start();   // to clear blink at timeout
    notify.secret_countdown();  //timeout secret number
    if(message_countdown-- == 0) {
      notify.dequeMessage();
      message_countdown = MESSAGE_LOOP_DELAY;  // manual messages should be immediate
    }
#ifdef SERIAL_DEBUGXX
    else if(!notify.msgqueueEmpty()) {
      Serial.print("dequeMessage delayed:");
      Serial.println(message_countdown);
    }
#endif
    if(alarm1.alarmNotifying()) alarm1.clearing_countup();
    delay(LOOP_DELAY_TIME);
}
