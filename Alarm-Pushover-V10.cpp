/*
  pushover alarm system for Particle Photon
  Version 1.0
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

#define SYSTEM_VERSION "V1.0"
#define SERIAL_DEBUG
#define SERIAL_WAIT
#include "application.h"
#include <queue>
#include <list>
#include "Adafruit_MCP9808.h"
#include "OW.h"
#include "parms.h"
#include "util.h"

int ledStatus = SYSTEM_STATUS_PIN;              //blink this LED at each loop pass
int ledNotice = SYSTEM_NOTIFICATION_PIN;      //blink this LED at each notification
int trigger3 = TRIGGER_PIN;
int ledStatusState = 0;
int ledStatusBlink = 0;                       // 0=solid,1=slow blink,2=fast blink
int ledStatusBlinkCount = 0;
int message_countdown = 1;
char event_message[40];

Adafruit_MCP9808 mcp9808;
OW ow(0);       //default 0 is 0x18 !!

#include "state.h"
State sys;

//sensor devices will all be loaded from EEPROM
//and added to a std::list
//1-wire devices are checked for presence
class Sensor {
public:
  Sensor() {
    // Move this to function in configuration read
    pinMode(LOOP_SENSOR_PIN, INPUT);
    pinMode(MOTION_LOOP_SENSOR_PIN,INPUT);
    pinMode(TAMPER_PIN,INPUT);
  }
  //bool readSensor(int);
  bool readSensor(device_t d);
  float readTemperature();
  float getLastTemperature();
  char* romFormat(char *buf, uint8_t rom[]);
private:
  float last_temp;  //used ??
};

// return TRUE if tripped
bool Sensor::readSensor(device_t d) {
    uint8_t senval;
    double tempF;
    bool rok;
    if(d.status != DEV_PRESENT) {
  #ifdef SERIAL_DEBUGXX
      Serial.print("Device Missing - ROM:");
      Serial.println(d.dev_rom[1],HEX);
  #endif
      return FALSE;
    }
    switch(d.dev_use) {
      case SWITCH:    //simple switch on PHOTON pin
  #ifdef SERIAL_DEBUGXX
          Serial.print("readSensor SWITCH:idx");
          Serial.print(sensor_idx);
          Serial.print(" PIN:");
          Serial.print(d.dev_addr[0]);
  #endif
         senval = digitalRead(d.dev_rom[0]);
         d.dev_reading = (float)senval;
  #ifdef SERIAL_DEBUGXX
          Serial.print(" value=");
          Serial.println(senval);
  #endif
          return senval==d.sense;
          break; //xxxx
      case OW_SENSOR:  //one-wire device on DS2482-100
          senval = ow.readPIOB(d.dev_rom);
          d.dev_reading = (float)senval;
          #ifdef SERIAL_DEBUG_OW
                  Serial.println("readSensor OW_SENSOR:");
                  Serial.print(" ow value=");
                  Serial.println(senval);
          #endif
                  return senval==d.sense;
          break;
      case MCP9808_THERMOMETER:  //one-wire device on DS2482-100
              // to be implemented
              // readTempF should use config parameters to get device info
              // i.e. i2c address etc.  This allows multiple devices on bus
              last_temp = mcp9808.readTempF();
              d.dev_reading = last_temp;
              // save to sensor structure
              // return value should  be tripped if outside temp range ??
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
              d.dev_reading = -1.0; //debug marker ??
            #endif
              rok = ow.readThermometer(d.dev_rom, &tempF);
              if(rok) {
                d.dev_reading = tempF;
                Serial.print("temp Fahrenheit x100:");
                Serial.println((int)tempF*100);
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

Sensor sensor;

// Alarm states xxx
typedef enum {alarm_disarmed=0,alarm_armed=1,alarm_tripped=2,alarm_notifying=4,alarm_clearing};

class Alarm {
public:
  Alarm() {
    readSavedState();
    prevTripState = FALSE;
    firstTrippedSensor=0;
  }
  bool setState(uint8_t new_state);
  bool readConfiguration();
  bool writeTestConfiguration();
  uint8_t buildDeviceList();
  void dump_device_list();
  bool validate_device_list();
  uint8_t wiredeviceAquire();
  bool wiredeviceValidate(uint8_t *ROM);
  bool readSavedState();
  bool writeSavedState();
  bool isTripped();
  bool prevTripped();
  device_t *firstTripped();
  bool isArmed();
  void doStatusBlink();
  void blinkTimeout();
  String getPendingMessage();
  bool checkSensors(void);
  String getLastTemperature();  //testing
private:
  device_t device;
  std::list<device_t> D; //??name
  std::list<device_t> trippedList;
  uint8_t curState;
  bool prevTripState;
  uint8_t firstTrippedSensor;
  uint8_t clearing_cnt;
  String message;  //??name
};

String Alarm::getLastTemperature() {
  //String t = String("100.00");  //will get from sensor.....???
  return String(sensor.getLastTemperature());
  //return t;
}
//typedef enum {alarm_disarmed=0,alarm_armed=1,alarm_tripped=2,alarm_notifying=4,alarm_clearing};

bool Alarm::setState(uint8_t new_state) {
  bool result = TRUE;
  curState = new_state;
  switch(curState) {
      case alarm_disarmed:
    #ifdef SERIAL_DEBUG
        Serial.println("alarm disarmed");
    #endif
        alarm_state.armed = FALSE;
        message = "DISARMED";
        ledStatusState = 0;
        ledStatusBlink = 0;
        break;
    case alarm_armed:
    #ifdef SERIAL_DEBUG
        Serial.println("alarm armed");
    #endif
        // add concept of prevTripped
        if(alarm_state.tripped) {
            //fast blink status
            ledStatusBlink = 2;
            message = "NOT CLEAR";
            result = FALSE;
            break;
        }
        ledStatusState = 1;
        ledStatusBlink = 0;
        if(alarm_state.armed)
          message = "ALARM RESET";
        else {
          alarm_state.armed = TRUE;
          message = "ALARM SET";
        }
        break;
    case alarm_tripped:
    #ifdef SERIAL_DEBUG
        Serial.println("alarm tripped");
    #endif
        ledStatusBlink = 1;
        message = "ALARM";
        break;
    case alarm_notifying:  //NOT used ???
    #ifdef SERIAL_DEBUG
        Serial.println("alarm notifying");
    #endif
        break;
    case alarm_clearing:
    #ifdef SERIAL_DEBUG
        Serial.print("alarm clearing:");
        Serial.println(clearing_cnt);
    #endif
        message = "CLEARING";
        ledStatusState = 1;
        ledStatusBlink = 2;
        alarm_state.tripped = FALSE;
        if(prevTripped()) {
          if(clearing_cnt++ > 4) {
            prevTripState = FALSE;
            clearing_cnt = 0;
            setState(alarm_armed);  //recursive
          }
        }
        break;
    default:
    #ifdef SERIAL_DEBUG
        Serial.println("invalid status");
    #endif
        message = "INVALID ALARM STATUS";
  }

  digitalWrite(ledStatus, ledStatusState);
  writeSavedState();
  return result;
}

uint8_t Alarm::buildDeviceList() {
  uint8_t dev_cnt = 0;
  char nbuf[SENSOR_NAME_SIZE+1];
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
    for(int i=0;i<8;i++)
      device.dev_rom[i] = configuration.dev_addr[j][i];
    device.dev_use = configuration.use[j];
    if(device.dev_use == SWITCH || device.dev_use == MCP9808_THERMOMETER )
      device.status = DEV_PRESENT;
    else
      device.status = DEV_MISSING;
    device.sense = configuration.sense[j];
    //write non-terminaed string copy to String routine??
    nbuf[0]=configuration.short_name[j][0];
    nbuf[1]=configuration.short_name[j][1];
    nbuf[2]=0;
    for(int k=0;k<SENSOR_NAME_SIZE;k++)
      nbuf[k]=configuration.name[j][k];
    nbuf[SHORT_NAME_SIZE]=0;
    device.short_name = String(nbuf);
    device.name = String(nbuf);
#ifdef SERIAL_DEBUGXX
    Serial.print("adding device: name=");
    Serial.println(device.name);
#endif
    device.dev_reading = 0.0;
    D.push_back(device);
    dev_cnt++;
  }
#ifdef SERIAL_DEBUGXX
  Serial.print("added to device_list:");
  Serial.println(dev_cnt);
#endif
  return dev_cnt;
}

//debug dump of sensor list
void Alarm::dump_device_list() {
  std::list<device_t>::iterator k;
for(k=D.begin(); k != D.end(); ++k) {
   device_t sp = *k;
   Serial.print(sp.short_name);
   Serial.print("  Status:");
   Serial.print(sp.status);
   Serial.print(" Use:");
   Serial.print(sp.dev_use);
   Serial.print(" ROM:");
   Serial.print(sp.dev_rom[0],HEX);
   Serial.print(":");
   Serial.print(sp.dev_rom[1],HEX);
   Serial.print(":");
   Serial.print(sp.dev_rom[2],HEX);
   Serial.print(":");
   Serial.print(sp.dev_rom[3],HEX);
   Serial.print(":");
   Serial.print(sp.dev_rom[4],HEX);
   Serial.print(":");
   Serial.print(sp.dev_rom[5],HEX);
   Serial.print(":");
   Serial.print(sp.dev_rom[6],HEX);
   Serial.print(":");
   Serial.print(sp.dev_rom[7],HEX);
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
  for(k=D.begin(); k != D.end(); ++k) {
    device_t sp = *k;
    //OW_SENSOR=2, OW_RELAY=3, OW_THERMOMETER=4,
    if(sp.dev_use >= OW_SENSOR && sp.dev_use <= OW_THERMOMETER)
      if(sp.status != 1) {
        all_present = FALSE;
        Serial.print("DEV not present:");
        Serial.println(sp.name);
      }
  }
  if(all_present)
    Serial.println("all dev present");
  return all_present;
}

bool Alarm::isTripped() {
  return alarm_state.tripped;
}
bool Alarm::prevTripped() {
  return prevTripState;
}
device_t* Alarm::firstTripped() {
  if(trippedList.empty()) return NULL;
  return &trippedList.front();
}
bool Alarm::isArmed() {
  return alarm_state.armed;
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
  for(k=D.begin(); k != D.end(); ++k) {
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
// DEBUG, for now just print them
uint8_t Alarm::wiredeviceAquire() {
  uint8_t ROM[8];
  uint8_t i,j,k;
  ow.wireResetSearch();
  for(i=0;i<MAX_ALLOW_OWDEV;i++) {
    j=ow.wireSearch(ROM); //CRASHES HERE !!!
    if(j==0) break;
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
#define TEST_SENSOR_2
//#define TEST_SENSOR_3
//#define TEST_SENSOR_4
//#define TEST_SENSOR_5
#define TEST_SENSOR_6
//#define TEST_SENSOR_7
  //Sensor 1
#ifdef TEST_SENSOR_1
  configuration.dev_addr[i][0] = MCP9808_I2CADDR;
  for(int k=1;k<8;k++)
    configuration.dev_addr[i][k] = 0xff;
  configuration.use[i] = MCP9808_THERMOMETER;
  configuration.sense[i] = 0;
  strncpy(configuration.short_name[i], "IT", SHORT_NAME_SIZE);
  strncpy(configuration.name[i], "INSIDE TEMP", SENSOR_NAME_SIZE);
  i++;
#endif
  //Sensor 2
#ifdef TEST_SENSOR_2
const uint8_t testrom1[8] = { 0x12,0x3a,0x84,0x72,0x00,0x00,0x00,0xd8 };
for(int k=0;k<8;k++)
  configuration.dev_addr[i][k] = testrom1[k];
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  strncpy(configuration.short_name[i], "DR", SHORT_NAME_SIZE);
  strncpy(configuration.name[i], "SHED DOOR", SENSOR_NAME_SIZE);
  i++;
#endif
  //Sensor 3
#ifdef TEST_SENSOR_3
const uint8_t testrom2[8] = { 0x05,0xe9,0xef,0x05,0x00,0x00,0x00,0x42 };
for(int k=0;k<8;k++)
  configuration.dev_addr[i][k] = testrom2[k];
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  strncpy(configuration.short_name[i], "O3", SHORT_NAME_SIZE);
  strncpy(configuration.name[i], "OWS SWITCH3", SENSOR_NAME_SIZE);
  i++;
#endif
  //Sensor 4
#ifdef TEST_SENSOR_4
const uint8_t testrom3[8] = { 0x10,0xc8,0xb6,0x1e,0x00,0x00,0x00,0x3b };
for(int k=0;k<8;k++)
  configuration.dev_addr[i][k] = testrom3[k];
  configuration.use[i] = OW_THERMOMETER;
  configuration.sense[i] = 0;
  strncpy(configuration.short_name[i], "T4", SHORT_NAME_SIZE);
  strncpy(configuration.name[i], "OUTSIDE TEMP4", SENSOR_NAME_SIZE);
  i++;
#endif
  //Sensor 5
#ifdef TEST_SENSOR_5
const uint8_t testrom4[8] = { 0x10,0x5d,0xab,0x4c,0x01,0x08,0x00,0xf3 };
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom4[k];
  configuration.use[i] = OW_THERMOMETER;
  configuration.sense[i] = 0;
  strncpy(configuration.short_name[i], "T5", SHORT_NAME_SIZE);
  strncpy(configuration.name[i], "DS1820 TEMP5", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef TEST_SENSOR_6
  const uint8_t testrom5[8] = { 0x12,0x6d,0x25,0x0a,0x00,0x00,0x00,0x39 };
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom5[k];
    configuration.use[i] = OW_SENSOR;
    configuration.sense[i] = SENSE_NORMAL_OPEN;
    strncpy(configuration.short_name[i], "MD", SHORT_NAME_SIZE);
    strncpy(configuration.name[i], "MOTION", SENSOR_NAME_SIZE);
    i++;
#endif
#ifdef TEST_SENSOR_7
//add:12:73:25:A:0:0:0:71:  ds2407
  const uint8_t testrom6[8] = { 0x12,0x73,0x25,0x0a,0x00,0x00,0x00,0x71 };
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom6[k];
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  strncpy(configuration.short_name[i], "O7", SHORT_NAME_SIZE);
  strncpy(configuration.name[i], "O7 SWITCH", SENSOR_NAME_SIZE);
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
bool Alarm::readSavedState() {
  EEPROM_readAnything(ALARM_STATE_ADDRESS, alarm_state);    //we will read unless button pressed to init network??
  if(alarm_state.magic==EE_MAGIC_STATE)
  {
#ifdef SERIAL_DEBUG
    Serial.println("VALID MAGIC STATE");
    Serial.print("sysState=");
    Serial.println(alarm_state.current_state);
    Serial.print("Armed=");
    Serial.println(alarm_state.armed);
#endif
    ledStatusState = alarm_state.armed;
    return TRUE;
  }
#ifdef SERIAL_DEBUG
  Serial.println("INVALID MAGIC STATE");
#endif
  return FALSE;
}
bool Alarm::writeSavedState() {
  alarm_state.magic = EE_MAGIC_STATE;
  alarm_state.current_state = curState;
  //just write current alarm_state values
#ifdef SERIAL_DEBUG
  Serial.print("writing SavedState: Armed=");
  Serial.println(alarm_state.armed);
#endif
  EEPROM_writeAnything(ALARM_STATE_ADDRESS, alarm_state);
  return TRUE;
}

bool Alarm::readConfiguration() {
  // READ configuration for updates
  // How to read eeprom on photon ??
  EEPROM_readAnything(CONFIGURATION_ADDRESS, configuration);    //we will read unless button pressed to init network??
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

// returns TRUE if any sensor tripped
bool Alarm::checkSensors(void) {
  bool temptrip = FALSE;
  //?? Must check all sensors here for alarm condition and notify
  // if not already notified and handle countdown to too many notifications
  // and reset conditions once alarm condition is cleared
  //here we pass thru configuration struct
  prevTripState = alarm_state.tripped;
  firstTrippedSensor = 0;
  trippedList.clear();
  //here we use will device_list
  //??? Add device scan here rather than
  // will require change to readSensor also
  std::list<device_t>::iterator k;
  for(k=D.begin(); k != D.end(); ++k) {
    device_t sp = *k;
    //if(sp.status == DEV_PRESENT) {  <=== add this ???
    if(sensor.readSensor(sp)) {
      temptrip = TRUE;
      trippedList.push_back(sp);
      #ifdef SERIAL_DEBUG
            Serial.println();
            Serial.print("sensor tripped:name=");
            Serial.println(sp.name);
      #endif
    }
  }
  alarm_state.tripped = temptrip;
  return temptrip;
}

Alarm alarm1;
//timer use CAUSES build to FAIL ???
//Timer blinkTimer(1000, &Alarm::blinkTimeout, alarm1, TRUE);

//Since timer in class does not build
void clear_blinking() {
  alarm1.blinkTimeout();
}

//Timer blink2Timer(5000, clear_blinking);

class Notifier {
public:
  Notifier(char *event_name, Sensor *s) {
    // move upd to Alarm class ???
    Particle.function("upd", &Notifier::upd, this);
    Particle.function("set", &Notifier::set, this);
    strcpy(webhook_id, event_name);
    sensor = s;
    msg_limit = 0;        //limit mesages per hour
    secret_timeout = 0;   //loop passes to timeout secret
    alarm_notify_cnt = 0;
    alarm_times_notified = 0;
  }
  int upd(String command);
  int set(String);
  void sendMessage(char* msg);  //send message to pushover
  void sendMessage(String);
  void queueMessage(String);    //add message to queue
  bool msgqueueEmpty();
  void dequeMessage(void);      //get next message from queue and send
  void hourlyReset();
  void pushAlarm(char* msg);
  void secret_countdown();
  void setStartTime();
  void checkTime();

private:
  std::queue<String> msg_queue;
  char webhook_id[20];
  char event_message[80];
  // to do, clear secret after xx loops ??? DEBUG
  int secret;
  unsigned int secret_timeout;
  int alarm_times_notified;
  int alarm_notify_cnt;
  int msg_limit;
  int hour = 0;
  int minute = 0;
  int lasthour = 0;   //use -1 to skip startup
  int lastminute = 0;
//  int elapsedminutes = 0;
  const char* float_to_cstr(float f);
  Sensor *sensor;
};

// set time at initialization
void Notifier::setStartTime() {
  hour=Time.hour();
  lasthour = hour - (hour % ALERT_HOURS);
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
  int tempHour = hour;

  hour=Time.hour();
  minute=Time.minute();
  /*
  // 10  18  18-10 = 8
  elapsedminutes = minute - lastminute;
  //50 10 10 - 50 + 60 = 20
  if(elapsedminutes < 0) elapsedminutes += 60;
  if(elapsedminutes >= WORRY_MINUTES) {
    elapsedminutes = minute;
  }
  */
  tempHour = hour;
  if(tempHour < lasthour) tempHour += 24;
  //ALERT_HOURS = 4  {4,8,12}
  //FIRST TIME EARLY: LAST_HOUR 3
  if(tempHour - lasthour >= ALERT_HOURS) { //NEEDS TO BE ON BOUNDARY
    if(lasthour!=-1){
      queueMessage("WORRY");
      String lt=alarm1.getLastTemperature();
      //String lt = String(sensor.getLastTemperature());
      Particle.publish("temperature", lt);
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

// push message on demand
int Notifier::upd(String command) {
  //do stuff
  queueMessage("UPD");
  //?? Compiler Error here ??? getLastT...
  //String lt = String(sensor.getLastTemperature());
  //Particle.publish("temperature", lt);
  return 1;
}

// set alarm state remotely
int Notifier::set(String command) {
  int i;
// use my old command-line parser here ???
  switch(command.charAt(0)) {
    case 'R' : // REQUEST Random Secret
        //command = 'REQUEST'
        secret = random(RANDOM_MIN,RANDOM_MAX); //small range for DEBUG
        secret_timeout = 60; // one minutes
        char msg[25];
        sprintf(msg, "SEC%d", secret);
        queueMessage(msg);
        break;
    case 'S' : // SET Alarm if Secret matches
        //command = 'SET<secret>'
        if(!secret) return 0;
        sscanf (command,"%s %d",msg,&i);
        if(i == secret) {
          secret = 0;
          // enable alarm here!!
          if(alarm1.setState(alarm_armed)) {
            #ifdef SERIAL_DEBUG
              Serial.print('set armed');
            #endif
          }
          queueMessage(alarm1.getPendingMessage());
        } else
          queueMessage("INVALID REQUEST");
        return 1;
        break;
    case 'D' : // Disarm Alarm if Secret matches
        if(!secret) return 0;
        //command = 'CLEAR <secret>'
        sscanf (command,"%s %d",msg,&i);
        if(i == secret) {
          secret = 0;
          // DISABLE alarm here!!
          alarm1.setState(alarm_disarmed);
          queueMessage(alarm1.getPendingMessage());
        } else
          queueMessage("INVALID REQUEST");
        return 1;
        break;
    case 'A' : // Acknowledge tripped Alarm
        if(!secret) return 0;
        sscanf (command,"%s %d",msg,&i);
        if(i == secret) {
          secret = 0;
          // clear blink state
          alarm1.setState(alarm_state.armed);
          queueMessage("ALARM ACKNOWLEDGED");
          queueMessage(alarm1.getPendingMessage());
        } else
          queueMessage("INVALID REQUEST");
          return 1;
          break;
    default:   // Error
        //queueMessage("UNKNOWN");
        return 0;
  }
  return 1;
}

//function to count down secret_timeout, and clear secret
void Notifier::secret_countdown() {
  if(!secret_timeout--) secret = 0;
}

const char* Notifier::float_to_cstr(float f) {
  char tempstr[8];
  //tempstr[0] = 0;
  //int inttemp = (int)f;
  //int fracttemp = (f - (float)inttemp) * 100;
  //%f conversion should work ????
  sprintf(tempstr,"%6.2f", f);
  //sprintf(tempstr,"%3d.%2d",inttemp,fracttemp);
  return tempstr;
}

bool Notifier::msgqueueEmpty() {
    return msg_queue.empty();
}
void Notifier::queueMessage(String msg) {
  msg_queue.push(msg);
}
void Notifier::hourlyReset() {
  msg_limit = 0;
}
void Notifier::dequeMessage() {
  String next_message;
  if(!msg_queue.empty()){
    next_message = msg_queue.front();
    if(msg_limit++ < MAX_MESSAGE_PER_HOUR) {
      msg_queue.pop();
      sendMessage(next_message);
    }
  }
}

void Notifier::sendMessage(String msg) {
  char message[80];
  strncpy(message, msg.c_str(), 80);
  sendMessage(message);
}
//change queueMessage to simply insert message into pending queue
//add method to throttle calls to dequeMessage (x minutes between)
//Rename this method to actuall push next message
//loop will call dequeMessage every pass
// send message to pushover
void Notifier::sendMessage(char* msg) {
  event_message[0] = 0;     //Why does strcpy NOT clear string?
  strcat(event_message,Time.timeStr().c_str());
  strcat(event_message,"Temp:");
  strcat(event_message, float_to_cstr(sensor->readTemperature()));

  if(alarm1.isTripped()) {
    strcat(event_message,"T*");
    device_t* ts;
    //NULL if trippedList empty
    ts = alarm1.firstTripped();
    if(ts != NULL)
      strncat(event_message,ts->short_name,2);
  }
  else if(alarm1.isArmed())
    strcat(event_message,"A ");
  else
    strcat(event_message,"D ");

  //strcat(event_message,"\n");
  strcat(event_message, msg);
  Spark.publish(webhook_id, event_message, 60, PRIVATE);
}
void Notifier::pushAlarm(char* msg) {
  //this can include global info about
  //what is tripped ??
  if(alarm_times_notified++ < MAX_ALARM_NOTIFIED) {
    event_message[0] = 0;
    strcat(event_message,Time.timeStr().c_str());
    /*
      strcat(event_message, name of alarm in trip); ??
    */
    strcat(event_message, msg);
    //publish should be to another webhook, ie webhook_alarm
    //and set quite times different for this hook
    Spark.publish(webhook_id, event_message, 60, PRIVATE);
  }
}
Notifier notify("safe-motion", &sensor);

void setup() {
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
    delay(1000);
    Serial.print("PhotonAlarm ");
    String version = String(SYSTEM_VERSION);
    Serial.println(version);
    Serial.println("initializing...");
#endif
//    digitalWrite(trigger3, HIGH); //DEBUG, trigger pin3 for logic 8 analyzer
    Time.zone(EST_OFFSET);
    uint8_t rv = mcp9808.begin(MCP9808_I2CADDR);  //bus master
    #ifdef SERIAL_DEBUG
        Serial.print("mcp begin() returned:");
        Serial.println(rv);
    #endif
    ow.reset();
    //if (1) Spark.connect();
    //should notify if rv (.begin()) failed ??
    if(!alarm1.readSavedState()) {
      alarm_state.armed = FALSE;
      alarm_state.tripped = FALSE;
      alarm1.writeSavedState();
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
      sys.sysState(sys_fail); //this will do HANG
    }
    //build device list from read configuration
    if(alarm1.buildDeviceList()) {
      alarm1.dump_device_list();
    }
    else notify.queueMessage("NODEVICE");
    // CHECK / COMPARE devices from config to
    // actual devices discovered
    // and mark PRESENT OR notify errors
    //CHECK FOR OW PRESENCE IF OW DEVICES IN CONFIGURE ???
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
    //sys.sysState(SYS_RUNNING);
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
      notify.queueMessage("DEVFAILVAL");
#ifdef SERIAL_DEBUG
    Serial.println("Device List NOT Valid");
#endif
    }

    sys.sysState(sys_running);
    // set lasthour to  be on ALERT_HOURS multiple
    notify.setStartTime();
    sensor.readTemperature();
    #ifdef SERIAL_DEBUG
        Serial.println("running...");
    #endif
}

void loop() {
#ifdef SERIAL_DEBUG
    Serial.print('.');
#endif
    // encapsulate all this loop time stuff into class :
    // into Notifier
    notify.checkTime();
    // MAKE this all a notify function
    if(alarm1.checkSensors()) {
      if(!alarm1.prevTripped()) {
        alarm1.setState(alarm_tripped);
        notify.queueMessage(alarm1.getPendingMessage());
        notify.queueMessage(alarm1.getPendingMessage()); //queue message twice
      }
    }
    else {
      if(alarm1.prevTripped()) {
        //goes back to armed state by ack message or timer timeout
        //or by loop countdown ???
        alarm1.setState(alarm_clearing);
        notify.queueMessage(alarm1.getPendingMessage());
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
    delay(LOOP_DELAY_TIME);
}
