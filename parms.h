/* parms.h
 *
 * Configuration parameters
 */
#ifndef __PARMS_H__
#define __PARMS_H__

#define EDT_OFFSET -4
#define EST_OFFSET -5

#define SENSOR_LIST_SCAN
#define ALERT_HOURS 3
#define LOOP_DELAY_TIME 250         //Delay ms between loop passes
#define MAX_ALARM_NOTIFIED 3
#define SYSTEM_STATUS_PIN D2        //RED if Armed, Blinking if any tripped
#define SYSTEM_NOTIFICATION_PIN D3  //GREEN blinks at each loop ??
#define TRIGGER_PIN D4              //debug pin for logic analyzer trigger
#define LOOP_SENSOR_PIN D4          //magnetic door loops
#define MOTION_LOOP_SENSOR_PIN D5   //motion detector
#define TAMPER_PIN D6               //system tamper switch loop
#define RANDOM_MIN 100              //range of random number for secret
#define RANDOM_MAX 999
#define WORRY_MINUTES 5             //time between follow-up notifications
#define MESSAGE_LOOP_DELAY 3        //number of loop passes btw notifications
#define MAX_MESSAGE_PER_HOUR 30     //absolute maximum of pushover messages per hour
#define MAXDEVICE 3                 //simple testing
#define MAX_ALLOW_OWDEV  10         //absolute maximum of ow devices
#define EE_MAGIC_CONFIG 0x16        // change this to write new test config
#define EE_MAGIC_STATE 0x90
#define FMLY_IBUTTON 0x01           //NOT USED ???
#define FMLY_2405 0x05
#define FMLY_2406 0x12              //and FMLY_2407
#define FMLY_1820 0x10
#define USE_FMLY_IBUTTON
#define USE_FMLY_2405
#define USE_FMLY_2406
#define USE_FMLY_1820
#define SENSOR_NAME_SIZE 16
#define SHORT_NAME_SIZE 2

#endif
