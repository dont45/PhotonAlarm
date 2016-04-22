/* notifier.h
 *
 */
 #ifndef __NOTIFIER_H__
 #define __NOTIFIER_H__

 #include "application.h"
 #include "util.h"
 #include "parms.h"
 #include "alarm.h"
 #include <queue>
 #include <list>

 class Notifier {
 public:
   Notifier(char *event_name, Sensor *s) {
     // move upd to Alarm class ???
     Particle.function("upd", &Notifier::upd, this);
     Particle.function("set", &Notifier::set, this);
     strcpy(webhook_id, event_name);
     p_sensor = s;
     msg_limit = 0;        //limit mesages per hour
     secret_timeout = 0;   //loop passes to timeout secret
     bad_secret_cnt = 0;
     alarm_notify_cnt = 0;
     alarm_times_notified = 0;
     hours_between_alert = ALERT_HOURS;
     digitalWrite(MESSAGE_PIN, LOW);
   }
   int upd(String command);
   int set(String);
   String updData();
   void sendMessage(char* msg);  //send message to pushover
   void sendMessage(String);
   void queueMessage(String);    //add message to queue
   bool msgqueueEmpty();
   void dequeMessage(void);      //get next message from queue and send
   void hourlyReset();
   void pushAlarm(char* msg);
   void secret_countdown();
   void setStartTime();
   void setAlertHours(int h);
   uint8_t getAlertHours();
   void checkTime();

 private:
   std::queue<String> msg_queue;
   char webhook_id[20];
   char event_message[1000];   //??reduce size of this ??
   // to do, clear secret after xx loops ??? DEBUG
   int secret;
   unsigned int secret_timeout;
   uint8_t bad_secret_cnt;
   int alarm_times_notified;
   int alarm_notify_cnt;
   int msg_limit;
   int hour = 0;
   int minute = 0;
   int lasthour = 0;   //use -1 to skip startup
   int lastminute = 0;
   int elapsedminutes = 0;
   uint8_t hours_between_alert;
   unsigned long lastSync = millis();
   Sensor *p_sensor;
   Alarm *p_alarm;  //??
 };
#endif
