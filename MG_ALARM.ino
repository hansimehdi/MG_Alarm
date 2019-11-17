#include "Adafruit_FONA.h"

#define FONA_RX 11
#define FONA_TX 10
#define FONA_RST 13
#include <SoftwareSerial.h>
#define NUM_SIZE 1
//fona initialization
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
char replybuffer[255];
char replybufferNum[255];
uint8_t type;

// owner number

char *numbers[] =
{
  (char *) "+21655450255",
  (char *) "+21694931981"
};


// System I/O
const byte SENSOR = 12;
const byte SIREN = 4;
const byte LOCKED_LED = 5;
const byte UNLOCKED_LED = 6;
const byte NETWORK_LED_OK = 7;
const byte NETWORK_LED_NO = 8;

//system stat
bool SYS_STAT;
bool DANGER_STAT;
bool NETWORKSTAT;


void setup() {
  pinMode(SENSOR, INPUT);
  pinMode(SIREN, OUTPUT);
  pinMode(LOCKED_LED, OUTPUT);
  pinMode(UNLOCKED_LED, OUTPUT);
  pinMode(NETWORK_LED_OK, OUTPUT);
  pinMode(NETWORK_LED_NO, OUTPUT);

  //init system
  sys_init();
  delay(30000);
  gsm_init();
  attachInterrupt(1, toggleStat, FALLING);
}

void loop() {
  checkNetwork();
  smsReaderToggler();
  if (SYS_STAT) {
    checkSensor();
    checkDanger();
  }
  delay(1000);
}


bool isIn(char text[], char *search) {
  char *p;
  p = &text[0];
  if (strstr(p, search) != NULL) {
    return true;
  } else {
    return false;
  }
}

void gsm_init() {
  while (!Serial);
  Serial.begin(115200);
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));
  fonaSerial->begin(9600);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }
  uint8_t n = fona.getNetworkStatus();
  Serial.print(F("Network status "));
  Serial.print(n);
  Serial.print(F(": "));

  if (n == 0) Serial.println(F("Not registered"));
  if (n == 1) {
    Serial.println(F("Registered (home)"));
    NETWORKSTAT = true;
  } else {
    NETWORKSTAT = false;
  }
  if (n == 2) Serial.println(F("Not registered (searching)"));
  if (n == 3) Serial.println(F("Denied"));
  if (n == 4) Serial.println(F("Unknown"));
  if (n == 5) Serial.println(F("Registered roaming"));
  fona.deleteAllSMS();
}

void sys_init() {
  digitalWrite(SIREN, LOW);
  digitalWrite(LOCKED_LED, LOW);
  digitalWrite(UNLOCKED_LED, HIGH);
  digitalWrite(NETWORK_LED_NO, HIGH);
  digitalWrite(UNLOCKED_LED, HIGH);

  SYS_STAT = false;
  DANGER_STAT = false;
  NETWORKSTAT = false;
}

void toggleStat() {
  SYS_STAT = false;
  DANGER_STAT = false;
  stopAlarm();
  switchLockLed();
}

void smsReaderToggler() {
  if (fona.getNumSMS() > 0) {
    for (int i = 0; i <= NUM_SIZE; i++)
    {
      char *x = numbers[i];
      if (fona.getSMSSender(1, replybufferNum, 250)) {
        if (isIn(replybufferNum, (char *) x)) {
          uint16_t smslen;
          if (fona.readSMS(1, replybuffer, 250, &smslen)) {
            if (isIn(replybuffer, (char *) "SYS_LOCK")) {
              SYS_STAT = true;
              switchLockLed();
              if (fona.sendSMS(x, (char *) "System has been locked successfully")) {
                fona.deleteAllSMS();
              } else {
                delay(5000);
              }
              break;
            } else if (isIn(replybuffer, (char *) "SYS_UNLOCK")) {
              stopAlarm();
              switchLockLed();
              if (fona.sendSMS(x, (char *) "System has been unlocked successfully")) {
                fona.deleteAllSMS();
              } else {
                delay(5000);
              }
            }
          }
        } else {
          fona.deleteAllSMS();
        }
      }
    }
  }
  fona.deleteAllSMS();
}

void alarm() {
  digitalWrite(SIREN, HIGH);
}

void inDanger() {
  if (SYS_STAT) {
    DANGER_STAT = true;
  }
}

void stopAlarm() {
  DANGER_STAT = false;
  SYS_STAT = false;
  digitalWrite(SIREN, LOW);
}

bool replayOwner(char *n, bool type) {
  if (type) {
    return fona.sendSMS(n, (char *) "SYS_LOCKED_SUCCESS");
  } else {
    return fona.sendSMS(n, (char *) "SYS_UNLOCKED_SUCCESS");
  }
}

void networkIndicator() {
  if (NETWORKSTAT) {
    digitalWrite(NETWORK_LED_OK, HIGH);
    digitalWrite(NETWORK_LED_NO, LOW);
  } else {
    digitalWrite(NETWORK_LED_OK, LOW);
    digitalWrite(NETWORK_LED_NO, HIGH);
  }
}

void checkNetwork() {
  uint8_t n = fona.getNetworkStatus();
  if (n == 1) {
    NETWORKSTAT = true;
  } else {
    NETWORKSTAT = false;
  }
  networkIndicator();
}

void checkSensor() {
  int x = digitalRead(SENSOR);
  if (x == LOW) {
    inDanger();
  }
}

void checkDanger() {
  if (DANGER_STAT) {
    alarm();
    for (int i = 0; i <= NUM_SIZE; i++)
    {
      char *x = numbers[i];
      if (SYS_STAT && DANGER_STAT) {
        int8_t callstat = fona.getCallStatus();
        if ((callstat == 0)) {
          fona.callPhone(x);
          delay(30000);
          fona.hangUp();
          delay(2000);
        }
        callstat = fona.getCallStatus();
        while(callstat!=0){
          delay(5000);
        }
        delay(100);
        smsReaderToggler();
      } else {
        delay(5000);
      }
      delay(5000);
    }
  }
}

void switchLockLed() {
  if (SYS_STAT) {
    digitalWrite(LOCKED_LED, HIGH);
    digitalWrite(UNLOCKED_LED, LOW);
  } else {
    digitalWrite(LOCKED_LED, LOW);
    digitalWrite(UNLOCKED_LED, HIGH);
  }
}
