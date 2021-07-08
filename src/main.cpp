#include <Arduino.h>
#include <ArduinoJson.h>

// ----------------- delays
#define DELAY 5
#define DEFAULT_HEARTBEAT (5 * 60 * 1000l)

#ifndef SERIAL_SPEED
#warning "SERIAL_SPEED not defined"
#define SERIAL_SPEED 9600
#endif

#ifndef SERIAL_CONFIG
#warning "SERIAL_CONFIG not defined"
#define SERIAL_CONFIG SERIAL_8N1
#endif

// ----------------- PINS
// OUTPUTS
#define GND 9      // D9 as GND output
#define GND2 3     // D3 as GND output
#define VCC PIN_A3 // A3 as VCC output
// INPUTS
#define POWER_PIN PIN_A5   // A5 as green led
#define BATTERY_PIN PIN_A0 // A0 as yellow led
#define OVERLOAD_PIN 4     // D4 as red led

const static int inputPins[] = {POWER_PIN, BATTERY_PIN, OVERLOAD_PIN};

// STATUS
#define FRONT_FLAG 0x2
#define FRONT_MASK (~FRONT_FLAG)

#define OFF LOW
#define ON HIGH
#define DOWN (FRONT_FLAG & ON)
#define UP (FRONT_FLAG & ON)
#define NB_STATE_VAL 4

// STATES
#define STATE_POWER 0
#define STATE_BATTERY 1
#define STATE_OVERLOAD 2
#define NB_STATE 3

#define NB_HISTO (1000 / DELAY)

int statesHisto[NB_HISTO][NB_STATE];
int states[NB_STATE];
int statePointer;
unsigned long lastMillis;
bool hasToPrint;
unsigned long heartbeat = DEFAULT_HEARTBEAT;

const static char *VALUES_STR[] = {
    // status
    "OFF",  // already off
    "ON",   // already on
    "DOWN", // just change to off
    "UP"    // just change to on
};

#define DEFAULT_SENSOR "Fortress 900 v2"
String sensor = DEFAULT_SENSOR;

void jsonResult(const String result,const bool help=false){
  Serial.print(F("{\"sensor\":\""));
  Serial.print(sensor);
  Serial.print(F("\",\"result\":\""));
  Serial.print(result);
  Serial.print(F("\""));
  if(help){
    Serial.print(F(",\"help\":{\"commands\":["
    "{\"cmnd\":\"status\"},"
    "{\"cmnd\":\"set\",\"parameters\":{"
    "\"sensor\":\"Fortress\","
    "\"heartbeat\":10000"
    "}}]}"));
  }
  Serial.println(F("}"));
}

void jsonStatus(unsigned long currentMillis){
  Serial.print(F("{\"sensor\":\""));
  Serial.print(sensor);
  Serial.print(F("\",\"status\":{\"power\":\""));
  Serial.print(VALUES_STR[states[STATE_POWER]]);
  Serial.print(F("\",\"battery\":\""));
  Serial.print(VALUES_STR[states[STATE_BATTERY]]);
  Serial.print(F("\",\"overload\":\""));
  Serial.print(VALUES_STR[states[STATE_OVERLOAD]]);
  Serial.print(F("},\"time\":"));
  Serial.print(currentMillis);
  Serial.print(F(",\"heartbeat\":"));
  Serial.print(heartbeat);
  Serial.println(F("}"));
}

void setup()
{
  // init state historic
  for (int h = 0; h < NB_HISTO; h++)
  {
    for (int s = 0; s < NB_STATE; s++)
    {
      statesHisto[h][s] = ON;
    }
  }
  for (int s = 0; s < NB_STATE; s++)
  {
    states[s] = ON;
  }
  statePointer = 0;
  lastMillis = 0;
  hasToPrint = true;

  // initialize serial communication
  Serial.begin(SERIAL_SPEED, SERIAL_CONFIG);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB
  }
  // make pins inputs:
  pinMode(POWER_PIN, INPUT_PULLUP);
  pinMode(BATTERY_PIN, INPUT_PULLUP);
  pinMode(OVERLOAD_PIN, INPUT_PULLUP);
  // D3 and D8 as GND
  pinMode(GND, OUTPUT);
  digitalWrite(GND, LOW);
  pinMode(GND2, OUTPUT);
  digitalWrite(GND2, LOW);
  // A3 as VCC
  pinMode(VCC, OUTPUT);
  digitalWrite(VCC, HIGH);

  jsonResult(F("setup complete"));
}

void loop()
{
  unsigned long currentMillis = millis();

  // read the input pins
  for (int s = 0; s < NB_STATE; s++)
  {
    int curState = HIGH - digitalRead(inputPins[s]);
    statesHisto[statePointer][s] = curState;

    if (statePointer % NB_HISTO / 2 == 0)
    {
      // compute mean state
      curState = OFF;
      for (int h = 0; h < NB_HISTO; h++)
      {
        curState |= statesHisto[h][s];
      }

      // detect front up/down
      if (curState != (states[s] & FRONT_MASK))
      {
        // set front flag
        states[s] = FRONT_FLAG | curState;
        hasToPrint = true;
      }
      else
      {
        // remove front flag
        states[s] = curState;
      }
    }
  }

  if (Serial.available())
  {
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, Serial);
    if (!err)
    {
      const char *cmnd = doc["cmnd"] | "";
      if (strcmp(cmnd, "status") == 0)
      {
        hasToPrint = true;
        jsonResult(F("OK"));
      }
      else if (strcmp(cmnd, "set") == 0)
      {
        const char * inSensor = doc["parameters"]["sensor"] | "NONE";
        bool notSensor = strcmp(inSensor, "NONE") == 0;
        const unsigned long inHeartbeat = doc["parameters"]["heartbeat"] | 0l;
        bool notHeartbeat = inHeartbeat == 0l;
        if(notHeartbeat && notSensor){
          jsonResult(F("Incorrect parameters"), true);
        } else {
          jsonResult(F("OK"));
          if(!notHeartbeat){
            heartbeat = inHeartbeat;
          }
          if(!notSensor){
            sensor = String(inSensor);
          }
          hasToPrint = true;
        }
      }
      else
      {
        jsonResult(F("Unknown command"), true);
      }
    }
    else
    {
      jsonResult(F("Bad syntax"), true);
    }
  }

  if (hasToPrint)
  {
    lastMillis = currentMillis;
  }
  else if (currentMillis - lastMillis > heartbeat)
  {
    lastMillis += heartbeat;
    hasToPrint = true;
  }

  // serial output (JSON) if there is any change or PRINT_TIMEOUT
  if (hasToPrint)
  {
    jsonStatus(currentMillis);
  }

  delay(DELAY);
  hasToPrint = false;
  statePointer = (statePointer + 1) % NB_HISTO;
}