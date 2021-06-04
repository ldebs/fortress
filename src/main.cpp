#include <Arduino.h>
#include <ArduinoJson.h>

// ----------------- delays
#define DELAY 100
#define PRINT_TIMEOUT (10 * 1000)

// ----------------- PINS
// OUTPUTS
#define GND 9      // D9 as GND output
#define GND2 3     // D3 as GND output
#define VCC PIN_A3 // A3 as VCC output
// INPUTS
#define POWER_PIN PIN_A5   // A5 as green led
#define BATTERY_PIN PIN_A0 // A0 as yellow led
#define OVERLOAD_PIN 4     // D4 as red led

const int inputPins[] = {POWER_PIN, BATTERY_PIN, OVERLOAD_PIN};

#define FRONT_FLAG 0x2
#define FRONT_MASK (~FRONT_FLAG)
int states[] = {LOW, LOW, LOW};
unsigned long lastMillis = 0;
bool hasToPrint = true;

#define NB_STATE 3
#define NB_STATE_VAL 4;
const char *JSON_FIELDS[] = {
    // {
    "sensor", // = sensor value
    "status", //=
    // {
    "power",    // = one of status values
    "battery",  // = one of status values
    "overload", // = one of status values
    // },
    "time" // = currentMillis
           // }
};
const char *VALUES_STR[] = {
    // sensor
    "fortress 900 v2",
    // status
    "OFF",  // already off
    "ON",   // already on
    "DOWN", // just change to off
    "UP"    // just change to on
};

void setup()
{
  // initialize serial communication
  Serial.begin(9600);
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
}

void loop()
{
  unsigned long currentMillis = millis();

  // read the input pins
  for (int i = 0; i < NB_STATE; i++)
  {
    int curState = HIGH - digitalRead(inputPins[i]);
    // detect front up/down
    if (curState != (states[i] & FRONT_MASK))
    {
      // set front flag
      states[i] = FRONT_FLAG | curState;
      hasToPrint = true;
    }
    else
    {
      // remove front flag
      states[i] = curState;
    }
  }

  if (hasToPrint)
  {
    lastMillis = currentMillis;
  }
  else if (currentMillis - lastMillis > PRINT_TIMEOUT)
  {
    lastMillis += PRINT_TIMEOUT;
    hasToPrint = true;
  }

  // serial output (JSON) if there is any change or PRINT_TIMEOUT
  if (hasToPrint)
  {
    int f = 0, v = 0;
    StaticJsonDocument<128> doc;
    doc[JSON_FIELDS[f++]] = VALUES_STR[v++];
    JsonObject state = doc.createNestedObject(JSON_FIELDS[f++]);
    for (int i = 0; i < NB_STATE; i++)
    {
      state[JSON_FIELDS[f++]] = VALUES_STR[v + states[i]];
    }
    v += NB_STATE_VAL;
    doc[JSON_FIELDS[f++]] = currentMillis;

    serializeJson(doc, Serial);
    Serial.println();
  }

  delay(DELAY);
  hasToPrint = false;
}