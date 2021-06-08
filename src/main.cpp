#include <Arduino.h>
#include <ArduinoJson.h>

// ----------------- delays
#define DELAY 5
#define PRINT_TIMEOUT (5 * 60 * 1000l)

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

const int inputPins[] = {POWER_PIN, BATTERY_PIN, OVERLOAD_PIN};

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

const char *VALUES_STR[] = {
    // status
    "OFF",  // already off
    "ON",   // already on
    "DOWN", // just change to off
    "UP"    // just change to on
};

#define PPCAT(a, b) a##b
#define JSON_BEGIN Serial.print("{\"sensor\":\"Fortress 900 v2\",")
#define JSON_OBJ(o) Serial.print(#o ":{")
#define JSON_KV(k, v) Serial.print(#k ":");Serial.print("\"");Serial.print(v);Serial.print("\"")
#define JSON_Kv(k, v) Serial.print(#k ":");Serial.print(v)
#define JSON_COMMA Serial.print(",")
#define JSON_ENDOBJ Serial.print("}")
#define JSON_END Serial.println("}")
#define JSON_RESULT(v) Serial.println("{\"sensor\":\"Fortress 900 v2\",\"RESULT\":" #v "}")

void setup()
{
  // init state historic
  for (int h = 0; h < NB_HISTO; h++)
  {
    for (int s = 0; s < NB_STATE; s++)
    {
      statesHisto[h][s] = OFF;
    }
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

  JSON_RESULT("setup complete");
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
    // Serial.println("read...");
    // String str=Serial.readString();
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, Serial);
    if (!err)
    {
      const char *cmnd = doc["cmnd"] | "";
      if (strcmp(cmnd, "status") == 0)
      {
        hasToPrint = true;
        JSON_RESULT("OK");
      }
      else
      {
        JSON_RESULT("Unknown command");
      }
    }
    else
    {
      JSON_RESULT("Bad syntax");
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
    JSON_BEGIN;
    JSON_OBJ("status");
    JSON_KV("power",VALUES_STR[states[STATE_POWER]]);
    JSON_COMMA;
    JSON_KV("battery",VALUES_STR[states[STATE_BATTERY]]);
    JSON_COMMA;
    JSON_KV("overload",VALUES_STR[states[STATE_OVERLOAD]]);
    JSON_ENDOBJ;
    JSON_COMMA;
    JSON_Kv("time",currentMillis);
    JSON_END;
  }

  delay(DELAY);
  hasToPrint = false;
  statePointer = (statePointer + 1) % NB_HISTO;
}