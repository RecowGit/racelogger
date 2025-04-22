#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <ESPAsyncWebServer.h>
#include <NimBLEDevice.h>
#include "WSLED.h"

extern WSLED wsled;

extern SPIClass hspi;            // HSPI-Instanz für SD-Karte
extern SdFs sd;                  // SdFat-FS für SD-Zugriff

extern bool sdInitialized;

// Webserver und Steuerung
extern AsyncWebServer server;
extern bool serverRunning;
extern bool bleRunning;

// Schalter-Pin (Webserver/BLE Toggle) und Entprellzeit
extern const uint8_t ServerSwitchPin;
extern const unsigned long debounceDelay;


extern bool rbconnected;

extern uint16_t header;
extern uint8_t messageClass;
extern uint8_t messageId;
extern uint16_t payloadLength;
extern uint32_t iTOW;
extern uint16_t year;
extern uint8_t month;
extern uint8_t day;
extern uint8_t hour;
extern uint8_t minute;
extern uint8_t second;
extern uint8_t validityFlags;
extern uint32_t timeAccuracy;
extern uint32_t nanoseconds;
extern uint8_t fixStatus;
extern uint8_t fixStatusFlags;
extern uint8_t dateTimeFlags;
extern uint8_t numSVs;
extern int32_t longitude;
extern int32_t latitude;
extern int32_t wgsAltitude;
extern int32_t mslAltitude;
extern uint32_t horizontalAccuracy;
extern uint32_t verticalAccuracy;
extern uint32_t gpsspeed;
extern uint32_t heading;
extern uint32_t speedAccuracy;
extern uint32_t headingAccuracy;
extern uint16_t pdop;
extern uint8_t latLonFlags;
extern uint8_t batteryStatus;
extern int16_t gForceX;
extern int16_t gForceY;
extern int16_t gForceZ;
extern int16_t rotRateX;
extern int16_t rotRateY;
extern int16_t rotRateZ;

extern float roll;
extern float pitch;
extern float lateral;

extern char frontString[256];
extern char rearString[256];

extern int distFront;
extern int distRear;
extern int frontTemps[8];
extern int rearTemps[8];
extern float frontTemp;
extern float rearTemp;

extern volatile int rxspeed;
extern volatile int rxrpm;
extern volatile int rxgear;
extern volatile int rxthrottle;