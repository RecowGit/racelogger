#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <stdint.h>
#include "globals.h"

// Öffentliche Schnittstellenfunktionen für RaceBox
void RaceBox_setup();
void RaceBox_loop();


// Berechnungsfunktionen
float getGpsSpeed();    // GPS-Geschwindigkeit (km/h, 2 Dezimalstellen)
int16_t getGeFX();        // gForce X (in g, 3 Dezimalstellen)
int16_t getGeFY();        // gForce Y (in g, 3 Dezimalstellen)
int16_t getGeFZ();        // gForce Z (in g, 3 Dezimalstellen)
int16_t getGyroX();       // Gyroskop X (deg/s, 2 Dezimalstellen)
int16_t getGyroY();       // Gyroskop Y (deg/s, 2 Dezimalstellen)
int16_t getGyroZ();       // Gyroskop Z (deg/s, 2 Dezimalstellen)
extern float getRoll();        // Rollwinkel in Grad
extern float getPitch();       // Nickwinkel in Grad
extern float getLateralForce();// Laterale Kraft in Newton

// Optional: Hilfsfunktion zum Runden
float roundToDecimals(float value, int decimals);

// Funktionen zur Generierung von NMEA-0183 Strings
String generateGPGGA();
String generateGPRMC();
extern String generateDateTime();


