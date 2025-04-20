#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include "racebox.h"

// -------------------------
// HSPI-Konfiguration und SD-Einstellungen
// -------------------------
extern SPIClass hspi;  // Definition in logger.cpp

// SD-Pin-Konfiguration
const int SD_CS    = 13;
const int HSPI_SCK = 14;
const int HSPI_MISO= 2;
const int HSPI_MOSI= 15;

#define SD_CONFIG SdSpiConfig(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(16), &hspi)
#define SD_FAT_TYPE 3  // 1 = FAT16/FAT32, 2 = exFAT, 3 = beide

// -------------------------
// Hardware-Pin-Konfiguration
// -------------------------
const int switchPin = 35;  // Taster zum Start/Stop des Loggings
const int ledPin    = 27;  // LED zeigt aktiven Logvorgang an

// -------------------------
// Double Circular Buffer Definitionen
// -------------------------
#define NUM_BUFFERS 2     // Zwei Buffer für Double Buffering
#define NUM_LINES   50    // Maximale Anzahl Zeilen pro Buffer
#define LINE_LENGTH 512   // Maximale Länge einer Zeile

// -------------------------
// Öffentliche Funktionen des Loggers
// -------------------------
void logger_setup();         // Initialisiert SD, Pins und startet den Logging-Task
void logger_task(void *param); // Logging-Task, läuft auf Core 1

#endif // LOGGER_H
