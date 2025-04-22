#include <Arduino.h>
#include "racebox.h"
#include <freertos/FreeRTOS.h>
#include <HardwareSerial.h>
#include <EasyTransfer.h>
#include "logger.h"
#include "globals.h"
#include <Wire.h>
#include "WSLED.h"
#include "logserver.h"
#include <ESPAsyncWebServer.h>

SPIClass hspi(HSPI); // Definition des HSPI-Objekts

// -------------------------
// HSPI-Konfiguration und SD-Einstellungen
// -------------------------
// extern SPIClass hspi;  // Definition in logger.cpp

// SD-Pin-Konfiguration
const int SD_CS    = 13;
const int HSPI_SCK = 14;
const int HSPI_MISO= 2;
const int HSPI_MOSI= 15;

#define SD_CONFIG SdSpiConfig(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(16), &hspi)
#define SD_FAT_TYPE 3  // 1 = FAT16/FAT32, 2 = exFAT, 3 = beide

HardwareSerial uart(2);
EasyTransfer ETin, ETout; 

AsyncWebServer server(80);    // Webserver-Instanz auf Port 80
bool serverRunning = false;
bool bleRunning = false;
const uint8_t ServerSwitchPin = 34;

#define RX_PIN 32
#define TX_PIN 33

WSLED wsled(26);

int distFront = 0;
int distRear = 0;
int frontTemps[8];
int rearTemps[8];
float frontTemp = 0.0;
float rearTemp = 0.0;
volatile int rxspeed;
volatile int rxrpm;
volatile int rxgear;
volatile int rxthrottle;
// volatile int16_t speed = 0;
// volatile int16_t rpm = 0;
// volatile int8_t gang = 0;
// volatile int8_t throttle = 0;

struct RECEIVE_DATA_STRUCTURE {
  int16_t distFront;
  int16_t frontTemps8[8];
  int16_t distRear;
  int16_t rearTemps8[8];
  int16_t speed;
  int16_t rpm;
  int8_t gang;
  int8_t throttle;
};

struct SEND_DATA_STRUCTURE {
  int16_t gForceX;
  int16_t gForceY;
  int16_t gForceZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
};

RECEIVE_DATA_STRUCTURE rxdata;
SEND_DATA_STRUCTURE txdata;


void printdata(){
  // Serial.print("GPS Speed: ");
  // Serial.print(getGpsSpeed());
  // Serial.print(" km/h");
  
  // Serial.print(" | ");

  // Serial.print("Roll: ");
  // Serial.print(getRoll());
  // Serial.print(" deg");

  // Serial.print(" | ");
  
  // Serial.print("Lateral Force: ");
  // Serial.print(getLateralForce());
  // Serial.print(" N");

}

void printNMEA() {
  Serial.println(generateGPGGA());
  Serial.println(generateGPRMC());
}

char frontString[256];
char rearString[256];

void processReceivedData(const RECEIVE_DATA_STRUCTURE &data) {
  // Umrechnung in Float erfolgt beispielhaft durch Division durch 10.0
  float frontDistance = data.distFront / 10.0;
  float rearDistance  = data.distRear  / 10.0;

  // Berechnung der Durchschnittstemperaturen
  float frontSum = 0.0;
  float rearSum = 0.0;
  for (int i = 0; i < 8; i++) {
    frontSum += data.frontTemps8[i] / 10.0;
    rearSum += data.rearTemps8[i] / 10.0;
  }
  float frontaverage = frontSum / 8.0;
  float rearaverage = rearSum / 8.0;

  // Erstelle den Front-String: "Front: Distance: x.x, Temps: t0, t1, ..., t7"
  sprintf(frontString, "%.1f, %.1f, ", frontaverage, frontDistance);
  for (int i = 0; i < 8; i++) {
    char tempStr[16];
    float tempValue = data.frontTemps8[i] / 10.0;
    sprintf(tempStr, "%.1f", tempValue);
    strcat(frontString, tempStr);
    if (i < 7) {
      strcat(frontString, ", ");
    }
    // Serial.println(frontString);
  }

  // Erstelle den Rear-String: "Rear: Distance: x.x, Temps: t0, t1, ..., t7"
  // sprintf(rearString, "Rear: Average Temp: %.1f, Distance: %.1f, Temps: ", rearaverage, rearDistance);
  sprintf(rearString, "%.1f, %.1f, ", rearaverage, rearDistance);
  for (int i = 0; i < 8; i++) {
    char tempStr[16];
    float tempValue = data.rearTemps8[i] / 10.0;
    sprintf(tempStr, "%.1f", tempValue);
    strcat(rearString, tempStr);
    if (i < 7) {
      strcat(rearString, ", ");
    }
  }
  // Serial.println(rearString);
}

void setup() {
  pinMode(ServerSwitchPin, INPUT_PULLUP);
  Serial.begin(115200);
  uart.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);  // RX: 4  TX: 2
  hspi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI);
  ETin.begin(details(rxdata), &uart);
  ETout.begin(details(txdata), &uart);

  hspi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI);

  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
  }

  logger_setup();
  logServerSetup();
  RaceBox_setup();
  
  wsled.begin();
  wsled.setBrightness(20);
  // wsled.on(WSLED::GREEN);
  wsled.tripleBlink(WSLED::GREEN, 150, 400);

}

void loop() {
  RaceBox_loop();
  logServerLoop();

  if (ETin.receiveData()) {
    // Serial.println("Empfang erfolgreich!");
    processReceivedData(rxdata);
} else {
//    Serial.println("Kein Datenempfang");
}

  distFront = rxdata.distFront;
  distRear = rxdata.distRear;

  for (int i = 0; i < 8; i++) {
    frontTemps[i] = rxdata.frontTemps8[i]; }

  for (int i = 0; i < 8; i++) {
    rearTemps[i] = rxdata.rearTemps8[i]; }

  rxspeed = rxdata.speed;
  rxrpm = rxdata.rpm;
  rxgear = rxdata.gang;
  rxthrottle = rxdata.throttle;
  // // Summe aller Werte
  // int32_t sum = 0; // 32-Bit-Variable zur Vermeidung von Überlauf
  // for (int i = 0; i < 8; i++) {
  //   sum += frontTemps[i];
  // }
  // // Durchschnitt berechnen (als float)
  // frontTemp = (float)sum / 8.0;

  // // Summe aller Werte
  // sum = 0; // 32-Bit-Variable zur Vermeidung von Überlauf
  // for (int i = 0; i < 8; i++) {
  //   sum += rearTemps[i];
  // }
  // // Durchschnitt berechnen (als float)
  // rearTemp = (float)sum / 8.0;

  txdata.gForceX = gForceX;
  txdata.gForceY = gForceY;
  txdata.gForceZ = gForceZ;
  txdata.gyroX = rotRateX;
  txdata.gyroY = rotRateY;
  txdata.gyroZ = rotRateZ;

  ETout.sendData();

  // Serial.println(frontString);
  // Serial.println(rearString);

  // printdata();
  // printNMEA();
}