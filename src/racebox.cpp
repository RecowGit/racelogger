#include "racebox.h"
#include <NimBLEDevice.h>
#include <math.h>
#include <cstdio>
#include "globals.h"
#include "WSLED.h"
#include "logserver.h"

// BLE UUIDs
static BLEUUID UART_service_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID TX_characteristic_UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

// Globale Statusvariablen
static bool doConnect = false;
static bool connected = false;
static bool updated_RaceBox_Data_Message = false; // Kennzeichnet, ob neue Daten vorliegen
bool rbconnected = false;

// Globale Zeiger für Remote-Characteristic und RaceBox-Gerät
static BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
static BLEAdvertisedDevice* myRaceBox = nullptr;

// Globale Variablen für RaceBox-Livedaten
uint16_t header;
uint8_t messageClass;
uint8_t messageId;
uint16_t payloadLength;
uint32_t iTOW;
uint16_t year;
uint8_t month;
uint8_t day;
uint8_t hour;
uint8_t minute;
uint8_t second;
uint8_t validityFlags;
uint32_t timeAccuracy;
uint32_t nanoseconds;
uint8_t fixStatus;
uint8_t fixStatusFlags;
uint8_t dateTimeFlags;
uint8_t numSVs;
int32_t longitude;
int32_t latitude;
int32_t wgsAltitude;
int32_t mslAltitude;
uint32_t horizontalAccuracy;
uint32_t verticalAccuracy;
uint32_t gpsspeed;
uint32_t heading;
uint32_t speedAccuracy;
uint32_t headingAccuracy;
uint16_t pdop;
uint8_t latLonFlags;
uint8_t batteryStatus;
int16_t gForceX;
int16_t gForceY;
int16_t gForceZ;
int16_t rotRateX;
int16_t rotRateY;
int16_t rotRateZ;

float headingDegrees;
String compass_direction;

// Hilfsvariablen für serielle Ausgabe
static unsigned long lastOutputTimeSerial = 0;
static const unsigned long outputIntervalMs_serial = 1000;

// Device Type: 0: RaceBox Mini/Mini S, 1: RaceBox Micro, -1: unbekannt
static int deviceType = -1;

// Konstante für die Masse (Motorrad + Fahrer)
static const float MASS_KG = 280.0;

// --- Vorwärtsdeklarationen interner Funktionen ---
static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
static void parsePayload(uint8_t* data);
static void parse_RaceBox_Data_Message_payload(uint8_t* data);
static String getCompassDirection(float headingDegrees);
static void decodeBatteryStatus(uint8_t batteryStatus);
static void calculateChecksum(uint8_t* data, uint16_t length, uint8_t& CK_A, uint8_t& CK_B);
static bool connectToRaceBox();
static void print_RaceBox_Data_to_serial();


// --- Callback-Klassen ---

// Client-Callback (angepasst an NimBLE-Arduino 2.2.2)
class ClientCallbacks : public NimBLEClientCallbacks {
public:
  void onConnect(NimBLEClient* pClient) {
    connected = true;
    Serial.println("RaceBox Connected!");
  }
  // Zusätzlicher Parameter 'reason' hinzugefügt
  void onDisconnect(NimBLEClient* pClient, int reason) {
    connected = false;
    Serial.println("Disconnected from RaceBox!");
    Serial.println("Hinweis: Disconnects können auftreten, wenn zu viel serielle Ausgabe den Code verzögert.");
    Serial.println("Versuche, die Verbindung wiederherzustellen...");
    doConnect = true;
  }
};

// AdvertisedDevice-Callback
class AdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
public:
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
    Serial.print("Advertised BLE Device found: ");
    Serial.println(advertisedDevice->toString().c_str());
  
    if (advertisedDevice->isAdvertisingService(UART_service_UUID)) {
      std::string deviceName = advertisedDevice->getName();
      if (deviceName.rfind("RaceBox", 0) == 0) { // Prüft, ob Name mit "RaceBox" beginnt
          if (deviceName.rfind("RaceBox Micro", 0) == 0) {               
              deviceType = 1;   
          }
          else if (deviceName.rfind("RaceBox Mini", 0) == 0) { 
              deviceType = 0;   
          }
          else {
              deviceType = -1;
          }
          
          #ifdef TARGET_DEVICE_ADDRESS
          std::string deviceAddress = advertisedDevice->getAddress().toString();
          if (deviceAddress == TARGET_DEVICE_ADDRESS) {
              Serial.printf("RaceBox found with address %s - matching TARGET_DEVICE_ADDRESS. Trying to connect...\n", deviceAddress.c_str());
              NimBLEDevice::getScan()->stop();
              myRaceBox = advertisedDevice;
              doConnect = true;
          } else {
              Serial.printf("Device name starts with RaceBox but address %s does not match TARGET_DEVICE_ADDRESS.\n", deviceAddress.c_str());
          }
          #else
          Serial.println("RaceBox found. TARGET_DEVICE_ADDRESS ist nicht gesetzt – Verbindung mit beliebiger RaceBox.");
          NimBLEDevice::getScan()->stop();
          Serial.printf("Verbinde zu RaceBox mit Adresse %s.... \n", advertisedDevice->getAddress().toString().c_str());
          myRaceBox = advertisedDevice;
          doConnect = true;
          #endif
      }
    }
  }
};

// --- Ende der Callback-Klassen ---

// Callback-Funktion für BLE-Notifications
static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    parsePayload(pData);
}

// --- Weitere interne Funktionen ---

static String getCompassDirection(float headingDegrees) {
  if (headingDegrees >= 337.5 || headingDegrees < 22.5) return "N";
  if (headingDegrees >= 22.5 && headingDegrees < 67.5) return "NO";
  if (headingDegrees >= 67.5 && headingDegrees < 112.5) return "O";
  if (headingDegrees >= 112.5 && headingDegrees < 157.5) return "SO";
  if (headingDegrees >= 157.5 && headingDegrees < 202.5) return "S";
  if (headingDegrees >= 202.5 && headingDegrees < 247.5) return "SW";
  if (headingDegrees >= 247.5 && headingDegrees < 292.5) return "W";
  if (headingDegrees >= 292.5 && headingDegrees < 337.5) return "NW";
  return "";
}

static void decodeBatteryStatus(uint8_t batteryStatus) {
    if (deviceType == 0) { // RaceBox Mini oder Mini S
        bool isCharging = (batteryStatus & 0x80) != 0;
        uint8_t batteryLevel = batteryStatus & 0x7F;
        Serial.print("RaceBox Mini/Mini S - Charging Status: ");
        Serial.println(isCharging ? "Charging" : "Not Charging");
        Serial.print("Battery Level: ");
        Serial.print(batteryLevel);
        Serial.println("%");
    } else if (deviceType == 1) { // RaceBox Micro
        Serial.print("RaceBox Micro - Input Voltage: ");
        float inputVoltage = batteryStatus / 10.0;
        Serial.print(inputVoltage, 1);
        Serial.println(" V");
    } else {
        Serial.println("Battery status: Unknown device type");
    }
}

static void calculateChecksum(uint8_t* data, uint16_t length, uint8_t& CK_A, uint8_t& CK_B) {
    CK_A = 0;
    CK_B = 0;
    for (int i = 2; i < length - 2; i++) {
        CK_A += data[i];
        CK_B += CK_A;
    }
}

static void parsePayload(uint8_t* data) {
    if (data[0] != 0xB5 || data[1] != 0x62) {
        Serial.println("Invalid frame start of payload data");
        return;
    }
    header = *(reinterpret_cast<uint16_t*>(data));
    messageClass = *(reinterpret_cast<uint8_t*>(data + 2));
    messageId = *(reinterpret_cast<uint8_t*>(data + 3));
    payloadLength = *(reinterpret_cast<uint16_t*>(data + 4));

    uint16_t packetLength = 6 + payloadLength + 2;
    if (packetLength > 512) {
        Serial.print("Received packet size exceeds maximum allowed size (512 bytes). Packet length: ");
        Serial.println(packetLength);
        return;
    }
    
    uint8_t CK_A, CK_B;
    calculateChecksum(data, packetLength, CK_A, CK_B);
    if (data[packetLength - 2] != CK_A || data[packetLength - 1] != CK_B) {
        Serial.println("*** Checksum validation failed. ***");
        return;
    }

    if (messageClass == 0xFF || messageId == 0x01) {
      parse_RaceBox_Data_Message_payload(data);
    }
}

static void parse_RaceBox_Data_Message_payload(uint8_t* data) {
  iTOW = *(reinterpret_cast<uint32_t*>(data + 6));                //e.g 0xA0 0xE7 0x0C 0x07
  year = *(reinterpret_cast<uint16_t*>(data + 10));               //e.g 0xE6 0x07 (2022) or 0xE8 0x07 (2024)
  month = *(reinterpret_cast<uint8_t*>(data + 12));               //0x01 (january) or 0x08 (august)
  day = *(reinterpret_cast<uint8_t*>(data + 13));                 //0x0A (10th) or 0x08 (8th)
  hour = *(reinterpret_cast<uint8_t*>(data + 14));                //0x08 (08 o'clock)
  minute = *(reinterpret_cast<uint8_t*>(data + 15));              //0x33 (51 min)
  second = *(reinterpret_cast<uint8_t*>(data + 16));              //0x08 (08 seconds)
  validityFlags = *(reinterpret_cast<uint8_t*>(data + 17));       //0x37 (Date/Time valid)
  timeAccuracy = *(reinterpret_cast<uint32_t*>(data + 18));       //0x19000000 (25 ns)
  nanoseconds = *(reinterpret_cast<uint32_t*>(data + 22));        //0x2AAD4D0E (239971626 ns = 0.239 seconds)
  fixStatus = *(reinterpret_cast<uint8_t*>(data + 26));           //0x03 (3D Fix)
  fixStatusFlags = *(reinterpret_cast<uint8_t*>(data + 27));      //0x01 (GNSS Fix OK)
  dateTimeFlags = *(reinterpret_cast<uint8_t*>(data + 28));       //0xEA (Date/Time Confirmed)
  numSVs = *(reinterpret_cast<uint8_t*>(data + 29));              //0x0B (11 satellites)
  longitude = *(reinterpret_cast<int32_t*>(data + 30));           //0xC693E10D (23.2887238 degrees)
  latitude = *(reinterpret_cast<int32_t*>(data + 34));            //0x3B376F19 (42.6719035 degrees)
  wgsAltitude = *(reinterpret_cast<int32_t*>(data + 38));         //0x618C0900 (625.761 meters)
  mslAltitude = *(reinterpret_cast<int32_t*>(data + 42));         //0x0F010900 (590.095 meters)
  horizontalAccuracy = *(reinterpret_cast<uint32_t*>(data + 46)); //0x9C030000 (0.924 meters)
  verticalAccuracy = *(reinterpret_cast<uint32_t*>(data + 50));   //0x2C070000 (1.836 meters)
  gpsspeed = *(reinterpret_cast<uint32_t*>(data + 54));           //0x23000000 (35 mm/s = 0.126 km/h)
  heading = *(reinterpret_cast<uint32_t*>(data + 58));            //0x00000000 (0 degrees)
  speedAccuracy = *(reinterpret_cast<uint32_t*>(data + 62));      //0xD0000000 (208 mm/s = 0.704 km/h)
  headingAccuracy = *(reinterpret_cast<uint32_t*>(data + 66));    //0x88A9DD00 (145.26856 degrees)
  pdop = *(reinterpret_cast<uint16_t*>(data + 70));               //0x2C01 (3)
  latLonFlags = *(reinterpret_cast<uint8_t*>(data + 72));         //0x00 (Coordinates valid)
  batteryStatus = *(reinterpret_cast<uint8_t*>(data + 73));       //has to be interpreted depending on if it is a RaceBox micro or mini, see my function void decodeBatteryStatus
  gForceX = *(reinterpret_cast<int16_t*>(data + 74));             //0xFDFF (-0.003 g)
  gForceY = *(reinterpret_cast<int16_t*>(data + 76));             //0x7100 (0.113 g)
  gForceZ = *(reinterpret_cast<int16_t*>(data + 78));             //0xCE03 (0.974 g)
  rotRateX = *(reinterpret_cast<int16_t*>(data + 80));            //0xFDFF (-0.003 deg/s)
  rotRateY = *(reinterpret_cast<int16_t*>(data + 82));            //0x7100 (0.113 deg/s)
  rotRateZ = *(reinterpret_cast<int16_t*>(data + 84));            //0xCE03 (0.974 deg/s)

    // gFX  = gForceX / 1000.0, 3;
    // gFY  = gForceY / 1000.0, 3;
    // gFZ  = gForceZ / 1000.0, 3;
    // gyroX = rotRateX / 100.0, 2;
    // gyroY = rotRateY / 100.0, 2;
    // gyroZ = rotRateZ / 100.0, 2;

    headingDegrees = heading / 100000.0;
    compass_direction = getCompassDirection(headingDegrees);
    updated_RaceBox_Data_Message = true;
}

// In dieser Version wird immer ein neuer Client erstellt.
static bool connectToRaceBox() {
  NimBLEClient* pClient = NimBLEDevice::createClient();
  pClient->setClientCallbacks(new ClientCallbacks());
  
  if (!pClient->connect(myRaceBox)) {
    Serial.println("Failed to connect.");
    NimBLEDevice::deleteClient(pClient);
    return false;
  }
  
  BLERemoteService* pService = pClient->getService(UART_service_UUID);
  if (pService != nullptr) {
    pRemoteCharacteristic = pService->getCharacteristic(TX_characteristic_UUID);
    if (pRemoteCharacteristic != nullptr) {
      if (!pRemoteCharacteristic->canNotify()) {
        Serial.println("Characteristic does not support notifications.");
        return false;
      }
      // subscribe statt registerForNotify verwenden:
      pRemoteCharacteristic->subscribe(true, notifyCallback);
      Serial.println("DEBUG: connectToRaceBox() returns true.");
      return true;
    }
  }
  return false;
}

static void print_RaceBox_Data_to_serial(){
    unsigned long currentTime = millis();
    if (currentTime - lastOutputTimeSerial >= outputIntervalMs_serial) {
        lastOutputTimeSerial = currentTime;
        Serial.println();
        Serial.println("--- Aktualisierte RaceBox-Daten ---");
        Serial.println("iTOW: " + String(iTOW) + " ms");
        Serial.println("Year: " + String(year));
        Serial.println("Month: " + String(month));
        Serial.println("Day: " + String(day));
        
        char timeString[9];
        sprintf(timeString, "%02d:%02d:%02d", hour, minute, second);
        Serial.println("Time (UTC): " + String(timeString));
        
        String fixStatusText;
        if (fixStatus == 0) {
            fixStatusText = "No Fix";
        } else if (fixStatus == 2) {
            fixStatusText = "2D Fix";
        } else if (fixStatus == 3) {
            fixStatusText = "3D Fix";
        } else {
            fixStatusText = "Unknown";
        }
        Serial.println("GPS: " + fixStatusText);
        Serial.println("Satellites: " + String(numSVs));
        Serial.println("Latitude: " + String(latitude / 1e7, 7) + " deg");
        Serial.println("Longitude: " + String(longitude / 1e7, 7) + " deg");
        Serial.println("WGS Altitude: " + String(wgsAltitude / 1000.0, 2) + " m");
        Serial.println("MSL Altitude: " + String(mslAltitude / 1000.0, 2) + " m");
        Serial.println("Horizontal Accuracy: " + String(horizontalAccuracy / 1000.0, 2) + " m");
        Serial.println("Vertical Accuracy: " + String(verticalAccuracy / 1000.0, 2) + " m");
        Serial.println("Speed: " + String(gpsspeed / 1000.0, 2) + " m/s");
        Serial.print("Heading: ");
        Serial.print(headingDegrees, 1);
        Serial.print(" deg, compass: ");
        Serial.println(compass_direction);
        Serial.println("PDOP: " + String(pdop / 100.0, 2));
        Serial.println("G-Force X: " + String(gForceX / 1000.0, 3) + " G");
        Serial.println("G-Force Y: " + String(gForceY / 1000.0, 3) + " G");
        Serial.println("G-Force Z: " + String(gForceZ / 1000.0, 3) + " G");
        decodeBatteryStatus(batteryStatus);
        Serial.println();
    } else {
      Serial.println("Serielle Ausgabe übersprungen (Update-Limit)");
    }
}

/* -----------------------------
   Neue Funktionen zur NMEA-Erzeugung
   ----------------------------- */

// Hilfsfunktion zur Berechnung der NMEA-Prüfsumme
static uint8_t nmeaChecksum(const char *sentence) {
  uint8_t checksum = 0;
  if (sentence[0] == '$') sentence++; // Überspringe '$'
  while (*sentence && *sentence != '*') {
    checksum ^= *sentence;
    sentence++;
  }
  return checksum;
}

// Generiert einen NMEA GPGGA-Datensatz
String generateGPGGA() {
  char temp[20];
  String nmea = "GPGGA,";
  
  // Zeit (hhmmss.sss)
  if (validityFlags & 0x01) { // Bit 0 = Zeit gültig
    uint32_t fracSec = (nanoseconds / 1000000) % 1000;
    snprintf(temp, sizeof(temp), "%02d%02d%02d.%03d", hour, minute, second, (int)fracSec);
  } else {
    strcpy(temp, "000000.000");
  }
  nmea += temp;
  nmea += ",";
  
  // Latitude (ddmm.mmmm,N/S)
  if (latitude != 0) {
    char dir = (latitude >= 0) ? 'N' : 'S';
    int32_t absLat = abs(latitude);
    int deg = absLat / 10000000L;         // Grad
    float min = (absLat % 10000000L) * 60.0 / 10000000.0; // Minuten
    snprintf(temp, sizeof(temp), "%02d%07.4f,%c,", deg, min, dir);
    nmea += temp;
  } else {
    nmea += "0000.0000,N,";
  }
  
  // Longitude (dddmm.mmmm,E/W)
  if (longitude != 0) {
    char dir = (longitude >= 0) ? 'E' : 'W';
    int32_t absLon = abs(longitude);
    int deg = absLon / 10000000L;         // Grad
    float min = (absLon % 10000000L) * 60.0 / 10000000.0; // Minuten
    snprintf(temp, sizeof(temp), "%03d%07.4f,%c,", deg, min, dir);
    nmea += temp;
  } else {
    nmea += "00000.0000,E,";
  }
  
  // Fix Quality (0-1)
  uint8_t fixQual = (fixStatus >= 1) ? 1 : 0;
  nmea += String(fixQual) + ",";
  
  // Anzahl Satelliten
  nmea += String(numSVs) + ",";
  
  // HDOP (aus 0.01 Einheiten umgerechnet)
  float hdop = pdop / 100.0;
  dtostrf(hdop, 0, 2, temp);
  nmea += temp;
  nmea += ",";
  
  // MSL Altitude (mm zu Metern)
  float altitude = mslAltitude / 1000.0;
  dtostrf(altitude, 0, 3, temp);
  nmea += temp;
  nmea += ",M,,,";
  
  // Checksumme berechnen
  uint8_t checksum = nmeaChecksum(nmea.c_str());
  
  // Kompletten NMEA-Satz zusammenfügen
  String fullNMEA = "$" + nmea + "*";
  if (checksum < 0x10) fullNMEA += "0";
  fullNMEA += String(checksum, HEX);
  fullNMEA.toUpperCase();
  
  return fullNMEA + "\r\n";
}

String generateDateTime() {
char temp[20];
String datetime;
snprintf(temp, sizeof(temp), "%02d-%02d-%04d_%02d-%02d", day, month, year % 100, hour, minute);
datetime = temp;
return datetime;
}

// Generiert einen NMEA GPRMC-Datensatz
String generateGPRMC() {
  char temp[20];
  String nmea = "GPRMC,";
  
  // Zeitstempel (hhmmss.sss)
  if (validityFlags & 0x01) {
    uint32_t fracSec = (nanoseconds / 1000000) % 1000;
    snprintf(temp, sizeof(temp), "%02d%02d%02d.%03d", hour, minute, second, (int)fracSec);
  } else {
    strcpy(temp, "000000.000");
  }
  nmea += temp;
  nmea += ",";
  
  // Status (A = aktiv, V = ungültig)
  char status = (fixStatus >= 1) ? 'A' : 'V';
  nmea += status;
  nmea += ",";
  
  // Latitude
  if (latitude != 0) {
    char dir = (latitude >= 0) ? 'N' : 'S';
    int32_t absLat = abs(latitude);
    int deg = absLat / 10000000L;
    float min = (absLat % 10000000L) * 60.0 / 10000000.0;
    snprintf(temp, sizeof(temp), "%02d%07.4f,%c,", deg, min, dir);
    nmea += temp;
  } else {
    nmea += "0000.0000,N,";
  }
  
  // Longitude
  if (longitude != 0) {
    char dir = (longitude >= 0) ? 'E' : 'W';
    int32_t absLon = abs(longitude);
    int deg = absLon / 10000000L;
    float min = (absLon % 10000000L) * 60.0 / 10000000.0;
    snprintf(temp, sizeof(temp), "%03d%07.4f,%c,", deg, min, dir);
    nmea += temp;
  } else {
    nmea += "00000.0000,E,";
  }
  
  // Geschwindigkeit in Knoten (Umrechnung m/s -> Knoten)
  float speedKnots = (gpsspeed / 1000.0) * 1.94384;
  dtostrf(speedKnots, 0, 2, temp);
  nmea += temp;
  nmea += ",";
  
  // Kurs über Grund (True Course)
  float trueCourse = heading / 100000.0;
  dtostrf(trueCourse, 0, 2, temp);
  nmea += temp;
  nmea += ",";
  
  // Datum (DDMMYY)
  snprintf(temp, sizeof(temp), "%02d%02d%02d", day, month, year % 100);
  nmea += temp;
  nmea += ",";
  
  // Magnetische Missweisung (nicht verfügbar -> leer)
  nmea += ",";
  
  // Modus (A = autonom, D = differenziell, E = geschätzt)
  char mode = (fixStatus == 2) ? 'D' : (fixStatus == 1 ? 'A' : 'N');
  nmea += mode;
  
  // Checksumme berechnen
  uint8_t checksum = nmeaChecksum(nmea.c_str());
  
  // Kompletten NMEA-Satz zusammenfügen
  String fullNMEA = "$" + nmea + "*";
  if (checksum < 0x10) fullNMEA += "0";
  fullNMEA += String(checksum, HEX);
  fullNMEA.toUpperCase();
  
  return fullNMEA + "\r\n";
}

/* -----------------------------
   Neue Funktionen zur Berechnung
   ----------------------------- */

// Hilfsfunktion zum Runden eines Wertes auf eine bestimmte Anzahl Dezimalstellen
float roundToDecimals(float value, int decimals) {
  float multiplier = pow(10, decimals);
  return round(value * multiplier) / multiplier;
}

// Berechnet die GPS-Geschwindigkeit (in km/h, 2 Dezimalstellen)
float getGpsSpeed() {
  return roundToDecimals((gpsspeed * 3.6 / 1000.0), 2);
}

// Berechnet die Beschleunigungswerte in g (3 Dezimalstellen)
int16_t getGeFX() {
  // return roundToDecimals((gForceX / 1000.0), 3);
  return gForceX;
}

int16_t getGeFY() {
  // return roundToDecimals((gForceY / 1000.0), 3);
  return gForceY;
}

int16_t getGeFZ() {
  // return roundToDecimals((gForceZ / 1000.0), 3);
  return gForceZ;
}

// Berechnet die Gyroskopwerte in deg/s (2 Dezimalstellen)
int16_t getGyroX() {
  // return roundToDecimals((rotRateX / 100.0), 2);
  return rotRateX;
}

int16_t getGyroY() {
  // return roundToDecimals((rotRateY / 100.0), 2);
  return rotRateY;
}

int16_t getGyroZ() {
  // return roundToDecimals((rotRateZ / 100.0), 2);
  return rotRateZ;
}

// Berechnet Roll- und Nickwinkel anhand der Beschleunigungswerte
float getRoll() {
  float ax = gForceX / 1000.0;
  float ay = gForceY / 1000.0;
  float az = gForceZ / 1000.0;
  return (atan2(ay, az) * 180.0 / PI);
}

float getPitch() {
  float ax = gForceX / 1000.0;
  float ay = gForceY / 1000.0;
  float az = gForceZ / 1000.0;
  return (atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI);
}

// Berechnet die laterale Kraft (in Newton)
// Hier wird der Lean-Winkel aus der normierten gForceZ berechnet, und
// anschließend die wahre laterale Beschleunigung (in m/s²) und Kraft.
float getLateralForce() {
  float normalizedGZ = gForceZ / 1000.0;
  float normalizedGY = gForceY / 1000.0;
  float leanAngle = acos(normalizedGZ);
  float gravityComponentY = sin(leanAngle);
  float lateralAccel_g = normalizedGY - gravityComponentY;
  float lateralAccel_ms2 = lateralAccel_g * 9.81;
  return MASS_KG * lateralAccel_ms2;
}

/* -----------------------------
   Öffentliche Schnittstellenfunktionen
   ----------------------------- */

void RaceBox_setup() {
  Serial.begin(115200);
  if (serverRunning) {
    Serial.println("Webserver aktiv – BLE wird nicht gestartet.");
    return;
  }
  NimBLEDevice::init("ESP32_RaceBox_Client");
  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  pScan->setInterval(45);
  pScan->setWindow(15);
  pScan->setActiveScan(true);
  pScan->start(0, false);
}

void RaceBox_loop() {
  if (serverRunning) {
    if (bleRunning) {
      stopBlePublic();  // BLE explizit stoppen
    }
    wsled.on(WSLED::GREEN);
    return;
  }

  if (!bleRunning) {
    startBlePublic();  // BLE wieder aktivieren, falls nicht aktiv
  }

  if (doConnect) {
    if (connectToRaceBox()) {
      Serial.println("Erfolgreich mit RaceBox verbunden.");
      NimBLEDevice::getScan()->stop();
    } else {
      Serial.println("Verbindung fehlgeschlagen. Erneuter Versuch...");
      NimBLEScan* pScan = NimBLEDevice::getScan();
      pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
      pScan->setInterval(45);
      pScan->setWindow(15);
      pScan->setActiveScan(true);
      pScan->start(0, false);
    }
    doConnect = false;
  }

  if (connected) {
    rbconnected = true;
    wsled.on(WSLED::BLUE);
    if (updated_RaceBox_Data_Message) {
      updated_RaceBox_Data_Message = false;
    }
  } else {
    rbconnected = false;
  }
}
