#include "logger.h"
#include "racebox.h"
#include "globals.h"

// -------------------------
// Globale Objekte und Variablen
// -------------------------
// SPIClass hspi(HSPI); // Definition des HSPI-Objekts

SdFs sd;            // SD-Karten-Objekt (SdFat)
FsFile logFile;     // Logdatei

// Double Circular Buffer Variablen
char circularBuffers[NUM_BUFFERS][NUM_LINES][LINE_LENGTH];
int bufferLineCount[NUM_BUFFERS] = {0, 0};
int activeBuffer = 0;

// -------------------------
// Interne Funktionsprototypen
// -------------------------
String generateFileName();
String createUniqueFileName();          // Generiert einen eindeutigen Dateinamen (mit Suffix bei Bedarf)
void writeHeaderToFile();
void generateTestData(char* buffer, unsigned long counter);
void pushLineToBuffer(const char* line);
void flushAllBuffers();

// -------------------------
// Globale Variablen für Distanzberechnung
// -------------------------
double cumulativeDistance = 0.0;      // Zur akkumulierten Distanz (in Metern)
unsigned long prevLogTime = 0;          // Letzter Log-Zeitpunkt (millis)

// -------------------------
// Lokale Funktionen
// -------------------------

// Generiert einen Dateinamen im Format "RC3_<datetime>.csv"
// (Hier wird ein statischer Platzhalter genutzt – ersetze diesen ggf. durch einen echten Zeitstempel)
String generateFileName() {
  return "RC3_"+ generateDateTime() +".csv";
}

String createUniqueFileName() {
    String baseName = "RC3_" + generateDateTime();  // Basisname ohne Endung
    String fileName = baseName + ".csv";
    int counter = 1;
    // Falls die Datei bereits existiert, wird der Zähler angehängt und hochgezählt,
    // bis ein noch nicht vorhandener Dateiname gefunden wurde.
    while (sd.exists(fileName.c_str())) {
      fileName = baseName + "_" + String(counter) + ".csv";
      counter++;
    }
    return fileName;
  }

// Schreibt den CSV-Header in die geöffnete Logdatei
void writeHeaderToFile() {
  String header = "Time (s),Session fragment #,Lap #,Trap name,"
                  "X-position (m),Y-position (m),Distance (m),Speed (m/s),"
                  "Altitude (m),Bearing (deg),Device update rate (Hz),Elapsed time (s),Latitude (deg),"
                  "Longitude (deg),Satellites (sats),Fix type,Accuracy (m),"
                  "Speed (m/s) *calc,Lateral acceleration (G) *calc,"
                  "Longitudinal acceleration (G) *calc,Lean angle (deg) *calc,"
                  "Combined acceleration (G) *calc,X acceleration (G) *acc,"
                  "Y acceleration (G) *acc,Z acceleration (G) *acc,"
                  "X rate of rotation (deg/s) *gyro,Y rate of rotation (deg/s) *gyro,"
                  "Z rate of rotation (deg/s) *gyro,Digital 1/RPM (rpm) *data,"
                  "Analog 1 *data,Analog 2 *data,Analog 3 *data,Analog 4 *data,"
                  "Analog 5 *data,Digital 2 *data,Analog 6 *data,Analog 7 *data,"
                  "Analog 8 *data,Analog 9 *data,Analog 10 *data,Analog 11 *data,"
                  "Analog 12 *data,Analog 13 *data,Analog 14 *data,Analog 15 *data,"
                  "Tyre temperature Rear (.C) *wheel,Suspension travel Rear (m) *wheel,"
                  "Tyre temperature Rear 1 (.C) *wheel,Tyre temperature Rear 2 (.C) *wheel,"
                  "Tyre temperature Rear 3 (.C) *wheel,Tyre temperature Rear 4 (.C) *wheel,"
                  "Tyre temperature Rear 5 (.C) *wheel,Tyre temperature Rear 6 (.C) *wheel,"
                  "Tyre temperature Rear 7 (.C) *wheel,Tyre temperature Rear 8 (.C) *wheel,"
                  "Tyre temperature Front (.C) *wheel,Suspension travel Front (m) *wheel,"
                  "Tyre temperature Front 1 (.C) *wheel,Tyre temperature Front 2 (.C) *wheel,"
                  "Tyre temperature Front 3 (.C) *wheel,Tyre temperature Front 4 (.C) *wheel,"
                  "Tyre temperature Front 5 (.C) *wheel,Tyre temperature Front 6 (.C) *wheel,"
                  "Tyre temperature Front 7 (.C) *wheel,Tyre temperature Front 8 (.C) *wheel";
  logFile.println(header);
  logFile.flush();
}

unsigned long logStartTime = 0;

int rpm = 0;
int speedsensor = 0.0;
int throttle = 0.0;
int gang = 0;
float lateral;
float roll;

String createCsvLogString() {
    String csv;
  
    // Hilfsfunktion zur Unix-Zeitberechnung (vereinfacht)
    auto computeUnixTime = [](uint16_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t min, uint8_t s) {
      return (unsigned long)(y - 1970) * 31536000UL + 
             (m - 1) * 2592000UL + 
             (d - 1) * 86400UL + 
             h * 3600UL + 
             min * 60UL + 
             s;
    };
  
    // Zeitberechnung
    unsigned long unixTime = computeUnixTime(year, month, day, hour, minute, second);
    double timestamp = (double)unixTime + nanoseconds / 1e9;
    unsigned long elapsedTime = millis() - logStartTime / 1000.0;
    
    
    // Koordinatenkonvertierung
    double latitudeDeg = latitude / 1e7;
    double longitudeDeg = longitude / 1e7;
  
    // Berechnung der verstrichenen Zeit und Integration der Distanz
    unsigned long currentTime = millis();
    double dt = 0.0;
    if (prevLogTime != 0) {
        dt = (currentTime - prevLogTime) / 1000.0;  // Zeitdifferenz in Sekunden
    }
    prevLogTime = currentTime;
        
    // Umrechnung der Geschwindigkeit in m/s (angenommen, gpsspeed liegt in Millimetern pro Sekunde vor)
    double speed_ms = gpsspeed / 1000.0;
    // Aktualisierung der zurückgelegten Distanz: deltaDistance = Geschwindigkeit * dt
    cumulativeDistance += speed_ms * dt;

    // Berechnung der Update Rate (geschriebene Logzeilen pro Sekunde)
    static unsigned long rateStartTime = 0;
    static int lineCountForRate = 0;
    static int measuredUpdateRate = 0;
    if (rateStartTime == 0) {
        rateStartTime = currentTime;
    }
    lineCountForRate++;
    if (currentTime - rateStartTime >= 1000) {
        measuredUpdateRate = lineCountForRate;
        lineCountForRate = 0;
        rateStartTime = currentTime;
    }

    // Beschleunigungswerte
    double accelX = gForceX / 1000.0;
    double accelY = gForceY / 1000.0;
    double accelZ = gForceZ / 1000.0;
    double combinedAccel = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
    double gyroX = rotRateX / 100.0;
    double gyroY = rotRateY / 100.0;
    double gyroZ = rotRateZ / 100.0;

    lateral = getLateralForce();
    roll = getRoll();


        // CSV-Zusammenstellung
        csv += String(timestamp, 3) + ",";              // Time (s)
        csv += "0,,,";                                  // Session, Lap, Trap
        csv += "0.000,0.000,";                          // X,Y,
        csv += String(cumulativeDistance) + ",";        // Distance
        csv += String(gpsspeed / 1000.0, 3) + ",";      // Speed
        csv += String(wgsAltitude / 1000.0, 3) + ",";   // Altitude
        csv += String(heading / 100000.0, 3) + ",";     // Bearing
        csv += String(measuredUpdateRate) + ",";        // Update Rate
        csv += String(elapsedTime, 3) + ",";            // Elapsed Time
        csv += String(latitudeDeg, 7) + ",";            // Latitude
        csv += String(longitudeDeg, 7) + ",";           // Longitude
        // csv += String(batteryStatus) + ",";          // Battery
        csv += String(numSVs) + ",";                    // Satellites
        csv += String(fixStatus) + ",";                 // Fix Type
        csv += String(horizontalAccuracy / 1000.0, 3) + ","; // Accuracy
    
        // *calc, *acc, *gyro und *data Felder
        csv += String(gpsspeed / 1000.0, 3) + ",";      // Speed (mm/s) *calc (leer)
        csv += String(lateral, 3) + ",";                // Lateral acceleration
        csv += ",";                                     // Longitudinal acceleration (leer)
        csv += String(roll, 3) + ",";                   // Lean angle
        csv += ",";                                     // Combined acceleration (leer)
        csv += String(accelX, 3) + ",";                 // X acceleration
        csv += String(accelY, 3) + ",";                 // Y acceleration
        csv += String(accelZ, 3) + ",";                 // Z acceleration
        csv += String(gyroX, 3) + ",";                  // X rotation rate
        csv += String(gyroY, 3) + ",";                  // Y rotation rate
        csv += String(gyroZ, 3) + ",";                  // Z rotation rate
        csv += String(rxrpm) + ",";                     // Digital 1/RPM
        csv += String(rxspeed, 0) + ",";                // Analog 1
        csv += String(rxthrottle, 1) + ",";             // Analog 2
        csv += ",,,,";                                  // Analog 3-5 (leer)
        csv += String(rxgear) + ",";                    // Digital 2
        csv += ",,,,,,,,,,";                            // Analog 6-15 (leer)
    
        // Reifen Temperaturen
        csv += String(frontString) + ",";
        csv += String(rearString);
      
        return csv;
  }

  void generateLogData(char* buffer, unsigned long counter) {
    // CSV-String generieren
    String logStr = createCsvLogString();
  
    // Zähler hinzufügen (z. B. für fortlaufende Protokollierung)
    logStr += counter;
  
    // String in char* umwandeln und in den Buffer kopieren
    strncpy(buffer, logStr.c_str(), LINE_LENGTH); // bufferSize sollte definiert sein!
  }

// Fügt eine Zeile in den aktiven Circular Buffer ein.
// Ist der Buffer voll, werden dessen Inhalte auf die SD-Karte geschrieben und der Buffer getauscht.
void pushLineToBuffer(const char* line) {
  int count = bufferLineCount[activeBuffer];
  if (count < NUM_LINES) {
    strncpy(circularBuffers[activeBuffer][count], line, LINE_LENGTH);
    circularBuffers[activeBuffer][count][LINE_LENGTH - 1] = '\0';
    bufferLineCount[activeBuffer]++;
  }
  if (bufferLineCount[activeBuffer] >= NUM_LINES) {
    int fullBuffer = activeBuffer;
    activeBuffer = (activeBuffer + 1) % NUM_BUFFERS;
    // Falls im neuen aktiven Buffer bereits Daten vorhanden sind, zuerst diese schreiben
    if (bufferLineCount[activeBuffer] > 0) {
      for (int i = 0; i < bufferLineCount[activeBuffer]; i++) {
        logFile.println(circularBuffers[activeBuffer][i]);
      }
      logFile.flush();
      bufferLineCount[activeBuffer] = 0;
    }
    // Schreibe den vollen Buffer in die SD-Datei
    for (int i = 0; i < NUM_LINES; i++) {
      logFile.println(circularBuffers[fullBuffer][i]);
    }
    logFile.flush();
    bufferLineCount[fullBuffer] = 0;
  }
}

// Schreibt alle im Buffer befindlichen Zeilen in die SD-Datei und leert die Buffer
void flushAllBuffers() {
  for (int b = 0; b < NUM_BUFFERS; b++) {
    for (int i = 0; i < bufferLineCount[b]; i++) {
      logFile.println(circularBuffers[b][i]);
    }
    logFile.flush();
    bufferLineCount[b] = 0;
  }
}

// -------------------------
// Öffentliche Logger-Funktionen
// -------------------------

// Initialisiert serielle Schnittstelle, Pins, HSPI und SD-Karte,
// öffnet die Logdatei und schreibt den Header.
// Anschließend wird der Logging-Task auf Core 1 gestartet.
void logger_setup() {
  // Serial.begin(115200);

  pinMode(switchPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // hspi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI);

  // if (!sd.begin(SD_CONFIG)) {
  //   sd.initErrorHalt(&Serial);
  // }

  String fileName = createUniqueFileName();
  Serial.print("Starte Logvorgang: ");
  Serial.println(fileName);
  logFile = sd.open(fileName.c_str(), O_RDWR | O_CREAT);
  // logFile = sd.open(fileName.c_str(), FILE_WRITE);
  if (!logFile) {
    Serial.println("Fehler beim Öffnen der Logdatei!");
    while (1);
  }
  // Falls die Datei neu ist (Größe 0), Header schreiben
  if (logFile.size() == 0) {
    writeHeaderToFile();
  }

  // Starte den Logging-Task auf Core 1 (zweiter Core)
  xTaskCreatePinnedToCore(
    logger_task,       // Task-Funktion
    "LoggerTask",      // Task-Name
    8192,              // Stackgröße in Bytes – ggf. anpassen
    NULL,              // Parameter
    2,                 // Priorität
    NULL,              // Task-Handle (optional)
    1                  // Core 1
  );
}

// Logging-Task: Wird auf Core 1 ausgeführt.
// Er prüft periodisch (25 Hz, also alle 40 ms) den Status des Tasters und
// führt den Logging-Vorgang (Testdatengenerierung, Buffer füllen, Flushen) aus.
void logger_task(void *param) {
  bool logging = false;
  unsigned long logCounter = 0;
  unsigned long lastLogTime = 0;

  // Debounce Variablen
  int lastButtonState = HIGH;
  int buttonState;
  unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;  // 50 ms Debounce-Zeit

  for (;;) {
    int reading = digitalRead(switchPin);
    unsigned long currentTime = millis();

    // Debounce-Logik
    if (reading != lastButtonState) {
      lastDebounceTime = currentTime;
    }

    if ((currentTime - lastDebounceTime) >= debounceDelay) {
      if (reading != buttonState) {
        buttonState = reading;
      }
    }
    lastButtonState = reading;

    if (buttonState == LOW) { // Taster entprellt gedrückt
      if (!logging) {
        logging = true;
        logCounter = 0;
        logStartTime = currentTime;
        digitalWrite(ledPin, HIGH);
        Serial.println("Logging gestartet.");
        lastLogTime = currentTime;
      }

      char dataLine[LINE_LENGTH];
      generateLogData(dataLine, logCounter);
      pushLineToBuffer(dataLine);
      logCounter++;
      // 25 Hz: 40 ms Wartezeit
      vTaskDelay(40 / portTICK_PERIOD_MS);
    }
    else { // Taster nicht gedrückt
      if (logging) {
        flushAllBuffers();
        logFile.close();
        digitalWrite(ledPin, LOW);
        Serial.println("Logvorgang beendet.");
        logging = false;
      }
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}


