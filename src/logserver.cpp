#include "globals.h"
#include "logserver.h"
#include <ESPmDNS.h>
#include <WiFi.h>
#include "racebox.h"
#include <SPIFFS.h>
#include "WSLED.h"
#include <ESPAsyncWebServer.h>
#include "logger.h"

// AP-Konfiguration
static const char* apSSID = "racelog";
static const char* apPassword = "12345678";  // leer = offenes Netzwerk
static const char* mdnsName = "racelog";

// Debounce-Variablen
static int lastSwitchState = HIGH;
static unsigned long lastDebounceTime = 0;

// HTTP-Handler: JSON-Endpoint liefert Array von {name, size} für Logdateien auf der SD-Karte
void handleList(AsyncWebServerRequest *request) {
    Serial.println("handleList() aufgerufen");
    String json = "[";

    FsFile root = sd.open("/");
    if (!root) {
        Serial.println("Fehler: Konnte Wurzelverzeichnis auf SD nicht öffnen.");
        request->send(500, "application/json", "[]");
        return;
    }

    FsFile file;
    bool first = true;
    while (file = root.openNextFile()) {
        char name[64];
        file.getName(name, sizeof(name));
        uint32_t size = file.fileSize();

        Serial.printf("Datei gefunden: %s (%d Bytes)\n", name, size);

        if (!first) json += ",";
        first = false;
        json += "{\"name\":\"" + String(name) + "\",\"size\":" + String(size) + "}";
        file.close();
    }

    root.close();
    json += "]";
    Serial.println("JSON gesendet: " + json);
    request->send(200, "application/json", json);
}

// HTTP-Handler: Startseite (HTML aus SPIFFS/SD interne Bereitstellung)
static void handleRoot(AsyncWebServerRequest *request) {
    request->redirect("/index.html");
}

// HTTP-Handler: Download einer CSV-Datei
static void handleDownload(AsyncWebServerRequest *request) {
    if (!request->hasParam("file")) {
      request->send(400, "text/plain", "Missing file param");
      return;
    }

    String fname = request->getParam("file")->value();
    String path = "/" + fname;

    if (!sd.exists(path.c_str())) {
      request->send(404, "text/plain", "File not found");
      return;
    }

    FsFile file = sd.open(path.c_str(), O_RDONLY);
    if (!file) {
      request->send(500, "text/plain", "Failed to open file");
      return;
    }

    AsyncResponseStream *response = request->beginResponseStream("text/csv");
    uint8_t buf[512];
    while (file.available()) {
      size_t len = file.read(buf, sizeof(buf));
      response->write(buf, len);
    }
    file.close();

    response->addHeader("Content-Disposition", "attachment; filename=" + fname);
    request->send(response);
}

// HTTP-Handler: Löschen einer Datei
static void handleDelete(AsyncWebServerRequest *request) {
    if (!request->hasParam("file")) {
      request->send(400, "text/plain", "Missing file param");
      return;
    }
    String fname = request->getParam("file")->value();
    String path = "/" + fname;
    if (sd.remove(path.c_str())) {
      request->send(200, "application/json", "{\"status\":\"ok\"}");
    } else {
      request->send(500, "application/json", "{\"status\":\"error\"}");
    }
}

// BLE-Steuerung
static void startBle() {
    if (bleRunning) return;
    RaceBox_setup();
    bleRunning = true;
}

static void stopBle() {
    if (!bleRunning) return;
    NimBLEDevice::deinit(true);
    bleRunning = false;
}

// Webserver starten
static void startServer() {
    if (serverRunning) return;
    stopBle();

    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSSID, apPassword);

    if (WiFi.softAPIP() == IPAddress(0, 0, 0, 0)) {
      Serial.println("Fehler beim Starten des Access Points!");
      return;
    }

    if (!MDNS.begin(mdnsName)) {
      Serial.println("Fehler beim Starten von mDNS");
      return;
    }

    MDNS.addService("http", "tcp", 80);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/list", HTTP_GET, handleList);
    server.on("/download", HTTP_GET, handleDownload);
    server.on("/delete", HTTP_DELETE, handleDelete);

    server.serveStatic("/index.html", SPIFFS, "/index.html");
    server.serveStatic("/style.css", SPIFFS, "/style.css");
    server.serveStatic("/header.png", SPIFFS, "/header.png");
    server.serveStatic("/logo.png", SPIFFS, "/logo.png");

    server.begin();
    // Serial.println("AP gestartet: " + String(apSSID));
    // Serial.println("IP: " + WiFi.softAPIP().toString());
    serverRunning = true;
    wsled.on(WSLED::GREEN);
}

// Stoppt den Webserver, trennt AP und startet BLE
static void stopServer() {
    if (!serverRunning) return;
    server.end();
    MDNS.end();
    WiFi.softAPdisconnect(true);
    startBle();
    serverRunning = false;
}

// Öffentliche API
void logServerSetup() {
    if (!SPIFFS.begin(true)) {
      Serial.println("SPIFFS mount failed");
      return;
    }
    // if (!sd.begin(SD_CONFIG)) {
    //     Serial.println("SD Card initialization failed in WebInterface!");
    //     return;
    //   }
    }

void logServerLoop() {
    int reading = digitalRead(ServerSwitchPin);  // z.B. Pin 34

    if (reading == LOW) {
        if (!serverRunning) {
            Serial.println("Pin LOW erkannt – Webserver wird gestartet.");
            startServer();
        }
    } else {
        if (serverRunning) {
            Serial.println("Pin HIGH erkannt – Webserver wird gestoppt.");
            stopServer();
        }
    }
}

void stopBlePublic() { stopBle(); }
void startBlePublic() { startBle(); }
