#ifndef LOGSERVER_H
#define LOGSERVER_H

/**
 * Setup für das Log-Server-Modul: Initialisiert Schalter-Pin.
 * Muss nach logger_setup() aufgerufen werden.
 */
void logServerSetup();

/**
 * Loop-Handler für Log-Server: Überwacht switchPin und toggelt Webserver/BLE.
 * Muss in loop() aufgerufen werden.
 */
void logServerLoop();

void stopBlePublic();
void startBlePublic();

#endif // LOGSERVER_H