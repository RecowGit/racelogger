/**
 * @file WSLED.cpp
 * @brief Implementierung der WSLED Klasse
 * @version 1.1
 * @date 2023-11-15
 * @license MIT
 */

 #include "WSLED.h"

 /**
  * @brief Konstruktor für die WSLED Klasse
  * @param pin Der GPIO-Pin, an dem die LED angeschlossen ist
  * @param numPixels Anzahl der LEDs in der Kette (Standard: 1)
  */
 WSLED::WSLED(uint8_t pin, uint16_t numPixels) 
   : _strip(numPixels, pin, NEO_GRB + NEO_KHZ800) {
   _blinkActive = false;
   _ledState = false;
   _previousMillis = 0;
 }
 
 /**
  * @brief Initialisiert den LED-Streifen
  * 
  * Muss im setup() aufgerufen werden bevor andere Funktionen verwendet werden.
  */
 void WSLED::begin() {
   _strip.begin();
   _strip.show(); // Initialisierung mit allen LEDs aus
 }
 
 /**
  * @brief Schaltet alle LEDs aus
  */
 void WSLED::off() {
   stopBlink();
   _strip.fill(0);
   _strip.show();
 }
 
 /**
  * @brief Schaltet die LED mit einer bestimmten Farbe ein
  * @param color Die Farbe als 32-bit Wert (0x00RRGGBB)
  */
 void WSLED::on(uint32_t color) {
   stopBlink();
   _strip.fill(color);
   _strip.show();
 }
 
 /**
  * @brief Schaltet die LED mit individuellen RGB-Werten ein
  * @param red Rotanteil (0-255)
  * @param green Grünanteil (0-255)
  * @param blue Blauanteil (0-255)
  */
 void WSLED::on(uint8_t red, uint8_t green, uint8_t blue) {
   stopBlink();
   _strip.fill(_strip.Color(red, green, blue));
   _strip.show();
 }
 
 /**
  * @brief Lässt die LED blinken
  * @param color Die Blinkfarbe als 32-bit Wert
  * @param interval Blinkintervall in Millisekunden
  */
 void WSLED::blink(uint32_t color, uint16_t interval) {
   _blinkColor = color;
   _blinkInterval = interval;
   _blinkActive = true;
   _previousMillis = millis();
   _ledState = true;
   _strip.fill(_blinkColor);
   _strip.show();
 }
 
 /**
  * @brief Lässt die LED blinken
  * @param red Rotanteil (0-255)
  * @param green Grünanteil (0-255)
  * @param blue Blauanteil (0-255)
  * @param interval Blinkintervall in Millisekunden
  */
 void WSLED::blink(uint8_t red, uint8_t green, uint8_t blue, uint16_t interval) {
   blink(_strip.Color(red, green, blue), interval);
 }
 
 void WSLED::tripleBlink(uint32_t color, uint16_t speed, uint16_t pause) {
    stopBlink();
    
    // Dreimaliges Blinken
    for(int i = 0; i < 3; i++) {
      _strip.fill(color);
      _strip.show();
      delay(speed);
      _strip.fill(0);
      _strip.show();
      if(i < 2) delay(speed); // Keine Pause nach dem letzten Blinken
    }
    
    delay(pause); // Pause nach der Sequenz
  }
  
  void WSLED::tripleBlink(uint8_t red, uint8_t green, uint8_t blue, uint16_t speed, uint16_t pause) {
    tripleBlink(_strip.Color(red, green, blue), speed, pause);
  }
  
 /**
  * @brief Stoppt das Blinken und schaltet die LED aus
  */
 void WSLED::stopBlink() {
   _blinkActive = false;
 }
 
 /**
  * @brief Setzt die Helligkeit aller LEDs
  * @param brightness Helligkeitswert (0-255)
  */
 void WSLED::setBrightness(uint8_t brightness) {
   _strip.setBrightness(brightness);
   _strip.show();
 }
 
 /**
  * @brief Aktualisiert den Blinkzustand (nicht-blockierend)
  * 
  * Diese Funktion sollte regelmäßig im loop() aufgerufen werden,
  * um nicht-blockierendes Blinken zu ermöglichen.
  */
 void WSLED::update() {
   if (!_blinkActive) return;
   
   unsigned long currentMillis = millis();
   
   if (currentMillis - _previousMillis >= _blinkInterval) {
     _previousMillis = currentMillis;
     _ledState = !_ledState;
     
     if (_ledState) {
       _strip.fill(_blinkColor);
     } else {
       _strip.fill(0);
     }
     _strip.show();
   }
 }
 
 /**
  * @brief Interne Funktion zur Blinksteuerung
  * 
  * Wird von update() aufgerufen um den Blinkzustand zu aktualisieren.
  * Kann für zukünftige Erweiterungen verwendet werden.
  */
 void WSLED::handleBlink() {
   update();
 }