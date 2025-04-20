/**
 * @file WSLED.h
 * @brief Bibliothek zur Steuerung von WS2812B/NeoPixel LEDs mit ESP32
 * @version 1.1
 * @date 2023-11-15
 * @license MIT
 */

 #ifndef WSLED_H
 #define WSLED_H
 
 #include <Arduino.h>
 #include <Adafruit_NeoPixel.h>
 
 /**
  * @class WSLED
  * @brief Klasse zur einfachen Steuerung von WS2812B/NeoPixel LEDs
  * 
  * Diese Klasse bietet eine vereinfachte Schnittstelle zur Steuerung von
  * WS2812B LEDs (auch bekannt als NeoPixel) mit einem ESP32 Mikrocontroller.
  * Sie baut auf der Adafruit NeoPixel Library auf und bietet zusätzliche
  * vereinfachte Funktionen.
  */
 class WSLED {
   public:
     /**
      * @brief Konstruktor für die WSLED Klasse
      * @param pin Der GPIO-Pin, an dem die LED angeschlossen ist
      * @param numPixels Anzahl der LEDs in der Kette (Standard: 1)
      */
     WSLED(uint8_t pin, uint16_t numPixels = 1);
     
     // Grundlegende Steuerfunktionen
     /**
      * @brief Initialisiert die LED-Streifen
      * 
      * Muss im setup() aufgerufen werden bevor andere Funktionen verwendet werden.
      */
     void begin();
     
     /**
      * @brief Schaltet alle LEDs aus
      */
     void off();
     
     /**
      * @brief Schaltet die LED mit einer bestimmten Farbe ein
      * @param color Die Farbe als 32-bit Wert (0x00RRGGBB)
      */
     void on(uint32_t color);
     
     /**
      * @brief Schaltet die LED mit individuellen RGB-Werten ein
      * @param red Rotanteil (0-255)
      * @param green Grünanteil (0-255)
      * @param blue Blauanteil (0-255)
      */
     void on(uint8_t red, uint8_t green, uint8_t blue);
     
     // Vordefinierte Farben (als 32-bit RGB Werte)
     static const uint32_t RED = 0xFF0000;    ///< Rot (255, 0, 0)
     static const uint32_t GREEN = 0x00FF00;  ///< Grün (0, 255, 0)
     static const uint32_t BLUE = 0x0000FF;   ///< Blau (0, 0, 255)
     static const uint32_t WHITE = 0xFFFFFF;  ///< Weiß (255, 255, 255)
     static const uint32_t YELLOW = 0xFFFF00; ///< Gelb (255, 255, 0)
     static const uint32_t PURPLE = 0xFF00FF; ///< Lila (255, 0, 255)
     static const uint32_t CYAN = 0x00FFFF;   ///< Cyan (0, 255, 255)
     static const uint32_t ORANGE = 0xFFA500; ///< Orange (255, 165, 0)
     
     // Blink-Funktionen
     /**
      * @brief Lässt die LED blinken
      * @param color Die Blinkfarbe als 32-bit Wert
      * @param interval Blinkintervall in Millisekunden
      */
     void blink(uint32_t color, uint16_t interval);
     
     /**
      * @brief Lässt die LED blinken
      * @param red Rotanteil (0-255)
      * @param green Grünanteil (0-255)
      * @param blue Blauanteil (0-255)
      * @param interval Blinkintervall in Millisekunden
      */
     void blink(uint8_t red, uint8_t green, uint8_t blue, uint16_t interval);
     
     void tripleBlink(uint32_t color, uint16_t speed = 100, uint16_t pause = 300);
    
     /**
      * @brief Lässt die LED 3x schnell hintereinander blinken
      * @param red Rotanteil (0-255)
      * @param green Grünanteil (0-255)
      * @param blue Blauanteil (0-255)
      * @param speed Geschwindigkeit des Einzelblitzes in Millisekunden
      * @param pause Pause zwischen den Blinksequenzen in Millisekunden
      */
     void tripleBlink(uint8_t red, uint8_t green, uint8_t blue, uint16_t speed = 100, uint16_t pause = 300);
     
     /**
      * @brief Stoppt das Blinken und schaltet die LED aus
      */
     void stopBlink();
     
     // Helligkeitssteuerung
     /**
      * @brief Setzt die Helligkeit aller LEDs
      * @param brightness Helligkeitswert (0-255)
      */
     void setBrightness(uint8_t brightness);
     
     /**
      * @brief Muss regelmäßig aufgerufen werden für nicht-blockierendes Blinken
      * 
      * Diese Funktion sollte im loop() aufgerufen werden, wenn nicht-blockierendes
      * Blinken verwendet wird. Sie aktualisiert den Blinkzustand.
      */
     void update();
     
   private:
     Adafruit_NeoPixel _strip;       ///< NeoPixel-Objekt für die LED-Ansteuerung
     uint16_t _blinkInterval;        ///< Aktuelles Blinkintervall in ms
     uint32_t _blinkColor;           ///< Aktuelle Blinkfarbe
     bool _blinkActive;              ///< Blinkstatus (aktiv/inaktiv)
     unsigned long _previousMillis;  ///< Zeitstempel des letzten Blinkwechsels
     bool _ledState;                 ///< Aktueller LED-Zustand (ein/aus)
     
     /**
      * @brief Interne Funktion zur Blinksteuerung
      * 
      * Wird von update() aufgerufen um den Blinkzustand zu aktualisieren.
      */
     void handleBlink();
 };
 
 #endif