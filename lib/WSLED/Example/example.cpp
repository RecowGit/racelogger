#include "WSLED.h"

// LED an Pin 15 anschließen
WSLED led(15);

void setup() {
  led.begin();
}

void loop() {
  // Einfache Farben
  led.on(WSLED::RED);
  delay(1000);
  led.on(WSLED::GREEN);
  delay(1000);
  led.on(WSLED::BLUE);
  delay(1000);
  
  // Benutzerdefinierte Farbe
  led.on(255, 100, 0); // Orange
  delay(1000);
  
  // Blinken lassen
  led.blink(WSLED::PURPLE, 500); // Lila, 500ms Intervall
  delay(5000);
  led.stopBlink();
  
  // Nicht-blockierendes Blinken im Hauptloop
  led.blink(WSLED::CYAN, 300);
  for(int i = 0; i < 50; i++) {
    led.update(); // Wichtig für nicht-blockierendes Blinken
    delay(10);
  }
  led.stopBlink();
  
  // Ausschalten
  led.off();
  delay(1000);
}