/*
  Simple Edge Test Sketch
  Venice1200 02/2019
*/

// Include Edge Header File
#include "Edge.h"

#define Button2 11
#define Button3 10
#define LEDL 13
#define LEDR 6

// Define Edge Objects
EDGE b1edge;
EDGE b2edge;

void setup() {
  Serial.begin(9600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  Serial.println("Starting Sketch !");

  pinMode(Button2, INPUT_PULLUP);
  pinMode(Button3, INPUT_PULLUP);

  // Init Edge Objects
  b1edge.init();
  b2edge.init();
}

void loop() {
  // You have to update all Edge Objects each cycle 
  // with your boolean Input Signal
  b1edge.update(digitalRead(Button2));
  b2edge.update(digitalRead(Button3));
  
  // Detect Signal changes for Input Button 2
  if (b1edge.rising())  Serial.println("B2 Rising Signal 0->1 detected !");
  if (b1edge.falling()) Serial.println("B2 Falling Signal 1->0 detected !");
  if (b1edge.changed()) Serial.println("B2 Signal change detected !");

  // Detect Signal changes for Input Button 3
  if (b2edge.rising())  Serial.println("B3 Rising Signal 0->1 detected !");
  if (b2edge.falling()) Serial.println("B3 Falling Signal 1->0 detected !");
  if (b2edge.changed()) Serial.println("B3 Signal change detected !");

  digitalWrite(LEDL, digitalRead(Button2)); 
  digitalWrite(LEDR, digitalRead(Button3)); 
}
