
#include "node.h"
#include "rplidar.h"


airlab::Node *espNode;
rplidarHandler *rplidar; 

void setup() {
  Serial.begin(115200);
  rplidar = new rplidarHandler();
  espNode = new airlab::Node(rplidar);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);
  rplidar->init();

  
}

void loop() {
  espNode->loop();
  rplidar->loop();
}