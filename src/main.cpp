#include <ArduinoOTA.h>
#include "rplidar.h"
#include "ros_node.h"


airlab::Node espNode;
rplidarHandler espLidar; 

void setup() {
  Serial.begin(115200);
  espLidar.init();

  // espNode.set_rplidar(&espLidar);
  espNode.init(&espLidar);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);
  ArduinoOTA.begin();

  
}

void loop() {
  espNode.loop();
  ArduinoOTA.handle();
}