#include <Arduino.h>
#define BAUD_RATE 115200
#define LED_BUILTIN 2
 String inputString = "";  

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
bool serialEvent(String& inputString) 
{

  bool data = false; 
  while (Serial.available()) {
    // get the new byte:
    Serial.print(Serial.read(), HEX);
    data = true; 
    
  }
 
  return data; 
}



void setup()
{
  // inititalize serial communication 
  Serial.begin(BAUD_RATE);
  // Serial2.begin(BAUD_RATE);

  // LED config 
  pinMode(LED_BUILTIN, OUTPUT);



}

void loop()
{
 if(serialEvent(inputString))
  {
 
    digitalWrite(LED_BUILTIN, HIGH);
     Serial.println("");
  }
  delay(10);
}