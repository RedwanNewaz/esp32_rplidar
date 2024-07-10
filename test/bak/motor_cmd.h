#pragma once
#include <Arduino.h>

// Constants for the car type and function
const byte HEAD = 0x01;        // Example value, adjust as needed
const byte DEVICE_ID = 0x01;   // Example value, adjust as needed
const byte FUNC_MOTION = 0x02; // Example value, adjust as needed
const byte CAR_TYPE = 0x01;    // Example value, adjust as needed
const byte COMPLEMENT = 0xFF;  // Example value, adjust as needed

// Delay time
const unsigned long delay_time = 10; // Adjust as needed

inline void set_car_motion(float v_x, float v_y, float v_z) {
  try {
    int vx_parms = int(v_x * 1000);
    int vy_parms = int(v_y * 1000);
    int vz_parms = int(v_z * 1000);

    byte cmd[12];
    cmd[0] = HEAD;
    cmd[1] = DEVICE_ID;
    cmd[2] = 0; // Placeholder for length, will be set later
    cmd[3] = FUNC_MOTION;
    cmd[4] = CAR_TYPE;

    // Pack the parameters
    cmd[5] = lowByte(vx_parms);
    cmd[6] = highByte(vx_parms);
    cmd[7] = lowByte(vy_parms);
    cmd[8] = highByte(vy_parms);
    cmd[9] = lowByte(vz_parms);
    cmd[10] = highByte(vz_parms);

    // Set the length byte
    cmd[2] = sizeof(cmd) - 1;

    // Calculate the checksum
    byte checksum = COMPLEMENT;
    for (int i = 0; i < sizeof(cmd) - 1; ++i) {
      checksum += cmd[i];
    }
    cmd[11] = checksum & 0xFF;

    // Serial.print("motion: ");
 
    for (int i = 0; i < sizeof(cmd); ++i) {
      Serial.print(cmd[i], HEX);
      // Serial.print(" ");
    }




    // Delay to ensure the command is processed
    delay(delay_time);
  } catch (...) {
    Serial.println("---set_car_motion error!---");
  }
}


   // FFFCA1210000001D