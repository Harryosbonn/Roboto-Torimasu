#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);             // Wait for serial monitor
  Serial.println("\nI2C Scanner");
}

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      if (address == 0x68) {
        Serial.println(">> MPU6050 CANDIDATE FOUND!");
        checkMPU();
      }

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}

void checkMPU() {
    Wire.beginTransmission(0x68);
    Wire.write(0x75); // WHO_AM_I register
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 1, true);
    if (Wire.available()) {
        byte c = Wire.read();
        Serial.print("   WHO_AM_I Register: 0x");
        Serial.println(c, HEX);
        if (c == 0x68) Serial.println("   HEALTH: GOOD (ID Matches)");
        else Serial.println("   HEALTH: SUSPICIOUS (ID Mismatch)");
    } else {
        Serial.println("   HEALTH: BAD (No Response to Read)");
    }
}
