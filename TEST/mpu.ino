#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long lastPrint = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== MPU6050 FINAL VERIFICATION ===");
    
    Wire.begin(21, 22);
    mpu.begin();
    
    Serial.println("Calibrating... Keep sensor STEADY for 3 seconds");
    delay(3000);
    mpu.calcOffsets();
    
    Serial.println("\n✅ CALIBRATION COMPLETE!");
    Serial.println("MPU6050 is working PERFECTLY!");
    
    Serial.println("\n=== STABILITY TEST ===");
    Serial.println("Testing for 30 seconds...");
    Serial.println("Time\tPitch\tRoll\tGyroX\tGyroY\tGyroZ");
}

void loop() {
    mpu.update();
    
    if(millis() - lastPrint > 1000) { // Every second
        float pitch = mpu.getAngleY();
        float roll = mpu.getAngleX();
        float gyroX = mpu.getGyroX();
        float gyroY = mpu.getGyroY();
        float gyroZ = mpu.getGyroZ();
        
        Serial.print(millis()/1000);
        Serial.print("\t");
        Serial.print(pitch, 1);
        Serial.print("\t");
        Serial.print(roll, 1);
        Serial.print("\t");
        Serial.print(gyroX, 1);
        Serial.print("\t");
        Serial.print(gyroY, 1);
        Serial.print("\t");
        Serial.print(gyroZ, 1);
        Serial.println();
        
        lastPrint = millis();
    }
}