#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check for the default address 0x28
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup(void) {
  Serial.begin(9600);
  Serial.println("BNO055 Orientation Sensor Test"); Serial.println("");

  /* Initialize the sensor */
  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
    
  /* Use the external crystal for better accuracy */
  bno.setExtCrystalUse(true);
}

void loop(void) {
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("  Y: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("  Z: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");

  delay(BNO055_SAMPLERATE_DELAY_MS);
}