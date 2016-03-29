

#include <Wire.h>
#include <I2Cdev.h>
#include <MS5803_I2C.h>

//const uint8_t MS_MODEL = 1; // MS5803-01BA
//const uint8_t MS_MODEL = 2; // MS5803-02BA
const uint8_t MS_MODEL = 5; // MS5803-05BA
//const uint8_t MS_MODEL = 14; // MS5803-14BA
//const uint8_t MS_MODEL = 30; // MS5803-30BA

MS5803 presstemp(0x76);
const uint8_t loop_delay = 10; // Seconds between readings
uint32_t wake_time = millis();


void setup() {
  Serial.begin(57600);
  Wire.begin();
  // Start up and get Calubration constants.
  presstemp.initialize(MS_MODEL);
  if ( presstemp.testConnection() ) Serial.println("We are communicating with MS5803 via I2C.");
  else Serial.println("I2C Communications with MS5803 failed.");
  // See what Calibration constants are. You can compare them with values in datasheet.
  //presstemp.debugCalConstants();
  
}
void loop(){
  if ( millis() >= wake_time){
    wake_time += loop_delay * 1000; // Calculate next wake time.
    Serial.print("Getting temperature");
    // Best to get temperature first as it is used in pressure calculations
    // presstemp.getTemperature(); // This returns a raw value, but if you just want to print it:
    presstemp.calcMeasurements(ADC_4096);
    Serial.print("The temperature is "); Serial.print(presstemp.getTemp_C()); Serial.println(" C");
    //Again, this retuns a value, but it's easiest to do this:
    Serial.print("The pressure is "); Serial.print(presstemp.getPress_mBar()); Serial.println(" mBar");

  }
  if ( millis() % 1000 == 0 ) Serial.print('.'); // progress bar
  delay(1);
}