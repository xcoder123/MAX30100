# MAX30100
Driver for MAX30100 using arduino


This code is part of tutorial on my blog: https://morf.lv/implementing-pulse-oximeter-using-max30100

# Sample Usage
```
#include "MAX30100.h"

MAX30100* pulseOxymeter;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Pulse oxymeter test!");

  //pulseOxymeter = new MAX30100( DEFAULT_OPERATING_MODE, DEFAULT_SAMPLING_RATE, DEFAULT_LED_PULSE_WIDTH, DEFAULT_IR_LED_CURRENT, true, true );
  pulseOxymeter = new MAX30100();
  pinMode(2, OUTPUT);

}

void loop() {
  //You have to call update with frequency at least 37Hz. But the closer you call it to 100Hz the better, the filter will work.
  pulseoxymeter_t result = pulseOxymeter->update();
  

  if( result.pulseDetected == true )
  {
    Serial.println("BEAT");
    
    Serial.print( "BPM: " );
    Serial.print( result.heartBPM );
    Serial.print( " | " );
  
    Serial.print( "SaO2: " );
    Serial.print( result.SaO2 );
    Serial.println( "%" );
  }
}
```
