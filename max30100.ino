
/*
MIT License

Copyright (c) 2017 Raivis Strogonovs (https://morf.lv)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
#include <Arduino.h>
#include <math.h>
#include <Wire.h>

#include "MAX30100.h"

MAX30100* pulseOxymeter;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Pulse oxymeter test!");

  //pulseOxymeter = new MAX30100( DEFAULT_OPERATING_MODE, DEFAULT_SAMPLING_RATE, DEFAULT_LED_PULSE_WIDTH, DEFAULT_IR_LED_CURRENT, true, true );
  pulseOxymeter = new MAX30100();
  pinMode(2, OUTPUT);

  //pulseOxymeter->printRegisters();
}

void loop() {
  //return;
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

    Serial.print("{P2|BPM|255,40,0|");
    Serial.print(result.heartBPM);
    Serial.print("|SaO2|0,0,255|");
    Serial.print(result.SaO2);
    Serial.println("}");
  }

  
    
  //These are special packets for FlexiPlot plotting tool
  Serial.print("{P0|IR|0,0,255|");
  Serial.print(result.dcFilteredIR);
  Serial.print("|RED|255,0,0|");
  Serial.print(result.dcFilteredRed);
  Serial.println("}");
  
  Serial.print("{P1|RED|255,0,255|");
  Serial.print(result.irCardiogram);
  Serial.print("|BEAT|0,0,255|");
  Serial.print(result.lastBeatThreshold);
  Serial.println("}");

  

  delay(10);

  //Basic way of determening execution of the loop via oscoliscope
  digitalWrite( 2, !digitalRead(2) );
}




