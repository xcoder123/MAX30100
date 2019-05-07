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

#include "MAX30100.h"

MAX30100::MAX30100(
        Mode mode, 
        SamplingRate samplingRate, 
        LEDPulseWidth pulseWidth, 
        LEDCurrent IrLedCurrent,
        bool highResMode,
        bool debug)
{
  this->debug = debug;
  currentPulseDetectorState = PULSE_IDLE;

  setMode(  mode );

  //Check table 8 in datasheet on page 19. You can't just throw in sample rate and pulse width randomly. 100hz + 1600us is max for that resolution
  setSamplingRate( samplingRate );
  setLEDPulseWidth( pulseWidth );

  redLEDCurrent = (uint8_t)STARTING_RED_LED_CURRENT;
  lastREDLedCurrentCheck = 0;

  this->IrLedCurrent = IrLedCurrent;
  setLEDCurrents(redLEDCurrent, IrLedCurrent );
  setHighresModeEnabled(highResMode);


  dcFilterIR.w = 0;
  dcFilterIR.result = 0;

  dcFilterRed.w = 0;
  dcFilterRed.result = 0;


  lpbFilterIR.v[0] = 0;
  lpbFilterIR.v[1] = 0;
  lpbFilterIR.result = 0;

  memset(meanDiffIR.values, 0, sizeof(float)*MEAN_FILTER_SIZE);
  meanDiffIR.index = 0;
  meanDiffIR.sum = 0;
  meanDiffIR.count = 0;

  // Erase all valuesBPM
  memset(valuesBPM, 0, sizeof(float)*PULSE_BPM_SAMPLE_SIZE);
  valuesBPMSum = 0;
  valuesBPMCount = 0;
  bpmIndex = 0;
  

  irACValueSqSum = 0;
  redACValueSqSum = 0;
  samplesRecorded = 0;
  pulsesDetected = 0;
  currentSaO2Value = 0;

  lastBeatThreshold = 0;

}

pulseoxymeter_t MAX30100::update()
{
  pulseoxymeter_t result = {
    /*bool pulseDetected*/ false,
    /*float heartBPM*/ 0.0,
    /*float irCardiogram*/ 0.0,
    /*float irDcValue*/ 0.0,
    /*float redDcValue*/ 0.0,
    /*float SaO2*/ currentSaO2Value,
    /*uint32_t lastBeatThreshold*/ 0,
    /*float dcFilteredIR*/ 0.0,
    /*float dcFilteredRed*/ 0.0
  };

  
  fifo_t rawData = readFIFO();  
  
  dcFilterIR = dcRemoval( (float)rawData.rawIR, dcFilterIR.w, ALPHA );
  dcFilterRed = dcRemoval( (float)rawData.rawRed, dcFilterRed.w, ALPHA );

  float meanDiffResIR = meanDiff( dcFilterIR.result, &meanDiffIR);
  lowPassButterworthFilter( meanDiffResIR/*-dcFilterIR.result*/, &lpbFilterIR );

  irACValueSqSum += dcFilterIR.result * dcFilterIR.result;
  redACValueSqSum += dcFilterRed.result * dcFilterRed.result;
  samplesRecorded++;

  if( detectPulse( lpbFilterIR.result ) && samplesRecorded > 0 )
  {
    result.pulseDetected=true;
    pulsesDetected++;

    float ratioRMS = log( sqrt(redACValueSqSum/samplesRecorded) ) / log( sqrt(irACValueSqSum/samplesRecorded) );

    if( debug == true )
    {
      Serial.print("RMS Ratio: ");
      Serial.println(ratioRMS);
    }

    //This is my adjusted standard model, so it shows 0.89 as 94% saturation. It is probably far from correct, requires proper empircal calibration
    currentSaO2Value = 110.0 - 18.0 * ratioRMS;
    result.SaO2 = currentSaO2Value;
    
    if( pulsesDetected % RESET_SPO2_EVERY_N_PULSES == 0)
    {
      irACValueSqSum = 0;
      redACValueSqSum = 0;
      samplesRecorded = 0;
    }
  }

  balanceIntesities( dcFilterRed.w, dcFilterIR.w );


  result.heartBPM = currentBPM;
  result.irCardiogram = lpbFilterIR.result;
  result.irDcValue = dcFilterIR.w;
  result.redDcValue = dcFilterRed.w;
  result.lastBeatThreshold = lastBeatThreshold;
  result.dcFilteredIR = dcFilterIR.result;
  result.dcFilteredRed = dcFilterRed.result;  
  
  
  return result;
}

bool MAX30100::detectPulse(float sensor_value)
{
  static float prev_sensor_value = 0;
  static uint8_t values_went_down = 0;
  static uint32_t currentBeat = 0;
  static uint32_t lastBeat = 0;

  if(sensor_value > PULSE_MAX_THRESHOLD)
  {
    currentPulseDetectorState = PULSE_IDLE;
    prev_sensor_value = 0;
    lastBeat = 0;
    currentBeat = 0;
    values_went_down = 0;
    lastBeatThreshold = 0;
    return false;
  }

  switch(currentPulseDetectorState)
  {
    case PULSE_IDLE:
      if(sensor_value >= PULSE_MIN_THRESHOLD) {
        currentPulseDetectorState = PULSE_TRACE_UP;
        values_went_down = 0;
      }
      break;

    case PULSE_TRACE_UP:
      if(sensor_value > prev_sensor_value)
      {
        currentBeat = millis();
        lastBeatThreshold = sensor_value;
      }
      else
      {

        if(debug == true) 
        {
          Serial.print("Peak reached: ");
          Serial.print(sensor_value);
          Serial.print(" ");
          Serial.println(prev_sensor_value);
        }

        uint32_t beatDuration = currentBeat - lastBeat;
        lastBeat = currentBeat;

        float rawBPM = 0;
        if(beatDuration > 0)
          rawBPM = 60000.0 / (float)beatDuration;
        if(debug == true) 
          Serial.println(rawBPM);

        currentPulseDetectorState = PULSE_TRACE_DOWN;

        // Reset moving avg after a while without pulses
        if(beatDuration > PULSE_BEAT_DURATION_RST){
          memset(valuesBPM, 0, sizeof(float)*PULSE_BPM_SAMPLE_SIZE);
          valuesBPMSum = 0;
          bpmIndex = 0;
          valuesBPMCount = 0;

          if(debug)
            Serial.println("Moving avg. reseted");
        }

        // Eliminate glitchy values
        if(rawBPM > PULSE_MAX_VALUE || rawBPM < PULSE_MIN_VALUE){
          if(debug)
            Serial.println("BPM out of bounds. Not adding to moving avg.");
            
          return false;
        }

        valuesBPMSum -= valuesBPM[bpmIndex];
        valuesBPM[bpmIndex] = rawBPM;
        valuesBPMSum += valuesBPM[bpmIndex];

        if(valuesBPMCount < PULSE_BPM_SAMPLE_SIZE)
          valuesBPMCount++;

        if(debug == true) 
        {
          Serial.print("CurrentMoving Avg: ");
          for(int i=0; i<valuesBPMCount; i++)
          {
            Serial.print(valuesBPM[i]);
            Serial.print(" ");
          }
  
          Serial.println(" ");
        }

        bpmIndex++;
        bpmIndex = bpmIndex % PULSE_BPM_SAMPLE_SIZE;

        currentBPM = valuesBPMSum / valuesBPMCount;
        if(debug == true) 
        {
          Serial.print("AVg. BPM: ");
          Serial.println(currentBPM);
        }

        return true;
      }
      break;

    case PULSE_TRACE_DOWN:
      if(sensor_value < prev_sensor_value)
      {
        values_went_down++;
      }


      if(sensor_value < PULSE_MIN_THRESHOLD)
      {
        currentPulseDetectorState = PULSE_IDLE;
      }
      break;
  }

  prev_sensor_value = sensor_value;
  return false;
}

void MAX30100::balanceIntesities( float redLedDC, float IRLedDC )
{
  
  if( millis() - lastREDLedCurrentCheck >= RED_LED_CURRENT_ADJUSTMENT_MS) 
  {
    //Serial.println( redLedDC - IRLedDC );
    if( IRLedDC - redLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent < MAX30100_LED_CURRENT_50MA) 
    {
      redLEDCurrent++;
      setLEDCurrents( redLEDCurrent, IrLedCurrent );
      if(debug == true) 
        Serial.println("RED LED Current +");
    } 
    else if(redLedDC - IRLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent > 0) 
    {
      redLEDCurrent--;
      setLEDCurrents( redLEDCurrent, IrLedCurrent );
      if(debug == true) 
        Serial.println("RED LED Current -");
    }

    lastREDLedCurrentCheck = millis();
  }
}


// Writes val to address register on device
void MAX30100::writeRegister(byte address, byte val)
{
  Wire.beginTransmission(MAX30100_DEVICE); // start transmission to device
  Wire.write(address); // send register address
  Wire.write(val); // send value to write
  Wire.endTransmission(); // end transmission
}

uint8_t MAX30100::readRegister(uint8_t address)
{
  Wire.beginTransmission(MAX30100_DEVICE);
  Wire.write(address);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX30100_DEVICE, 1);

  return Wire.read();
}

// Reads num bytes starting from address register on device in to _buff array
void MAX30100::readFrom(byte address, int num, byte _buff[])
{
  Wire.beginTransmission(MAX30100_DEVICE); // start transmission to device
  Wire.write(address); // sends address to read from
  Wire.endTransmission(false); // end transmission

  Wire.requestFrom(MAX30100_DEVICE, num); // request 6 bytes from device Registers: DATAX0, DATAX1, DATAY0, DATAY1, DATAZ0, DATAZ1

  int i = 0;
  while(Wire.available()) // device may send less than requested (abnormal)
  {
    _buff[i++] = Wire.read(); // receive a byte
  }

  Wire.endTransmission(); // end transmission
}

void MAX30100::setMode(Mode mode)
{
  byte currentModeReg = readRegister( MAX30100_MODE_CONF );
  writeRegister( MAX30100_MODE_CONF, (currentModeReg & 0xF8) | mode );
}

void MAX30100::setHighresModeEnabled(bool enabled)
{
    uint8_t previous = readRegister(MAX30100_SPO2_CONF);
    if (enabled) {
        writeRegister(MAX30100_SPO2_CONF, previous | MAX30100_SPO2_HI_RES_EN);
    } else {
        writeRegister(MAX30100_SPO2_CONF, previous & ~MAX30100_SPO2_HI_RES_EN);
    }
}

void MAX30100::setSamplingRate(SamplingRate rate)
{
  byte currentSpO2Reg = readRegister( MAX30100_SPO2_CONF );
  writeRegister( MAX30100_SPO2_CONF, ( currentSpO2Reg & 0xE3 ) | (rate<<2) );
}

void MAX30100::setLEDPulseWidth(LEDPulseWidth pw)
{
  byte currentSpO2Reg = readRegister( MAX30100_SPO2_CONF );
  writeRegister( MAX30100_SPO2_CONF, ( currentSpO2Reg & 0xFC ) | pw );
}

void MAX30100::setLEDCurrents( byte redLedCurrent, byte IRLedCurrent )
{
  writeRegister( MAX30100_LED_CONF, (redLedCurrent << 4) | IRLedCurrent );
}

float MAX30100::readTemperature()
{
  byte currentModeReg = readRegister( MAX30100_MODE_CONF );
  writeRegister( MAX30100_MODE_CONF, currentModeReg | MAX30100_MODE_TEMP_EN );

  delay(100); //This can be changed to a while loop, there is an interrupt flag for when temperature has been read.

  int8_t temp = (int8_t)readRegister( MAX30100_TEMP_INT );
  float tempFraction = (float)readRegister( MAX30100_TEMP_FRACTION ) * 0.0625;

  return (float)temp + tempFraction;
}

fifo_t MAX30100::readFIFO()
{
  fifo_t result;

  byte buffer[4];
  readFrom( MAX30100_FIFO_DATA, 4, buffer );
  result.rawIR = (buffer[0] << 8) | buffer[1];
  result.rawRed = (buffer[2] << 8) | buffer[3];

  return result;
}

dcFilter_t MAX30100::dcRemoval(float x, float prev_w, float alpha)
{
  dcFilter_t filtered;
  filtered.w = x + alpha * prev_w;
  filtered.result = filtered.w - prev_w;

  return filtered;
}

void MAX30100::lowPassButterworthFilter( float x, butterworthFilter_t * filterResult )
{  
  filterResult->v[0] = filterResult->v[1];

  //Fs = 100Hz and Fc = 10Hz
  filterResult->v[1] = (2.452372752527856026e-1 * x) + (0.50952544949442879485 * filterResult->v[0]);

  //Fs = 100Hz and Fc = 4Hz
  //filterResult->v[1] = (1.367287359973195227e-1 * x) + (0.72654252800536101020 * filterResult->v[0]); //Very precise butterworth filter 

  filterResult->result = filterResult->v[0] + filterResult->v[1];
}

float MAX30100::meanDiff(float M, meanDiffFilter_t* filterValues)
{
  float avg = 0;

  filterValues->sum -= filterValues->values[filterValues->index];
  filterValues->values[filterValues->index] = M;
  filterValues->sum += filterValues->values[filterValues->index];

  filterValues->index++;
  filterValues->index = filterValues->index % MEAN_FILTER_SIZE;

  if(filterValues->count < MEAN_FILTER_SIZE)
    filterValues->count++;

  avg = filterValues->sum / filterValues->count;
  return avg - M;
}

void MAX30100::printRegisters()
{
  Serial.println(readRegister(MAX30100_INT_STATUS),  HEX);
  Serial.println(readRegister(MAX30100_INT_ENABLE),  HEX);
  Serial.println(readRegister(MAX30100_FIFO_WRITE), HEX);
  Serial.println(readRegister(MAX30100_FIFO_OVERFLOW_COUNTER), HEX);
  Serial.println(readRegister(MAX30100_FIFO_READ), HEX);
  Serial.println(readRegister(MAX30100_FIFO_DATA),   HEX);
  Serial.println(readRegister(MAX30100_MODE_CONF), HEX);
  Serial.println(readRegister(MAX30100_SPO2_CONF), HEX);
  Serial.println(readRegister(MAX30100_LED_CONF),  HEX);
  Serial.println(readRegister(MAX30100_TEMP_INT),   HEX);
  Serial.println(readRegister(MAX30100_TEMP_FRACTION),   HEX);
  Serial.println(readRegister(MAX30100_REV_ID),      HEX);
  Serial.println(readRegister(MAX30100_PART_ID),     HEX);
}

