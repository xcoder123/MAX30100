/*
Copyright (c) 2017 Raivis Strogonovs
 
Permission is hereby granted, free of charge, to any person obtaining a copy of 
this software and associated documentation files (the "Software"), to deal in the 
Software without restriction, including without limitation the rights to use, copy, modify, 
merge, publish, distribute, sublicense, and/or sell copies of the Software, and to 
permit persons to whom the Software is furnished to do so, subject to the following conditions:
 
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#ifndef MAX30100_H
#define MAX30100_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
/*----------------------------------------------*/
/* Config defines, you can tailor to your needs */
/*----------------------------------------------*/

/* MAX30100 parameters */
#define DEFAULT_OPERATING_MODE            MAX30100_MODE_SPO2_HR    
/*!!!IMPORTANT
 * You can't just throw these two values at random. Check Check table 8 in datasheet on page 19.
 * 100hz + 1600us is max for that resolution
 */
#define DEFAULT_SAMPLING_RATE             MAX30100_SAMPLING_RATE_100HZ
#define DEFAULT_LED_PULSE_WIDTH           MAX30100_PULSE_WIDTH_1600US_ADC_16

#define DEFAULT_IR_LED_CURRENT            MAX30100_LED_CURRENT_50MA
#define STARTING_RED_LED_CURRENT          MAX30100_LED_CURRENT_27_1MA


/* Adjust RED LED current balancing*/
#define MAGIC_ACCEPTABLE_INTENSITY_DIFF         65000
#define RED_LED_CURRENT_ADJUSTMENT_MS           500

/* SaO2 parameters */
#define RESET_SPO2_EVERY_N_PULSES     4

/* Filter parameters */
#define ALPHA 0.95  //dc filter alpha value
#define MEAN_FILTER_SIZE        15  

/* Pulse detection parameters */
#define PULSE_MIN_THRESHOLD         100 //300 is good for finger, but for wrist you need like 20, and there is shitloads of noise
#define PULSE_MAX_THRESHOLD         2000
#define PULSE_GO_DOWN_THRESHOLD     1

#define PULSE_BPM_SAMPLE_SIZE       10 //Moving average size





/* Enums, data structures and typdefs. DO NOT EDIT */
struct pulseoxymeter_t {
  bool pulseDetected;
  float heartBPM;

  float irCardiogram;

  float irDcValue;
  float redDcValue;

  float SaO2;

  uint32_t lastBeatThreshold;

  float dcFilteredIR;
  float dcFilteredRed;
};

typedef enum PulseStateMachine {
    PULSE_IDLE,
    PULSE_TRACE_UP,
    PULSE_TRACE_DOWN
} PulseStateMachine;

struct fifo_t {
  uint16_t rawIR;
  uint16_t rawRed;
};

struct dcFilter_t {
  float w;
  float result;
};

struct butterworthFilter_t
{
  float v[2];
  float result;
};

struct meanDiffFilter_t
{
  float values[MEAN_FILTER_SIZE];
  byte index;
  float sum;
  byte count;
};

/* MAX30100 register and bit defines, DO NOT EDIT */
#define MAX30100_DEVICE                   0x57

//Part ID Registers
#define MAX30100_REV_ID                   0xFE
#define MAX30100_PART_ID                  0xFF

//status registers
#define MAX30100_INT_STATUS               0x00
#define MAX30100_INT_ENABLE               0x01

//Fifo registers
#define MAX30100_FIFO_WRITE               0x02
#define MAX30100_FIFO_OVERFLOW_COUNTER    0x03
#define MAX30100_FIFO_READ                0x04
#define MAX30100_FIFO_DATA                0x05

//Config registers
#define MAX30100_MODE_CONF                0x06
#define MAX30100_SPO2_CONF                0x07
#define MAX30100_LED_CONF                 0x09

//Temperature registers
#define MAX30100_TEMP_INT                 0x16
#define MAX30100_TEMP_FRACTION            0x17


//Bit defines MODE Regsiter
#define MAX30100_MODE_SHDN                (1<<7)
#define MAX30100_MODE_RESET               (1<<6)
#define MAX30100_MODE_TEMP_EN             (1<<3)

typedef enum Mode {
    MAX30100_MODE_HR_ONLY                 = 0x02,
    MAX30100_MODE_SPO2_HR                 = 0x03
} Mode;

//Bit defines SpO2 register
#define MAX30100_SPO2_HI_RES_EN           (1 << 6)
typedef enum SamplingRate {
    MAX30100_SAMPLING_RATE_50HZ           = 0x00,
    MAX30100_SAMPLING_RATE_100HZ          = 0x01,
    MAX30100_SAMPLING_RATE_167HZ          = 0x02,
    MAX30100_SAMPLING_RATE_200HZ          = 0x03,
    MAX30100_SAMPLING_RATE_400HZ          = 0x04,
    MAX30100_SAMPLING_RATE_600HZ          = 0x05,
    MAX30100_SAMPLING_RATE_800HZ          = 0x06,
    MAX30100_SAMPLING_RATE_1000HZ         = 0x07
} SamplingRate;

typedef enum LEDPulseWidth {
    MAX30100_PULSE_WIDTH_200US_ADC_13     = 0x00,
    MAX30100_PULSE_WIDTH_400US_ADC_14     = 0x01,
    MAX30100_PULSE_WIDTH_800US_ADC_15     = 0x02,
    MAX30100_PULSE_WIDTH_1600US_ADC_16    = 0x03,
} LEDPulseWidth;

typedef enum LEDCurrent {
    MAX30100_LED_CURRENT_0MA              = 0x00,
    MAX30100_LED_CURRENT_4_4MA            = 0x01,
    MAX30100_LED_CURRENT_7_6MA            = 0x02,
    MAX30100_LED_CURRENT_11MA             = 0x03,
    MAX30100_LED_CURRENT_14_2MA           = 0x04,
    MAX30100_LED_CURRENT_17_4MA           = 0x05,
    MAX30100_LED_CURRENT_20_8MA           = 0x06,
    MAX30100_LED_CURRENT_24MA             = 0x07,
    MAX30100_LED_CURRENT_27_1MA           = 0x08,
    MAX30100_LED_CURRENT_30_6MA           = 0x09,
    MAX30100_LED_CURRENT_33_8MA           = 0x0A,
    MAX30100_LED_CURRENT_37MA             = 0x0B,
    MAX30100_LED_CURRENT_40_2MA           = 0x0C,
    MAX30100_LED_CURRENT_43_6MA           = 0x0D,
    MAX30100_LED_CURRENT_46_8MA           = 0x0E,
    MAX30100_LED_CURRENT_50MA             = 0x0F
} LEDCurrent;

class MAX30100 
{
  public:
    MAX30100( Mode mode = DEFAULT_OPERATING_MODE, 
              SamplingRate samplingRate = DEFAULT_SAMPLING_RATE, 
              LEDPulseWidth pulseWidth = DEFAULT_LED_PULSE_WIDTH, 
              LEDCurrent IrLedCurrent = DEFAULT_IR_LED_CURRENT,
              bool highResMode = true,
              bool debug = false 
             );

    pulseoxymeter_t update();

    void setMode(Mode mode);
    void setHighresModeEnabled(bool enabled);
    void setSamplingRate(SamplingRate rate);
    void setLEDPulseWidth(LEDPulseWidth pw);
    void setLEDCurrents( byte redLedCurrent, byte IRLedCurrent );
    float readTemperature();
    fifo_t readFIFO();
    void printRegisters();

    dcFilter_t dcRemoval(float x, float prev_w, float alpha);
    void lowPassButterworthFilter( float x, butterworthFilter_t * filterResult );
    float meanDiff(float M, meanDiffFilter_t* filterValues);

  private:
    bool detectPulse(float sensor_value);
    void balanceIntesities( float redLedDC, float IRLedDC );
    
    void writeRegister(byte address, byte val);
    uint8_t readRegister(uint8_t address);
    void readFrom(byte address, int num, byte _buff[]);

  private:
    bool debug;
    
    uint8_t redLEDCurrent;
    float lastREDLedCurrentCheck;
    
    uint8_t currentPulseDetectorState;
    float currentBPM;
    float valuesBPM[PULSE_BPM_SAMPLE_SIZE];
    float valuesBPMSum;
    uint8_t valuesBPMCount;
    uint8_t bpmIndex;
    uint32_t lastBeatThreshold;
    
    fifo_t prevFifo;
    
    dcFilter_t dcFilterIR;
    dcFilter_t dcFilterRed;
    butterworthFilter_t lpbFilterIR;
    meanDiffFilter_t meanDiffIR;

    float irACValueSqSum;
    float redACValueSqSum;
    uint16_t samplesRecorded;
    uint16_t pulsesDetected;
    float currentSaO2Value;

    LEDCurrent IrLedCurrent;
};

#endif
