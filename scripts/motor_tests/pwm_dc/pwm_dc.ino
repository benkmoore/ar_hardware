

#include "Adafruit_MCP4728.h"

#define MAX_PWM 2700                                  // 12 bit value (0 -> 4095) converted to analog voltage (0v -> 2.048v)
#define MIN_PWM 2000                                 // 12 bit value converted to analog voltage
#define MAX_VEL 1.04                                  // m/s
#define MIN_VEL 0.25
#define PULSEHIGH 2300

Adafruit_MCP4728 mcp;
int i = 0;

void setup() {
  // Setup analog board to use 2.048v as vref
  mcp.begin();
  mcp.setChannelValue(MCP4728_CHANNEL_A, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
  mcp.setChannelValue(MCP4728_CHANNEL_B, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
  mcp.setChannelValue(MCP4728_CHANNEL_C, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
  mcp.setChannelValue(MCP4728_CHANNEL_D, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
  mcp.saveToEEPROM();
}

void loop() {
//    i += 1;
//    if(i%4 == 0){
    mcp.fastWrite(0, 0, 0, 0);
//    delayMicroseconds(1);
//    }


    mcp.fastWrite(PULSEHIGH, PULSEHIGH, PULSEHIGH, PULSEHIGH);
    delayMicroseconds(500);
    // delay(20);
  
}
