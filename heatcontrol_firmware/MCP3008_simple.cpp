
#include "MCP3008_simple.h"

#include <SPI.h>

void setup_mcp3008(uint8_t chip_select) 
{
    pinMode(chip_select, OUTPUT);
    digitalWrite(chip_select, HIGH);
}

int read_mcp3008_value(uint8_t chip_select, uint8_t channel) 
{  
    digitalWrite(chip_select, LOW);

    SPI.transfer(channel + 0b0011000);
    uint8_t value1 = SPI.transfer(0);
    uint8_t value2 = SPI.transfer(0);

    int value = ((value1 & 0x3f) << 4) + ((value2 & 0xf0) >> 4); 

    digitalWrite(chip_select, HIGH);

    return value;
}

