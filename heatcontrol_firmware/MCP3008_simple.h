#ifndef _MCP3008_simple_h
#define _MCP3008_simple_h

#include <Arduino.h>

// adapted from: https://www.reddit.com/r/arduino/comments/2wz9kt/reading_a_mcp3008_via_spi_simply/

void setup_mcp3008(uint8_t chip_select);

int read_mcp3008_value(uint8_t chip_select, uint8_t channel);

#endif
