  /*
  Read temperatures from NTC20 and PT1000 with Heatcontrol board V1.2.0.
 
  Started: 14. 3.2010
  Current:  4. 1.2018

  (c) 2010-2018 by Michael Neuroth

  Heatincontrol board:

(Output-Kanal via RS232)              MCP3008  MCP3008  ATMEL
    Logische Klemme     Board Klemme    IC4     IC5      IC2 
    -----------------------------------------------------------
            1               KL8         ADC7            ADC0
            2               KL7         ADC6            ADC1
            3               KL6         ADC5            ADC2
            4               KL5         ADC4            ADC3
            5               KL4         ADC3            ADC4
            6               KL3         ADC2            ADC5
            7               KL2         ADC1            ADC6
            8               KL1         ADC0            ADC7

            9               KL16                ADC7
           10               KL15                ADC6  
           11               KL14                ADC5
           12               KL13                ADC4
           13               KL12                ADC3
           14               KL11                ADC2
           15               KL10                ADC1
           16               KL9                 ADC0
 */

// *************************************************
// *** configuration ***
// *************************************************

#define BIG

#define _WITH_MCP3008
#define _WITH_IOS
#define _WITH_RTC
#define _WITH_MC_ADC
#undef _WITH_SD
#undef _WITH_LAN
#define _WITH_WATCHDOG

// *************************************************
// *** includes ***
// *************************************************

#include <SPI.h>

#ifdef _WITH_MCP3008
#include <math.h>
#include "MCP3008_simple.h"
// REMARK: if mcp3008 is disabled, the linker will remove (not referenced) the code in MCP3008_simple.cpp automatically
#endif

#ifdef _WITH_IOS
#include "MCP23S17.h"
#endif

#ifdef _WITH_SD
#include <SD.h>
#endif

#ifdef _WITH_RTC
#include "RTClib.h"   // use Real Time Clock Library from Adafruit: https://github.com/adafruit/RTClib
#endif

#ifdef _WITH_WATCHDOG
#include <avr/wdt.h>
#endif

#include <EEPROM.h> 

// *************************************************
// *** defines ***
// *************************************************

#define VERSION "2.0 from 29.1.2018"

#define SPI_CLOCK_RATE  20000     // 10000 .. 50000 does not help with quality of MCP3008 ADC 

#define CMD_READ_DATA   "READ_DATA"
#define CMD_READ_TEMP   "READ_TEMP"
#define CMD_READ_RAW    "READ_RAW"
#define CMD_READ_IO     "READ_IO"
#define CMD_READ_RELAIS "READ_RELAIS"
#define CMD_SET_RELAIS  "SET_RELAIS"
#define CMD_READ_TIME   "READ_TIME"
#define CMD_SET_TIME    "SET_TIME"
#define CMD_SD_DIR      "DIR"
#define CMD_DUMP_TEMP   "DUMP_TEMP"
#define CMD_DUMP_STOP   "DUMP_STOP"
#define CMD_DEBUG       "DEBUG"
#define CMD_PING        "PING"
#define CMD_VERSION     "VERSION"
#define CMD_HELP        "HELP"
#define CMD_TEST        "TEST"
#define CMD_RESTARTS    "RESTARTS"

// TODO: commands:
// READ_ADC_RAW
// SET_IO
// READ_SD_CARD
// READ_STATUS  --> Anzeige welche Hardware ok ist, z. B. RTC, etc.
// Ethernet-Card Befehle ?
// MC-ADC ebenfalls ausgeben (DEBUG-Modus)
// ggf. kompatibilitaets modus mit 12 Kanaelen realisieren, bzw. neues Protokoll einfuehren mit erweiterten Daten
// ggf. debug modus fuer firmware via #ifdef
// ggf. write_sensor_values_to_rs232() und CMD entfernen. Fuer was braucht man dieses Command?

#define LED_1_PIN                   13
#define LED_2_PIN                   14

#define MCP3008_SLAVE_SELECT_CHIP1  10
#define MCP3008_SLAVE_SELECT_CHIP2  11 
#define MCP23S17_SLAVE_SELECT_PIN   12 
#define MCP23S17_ADDRESS             0
#define SD_SLAVE_SELECT_PIN          3    // org: 3   // aber: SS == 5 fuer ATMega32
#define LAN_SLAVE_SELECT_PIN         2    // org: 2

#define EEPROM_ADRESS_RESTART_COUNTER   42

// *************************************************
// *** constants ***
// *************************************************

const double PT1000_B   = -345.27;    // data from: http://www.fuehlersysteme.de/resistance_characteristics_de.pdf
const double NTC20_B    = 3976.0;     // from: http://www.produktinfo.conrad.com/datenblaetter/500000-524999/502362-da-01-de-NTC_TEMP_SENSOR_TS_NTC_203_60_150_C.pdf
//const double NTC20_B  = 4275.76;    // see: http://www.umnicom.de/Elektronik/Schaltungssammlung/Temperatur/Ntc/Ntc.html
                                      // data from: http://www.fuehlersysteme.de/resistance_characteristics_de.pdf
//const double NTC20_B  = 4283.72;    // data from: Honeywell Installationsanleitung

const double PT1000_R1  = 1000.0;     // org: 820.0;
const double PT1000_R2  = 917.0;      // == 1kOhm || 11kOhm == 916.6 kOhm (gemessen 917kOhm)
const double NTC20_R1   = 11000.0;    // 11kOhm for NTC20
const double V_REF      = 5.0;
const char SEPARATOR    = ';';

const int MAX_BUFFER_SIZE = 1024;     // org: 1024;      // until 2.1.2018 == 256

#ifndef _WITH_MC_ADC
const int AVERAGE_COUNT = 20;         // org: 50;        // number of measurements to average: until 26.4.2010 == 10, until 2.1.2018 == 8 (bei 25 funktioniert IOS nicht mehr --> 1660 Bytes Ram)
                                                         //                                                                               (bei 20 funktioniert IOS --> 1500 Bytes Ram)
const int MAX_CHANNELS  = 16;         // org: 12;
#else
const int AVERAGE_COUNT = 12;
const int MAX_CHANNELS  = 16+8;
#endif

// *************************************************
// *** variables ***
// *************************************************

int count = 0;
int g_bDump = 0;
unsigned long g_iMsgCount = 0;

// for RS232 communication
char g_sReadBuffer[MAX_BUFFER_SIZE];
int g_iReadBufferPos = 0;

#ifdef _WITH_SD
int g_bSDOk = 0;
#endif

#ifdef _WITH_MCP3008
int g_aAverageADCValue[MAX_CHANNELS][AVERAGE_COUNT];      // for average of sensor value // fist index is logical channel
int g_iActMeasurementCount = 0;
#endif

#ifdef _WITH_IOS
MCP g_ioChip(MCP23S17_ADDRESS, MCP23S17_SLAVE_SELECT_PIN);             
#endif

#ifdef _WITH_RTC
boolean g_bRTCOk = false;
RTC_DS1307 g_rtc;
#endif


// *************************************************
// *** code / functions ***
// *************************************************

// from: http://shelvin.de/eine-integer-zahl-in-das-arduiono-eeprom-schreiben/

void eepromWriteInt(int adr, int wert) 
{
  byte low, high;
  low=wert&0xFF;
  high=(wert>>8)&0xFF;
  EEPROM.write(adr, low); // dauert 3,3ms 
  EEPROM.write(adr+1, high);
}

int eepromReadInt(int adr) 
{
  byte low, high;
  low=EEPROM.read(adr);
  high=EEPROM.read(adr+1);
  return low + ((high << 8)&0xFF00);
}

// *************************************************
// *** code for ADCs ***
// *************************************************

#ifdef _WITH_MCP3008

double calc_voltage(double dDigitalValue)
{
    return dDigitalValue * V_REF / 1024.0;
}

double calc_resistor(double U1, double U2, double R1)
{
    return R1 * ( U1 / U2 - 1.0 );
}

// see: http://www.umnicom.de/Elektronik/Schaltungssammlung/Temperatur/Ntc/Ntc.html
double calc_temperature(double R, double RN, double TN, double B)
{
    return B * TN / (B + log(R/RN)*TN ) - 273.16;
}

double get_averaged_adc_value(int iAdcChannel)
{
    double dSum = 0.0;

    for( int j=0; j<AVERAGE_COUNT; j++ )
    {
        dSum += (double)g_aAverageADCValue[iAdcChannel][j];
    }    
    return dSum/((double)AVERAGE_COUNT);
}

void read_all_adc_values()
{
#ifdef _ADC_FROM_MC
    for( int logicalChannelNo=0; logicalChannelNo<MAX_CHANNELS; logicalChannelNo++ )
    {
        if( logicalChannelNo<8 )
        {
            g_aAverageADCValue[logicalChannelNo][g_iActMeasurementCount] = analogRead(logicalChannelNo);
        }
        else
        {
            g_aAverageADCValue[logicalChannelNo][g_iActMeasurementCount] = 0;  // not avaliable !        
        }
    }
    g_iActMeasurementCount++;
    if( g_iActMeasurementCount>=AVERAGE_COUNT )
    {
        g_iActMeasurementCount = 0;
    }
#else
    SPI.beginTransaction(SPISettings(SPI_CLOCK_RATE, MSBFIRST, SPI_MODE0));  // use slow sample rates, this is more precise ! (10000 - 300000)
    int MAX = MAX_CHANNELS>16 ? 16 : MAX_CHANNELS;    // only two MCO3008 available on HeatingControlBoard !
    const int MEASURE_AVERAGE_COUNT = 5;
    for( int logicalChannelNo=0; logicalChannelNo<MAX; logicalChannelNo++ )
    {
        uint8_t channelIndex = 0;
        uint8_t chipSelect = 0;
        if( logicalChannelNo<8 )
        {
            channelIndex = (uint8_t)(7-logicalChannelNo);
            chipSelect = (uint8_t)MCP3008_SLAVE_SELECT_CHIP1;
        }
        else
        {       
            // 0 --> 15 und 7 --> 8 log               // 8..15   =>  7..0  // 15-8=7  15-9=6 ...
            channelIndex = (uint8_t)(15-logicalChannelNo);
            chipSelect = (uint8_t)MCP3008_SLAVE_SELECT_CHIP2;
        }

        int sum = 0;
        // average the single measurement, because the MCP3008 ist much faster (and therefore not so precisely) than the ADC of the ATMega32 !
        for( int n=0; n<MEASURE_AVERAGE_COUNT; n++ )
        {
            sum += read_mcp3008_value(chipSelect, channelIndex);
        }
        g_aAverageADCValue[logicalChannelNo][g_iActMeasurementCount] = sum/MEASURE_AVERAGE_COUNT;

        //g_aAverageADCValue[logicalChannelNo][g_iActMeasurementCount] = read_mcp3008_value(chipSelect, channelIndex);          
    }
    SPI.endTransaction();
#ifdef _WITH_MC_ADC
    for( int logicalChannelNo=0; logicalChannelNo<8; logicalChannelNo++ )
    {
        if( logicalChannelNo<8 )
        {
            g_aAverageADCValue[logicalChannelNo+16][g_iActMeasurementCount] = analogRead(logicalChannelNo);
        }
    }
#endif
    g_iActMeasurementCount++;
    if( g_iActMeasurementCount>=AVERAGE_COUNT )
    {
        g_iActMeasurementCount = 0;
    }
#endif
}

#ifdef BIG
void write_sensor_values_to_rs232()
{
    Serial.print(F("Vref="));
    Serial.print(V_REF);
    Serial.print(SEPARATOR);
    for( int i=0; i<MAX_CHANNELS; i++ )
    {
        double R,T;
        double dMean = get_averaged_adc_value(i);
        double U2 = calc_voltage(dMean);
        Serial.print(F("Ch"));
        Serial.print(i);
        Serial.print(F("="));
        Serial.print(g_aAverageADCValue[i][g_iActMeasurementCount]);
        Serial.print(F(":"));
        Serial.print(U2);
        Serial.print(F(":"));
        if( i==0 || i==15 || i==16 )    // pt1000
        {
            R = calc_resistor(V_REF,U2,PT1000_R1);
            T = calc_temperature(R,1000.0,273.16,PT1000_B);
        }
// original:        
//        else if( /*i==10 ||*/ i==11 )
//        {
//            T = dMean;    // for ligth sensors return digital value
//        }
//        else if( i==15 )    // original channel 9  // pt1000 for icing protection heat pump (6.11.2010)
//        {
//            R = calc_resistor(V_REF,U2,PT1000_R1);
//            T = calc_temperature(R,1000.0,273.16,PT1000_B);
//        }
        else
        {
            R = calc_resistor(V_REF,U2,NTC20_R1);
            T = calc_temperature(R,20000.0,298.16,NTC20_B);
        }
        Serial.print(R);
        Serial.print(F(":"));
        Serial.print(T);
        Serial.print(SEPARATOR);
    }
    Serial.print(g_iMsgCount++);
    Serial.println();
}

// process something like: READ_RAW:Ch=0;
void write_raw_values_to_rs232(char * sCommand)
{
    char sExpr[8];
    memset(sExpr,0,8);
    strncpy(sExpr,sCommand+strlen(CMD_READ_RAW)+1,sizeof(sExpr));   // == Ch=0;
    char * sFound = strstr(sExpr,"=");
    char * sTermCmd = strstr(sExpr,";");
/*
    Serial.print(sCommand);
    Serial.print("#!#");
    Serial.print(sExpr);
    Serial.print("#");
    Serial.print(sFound);
    Serial.print("#");
    Serial.print(sTermCmd);
    Serial.print("#");
*/    
  //  Serial.println();
  //  return;
    
    if( sFound!=0 && sTermCmd!=0 )
    {
        char sChannelNo[3];
        memset(sChannelNo,0,3);
        strncpy(sChannelNo,sFound+1,sTermCmd-sFound-1);
        int iChannelNo = atoi(sChannelNo);
        Serial.print(F("OK:Channel="));
        Serial.print(iChannelNo);
        Serial.print(F(";"));
        Serial.print(F("Count="));
        Serial.print(AVERAGE_COUNT);
        Serial.print(F(";"));
        if( iChannelNo>=0 && iChannelNo<MAX_CHANNELS )
        {
            for( int i=0; i<AVERAGE_COUNT; i++ )
            {
                Serial.print(g_aAverageADCValue[iChannelNo][i]);        
                Serial.print(F(";"));
            }
        }
        Serial.println();
        return;
    }
    Serial.println(F("ERROR: in CMD_READ_RAW;"));
}
#endif

void write_temp_values_to_rs232()
{
    for( int i=0; i<MAX_CHANNELS; i++ )
    {
        double R,T;
        double dMean = get_averaged_adc_value(i);
        double U2 = calc_voltage(dMean);
        Serial.print(F("Ch"));
        Serial.print(i);
        Serial.print(F("="));
        if( i==0 || i==15 || i==16)    // pt1000
        {
            R = calc_resistor(V_REF,U2,PT1000_R1);
            T = calc_temperature(R,1000.0,273.16,PT1000_B);
        }
// original:        
//        else if( /*i==10 ||*/ i==11 )
//        {
//            T = dMean;    // for ligth sensors return digital value
//        }
//        else if( i==15 )   // original channel 9  // pt1000 for icing protection heat pump (6.11.2010)
//        {
//            R = calc_resistor(V_REF,U2,PT1000_R1);      // org: PT1000_R2
//            T = calc_temperature(R,1000.0,273.16,PT1000_B);
//||        }
        else
        {
            R = calc_resistor(V_REF,U2,NTC20_R1);
            T = calc_temperature(R,20000.0,298.16,NTC20_B);
        }
        Serial.print(T);
        //Serial.print(F("°C"));
        Serial.print(SEPARATOR);
    }
    Serial.print(g_iMsgCount++);
    Serial.println();
}

void init_all_adc_measurements()
{
    for( int j=0; j<AVERAGE_COUNT; j++ )
    {
        read_all_adc_values();
    }
}

#endif //_WITH_MCP3008

// *************************************************
// *** code for input output signals (relais) ***
// *************************************************

#ifdef _WITH_IOS

// writes something like: io1=0;io2=1;io3=1;io4=0<NL>
void write_io_values_to_rs232()
{
    const unsigned int MAX = 4;
    unsigned int inputData = g_ioChip.digitalRead();   
    unsigned int the4Bits = (inputData & 0x0F00) >> 8;
    for(unsigned int i=0; i<MAX; i++)
    {
        Serial.print(F("io"));
        Serial.print(i+1);    
        Serial.print(F("="));
        unsigned int val = (the4Bits & (1 << i)) >> i;
        Serial.print(val);
        if( i<MAX-1 )
        {
            Serial.print(F(";"));          
        }
    }
    Serial.println();
}

// writes something like: rel1=0;rel2=1;rel3=1;...<NL>
void write_relais_values_to_rs232()
{
    unsigned int inputData = g_ioChip.digitalRead();
    unsigned int first8Relais = inputData & 0xFF;
    unsigned int last4Relais = (inputData & 0xF000) >> 12;
    for(int i=7; i>=0; i--)
    {
        int j = 7-i;
        Serial.print(F("Rel"));
        Serial.print(j+1);    
        Serial.print(F("="));
        unsigned int val = (first8Relais & (1 << i)) >> i;
        Serial.print(val);    
        Serial.print(F(";"));
    }
    for(int i=3; i>=0; i--)
    {
        int j = 3-i;
        Serial.print(F("Rel"));
        Serial.print(8+j+1);    
        Serial.print(F("="));
        unsigned int val = (last4Relais & (1 << i)) >> i;
        Serial.print(val);    
        if( i>0 )
        {
            Serial.print(F(";"));          
        }
    }
    Serial.println();
}

// process something like: SET_RELAIS:rel1=0;<NL>
void set_relais_values_and_write_result_to_rs232(char * sCommand)
{
// TODO: bessere pruefung auf falsche input parameter, siehe auch Funktion set_rtc(...): ggf syntax: SET_RELAIS:rel01=1; 
// TODO: syntax pruefung durchfuehren fuer Rel2=1;
    char sExpr[8];
    memset(sExpr,0,8);
    strncpy(sExpr,sCommand+strlen(CMD_SET_RELAIS)+4,sizeof(sExpr));
    char * sFound = strstr(sExpr,"=");
    char * sTermCmd = strstr(sExpr,";");
    if( sFound!=0 && sTermCmd!=0 )
    {
        char sRelaisNo[3];
        memset(sRelaisNo,0,3);
        strncpy(sRelaisNo,sExpr,sFound-sExpr);
        int iRelaisNo = atoi(sRelaisNo);
        char sValue[3];
        memset(sValue,0,3);
        strncpy(sValue,sFound+1,sTermCmd-sFound-1);
        int iValue = atoi(sValue);
        Serial.print(F("OK:Rel"));
        Serial.print(iRelaisNo);
        Serial.print(F("="));
        Serial.print(iValue);
        Serial.println();
        if( iRelaisNo<9 )
        {
            // 1 --> pin 8, 2 --> pin 7
            g_ioChip.digitalWrite((uint8_t)(9-iRelaisNo),(uint8_t)(iValue==1));
        }
        else
        {
            // 9 --> pin 16, 10 --> pin 15
            g_ioChip.digitalWrite((uint8_t)(16-(iRelaisNo-9)),(uint8_t)(iValue==1));          
        }
    }
    else
    {
        Serial.println(F("ERROR: in CMD_SET_RELAIS;"));
    }
}

#endif //_WITH_IOS

#ifdef TEST
// process something like: TEST:1;<NL>
void do_test(char * sCommand)
{
    // GULP
    digitalWrite(1, LOW);
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);

    char sExpr[8];
    memset(sExpr,0,8);
    strncpy(sExpr,sCommand+strlen(CMD_TEST)+1,sizeof(sExpr));
    char * sTermCmd = strstr(sExpr,";");
    if( sTermCmd!=0 )
    {
        char sNo[3];
        memset(sNo,0,3);
        strncpy(sNo,sExpr,sTermCmd-sExpr);
        int iNo = atoi(sNo);
        digitalWrite(iNo, HIGH);
        Serial.print(F("TEST:"));
        Serial.print(iNo);
        Serial.println(F(";"));
    }
    else
    {
        Serial.println(F("ERROR: in CMD_TEST;"));
    }
}
#endif
// *************************************************
// *** code for SD card ***
// *************************************************

#ifdef _WITH_SD

// from arduino examples
void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

#endif

// *************************************************
// *** code for the real time clock ***
// *************************************************

#ifdef _WITH_RTC

void read_rtc()
{
/*
    if( g_bRTCOk )
    {
        Serial.println("RTC OK!");   
    }
    else
    {
        Serial.println("ERROR: RTC N O T OK!");   
    }

    if( g_rtc.isrunning() )
    {
        Serial.println("RTC running!");
    }
    else
    {
        Serial.println("ERROR: RTC N O T running!");
    }
 */   
    if( g_bRTCOk && g_rtc.isrunning() )
    {
        DateTime now = g_rtc.now();
        Serial.print(F("TIME="));
        char buffer[11];
        sprintf(buffer, "%4d-%02d-%02d", now.year(), now.month(), now.day());
        Serial.print(buffer);
        Serial.print(F(","));
        sprintf(buffer, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
        Serial.print(buffer);
        Serial.println(F(";"));        
    }
    else
    {
        Serial.println(F("ERROR: real time clock not available or not running;"));
    }
}

// process something like: SET_TIME:2017-12-31+15:24;<NL>
//                     or: SET_TIME:2017-01-06+03:05;<NL>
void set_rtc(char * sCommand)
{
    const char dateSep = '-';
    const char sep = '+';
    const char timeSep = ':';

    char sDateTime[20];
    memset(sDateTime,0,20);
    strncpy(sDateTime,sCommand+strlen(CMD_SET_TIME)+1,sizeof(sDateTime));

    if( !g_bRTCOk )
    {
        Serial.println(F("ERROR: real time clock not available;"));
        return;
    }

    // check for expected date/time format
    if( sDateTime[4]!=dateSep || sDateTime[7]!=dateSep || sDateTime[10]!=sep || sDateTime[13]!=timeSep || sDateTime[16]!=';' )
    {
        Serial.println(F("ERROR: invalid time format (expected: SET_TIME:2017-12-31+15:24;);"));
    }
    else
    {
        String s;

        char sYear[5];
        memset(sYear,0,5);
        strncpy(sYear,sDateTime,4);
        sYear[4] = 0;
        char sMonth[3];
        memset(sMonth,0,3);
        strncpy(sMonth,sDateTime+5,2);
        sMonth[2] = 0;
        char sDay[3];
        memset(sDay,0,3);
        strncpy(sDay,sDateTime+8,2);
        sDay[2] = 0;
        char sHour[3];
        memset(sHour,0,3);
        strncpy(sHour,sDateTime+11,2);
        sHour[2] = 0;
        char sMinute[3];
        memset(sMinute,0,3);
        strncpy(sMinute,sDateTime+14,2);
        sMinute[2] = 0;

        s = sYear;
        int iYear = s.toInt();
        s = sMonth;
        int iMonth = s.toInt();
        s = sDay;
        int iDay = s.toInt();
        s = sHour;
        int iHour = s.toInt();
        s = sMinute;
        int iMinute = s.toInt();

        g_rtc.adjust(DateTime(iYear,iMonth,iDay,iHour,iMinute,0));
        /*
        Serial.print(iYear);
        Serial.print(" ");
        Serial.print(iMonth);
        Serial.print(" ");
        Serial.print(iDay);
        Serial.print(" ");
        Serial.print(iHour);
        Serial.print(" ");
        Serial.println(iMinute);
        */
        Serial.println(F("OK: Set RTC;"));
    }
}

#endif 

// *************************************************
// *** code for RS232 communication ***
// *************************************************

void try_read_rs232()
{
    while( Serial.available()>0 )
    {
        int iByte = Serial.read();
        if( g_iReadBufferPos+1<MAX_BUFFER_SIZE )
        {
            g_sReadBuffer[g_iReadBufferPos] = (char)iByte;
            g_iReadBufferPos++;
            g_sReadBuffer[g_iReadBufferPos] = 0;
        }
    }
}

int get_command_from_buffer(char * pBuffer, int iBufferSize)
{
    for( int i=0; i<g_iReadBufferPos; i++ )
    {
        if(  g_sReadBuffer[i]==';' )
        {
            strncpy(pBuffer,g_sReadBuffer,i+1<iBufferSize?i+1:iBufferSize);
            if( i+1<iBufferSize )
            {
                pBuffer[i+1] = 0;
            }
            memmove(g_sReadBuffer,g_sReadBuffer+i+1,g_iReadBufferPos-i+1);
            g_iReadBufferPos -= i+1;
            return i+1;
        }
    }
    return 0;
}

void process_command_from_rs232()
{
    char sBuffer[MAX_BUFFER_SIZE];
    
    try_read_rs232();
    if( get_command_from_buffer(sBuffer,MAX_BUFFER_SIZE)>0 )
    {
        if( strncmp(sBuffer,CMD_PING,strlen(CMD_PING))==0 )
        {
            Serial.println(F("PONG;"));
        }
        else if( strncmp(sBuffer,CMD_DEBUG,strlen(CMD_DEBUG))==0 )
        {
            write_debug_to_rs232();
        }
#ifdef _WITH_MCP3008
        else if( strncmp(sBuffer,CMD_READ_TEMP,strlen(CMD_READ_TEMP))==0 )
        {
            write_temp_values_to_rs232();
        }
#ifdef BIG
        else if( strncmp(sBuffer,CMD_READ_DATA,strlen(CMD_READ_DATA))==0 )
        {
            write_sensor_values_to_rs232();
        }
        else if( strncmp(sBuffer,CMD_READ_RAW,strlen(CMD_READ_RAW))==0 )
        {
            write_raw_values_to_rs232(sBuffer);
        }
#endif
        else if( strncmp(sBuffer,CMD_DUMP_TEMP,strlen(CMD_DUMP_TEMP))==0 )
        {
            g_bDump = 1;
        }
        else if( strncmp(sBuffer,CMD_DUMP_STOP,strlen(CMD_DUMP_STOP))==0 )
        {
            g_bDump = 0;
        }
#endif
#ifdef _WITH_IOS
        else if( strncmp(sBuffer,CMD_READ_IO,strlen(CMD_READ_IO))==0 )
        {
            write_io_values_to_rs232();
        }
        else if( strncmp(sBuffer,CMD_READ_RELAIS,strlen(CMD_READ_RELAIS))==0 )
        {
            write_relais_values_to_rs232();
        }        
        else if( strncmp(sBuffer,CMD_SET_RELAIS,strlen(CMD_SET_RELAIS))==0 )
        {
            set_relais_values_and_write_result_to_rs232(sBuffer);
        }       
#endif 
#ifdef _WITH_RTC        
        else if( strncmp(sBuffer,CMD_READ_TIME,strlen(CMD_READ_TIME))==0 )
        {
            read_rtc();
        }        
        else if( strncmp(sBuffer,CMD_SET_TIME,strlen(CMD_SET_TIME))==0 )
        {
            set_rtc(sBuffer);
        }       
#endif         
        else if( strncmp(sBuffer,CMD_VERSION,strlen(CMD_VERSION))==0 )
        {
            Serial.print(F("VERSION="));
            Serial.print(VERSION);
            Serial.println(F(";"));
        }
#ifdef _WITH_SD        
        else if( strncmp(sBuffer,CMD_SD_DIR,strlen(CMD_SD_DIR))==0 )
        {
            Serial.println(F("try read RS232!"));
            //digitalWrite(SD_SLAVE_SELECT_PIN, HIGH);
            //digitalWrite(LAN_SLAVE_SELECT_PIN, HIGH);

            if( g_bSDOk )
            {
                Serial.println(F("exeucte read RS232!"));
                File root = SD.open("/");

                Serial.println(root);
                printDirectory(root, 0);
            }

        }
#endif
#ifdef BIG
        else if( strncmp(sBuffer,CMD_HELP,strlen(CMD_HELP))==0 )
        {
            Serial.print(F("Help: List of commands: "));
            Serial.print(CMD_READ_DATA);
            Serial.print(F(","));
            Serial.print(CMD_READ_TEMP);
            Serial.print(F(","));
            Serial.print(CMD_READ_RAW);
            Serial.print(F(","));
            Serial.print(CMD_READ_IO);
            Serial.print(F(","));
            Serial.print(CMD_READ_RELAIS);
            Serial.print(F(","));
            Serial.print(CMD_SET_RELAIS);
            Serial.print(F(","));
            Serial.print(CMD_READ_TIME);
            Serial.print(F(","));
            Serial.print(CMD_SET_TIME);
            Serial.print(F(","));
            Serial.print(CMD_DUMP_TEMP);
            Serial.print(F(","));
            Serial.print(CMD_DUMP_STOP);
            Serial.print(F(","));
            Serial.print(CMD_DEBUG);
            Serial.print(F(","));
            Serial.print(CMD_PING);
            Serial.print(F(","));
            Serial.print(CMD_VERSION);
            Serial.print(F(","));
            Serial.print(CMD_HELP);
            Serial.print(F(","));
            Serial.print(CMD_RESTARTS);

            Serial.println(F(";"));
        }
#endif
#ifdef TEST
        else if( strncmp(sBuffer,CMD_TEST,strlen(CMD_TEST))==0 )
        {
            do_test(sBuffer);
        }
#endif   
        else if( strncmp(sBuffer,CMD_RESTARTS,strlen(CMD_RESTARTS))==0 )
        {
            Serial.print(F("Restarts="));
            Serial.print(eepromReadInt(EEPROM_ADRESS_RESTART_COUNTER));    
            Serial.println(F(";"));
        }
        else
        {
            // echo all other commands
            Serial.print(F("ERROR: unknown command: "));
            Serial.print(sBuffer);
            Serial.println(F(";"));
        }
    }
    if( g_bDump )
    {
#ifdef _WITH_MCP3008
        write_temp_values_to_rs232();
#endif
    }
}

void write_debug_to_rs232()
{
    Serial.print(F("DEBUG:"));

#ifdef _WITH_MCP3008    
    for( int i=0; i<MAX_CHANNELS; i++ )
    {
        double dValueMean = get_averaged_adc_value(i);
        Serial.print(F("Ch"));
        Serial.print(i);
        Serial.print(F("="));
        Serial.print(dValueMean);
        Serial.print(F(";"));
    }
#endif

#ifdef _WITH_IOS
    unsigned int inputData = g_ioChip.digitalRead(); 
    Serial.print(F("io="));
    Serial.print(inputData);
#endif

    Serial.println();
}


void blink_leds()
{
    if( millis()/100 % 10 == 1 )
    {
        digitalWrite(LED_1_PIN, HIGH);   // set the LED on
        digitalWrite(LED_2_PIN, HIGH);   // set the LED on
    }
    else
    {
        digitalWrite(LED_1_PIN, LOW);    // set the LED off
        digitalWrite(LED_2_PIN, LOW);    // set the LED off
    }
}

#ifdef _TEST
void blink_ios()
{
    if( millis()/1000 % 2 == 1 )
    {
        digitalWrite(SD_SLAVE_SELECT_PIN, HIGH);
        //digitalWrite(LAN_SLAVE_SELECT_PIN, HIGH);
    }
    else
    {
        digitalWrite(SD_SLAVE_SELECT_PIN, LOW);
        //digitalWrite(LAN_SLAVE_SELECT_PIN, LOW);
    }  
}
#endif

// *************************************************
// *** the initialization function ***
// *************************************************

void setup() 
{    
#ifdef TEST
    // TEST
    pinMode(1, OUTPUT);
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    digitalWrite(1, LOW);
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
#endif
    // disable SD card board and LAN board --> for testing
    //pinMode(2, OUTPUT);
    //pinMode(3, OUTPUT);
    //digitalWrite(2, HIGH);
    //digitalWrite(3, HIGH);
  
    // initialize serial communications at 19200 bps:
    Serial.begin(19200, SERIAL_8N1); 

    // initialize io pins
    pinMode(LED_1_PIN, OUTPUT);     
    pinMode(LED_2_PIN, OUTPUT);     

    SPI.begin();   
    
#ifdef _WITH_MCP3008
    setup_mcp3008(MCP3008_SLAVE_SELECT_CHIP1);
    setup_mcp3008(MCP3008_SLAVE_SELECT_CHIP2);

    init_all_adc_measurements();
#endif

#ifdef _WITH_IOS
    g_ioChip.begin();
    g_ioChip.pinMode(0xF0FF);   // Port GPB = 0xFF00   und Port GPA = 0x00FF
    g_ioChip.digitalWrite(0x0000);
#endif

#ifdef _WITH_SD
    pinMode(SD_SLAVE_SELECT_PIN, OUTPUT);     
    g_bSDOk = SD.begin(SD_SLAVE_SELECT_PIN); 
    if (!g_bSDOk) 
    {
        Serial.println(F("SD Card initialization failed!"));
    }
#endif

#ifdef _WITH_LAN
    pinMode(LAN_SLAVE_SELECT_PIN, OUTPUT);     
#endif

#ifdef _WITH_RTC
    g_bRTCOk = g_rtc.begin();
    if (!g_bRTCOk)
    {
        Serial.println(F("RTC initialization failed!"));
    }
#endif

    // increment restart counter
    int iRestarts = eepromReadInt(EEPROM_ADRESS_RESTART_COUNTER);
    eepromWriteInt(EEPROM_ADRESS_RESTART_COUNTER,iRestarts+1);

#ifdef _WITH_WATCHDOG
    wdt_enable(WDTO_8S);
#endif
}

// *************************************************
// *** the main processing loop ***
// *************************************************

void loop() 
{
    static int count = 0;
    static uint8_t toggle = 1;                      // Set up a static variable for toggling portB on/off

#ifdef _WITH_WATCHDOG
    wdt_reset();
#endif

    blink_leds();
    //blink_ios();
    count++;

#ifdef _WITH_MCP3008
    read_all_adc_values();
#endif
  
    process_command_from_rs232();  
//    write_temp_values_to_rs232();

//    write_sensor_values_to_rs232();

//    unsigned int inputData = g_ioChip.digitalRead(); 
/* 
    if(count > 100)
    {
        toggle = !toggle; 
        g_ioChip.digitalWrite(toggle ? 0xf0ff : 0x0);
        //g_ioChip.digitalWrite(0x0);
        count = 0;
    }
    else
    {
        //g_ioChip.digitalWrite(0xf0);
        g_ioChip.digitalWrite(toggle ? 0xf0ff : 0x0);
    }
*/    
    // wait at least 10 milliseconds before the next loop
    // for the analog-to-digital converter to settle
    // after the last reading:
    delay(10);                     
}

