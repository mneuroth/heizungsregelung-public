/*********
  Rui Santos
  Complete project details at http://randomnerdtutorials.com  
  https://randomnerdtutorials.com/esp32-web-server-arduino-ide/
*********/

#define WITH_WIFI 

#ifdef WITH_WIFI
// Load Wi-Fi library
#include <WiFi.h>
#endif

//***********************************************************

int ledPin = 2;
int analogPinIn = 36;

const double NTC20_R1   = 10970.0;    //org/default: 11000.0;    // 11kOhm for NTC20, Value measured on 10.11.2024: 10970.0 Ohm
const double V_REF      = 3.3;        // Value measured on 10.11.20204: 3.301 V
const double NTC20_B    = 3976.0;     // from: http://www.produktinfo.conrad.com/datenblaetter/500000-524999/502362-da-01-de-NTC_TEMP_SENSOR_TS_NTC_203_60_150_C.pdf

double calc_voltage(double dDigitalValue)
{
  return dDigitalValue * V_REF / 4096.0 /*1024.0*/;  // ESP has 12 Bit ADCs, Arduino has 10 Bit ADCs
}

double calc_resistor(double U1, double U2, double R1)
{
  return R1 * ( U1 / U2 - 1.0 );
}

// see: http://www.umnicom.de/Elektronik/Schaltungssammlung/Temperatur/Ntc/Ntc.html
double calc_temperature(double R, double RN, double TN, double B)
{
  return B * TN / (B + log(R / RN) * TN ) - 273.16;
}

double calculateStandardDeviation(double v1, double v2, double v3, double v4, double v5, double mean) {
  return sqrt(((v1 - mean)*(v1 - mean) + (v2 - mean)*(v2 - mean) + (v3 - mean)*(v3 - mean) + (v4 - mean)*(v4 - mean) + (v5 - mean)*(v5 - mean)) / 5.0);
}

//***********************************************************

// Replace with your network credentials
const char* ssid = "MIMAWLAN";
const char* password = "34956597662374754122";

#ifdef WITH_WIFI
// Set web server port number to 80
WiFiServer server(80);
#endif

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
//String output26State = "off";
//String output27State = "off";

// Assign output variables to GPIO pins
//const int output26 = 26;
//const int output27 = 27;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup() {
  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);
  
  // Initialize the output variables as outputs
//  pinMode(output26, OUTPUT);
//  pinMode(output27, OUTPUT);
  // Set outputs to LOW
//  digitalWrite(output26, LOW);
//  digitalWrite(output27, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
#ifdef WITH_WIFI
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
#endif
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
#ifdef WITH_WIFI  
  Serial.println(WiFi.localIP());
  server.begin();
#endif

  //set the resolution to 12 bits (0-4096)
  analogReadResolution(12);

}

void loop(){
  digitalWrite(ledPin, LOW);

      //int analogValue = analogRead(analogPinIn);
      //Serial.println(analogValue);
      int analogVolts = analogReadMilliVolts(analogPinIn);
      //Serial.println(analogVolts);
      double R1,R2,R3,R4,R5, T1,T2,T3,T4,T5, T, stddev_T;
      //double dMean = analogVolts*0.001; // get_averaged_adc_value(i);
      double U2 = ((double)analogVolts)*0.001; //calc_voltage(dMean);
      R1 = calc_resistor(V_REF, U2, NTC20_R1);
      T1 = calc_temperature(R1, 20000.0, 298.16, NTC20_B);
      delay(200);
      analogVolts = analogReadMilliVolts(analogPinIn);
      U2 = ((double)analogVolts)*0.001;
      R2 = calc_resistor(V_REF, U2, NTC20_R1);
      T2 = calc_temperature(R2, 20000.0, 298.16, NTC20_B);
      delay(200);
      analogVolts = analogReadMilliVolts(analogPinIn);
      U2 = ((double)analogVolts)*0.001;
      R3 = calc_resistor(V_REF, U2, NTC20_R1);
      T3 = calc_temperature(R3, 20000.0, 298.16, NTC20_B);
      delay(200);
      analogVolts = analogReadMilliVolts(analogPinIn);
      U2 = ((double)analogVolts)*0.001;
      R4 = calc_resistor(V_REF, U2, NTC20_R1);
      T4 = calc_temperature(R4, 20000.0, 298.16, NTC20_B);
      delay(200);
      analogVolts = analogReadMilliVolts(analogPinIn);
      U2 = ((double)analogVolts)*0.001;
      R5 = calc_resistor(V_REF, U2, NTC20_R1);
      T5 = calc_temperature(R5, 20000.0, 298.16, NTC20_B);
        //delay(500);
        //Serial.println(T);
      T = (T1+T2+T3+T4+T5)/5.0;
//      Serial.println(T); 
//      Serial.println(T1); 
//      Serial.println(T2); 
//      Serial.println(T3); 
//      Serial.println(T4); 
//      Serial.println(T5);
//      Serial.println("-------------"); 
      stddev_T = calculateStandardDeviation(T1, T2, T3, T4, T5, T);      
//      Serial.println(stddev_T); 

#ifdef WITH_WIFI  
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client

    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected

      digitalWrite(ledPin, HIGH);   // show that a client is connected
/*
      // read the analog / millivolts value for pin:
      int analogValue = analogRead(analogPinIn);
      //Serial.println(analogValue);
      int analogVolts = analogReadMilliVolts(analogPinIn);
      //Serial.println(analogVolts);
      double R, T;
      //double dMean = analogVolts*0.001; // get_averaged_adc_value(i);
      double U2 = ((double)analogVolts)*0.001; //calc_voltage(dMean);
      R = calc_resistor(V_REF, U2, NTC20_R1);
      T = calc_temperature(R, 20000.0, 298.16, NTC20_B);
*/
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
/*            
            // turns the GPIOs on and off
            if (header.indexOf("GET /26/on") >= 0) {
              Serial.println("GPIO 26 on");
              output26State = "on";
              digitalWrite(output26, HIGH);
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("GPIO 26 off");
              output26State = "off";
              digitalWrite(output26, LOW);
            } else if (header.indexOf("GET /27/on") >= 0) {
              Serial.println("GPIO 27 on");
              output27State = "on";
              digitalWrite(output27, HIGH);
            } else if (header.indexOf("GET /27/off") >= 0) {
              Serial.println("GPIO 27 off");
              output27State = "off";
              digitalWrite(output27, LOW);
            }
*/
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            //client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            //client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            //client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            //client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            //client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            //client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP32 Temperature Measurement Server</h1>");
/*            
            // Display current state, and ON/OFF buttons for GPIO 26  
            client.println("<p>GPIO 26 - State " + output26State + "</p>");
            // If the output26State is off, it displays the ON button       
            if (output26State=="off") {
              client.println("<p><a href=\"/26/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
               
            // Display current state, and ON/OFF buttons for GPIO 27  
            client.println("<p>GPIO 27 - State " + output27State + "</p>");
            // If the output27State is off, it displays the ON button       
            if (output27State=="off") {
              client.println("<p><a href=\"/27/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/27/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
*/
            char buffer[120];
            sprintf(buffer, "<p>Temperature: %f &deg;C +/- %f &deg;C</p>", T, stddev_T);
            client.println(buffer);
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
#endif
}
