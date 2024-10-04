// Load Wi-Fi library
#include <ESP8266WiFi.h>
// sound sensor data printing
 const int SAMPLE_TIME = 10;
 unsigned long millisCurrent;
 unsigned long millisLast = 0;
 unsigned long millisElapsed = 0;
 int sampleBufferValue = 0;
 //sound sensor data printing end

// Replace with your network credentials
const char* ssid = "YOUR SSID HERE";
const char* password = "YOUR PASSWORD HERE";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
// bunlar web sitesinde printlenecek bir sensor bir şey algılıyorsa
// "on" diyecek, yoksa "off" diyecek.
String output0State = ""; // PIR STATE
String output5State = ""; // SOUND STATE
String output4State = ""; // FLAME STATE

// Assign output variables to GPIO pins
// bu pinlere videodaki GPIO pin yerlerini gösteren kısımdan bak
// sadece ledler GPIOlara girecek, sensorler aşağıda dediğim gibi
// D3, D2 ve A0'a girecek. Eğer D3, D2 ve A0dan herhangi biri ledler
// için ayırdığımız pinlere tekabil ediyorsa başka pin seçeriz.
//const int redLed = 0; // PIR LED RED (D3)
//const int greenLed = 5; // SOUND LED GREEN (D1)
//const int blueLed = 4; // FLAME LED BLUE (D2)

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

///////wifisız koddan alıntı start

int pirSensor = 13; // PIR Sensor input GPIO 13 (D7)
int soundSensor = 12; // Sound Sensor input GPIO 12 (D6)
int flameSensor = A0; // Flame Sensor input A0

boolean greenLedStatus = false;
int state = LOW; // by default, no motion detected
int val = 0; // variable to store the sensor status (value)

// lowest and highest flame sensor readings:
const int flameSensorMin = 0;     // flame sensor minimum
const int flameSensorMax = 1024;  // flame sensor maximum

///////wifisız koddan alıntı end

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
    // Initialize the output variables as outputs
//  pinMode(redLed, OUTPUT);
//  pinMode(greenLed, OUTPUT);
//  pinMode(blueLed, OUTPUT);
    pinMode(soundSensor,INPUT);
    pinMode(pirSensor, INPUT);    // initialize sensors as inputs

  // Set outputs to LOW
//  digitalWrite(redLed, LOW);
//  digitalWrite(greenLed, LOW);
//  digitalWrite(blueLed, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

  //////vidyoya göre kesinlikle değiştirilmemesi gereken kısım start

    WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    currentTime = millis();
    previousTime = currentTime;
    while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
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

  //////vidyoya göre kesinlikle değiştirilmemesi gereken kısım end

  //// ana kodun geleceği yer start

  //////////////pir sensor code start//////////////
 // DİKKAT: PIR kodu ters çalışabilir, yani motion yokken ışık yakıp
 // varken söndürebilir, böyle olursa haber ver değiştiririm.
  
  val = digitalRead(pirSensor); // read PIR sensor value
  if (val == HIGH) { // check if the PIR sensor is HIGH
//    digitalWrite(redLed, HIGH); // turn red LED ON
    delay(500);
    
    if (state == LOW) {
      Serial.println("Motion detected!");
      output0State = "on"; 
      state = HIGH; // update variable state to HIGH
    }
  } 
  else {
//      digitalWrite(redLed, LOW); // turn red LED OFF
      delay(500);
      
      if (state == HIGH){
        Serial.println("Motion stopped!");
        output0State = "off";
        state = LOW; // update variable state to LOW
    }
  } 
  
  //////////////pir sensor code end//////////////
  
  //////////////sound sensor code start//////////////

  int SensorData=digitalRead(soundSensor); 
  if(SensorData==1){

    if(greenLedStatus==false){
        greenLedStatus=true;
//        digitalWrite(greenLed,HIGH);
        output5State = "on";
    }
    else{
        greenLedStatus=false;
//        digitalWrite(greenLed,LOW);
        output5State = "off";
    }
// sound data printing start
         millisCurrent = millis();
   millisElapsed = millisCurrent - millisLast;
 
   if (digitalRead(soundSensor) == LOW) {
     sampleBufferValue++;
   }
 
   if (millisElapsed > SAMPLE_TIME) {
   //  Serial.println(sampleBufferValue);
     sampleBufferValue = 0;
     millisLast = millisCurrent;
   }
   //sound data printing end
  } 
  
  //////////////sound sensor code end//////////////

  //////////////flame sensor code start//////////////
  
  // read the flame sensor on analog A0:
  int flameSensorReading = analogRead(flameSensor);
  // map the sensor range (four options):
  // ex: 'long int map(long int, long int, long int, long int, long int)'
  int range = map(flameSensorReading, flameSensorMin, flameSensorMax, 0, 3);
  
  // range value:
  switch (range) {
  case 0:    // A fire closer than 1.5 feet away.
    Serial.println("** Close Fire **");
//    digitalWrite(blueLed,HIGH);
    output4State = "on";
    break;
  case 1:    // A fire between 1-3 feet away.
    Serial.println("** Distant Fire **");
//    digitalWrite(blueLed,HIGH);
    output4State = "on";
    break;
  case 2:    // No fire detected.
    //Serial.println("No Fire");
//    digitalWrite(blueLed,LOW);
    output4State = "off";
    break;
  }
  delay(1);  // delay between reads
  
  //////////////flame sensor code end//////////////

  //// ana kodun geleceği yer end
  
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>"); // always start with this, indicates website will display html text
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"); // makes webpage responsive in any browser
            client.println("<meta http-equiv=\"refresh\" content=\"0.1\">"); //refresh website every 0.1 seconds
            client.println("<link rel=\"icon\" href=\"data:,\">");


            // Web Page Heading
            client.println("<body><h1>Home Security Web App</h1>");

            // Display current state, and ON/OFF buttons for GPIO 0  (PIR SENSOR)
            client.println("<p>PIR Sensor - State " + output0State + "</p>");

            // Display current state, and ON/OFF buttons for GPIO 5 (SOUND SENSOR) 
            client.println("<p>Sound Sensor - State " + output5State + " " + sampleBufferValue + "</p>");

            // Display current state, and ON/OFF buttons for GPIO 4 (FLAME SENSOR)  
            client.println("<p>Flame Sensor - State " + output4State + "</p>");
            client.println("</body></html>");

//// ne olduğunu anlamadığım ama gerekliymiş gibi gözüken kısım start

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
  
//// ne olduğunu anlamadığım ama gerekliymiş gibi gözüken kısım end
}
