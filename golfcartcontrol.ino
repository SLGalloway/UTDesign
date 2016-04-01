#include <WiFi.h>
#include <WiFiClient.h>
#include <SPI.h>

// motor pin-map 
#define POWER 10
#define TURN_L 2
#define TURN_R 3

//controls sent from phone, values to be passed to each motor
int controls[4];
int motorValue, turn;

// the network name you want to created
char ssid[] = "asd1234";
// your network password
char password[] = "12345678";
WiFiServer server(80);
boolean alreadyConnected = false; // whether or not the client was connected previously

String inData;

void setup()
{
  // initialize serial communication at 9600 bits per second:
  initPins();
  
  Serial.begin(115200);      // initialize serial communication
  Serial.print("Starting create network...");
  WiFi.beginNetwork(ssid, password);
  Serial.println("network created done.");
  printWifiStatus();
  Serial.println("Starting webserver on port 80");
  server.begin();                           // start the web server on port 80
  Serial.println("Webserver started!");
}

void loop(){
  WiFiClient client = server.available();   // listen for incoming clients
  // when the client sends the first byte, say hello:
  if (client) {
    if (!alreadyConnected) {
      // clead out the input buffer:
      client.flush();
      Serial.println("We have a new client");
      client.print("Hello, client!");
      alreadyConnected = true;
    }
    
    if (client.available() > 0) {
      // read the bytes incoming from the client:
      char thisChar = client.read();      
      // Process message when new line or carriage return character is recieved
      inData += thisChar;
      Serial.print(thisChar);
      if(thisChar == '\n' || thisChar == '\r'){
        Serial.print(thisChar);
        getControls(inData);
        if(controls[2] == 1)
        {
          getMotorValues();
          analogWrite(POWER, motorValue);
          if(turn == 0)
          {
            digitalWrite(TURN_L, LOW);
            digitalWrite(TURN_R, LOW);
          }
          else if(turn == 1)
          {
            digitalWrite(TURN_L, HIGH);
            digitalWrite(TURN_R, LOW);
          }
          else if(turn == 2)
          {
            digitalWrite(TURN_L, LOW);
            digitalWrite(TURN_R, HIGH);
          }
        }
        else
        {
          analogWrite(POWER, 0);
          digitalWrite(TURN_L, LOW);
          digitalWrite(TURN_R, LOW);
        }

        inData = ""; // Clear recieved buffer
        motorValue = 0;
        turn = 0;
      }
    }
  }
}

void getControls(String input)
{
  String splitStr[4];
  
  splitStr[0] = input.substring(0, 4);
  splitStr[1] = input.substring(5, 9);
  splitStr[2] = input.substring(10, 11);
  splitStr[3] = input.substring(12, 13);
  
  for(int i = 0; i < 4; i++)
  {
    Serial.println(splitStr[i]);
  }
  

  for(int i = 0; i < 4; i++)
  {
    controls[i] = splitStr[i].toInt();
    if(i < 2)
    {
      Serial.println(controls[i]);
    }
  }
}

void getMotorValues()
{
  motorValue = controls[0];
  motorValue = motorValue * 2.55;
  
  if(motorValue > 255)
  {
    motorValue = 255;
  }
  
  if(controls[1] == 0)
  {
    turn = 0;
  }
  
  else if(controls[1] < 0)
  {
    turn = 1;
  }
  
  else if(controls[1] > 0)
  {
    turn = 2;
  }
  
  Serial.println(motorValue);
}

void initPins() {
  pinMode(POWER, OUTPUT);
  pinMode(TURN_L, OUTPUT);
  pinMode(TURN_R, OUTPUT);
  analogWrite(POWER, 0);
  digitalWrite(TURN_L, LOW);
  digitalWrite(TURN_R, LOW);

}
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(ssid);
  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("Server IP Address: ");
  Serial.println(ip);
}

