#include<WiFi.h>
#include<WiFiClient.h>
#include<SPI.h>
#include<aJSON.h>

//motor pin map
#define POWER 10          //analog pin to cart's motor controller for movement
#define TURN_R 2          //digital pin to steering actuator, high when turning right
#define TURN_L 3          //digital pin to steering actuator, high when turning left
#define TURN_PWR 4        //digital pin to steering actuator, high when turning
#define BRAKE_DWN 5       //digital pin to braking actuator, high when pressing brake
#define BRAKE_UP 6        //digital pin to braking actuator, high when releasing brake
#define BRAKE_PWR 7       //digital pin to braking actuator, high when pressing or releasing brake
#define REV_ON 23         //digital pin to reverse switch actuator, high when engaging reverse
#define REV_OFF 24        //digital pin to reverse switch actuator, high when disengaging reverse
#define REV_PWR 25        //digital pin to reverse switch actuator, high when engaging or disengaging reverse

//UltraSonic pin map
/*#define FC_TRIGGER 26
#define FC_ECHO 27
#define FL_TRIGGER 28
#define FL_ECHO 29
#define FR_TRIGGER 40
#define FR_ECHO 39
#define LF_TRIGGER 38
#define LF_ECHO 37
#define LB_TRIGGER 36
#define LB_ECHO 35
#define RF_TRIGGER 34
#define RF_ECHO 33
#define RB_TRIGGER 32
#define RB_ECHO 31*/

int power, turn, turnCount, newWaypoint, enable, rev, waypointCount, loopCount;
double latitude, goalLat, longitude, goalLong, heading, optimalHeading;
double waypoints[5][2];
boolean autoNav, fcCol, flCol, frCol, lfCol, lbCol, rfCol, rbCol, goLeft, goRight, goForward, collision;
boolean visited[5];

String inData = "";

// the network name you want to created
char ssid[] = "asd1234";
// your network password
char password[] = "12345678";
WiFiServer server(80);
boolean alreadyConnected = false;

void setup()
{
  initPins();
  
  for(int i = 0; i < 5; i++)
  {
    visited[i] = false;
  }
  turnCount = 0;
  loopCount = 0;
  waypointCount = 0;
  
  Serial.begin(115200);
  Serial.print("Creating wireless network: ");
  WiFi.beginNetwork(ssid, password);
  Serial.println("done.");
  printWifiStatus();
  Serial.println("Starting webserver on port 80");
  server.begin();
  Serial.println("Webserver started!");
}

void loop()
{
  
  WiFiClient client = server.available();   
  // listen for incoming clients 
  //when the client sends the first byte, say hello:
  if (client) 
  {
    if (!alreadyConnected) 
    {
      // clead out the input buffer:
      client.flush();
      Serial.println("We have a new client");
      client.print("Hello, client!");
      alreadyConnected = true;
    }
    
    while (client.available() > 0)
    {
      // read the bytes incoming from the client:
      char thisChar = client.read();      
      // Process message when new line or carriage return character is recieved
      inData += thisChar;
      //Serial.print(thisChar);
      if(thisChar == '}')
      {
        jsonParse();
        if(!autoNav)
        {
          if(enable == 1)
          {
            phoneControl();
          }
          else idle();
        }
        else if(autoNav)
        {
          autoNavigate();
        }
        inData = "";
      }
    }
  }  
}

//sends commands to cart based on manual controls if that mode is engaged
void phoneControl()
{
  if(rev == 0)
  {
    if(power > 0)
    {
      if(turn == 0)
      {
        go();
      }
      
      if(turn > 0)
      {
        turnR();
      }
      
      if(turn < 0)
      {
        turnL();
      }
    }
    else if(power < 0)
    {
      brake();
    }
    else idle();
  }
  else if(rev == 1)
  {
    if(power < 0)
    {
      reverse();
    }
    else if(power > 0)
    {
      brake();
    }
    else idle();
  }   
}

//parses input from mobile app as json object
void jsonParse()
{
  char jsonarray[500];
  inData.toCharArray(jsonarray, 500);
  aJsonObject *jsonObject = aJson.parse(jsonarray);
  aJsonObject *jgpsNav = aJson.getObjectItem(jsonObject, "gpsNav");
  newWaypoint = (jgpsNav->valueint);
  if(newWaypoint == 1)
  {
    autoNav = true;
  }
  aJsonObject *jenable = aJson.getObjectItem(jsonObject, "enable");
  enable = (jenable->valueint);
  Serial.print("\tEnable: ");
  Serial.print(enable);
  if(!autoNav)
  {
    aJsonObject *jpower = aJson.getObjectItem(jsonObject, "power");
    power = (jpower->valueint);
    Serial.print("Power: ");
    Serial.print(power);
    aJsonObject *jturn = aJson.getObjectItem(jsonObject, "turn");
    turn = (jturn->valueint);
    Serial.print("\tTurn: ");
    Serial.print(turn);
    aJsonObject *jreverse = aJson.getObjectItem(jsonObject, "reverse");
    rev = (jreverse->valueint);
    Serial.print("\tReverse: ");
    Serial.println(rev);
  }
  else if(autoNav)
  { 
    aJsonObject *jlatitude = aJson.getObjectItem(jsonObject, "latitude");
    latitude = (jlatitude->valuefloat);
    Serial.print("Latitude: ");
    Serial.print(latitude, 6);
    aJsonObject *jlongitude = aJson.getObjectItem(jsonObject, "longitude");
    longitude = (jlongitude->valuefloat);
    Serial.print("\tLongitude: ");
    Serial.print(longitude, 6);
    aJsonObject *jheading = aJson.getObjectItem(jsonObject, "heading");
    heading = (jheading->valuefloat);
    heading = 90 - heading;
    if(heading >= 360.0)
    {
      heading = heading - 360.0;
    }
    if(heading < 0)
    {
      heading = heading + 360.0;
    }
    Serial.print("\tHeading: ");
    Serial.println(heading);
    if(newWaypoint == 1)
    {
      Serial.print("New waypoint. ");
      aJsonObject *jwaypoints = aJson.getObjectItem(jsonObject, "waypoints");
      int tmp = aJson.getArraySize(jwaypoints);
      Serial.print("Total waypoints: ");
      Serial.println(tmp);
      for(int i = 0; i < tmp; i++)
      {
        aJsonObject *coordSet = aJson.getArrayItem(jwaypoints, i);
        aJsonObject *jlat = aJson.getArrayItem(coordSet, 0);
        waypoints[i][0] = (jlat->valuefloat);
        Serial.print("\tWaypoints[");
        Serial.print(i);
        Serial.print("[0]: ");
        Serial.print(waypoints[i][0]);
        aJsonObject *jlon = aJson.getArrayItem(coordSet, 1);
        waypoints[i][1] = (jlon->valuefloat);
        Serial.print("\tWaypoints[");
        Serial.print(i);
        Serial.print("][1]: ");
        Serial.println(waypoints[i][1]);
        waypointCount++;
      }
    }
  }
  aJson.deleteItem(jsonObject);
}

//calculates optimal heading based on current and goal locations
void getOptimalHeading()
{
  float adjacent = goalLong - longitude;
  float opposite = goalLat - latitude;
  
  optimalHeading = atan2(opposite, adjacent);
  
  optimalHeading = optimalHeading / 3.1415;
  optimalHeading = optimalHeading * 180.0;
  
  if(optimalHeading < 0.0)
  {
   optimalHeading = optimalHeading + 360.0; 
  }
  
  goForward = false;
  goLeft = false;
  goRight = false;
  
  Serial.print("Optimal heading: ");
  Serial.print(optimalHeading);
  
  Serial.print("\tDifference: ");
  Serial.println(optimalHeading - heading);
  
  if(((optimalHeading - heading) <= 10.0 && (optimalHeading - heading) >= -10.0) || ((optimalHeading - heading) >= 350.0 && (optimalHeading - heading) < 360.0) || ((optimalHeading - heading) <= -350.0 && (optimalHeading - heading) > -360.0))
  {
    goForward = true;
  }
  
  else if(((optimalHeading - heading) < 180.0 && (optimalHeading - heading) > 10.0) || (optimalHeading - heading) < -180.0)
  {
    goLeft = true;
  }
  
  else if(((optimalHeading - heading) > -180.0 && (optimalHeading - heading) < -10.0) || (optimalHeading - heading) > 180.0)
  {
    goRight = true;
  }
}

//auto navigation when no collisions are detected
void moveToGoal()
{
  Serial.print("Enable: ");
  Serial.println(enable);
  if(enable == 0)
  {
    idle();
  }
  else if(goForward)
  {
    forward();
  }
  
  else if(goLeft)
  {
    go();
    turnL();
  }
  
  else if(goRight)
  {
    go();
    turnR();
  }
}

//movement that takes collision data into account if some are detected
/*void moveAvoidingCollision()
{
  if(fcCol)
  {
    if(!lfCol && !lbCol && goLeft)
    {
      turnL();
    }
    else if(!rfCol && !rbCol && goRight)
    {
      turnR();
    }
    
    else if(!lfCol && !lbCol)
    {
      turnL();
    }
    
    else if(!rfCol && !rbCol)
    {
      turnR();
    }
    
    else brake();
  }
  
  else if(flCol)
  {
    if(!frCol && !rfCol && !rbCol)
    {
      turnR();
    }
    else brake();
  }
  
  else if(frCol)
  {
    if(!flCol && !lfCol && !lbCol)
    {
      turnL();
    }
    else brake();
  }
  
  else if(lfCol)
  {
    if(!rfCol)
    {
      turnR();
    }
    else brake();
  }
  
  else if(lbCol)
  {
    moveToGoal();
  }
  
  else if(rfCol)
  {
    turnL();
  }
  
  else if(rbCol)
  {
    moveToGoal();
  }
}*/

//sets all actuators and cart power to neutral
void idle()
{
  analogWrite(POWER, 0);
  digitalWrite(TURN_L, LOW);
  digitalWrite(TURN_R, LOW);
  digitalWrite(TURN_PWR, LOW);
  digitalWrite(BRAKE_DWN, LOW);
  digitalWrite(BRAKE_UP, LOW);
  digitalWrite(BRAKE_PWR, LOW);
  digitalWrite(REV_ON, LOW);
  digitalWrite(REV_OFF, LOW);
  digitalWrite(REV_PWR, LOW);
  Serial.println("Idle");
}


//initiates braking
void brake()
{
  analogWrite(POWER, 0);
  digitalWrite(TURN_L, LOW);
  digitalWrite(TURN_R, LOW);
  digitalWrite(TURN_PWR, LOW);
  digitalWrite(BRAKE_DWN, HIGH);
  digitalWrite(BRAKE_UP, LOW);
  digitalWrite(BRAKE_PWR, HIGH);
  digitalWrite(REV_ON, LOW);
  digitalWrite(REV_OFF, HIGH);
  digitalWrite(REV_PWR, HIGH);
  Serial.println("Braking");
}

//powers back wheels
void go()
{
  Serial.println("Power to back wheels.");
  analogWrite(POWER, 100);
  digitalWrite(BRAKE_DWN, LOW);
  digitalWrite(BRAKE_UP, HIGH);
  digitalWrite(BRAKE_PWR, HIGH);
  digitalWrite(REV_ON, LOW);
  digitalWrite(REV_OFF, HIGH);
  digitalWrite(REV_PWR, HIGH);
}

//straightens wheels and moves straight forward
void forward()
{
  go();
  Serial.println("Going forward.");
  if(turnCount < 0)
  {
    turnR();
  }
  
  else if(turnCount > 0)
  {
    turnL();
  }
  
  else{
    digitalWrite(TURN_R, LOW);
    digitalWrite(TURN_L, LOW);
    digitalWrite(TURN_PWR, LOW);
  }
}

//called when reverse should be engaged
void reverse()
{
  analogWrite(POWER, 100);
  digitalWrite(TURN_L, LOW);
  digitalWrite(TURN_R, LOW);
  digitalWrite(TURN_PWR, LOW);
  digitalWrite(BRAKE_DWN, LOW);
  digitalWrite(BRAKE_UP, HIGH);
  digitalWrite(BRAKE_PWR, HIGH);
  digitalWrite(REV_ON, HIGH);
  digitalWrite(REV_OFF, LOW);
  digitalWrite(REV_PWR, HIGH);
  
  Serial.println("Reverse");
}

//called when steering needs to turn left
void turnL()
{
  if(turnCount > -3 && autoNav)
  {
    turnCount--;
  }
  
  Serial.println("Turning left.");
  digitalWrite(TURN_L, HIGH);
  digitalWrite(TURN_R, LOW);
  digitalWrite(TURN_PWR, HIGH);
}

//called when steering needs to turn right
void turnR()
{
  analogWrite(POWER, 100);
  if(turnCount < 3 && autoNav)
  {
    turnCount++;
  }
  Serial.println("Turning right.");
  digitalWrite(TURN_L, LOW);
  digitalWrite(TURN_R, HIGH);
  digitalWrite(TURN_PWR, HIGH);
}

//test each ultrasonic, store info about ones that detect incoming collisions
/*void detectCollisions()
{
  fcCol = false;
  flCol = false;
  frCol = false;
  lfCol = false;
  rfCol = false;
  lbCol = false;
  rbCol = false;
  
  long distances[7]; 
  distances[0] = UltraSonicTest(FC_TRIGGER, FC_ECHO);
  distances[1] = UltraSonicTest(LF_TRIGGER, LF_ECHO);
  distances[2] = UltraSonicTest(RF_TRIGGER, RF_ECHO);
  distances[3] = UltraSonicTest(FL_TRIGGER, FL_ECHO);
  distances[4] = UltraSonicTest(LB_TRIGGER, LB_ECHO);
  distances[5] = UltraSonicTest(RB_TRIGGER, RB_ECHO);
  distances[6] = UltraSonicTest(FR_TRIGGER, FR_ECHO);
  
  if(distances[0] < 300)
  {
    fcCol = true;
  }
  if(distances[1] < 100)
  {
    lfCol = true;
  }
  if(distances[2] < 100)
  {
    rfCol = true;
  }
  if(distances[3] < 300)
  {
    flCol = true;
  }
  if(distances[4] < 100)
  {
    lbCol = true;
  }
  if(distances[5] < 100)
  {
    rbCol = true;
  }
  if(distances[6] < 300)
  {
    frCol = true;
  }
  if(fcCol || lfCol || lbCol || flCol || frCol || rfCol || rbCol)
  {
    collision = true;
  }
}

//test a single ultrasonic to get collision data
long UltraSonicTest(int trigger, int echo)
{
  long dur, dis;
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  dur = pulseIn(echo, HIGH);
  dis = dur/58.2;
  delay(10);
  return dis;
}*/

//find waypoint to navigate to, avoid collisions if necessary, otherwise move to goal
void autoNavigate()
{
  Serial.println("In autoNavigate.");
  for(int i = 0; i < waypointCount; i++)
  {
    if(!visited[i])
    {
      Serial.print("Going to waypoint ");
      Serial.println(i+1);
      goalLat = waypoints[i][0];
      goalLong = waypoints[i][1];
      break;
    }
  }
  
  double latDif = goalLat - latitude;
  double longDif = goalLong - longitude;
  if(latDif < .0001 && latDif > -.0001 && longDif < .0001 && longDif > -.0001)
  {
    for(int i = 0; i < waypointCount; i++)
    {
      if(!visited[i])
      {
        visited[i] = true;
        Serial.println("Waypoint reached.");
        break;
      }
    }
  }
  
  boolean allVisited = true;
  for(int i = 0; i < waypointCount; i++)
  {
    if(!visited[i])
    {
      allVisited = false;
    }
  }
  if(allVisited)
  {
    autoNav = false;
  }
  
  getOptimalHeading();
  /*detectCollisions();
  if(collision)
  {
    moveAvoidingCollision();
  }*/
  moveToGoal();
}

//initialize all used pins at start of program
void initPins() 
{
  pinMode(POWER, OUTPUT);
  pinMode(TURN_L, OUTPUT);
  pinMode(TURN_R, OUTPUT);
  pinMode(TURN_PWR, OUTPUT);
  pinMode(BRAKE_DWN, OUTPUT);
  pinMode(BRAKE_UP, OUTPUT);
  pinMode(BRAKE_PWR, OUTPUT);
  pinMode(REV_ON, OUTPUT);
  pinMode(REV_OFF, OUTPUT);
  pinMode(REV_OFF, OUTPUT);
  
  /*pinMode(FC_TRIGGER,OUTPUT);
  pinMode(FC_ECHO,INPUT);
  pinMode(FL_TRIGGER,OUTPUT);
  pinMode(FL_ECHO,INPUT);
  pinMode(FR_TRIGGER,OUTPUT);
  pinMode(FR_ECHO,INPUT);
  pinMode(LF_TRIGGER,OUTPUT);
  pinMode(LF_ECHO,INPUT);
  pinMode(LB_TRIGGER,OUTPUT);
  pinMode(LB_ECHO,INPUT);
  pinMode(RF_TRIGGER,OUTPUT);
  pinMode(RF_ECHO,INPUT);
  pinMode(RB_TRIGGER,OUTPUT);
  pinMode(RB_ECHO,INPUT);*/
  
  analogWrite(POWER, 0);
  digitalWrite(TURN_L, LOW);
  digitalWrite(TURN_R, LOW);
  digitalWrite(TURN_PWR, LOW);
  digitalWrite(BRAKE_DWN, LOW);
  digitalWrite(BRAKE_UP, LOW);
  digitalWrite(BRAKE_PWR, LOW);
  digitalWrite(REV_ON, LOW);
  digitalWrite(REV_OFF, LOW);
  digitalWrite(REV_PWR, LOW);
  
  /*digitalWrite(FC_TRIGGER, LOW);
  digitalWrite(FL_TRIGGER, LOW);
  digitalWrite(FR_TRIGGER, LOW);
  digitalWrite(LF_TRIGGER, LOW);
  digitalWrite(LB_TRIGGER, LOW);
  digitalWrite(RF_TRIGGER, LOW);
  digitalWrite(RB_TRIGGER, LOW);*/
} 

void printWifiStatus() 
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(ssid);
  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("Server IP Address: ");
  Serial.println(ip);
}
