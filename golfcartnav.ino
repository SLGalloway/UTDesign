#include<WiFi.h>
#include<WiFiClient.h>
#include<SPI.h>
#include<aJSON.h>

//motor pin map
#define POWER 10
#define TURN_R 2
#define TURN_L 3
#define TURN_PWR 4
#define BRAKE_DWN 5
#define BRAKE_UP 6
#define BRAKE_PWR 7
#define REV_ON 23
#define REV_OFF 24
#define REV_PWR 25

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
boolean alreadyConnected = false; // whether or not the client was connected previously

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
  Serial.print("Starting create network...");
  WiFi.beginNetwork(ssid, password);
  Serial.println("network created done.");
  printWifiStatus();
  Serial.println("Starting webserver on port 80");
  server.begin();                           // start the web server on port 80
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

void phoneControl()
{
  if(rev == 0)
  {
    if(power > 0)
    {
      if(turn == 0)
      {
        forward();
      }
      
      else if(turn > 0)
      {
        turnR();
      }
      
      else if(turn < 0)
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
    if(power > 0)
    {
      reverse();
    }
    else if(power < 0)
    {
      brake();
    }
    else idle();
  }   
}

void jsonParse()
{
  char jsonarray[200];
  inData.toCharArray(jsonarray, 200);
  aJsonObject *jsonObject = aJson.parse(jsonarray);
  aJsonObject *jgpsNav = aJson.getObjectItem(jsonObject, "gpsNav");
  newWaypoint = (jgpsNav->valueint);
  if(newWaypoint == 1)
  {
    autoNav = true;
  }
  if(!autoNav)
  {
    aJsonObject *jpower = aJson.getObjectItem(jsonObject, "power");
    power = (jpower->valueint);
    //Serial.print("Power: ");
    //Serial.println(power);
    aJsonObject *jturn = aJson.getObjectItem(jsonObject, "turn");
    turn = (jturn->valueint);
    //Serial.print("Turn: ");
    //Serial.println(turn);
    aJsonObject *jenable = aJson.getObjectItem(jsonObject, "enable");
    enable = (jenable->valueint);
    //Serial.print("Enable: ");
    //Serial.println(enable);
    aJsonObject *jreverse = aJson.getObjectItem(jsonObject, "reverse");
    rev = (jreverse->valueint);
    //Serial.print("Reverse: ");
    //Serial.println(rev);
  }
  else if(autoNav)
  { 
    aJsonObject *jlatitude = aJson.getObjectItem(jsonObject, "latitude");
    latitude = (jlatitude->valuefloat);
    //Serial.print("Latitude: ");
    //Serial.println(latitude);
    aJsonObject *jlongitude = aJson.getObjectItem(jsonObject, "longitude");
    longitude = (jlongitude->valuefloat);
    //Serial.print("Longitude: ");
    //Serial.println(longitude);
    aJsonObject *jheading = aJson.getObjectItem(jsonObject, "heading");
    heading = (jheading->valuefloat);
    heading = heading + 90.0;
    if(heading >= 360.0)
    {
      heading = heading - 360.0;
    }
    //Serial.print("Heading: ");
    //Serial.println(heading);
    if(newWaypoint == 1)
    {
      aJsonObject *jwaypoints = aJson.getObjectItem(jsonObject, "waypoints");
      int tmp = aJson.getArraySize(jwaypoints);
      for(int i = 0; i < tmp; i++)
      {
        aJsonObject *coordSet = aJson.getArrayItem(jwaypoints, i);
        aJsonObject *jlat = aJson.getArrayItem(coordSet, 0);
        waypoints[i][0] = (jlat->valuefloat);
        //Serial.print("Waypoints[");
        //Serial.print(i);
        //Serial.print("[0]: ");
        //Serial.println(waypoints[i][0]);
        aJsonObject *jlon = aJson.getArrayItem(coordSet, 1);
        waypoints[i][1] = (jlon->valuefloat);
        //Serial.print("Waypoints[");
        //Serial.print(i);
        //Serial.print("][1]: ");
        //Serial.println(waypoints[i][1]);
        waypointCount++;
      }
    }
  }
}
void getOptimalHeading()
{
  float opposite = goalLong - longitude;
  float adjacent = goalLat - latitude;
  
  optimalHeading = atan2(opposite, adjacent);
  
  optimalHeading = optimalHeading / 3.1415;
  optimalHeading = optimalHeading * 180.0;
  
  goForward = false;
  goLeft = false;
  goRight = false;
  
  if((optimalHeading - heading) < 3 && (optimalHeading - heading) > -3)
  {
    goForward = true;
  }
  
  else if(((optimalHeading - heading) < 180 && (optimalHeading - heading) > 3) || (optimalHeading - heading) < -180)
  {
    goLeft = true;
  }
  
  else if(((optimalHeading - heading) > -180 && (optimalHeading - heading) < -3) || (optimalHeading - heading) > 180)
  {
    goRight = true;
  }
}

void moveToGoal()
{
  if(goForward)
  {
    forward();
  }
  
  else if(goLeft)
  {
    turnL();
  }
  
  else if(goRight)
  {
    turnR();
  }
}

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

void idle()
{
  analogWrite(POWER, 0);
  digitalWrite(TURN_L, LOW);
  digitalWrite(TURN_R, LOW);
  digitalWrite(TURN_PWR, LOW);
  digitalWrite(BRAKE_DWN, LOW);
  digitalWrite(BRAKE_UP, HIGH);
  digitalWrite(BRAKE_PWR, HIGH);
  digitalWrite(REV_ON, LOW);
  digitalWrite(REV_OFF, HIGH);
  digitalWrite(REV_PWR, HIGH);
  Serial.println("Idle");
}

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

void forward()
{
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
    analogWrite(POWER, 100);
    digitalWrite(TURN_L, LOW);
    digitalWrite(TURN_R, LOW);
    digitalWrite(TURN_PWR, LOW);
    digitalWrite(BRAKE_DWN, LOW);
    digitalWrite(BRAKE_UP, HIGH);
    digitalWrite(BRAKE_PWR, HIGH);
    digitalWrite(REV_ON, LOW);
    digitalWrite(REV_OFF, HIGH);
    digitalWrite(REV_PWR, HIGH);
  }
}

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

void turnL()
{
  Serial.println("Turn count: ");
  Serial.println(turnCount);
  analogWrite(POWER, 100);
  if(turnCount > -11)
  {
    Serial.println("Turning left.");
    digitalWrite(TURN_L, HIGH);
    digitalWrite(TURN_R, LOW);
    digitalWrite(TURN_PWR, HIGH);
    turnCount--;
  }
  else{
    Serial.println("Turn count too low, staying forward.");
    digitalWrite(TURN_L, LOW);
    digitalWrite(TURN_R, LOW);
    digitalWrite(TURN_PWR, LOW);
  }
  digitalWrite(BRAKE_DWN, LOW);
  digitalWrite(BRAKE_UP, HIGH);
  digitalWrite(BRAKE_PWR, HIGH);
  digitalWrite(REV_ON, LOW);
  digitalWrite(REV_OFF, HIGH);
  digitalWrite(REV_PWR, HIGH);
}

void turnR()
{
  Serial.println("Turn count: ");
  Serial.println(turnCount);
  analogWrite(POWER, 100);
  if(turnCount < 11)
  {
    Serial.println("Turning right.");
    digitalWrite(TURN_L, LOW);
    digitalWrite(TURN_R, HIGH);
    digitalWrite(TURN_PWR, HIGH);
    turnCount++;
  }
  else{
    Serial.println("Turn count too high, staying forward.");
    digitalWrite(TURN_L, LOW);
    digitalWrite(TURN_R, LOW);
    digitalWrite(TURN_PWR, LOW);
  }
  digitalWrite(BRAKE_DWN, LOW);
  digitalWrite(BRAKE_UP, HIGH);
  digitalWrite(BRAKE_PWR, HIGH);
  digitalWrite(REV_ON, LOW);
  digitalWrite(REV_OFF, HIGH);
  digitalWrite(REV_PWR, HIGH);
}

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

void autoNavigate()
{
  Serial.println("In autoNavigate.");
  for(int i = 0; i < waypointCount; i++)
  {
    if(!visited[i])
    {
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
