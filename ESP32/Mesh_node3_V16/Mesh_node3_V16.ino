//_REMOTE_//

#include "painlessMesh.h"
#include <Arduino_JSON.h>
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel pixels(1, 27, NEO_GRB + NEO_KHZ800);

// MESH Details
#define   MESH_PREFIX     "RNTMESH" //name for your MESH
#define   MESH_PASSWORD   "MESHpassword" //password for your MESH
#define   MESH_PORT       5555 //default port

//Number for this node
int nodeNumber = 3;
int strint = 1;
int movE = 0;
String a;

int pushButton1 = 14;
int pushButton2 = 12;
int pushButton3 = 13;
int buttonState1 = 0;
int buttonState2 = 0;
int buttonState3 = 0;

int eCount = 0;
int forwardDirCnt = 0;

int node2 = 25;
int ESP1AutoMode = 0;
int ESP1AutoOpDone = 0;

int rightTurnFlg = 0;
int leftTurnFlg = 0;
int cRTurn = 0;
int cLTurn = 0;
int pRTurn = 0;
int pLTurn = 0;
int rightTurnCnt = 0;
int leftTurnCnt = 0;
int ESP1TurnCount = 0;

int spare = 0;
int spare2 = 0;

int plannerLines = 0;
int AutoMode = 0;
int AutoOpDone = 0;
int TurnDone = 0;
int TotalTurn = 0;
int TotalLength = 0;
int cTurn = 0;

int Length = 75;
int Width = 60;
int FcmdLength = 0;

int cmd = 0;
int Esp1Fcmd = 5;
int Esp1Tcmd = 5;
int Esp1FCntcmd = 0;
int Esp1TCntcmd = 0;
int Esp2Tcmd = 5;
int Esp2TDcmd = 1;

int E1rightTurnCnt = 0;
int E1leftTurnCnt = 0;
int E1TotalTurnCnt = 0;

int ESP1PosAverage = 0;
int ESP1LengthCovered = 0;

int ESP1TCntDone = 0;
int ESP1FcmdDone = 0;
int ESP1FCntcmdDone = 0;
int ESP1TCntcmdDone = 0;
int ESP2TurnDone = 0;

int ESP1Activation = 0;
int ESP2Activation = 0;
int ESPConnection = 0;


int CStart = 0;

unsigned long cMillis1;
unsigned long pMillis2;
const unsigned long tPeriod = 2500;

//String to send to other nodes with readings
String readings;

Scheduler userScheduler; // to control your personal task
painlessMesh  mesh;

// User stub
void sendMessage() ; // Prototype so PlatformIO doesn't complain
String getReadings(); // Prototype for sending sensor readings

TaskHandle_t Task1;
TaskHandle_t Task2;
//Create tasks: to send messages and get readings;
Task taskSendMessage(TASK_SECOND , TASK_FOREVER, &sendMessage);

String getReadings () {
  JSONVar jsonReadings;
  jsonReadings["Node"] = nodeNumber;
  jsonReadings["CMD"] = cmd;
  jsonReadings["ESP1ForwardCMD"]      =  Esp1Fcmd;
  jsonReadings["ESP1TurnCMD"]         =  Esp1Tcmd;
  jsonReadings["ESP1TurnCntCMD"]      =  Esp1TCntcmd;
  jsonReadings["ESP1ForwardCntCMD"]   =  Esp1FCntcmd;
  jsonReadings["ESP2TurnCMD"]         =  Esp2Tcmd;
  jsonReadings["ESP2TurnCntCMD"]      =  Esp2TDcmd;
  jsonReadings["AutoMode"]            =  AutoMode;
  jsonReadings["ESPConnection"]       =  ESPConnection;
  readings = JSON.stringify(jsonReadings);
  return readings;
}

void sendMessage () {
  String msg = getReadings();
  mesh.sendBroadcast(msg);
}

// Needed for painless library
void receivedCallback( uint32_t from, String &msg ){
  Serial.printf("Received from %u msg=%s\n", from, msg.c_str());
  JSONVar myObject = JSON.parse(msg.c_str());
  int node = myObject["node"];

  double countE = myObject["ROBOT_ESP1"];
  eCount = countE;

  double forwDirCnt = myObject["ROBOT_ESP1_forwardDirCnt"];
  if(forwDirCnt<10000) {  forwardDirCnt = forwDirCnt; }

  double rTurnCnt = myObject["ROBOT_ESP1_rightTurnCnt"];
  if(rTurnCnt<10000) {  E1rightTurnCnt = rTurnCnt; }
  
  double lTurnCnt = myObject["ROBOT_ESP1_leftTurnCnt"];
  if(lTurnCnt<10000) {  E1leftTurnCnt = lTurnCnt; }

  double ESP1Cmd = myObject["ROBOT_ESP1_CMD"];
  if(ESP1Cmd<10000) {  node2 = ESP1Cmd; }

  double ESP1Amd = myObject["ROBOT_ESP1_AMD"];
  if(ESP1Amd<10000) {  ESP1AutoMode = ESP1Amd; }

  double ESP1AutoOperationDone = myObject["ROBOT_ESP1_AutoOpDone"];
  if(ESP1AutoOperationDone<10000) {  ESP1AutoOpDone = ESP1AutoOperationDone; }

  double ESP1CountTurn = myObject["ROBOT_ESP1_TurnCount"];
  if(ESP1CountTurn<10000) {  ESP1TurnCount = ESP1CountTurn; }

  double ESP1CoveredLength = myObject["ROBOT_ESP1_LengthCovered"];
  if(ESP1CoveredLength<10000) {  ESP1LengthCovered = ESP1CoveredLength; ESP1LengthCovered = ESP1LengthCovered - 200; }

  double ESP1Posavg = myObject["ROBOT_ESP1_Posavg"];
  if(ESP1Posavg<10000) {  ESP1PosAverage = ESP1Posavg; ESP1PosAverage = ESP1PosAverage - 200; }

  double ESP1TurnCntDone = myObject["ROBOT_ESP1_TurnCntDone"];
  if(ESP1TurnCntDone<10000) {  ESP1TCntDone = ESP1TurnCntDone; }

  double E1FcmdDone = myObject["ROBOT_ESP1_FcmdDone"];
  if(E1FcmdDone<10000) {  ESP1FcmdDone = E1FcmdDone; }
  
  double E1FCntcmdDone = myObject["ROBOT_ESP1_FCntcmdDone"];
  if(E1FCntcmdDone<10000) {  ESP1FCntcmdDone = E1FCntcmdDone; }
  
  double E1TCntcmdDone = myObject["ROBOT_ESP1_TCntcmdDone"];
  if(E1TCntcmdDone<10000) {  ESP1TCntcmdDone = E1TCntcmdDone; }

  double E2TurnDone = myObject["ROBOT_ESP2_TurnDone"];
  if(E2TurnDone<10000) {  ESP2TurnDone = E2TurnDone; }

  double ESP1Active = myObject["ROBOT_ESP1_Activation"];
  if(ESP1Active<10000) {  ESP1Activation = ESP1Active; }

  double ESP2Active = myObject["ROBOT_ESP2_Activation"];
  if(ESP2Active<10000) {  ESP2Activation = ESP2Active; }

  E1TotalTurnCnt = E1rightTurnCnt + E1leftTurnCnt;
}

void newConnectionCallback(uint32_t nodeId){
  Serial.printf("New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback(){
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset){
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
}

void setup()
{
  Serial.begin(115200);
  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();
  xTaskCreatePinnedToCore(Task2code,"Task2",10000,NULL,1,&Task2,1);

  pinMode(pushButton1, INPUT_PULLUP);
  pinMode(pushButton2, INPUT_PULLUP);
  pinMode(pushButton3, INPUT_PULLUP);
  pixels.begin();
  pixels.setBrightness(100);
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();
  delay(200);
}

void Task2code( void * pvParameters )
{
  for(;;)
  {
    if(Serial.available( ) > 0)
    { 
      a = Serial.readString( );
      strint = a.toInt();
      Serial.print(" strint = ");
      Serial.print(strint);
      Serial.println();
      //pixels.clear();
      //pixels.setPixelColor(0, pixels.Color(0, 255, 0));
      //pixels.show();
    }
    getButton();
    CMD();
    print();
  }
}

void getButton()
{
  buttonState2 = digitalRead(pushButton2);
  buttonState3 = digitalRead(pushButton3);

  if(buttonState1 == 1 && buttonState2 == 0 && buttonState3 == 0)
  {
    Esp2TDcmd = 1;
  }
  else if(buttonState1 == 1 && buttonState2 == 0 && buttonState3 == 1)
  {
    Esp2TDcmd = 3;
  }
  else if(buttonState1 == 1 && buttonState2 == 1 && buttonState3 == 0)
  {
    Esp2TDcmd = 5;
  }
  else if(buttonState1 == 1 && buttonState2 == 1 && buttonState3 == 1)
  {
    Esp2TDcmd = Esp2TDcmd;
  }
  else
  {
    Esp2TDcmd = Esp2TDcmd;
  }
}

void CMD()
{
  if(ESP1Activation == 0 && ESP2Activation == 0)
  {
    ESPConnection = 0;
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
  }
  else if(ESP1Activation == 1 && ESP2Activation == 1)
  {
    ESPConnection = 1;
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    pixels.show();
  }

  buttonState1 = digitalRead(pushButton1);
  if(buttonState1 == 0 && buttonState2 == 1 && buttonState3 == 1)
  {
    strint = 5001;
  }

  if(strint < 4999)
  {
    area();
    if(strint>0 && strint<9)
    {
      Esp2TDcmd = strint;
    }
    else if(strint>10 && strint<20)
    {
      Esp1Fcmd = strint - 10;
    }
    else if(strint>20 && strint<30)
    {
      Esp1Tcmd = strint - 20;
      Esp2Tcmd = strint - 20;
    }
    else if(strint>99 && strint<199)
    {
      Esp1TCntcmd = strint - 100;
    }
    else if(strint>199 && strint<1000)
    {
      Esp1FCntcmd = strint;
    }
    else if(strint == 0)
    {
      Esp1FCntcmd = strint;
      Esp1TCntcmd = strint;
      Esp1Tcmd = strint;
      Esp1Fcmd = strint;
      Esp2Tcmd = strint;
    }
    
    cmd = strint;
    AutoMode = 0;  
  }

  else if(strint == 5000)
  {
    cmd = strint;
  }

  else if(strint == 5002)
  {
    cmd = strint;
    Esp1TCntcmd = 0;
    Esp1FCntcmd = 200;
  }

  else if(strint == 5001)
  {
    AutoMode = 1;

    if(TotalLength != ESP1LengthCovered)
    {
      cmd = strint;
      AutoMode = 1;
      if(Length != ESP1PosAverage)
      {
        if(CStart == 0)
        {
          Esp1FCntcmd = FcmdLength;
          Esp1Tcmd = 5;
          Esp2Tcmd = 5;
          Esp1TCntcmd = 0;
        }
        else if(CStart == 1)
        {
          if(ESP2TurnDone == 6 && ESP1FCntcmdDone == 200 && ESP1TCntDone == 55)
          {
            Esp1Tcmd = 5;
            Esp2Tcmd = 5;
          }
          else if(ESP2TurnDone == 5 && ESP1FCntcmdDone == 200 && ESP1TCntDone == 55)
          {
            Esp1TCntcmd = 0;
          }
          else if(ESP2TurnDone == 5 && ESP1FCntcmdDone == 200 && ESP1TCntDone == 0)
          {
            CStart = 0;
          }

          else if(ESP2TurnDone == 4 && ESP1FCntcmdDone == 200 && ESP1TCntDone == 49)
          {
            Esp1Tcmd = 5;
            Esp2Tcmd = 5;
          }
          else if(ESP2TurnDone == 5 && ESP1FCntcmdDone == 200 && ESP1TCntDone == 49)
          {
            Esp1TCntcmd = 0;
          }
          else if(ESP2TurnDone == 5 && ESP1FCntcmdDone == 200 && ESP1TCntDone == 0)
          {
            CStart = 0;
          }
        }
      }
      else if(Length == ESP1PosAverage)
      {
        if(E1TotalTurnCnt != TotalTurn)
        {
          if(E1TotalTurnCnt == 0)
          {
            if(ESP2TurnDone != 6)
            {
              Esp1Tcmd = 6;
              Esp2Tcmd = 6;
            }
            else if(ESP2TurnDone == 6 && ESP1FCntcmdDone == FcmdLength)
            {
              Esp1FCntcmd = 200;
            }
            else if(ESP2TurnDone == 6 && ESP1FCntcmdDone == 200)
            {
              Esp1TCntcmd = 55;
              CStart = 1;
            }
          }
          else if(E1TotalTurnCnt > 0 && E1TotalTurnCnt % 2 == 0)
          {
            if(ESP2TurnDone != 6)
            {
              Esp1Tcmd = 6;
              Esp2Tcmd = 6;
            }
            else if(ESP2TurnDone == 6 && ESP1FCntcmdDone == FcmdLength)
            {
              Esp1FCntcmd = 200;
            }
            else if(ESP2TurnDone == 6 && ESP1FCntcmdDone == 200)
            {
              Esp1TCntcmd = 55;
              CStart = 1;
            }
          }
          else if(E1TotalTurnCnt > 0 && E1TotalTurnCnt % 2 != 0)
          {
            if(ESP2TurnDone != 4)
            {
              Esp1Tcmd = 4;
              Esp2Tcmd = 4;
            }
            else if(ESP2TurnDone == 4 && ESP1FCntcmdDone == FcmdLength)
            {
              Esp1FCntcmd = 200;
            }
            else if(ESP2TurnDone == 4 && ESP1FCntcmdDone == 200)
            {
              Esp1TCntcmd = 49;
              CStart = 1;
            }
          }
        }
      }
    }
    else if(ESP1LengthCovered >= TotalLength)
    {
      cmd = 5002;
      strint = 5002;
      Esp1TCntcmd = 0;
      Esp1FCntcmd = 200;
      //timer1();
    }

  }
}

void timer1()
{
  cMillis1 = millis();
  if((cMillis1 - pMillis2)>=1000)
  {
    strint = 5000;
    pMillis2 = cMillis1;
  }
}

void area()
{
  if(strint > 1000 && strint < 2000)
  {
    Length = strint - 1000;
  }
  else if(strint > 2000 && strint < 3000)
  {
    Width = strint - 2000;
    plannerLines = Width/20;
    TotalLength = Length * plannerLines;

    if(plannerLines % 2 == 0)
    {
      //Even
      rightTurnCnt = plannerLines / 2;
      leftTurnCnt = rightTurnCnt - 1;
    }
    else if(plannerLines % 2 != 0)
    {
      //Odd
      rightTurnCnt = plannerLines / 2;
      leftTurnCnt = rightTurnCnt;
    }
    FcmdLength = Length+200;
    TotalTurn = rightTurnCnt + leftTurnCnt;
  }
  else
  {
    Length = Length;
    Width = Width;
    plannerLines = Width/20;
    TotalLength = Length * plannerLines;

    if(plannerLines % 2 == 0)
    {
      //Even
      rightTurnCnt = plannerLines / 2;
      leftTurnCnt = rightTurnCnt - 1;
    }
    else if(plannerLines % 2 != 0)
    {
      //Odd
      rightTurnCnt = plannerLines / 2;
      leftTurnCnt = rightTurnCnt;
    }
    FcmdLength = Length+200;
    TotalTurn = rightTurnCnt + leftTurnCnt;
  }
}

void print()
{
  //Serial.print(" I/P= ");
  Serial.print(strint);
  //Serial.print(" L= ");
  //Serial.print(Length);
  //Serial.print(" T.Length= ");
  //Serial.print(TotalLength);
  
  //Serial.print(" W= ");
  //Serial.print(Width);

  Serial.print(" LTurn= ");
  Serial.print(E1leftTurnCnt);
  Serial.print(" RTurn= ");
  Serial.print(E1rightTurnCnt);
  Serial.print(" TTurn= ");
  Serial.print(TotalTurn);
  Serial.print(" PLn= ");
  Serial.print(plannerLines);

  Serial.print(" E1Fcmd= ");
  Serial.print(Esp1Fcmd);
  Serial.print(" E1Tcmd= ");
  Serial.print(Esp1Tcmd);
  Serial.print(" E2Tcmd= ");
  Serial.print(Esp2Tcmd);
  Serial.print(" E1TCnt= ");
  Serial.print(Esp1TCntcmd);
  Serial.print(" E1FCnt= ");
  Serial.print(Esp1FCntcmd);
  Serial.print(" E1LCovered= ");
  Serial.print(ESP1LengthCovered);
  Serial.print(" E1PosAvg= ");
  Serial.print(ESP1PosAverage);
  
  Serial.print(" spr2= ");
  Serial.print(spare2);
  Serial.print(" E1TCntDn= ");
  Serial.print(ESP1TCntDone);
  Serial.print(" E2TDn= ");
  Serial.print(ESP2TurnDone);

  Serial.print(" E1FDn= ");
  Serial.print(ESP1FcmdDone);
  Serial.print(" E1FCntDn= ");
  Serial.print(ESP1FCntcmdDone);
  Serial.print(" E1TCntDn= ");
  Serial.print(ESP1TCntcmdDone);
  Serial.print(" Esp2TDcmd= ");
  Serial.print(Esp2TDcmd);

  Serial.println();
}

void loop() {
  mesh.update();
}