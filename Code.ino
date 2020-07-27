//Nissan Leaf BMS 30KWH data request and read via CANBUS
//
//Written by David Blackhurst 27/7/2020
//
//Good to use on a personal basis, not for commercial use unless preagreed with myself
//
//Lots of time has been spent on this to make it as easy to use as possible, I use the Arduino with Can Bus shield from the below link
//
//https://www.hobbytronics.co.uk/leonardo-canbus?keyword=canbus
//
//If you feel like contributing to my ongoing Electric Car Conversions and code writing please do,
//
//https://www.paypal.me/DBlackhurst
//
//This code will query the BMS over the EV CanBus, it will collect...
//Cell Voltages, Cell Shunt Activity, SOC, SOH, HV Voltage, LV Voltage, Capacity, Temperatures (3), Current
//
//This code is without warranty, it is untested on all Nissan Leaf variations and I cannot be held responsible for any damage caused by this code
//
//USE AT YOUR OWN RISK

#include <Arduino.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

#define CANint 2
#define LED2 8
#define LED3 7

unsigned char len = 0;
unsigned char buf[8];
unsigned long ID = 0;
unsigned long line = 0;
unsigned long time;

MCP_CAN CAN0(17); // Set CS to pin 17

//Data to send [group] [request line]
byte sendGroup1[8] = {0x02,0x21,0x01,0,0,0,0,0};
byte sendGroup2[8] = {0x02,0x21,0x02,0,0,0,0,0};
byte sendGroup3[8] = {0x02,0x21,0x03,0,0,0,0,0};
byte sendGroup4[8] = {0x02,0x21,0x04,0,0,0,0,0};
byte sendGroup6[8] = {0x02,0x21,0x06,0,0,0,0,0};
byte sendNextLine[8] = {0x30,0x01,0,0xFF,0xFF,0xFF,0xFF,0xFF};

int BMSQuery = 0;
boolean HVVoltageCheck = false;

boolean tempsReading = false;
byte temps[4]; //Battery Pack Temperatures

boolean voltageReading = false;
int maxVolt = 0;
int minVolt = 0;

boolean packInfoReading = false;
float SOC = 0;
float SOH = 0;
float capacity = 0;
float current = 0;
float LVBattery = 0;
float HVBattery = 0;

boolean cellVoltagesReading = false;
float cellVoltage[96]; // All Cells Voltages
int cellCount = 0;
byte memVoltage = 0;

boolean shuntsReading = false;
boolean shunts[96]; // All shunt values
int shuntCount = 0;

void setup() {
  Serial.begin(115200);
  
  while (!Serial) {
    Serial.print("I will wait here forever...");
      delay(1000);
  }
  
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);

  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(CANint, INPUT);
  digitalWrite(LED2, LOW);
  
  Serial.println("CAN init:");

  if (CAN0.begin(CAN_500KBPS) == CAN_OK) {
    Serial.println("Can Init Success");
  } else {
    Serial.println("Can Init Failed");
    while (1) {
      Serial.print("I will wait here forever...");
      delay(1000);
    }
  }

  Serial.println("Good to go!");
}

void decode1DB() {
  HVVoltageCheck = true;
  HVBattery = (buf[2] + (float)(buf[3] >> 6)) * 2;
  
  if(buf[0] & B10000000 == B10000000) { //negative current
    current = (~buf[0] + ~(buf[1] >> 5) + 1) * 2; //invert the bytes and add one
  } else {
    current = (buf[0] + (buf[1] >> 5)) * 2;
  }
}

void decode7BB() {
  if ((buf[0] == 0x10 && buf[3] == 0x01) || BMSQuery == 1) { //[0] First Line, Group 1 - Pack Info
    readPackInfo();
  } else if ((buf[0] == 0x10 && buf[3] == 0x02) || BMSQuery == 3) { //[0] First Line, Group 2 - Cell Voltages
    readCellVoltages();
  } else if ((buf[0] == 0x10 && buf[3] == 0x03) || BMSQuery == 5) { //[0] First Line, Group 3 - Pack Voltages
    readVoltage();
  } else if ((buf[0] == 0x10 && buf[3] == 0x04) || BMSQuery == 6) { //[0] First Line, Group 4 - Pack Temperatures
    readTemps();
  } else if ((buf[0] == 0x10 && buf[3] == 0x06) || BMSQuery == 7) { //[0] First Line, Group 6 - Cell Shunts
    readShunts();
  }
}

void requestPackInfo() {
  packInfoReading = true;
  CAN0.sendMsgBuf(0x79b, 0, sizeof(sendGroup1), sendGroup1);
}

void requestCellVoltages() {
  cellVoltagesReading = true;
  CAN0.sendMsgBuf(0x79b, 0, sizeof(sendGroup2), sendGroup2);
}

void requestVoltage() {
  voltageReading = true;
  CAN0.sendMsgBuf(0x79b, 0, sizeof(sendGroup3), sendGroup3);
}

void requestTemps() {
  tempsReading = true;
  CAN0.sendMsgBuf(0x79b, 0, sizeof(sendGroup4), sendGroup4);
}

void requestShunts() {
  shuntsReading = true;
  CAN0.sendMsgBuf(0x79b, 0, sizeof(sendGroup6), sendGroup6);
}

void readPackInfo() {
  if (buf[0] == 0x10 && buf[3] == 0x01) { // First Line
    packInfoReading = true;
    CAN0.sendMsgBuf(0x79b, 0, sizeof(sendNextLine), sendNextLine);
  } else if (buf[0] == 0x21) {
    CAN0.sendMsgBuf(0x79b, 0, sizeof(sendNextLine), sendNextLine);
  } else if (buf[0] == 0x22) {
    CAN0.sendMsgBuf(0x79b, 0, sizeof(sendNextLine), sendNextLine);
  } else if (buf[0] == 0x23) {
    LVBattery = ((float)(buf[3] * 256) + buf[4]) / 1024;
    CAN0.sendMsgBuf(0x79b, 0, sizeof(sendNextLine), sendNextLine);
  } else if (buf[0] == 0x24) {
    SOH = ((float)(buf[2] * 256) + buf[3]) / 100;
    SOC = ((float)(buf[5] * 65536) + (float)(buf[6] * 256) + buf[7]) / 10000;
    CAN0.sendMsgBuf(0x79b, 0, sizeof(sendNextLine), sendNextLine);
  } else if (buf[0] == 0x25) {
    capacity = ((float)(buf[2] * 65536) + (float)(buf[3] * 256) + buf[4]) / 10000;
  }
}

void addCell(byte highByte, byte lowByte) {
  if(cellCount < 96) {
    cellVoltage[cellCount] = ((float)(highByte * 256) + lowByte) / 1000;
    cellCount = cellCount + 1;
  }
}

void readCellVoltages() {
  int dataType = (buf[0] %2); // Two sets of data, one with split cell at the end and one at the start
  if (buf[0] == 0x10 && buf[3] == 0x02) { // First Line
    cellCount = 0;
    addCell(buf[4],buf[5]);
    addCell(buf[6],buf[7]);
    CAN0.sendMsgBuf(0x79b, 0, sizeof(sendNextLine), sendNextLine);
  } else if (dataType == 1) {
    addCell(buf[1],buf[2]);
    addCell(buf[3],buf[4]);
    addCell(buf[5],buf[6]);
    memVoltage = buf[7];
    CAN0.sendMsgBuf(0x79b, 0, sizeof(sendNextLine), sendNextLine);
  } else if (dataType == 0) {
    addCell(memVoltage,buf[1]);
    addCell(buf[2],buf[3]);
    addCell(buf[4],buf[5]);
    addCell(buf[6],buf[7]);
    CAN0.sendMsgBuf(0x79b, 0, sizeof(sendNextLine), sendNextLine);
  }
}

//This contains Max and Min Pack Voltage, I don't really need these so not coded them.
void readVoltage() {
  if (buf[0] == 0x10 && buf[3] == 0x03) { // First Line
    CAN0.sendMsgBuf(0x79b, 0, sizeof(sendNextLine), sendNextLine);
  } else if (buf[0] == 0x21) {
    CAN0.sendMsgBuf(0x79b, 0, sizeof(sendNextLine), sendNextLine);
  } else if (buf[0] == 0x22) {
    CAN0.sendMsgBuf(0x79b, 0, sizeof(sendNextLine), sendNextLine);
  } else if (buf[0] == 0x23) {
    CAN0.sendMsgBuf(0x79b, 0, sizeof(sendNextLine), sendNextLine);
  } else if (buf[0] == 0x24) {
  }
}

void readTemps() {
  if (buf[0] == 0x10 && buf[3] == 0x04) { // First Line
    temps[0] = buf[6];
    CAN0.sendMsgBuf(0x79b, 0, sizeof(sendNextLine), sendNextLine);
  } else if (buf[0] == 0x21) { // Second Line
    temps[1] = buf[2];
    temps[2] = buf[5];
    CAN0.sendMsgBuf(0x79b, 0, sizeof(sendNextLine), sendNextLine);
  } else if (buf[0] == 0x22) { // Third Line
    temps[3] = buf[1];
  }
}

void addShunts(byte data) {
  shunts[shuntCount] = data && 00001000;
  shunts[shuntCount] = data && 00000100;
  shunts[shuntCount] = data && 00000010;
  shunts[shuntCount] = data && 00000001;
  shuntCount = shuntCount + 4;
}

void readShunts() {
  if (buf[0] == 0x10 && buf[3] == 0x06) { // First Line
    shuntCount = 0;
    for(int i = 4; i < 8; i++) {
      addShunts(buf[i]);
    }
    CAN0.sendMsgBuf(0x79b, 0, sizeof(sendNextLine), sendNextLine);
  } else if (buf[0] == 0x21 || buf[0] == 0x22) {
    for(int i = 0; i < 8; i++) {
      addShunts(buf[i]);
    }
    CAN0.sendMsgBuf(0x79b, 0, sizeof(sendNextLine), sendNextLine);
  } else if (buf[0] == 0x23) {
    for(int i = 0; i < 4; i++) {
      addShunts(buf[i]);
    }
  }
}

//Use this function to output the details of the currently captured piece of CanBus data
void output() {
  Serial.print(ID,HEX); // Output HEX Header
  Serial.print("\t");
  for(int i = 0; i<len; i++) { // Output Bytes of data in Dec, Length dependant on data
    Serial.print(buf[i]);
    Serial.print("\t");
  }
  Serial.println("");
}

void outputResults() {
  for(int i = 0; i < 96; i++) { // Display Cell Voltage and Shunts
    Serial.print("Cell ");
    Serial.print(i);
    Serial.print(" ");
    Serial.print(cellVoltage[i]);
    Serial.print("v Shunt = ");
    Serial.println(shunts[i]);
    cellVoltage[i] = 0; 
    shunts[i] = false;
  }
  
  for(int i = 1; i < 5; i++) {  // Display Temps
      Serial.print("Temp ");
      Serial.print(i);
      Serial.print(" ");
      Serial.println(temps[i-1]);
      temps[i-1] = 0; 
  }
  
  Serial.print("HV Battery: "); //Display the rest of the data
  Serial.print(HVBattery);
  Serial.println("V");
  Serial.print("LV Battery: ");
  Serial.print(LVBattery);
  Serial.println("V");
  Serial.print("SOH: ");
  Serial.print(SOH);
  Serial.println("%");
  Serial.print("SOC: ");
  Serial.print(SOC);
  Serial.println("%");
  Serial.print("Capacity: ");
  Serial.print(capacity);
  Serial.println("Ah");
  Serial.print("Current: ");
  Serial.print(current);
  Serial.println("amps");
}

void loop() {
  time = millis();
  
  BMSQuery = (time / 1000) % 10; //Each Second ask a different question

  if(CAN_MSGAVAIL == CAN0.checkReceive()) { // Check to see whether data is read
    CAN0.readMsgBufID(&ID, &len, buf);    // Read data
  } else {
    ID = 0; // No data so reset ID
  }
  
  switch(BMSQuery) {
    case 1:
      if(!packInfoReading) requestPackInfo();
      break;
    case 3:
      if(!cellVoltagesReading) requestCellVoltages();
      break;
    case 5:      
      if(!voltageReading) requestVoltage();
      break;
    case 6:
      if(!tempsReading) requestTemps();
      break;
    case 7:
      if(!shuntsReading) requestShunts();
      break;
    case 8:
      if(ID == 0x1db && !HVVoltageCheck) {
        decode1DB();
        outputResults();
      }
      break;
    case 9: //resets everything, just in case packets of data are missed the program wont get stuck
      packInfoReading = false;
      cellVoltagesReading = false;
      voltageReading = false;
      tempsReading = false;
      shuntsReading = false;
      HVVoltageCheck = false;
    default:
      break;
  }
  
  if(ID == 0x7bb) { //decode response from BMS
    decode7BB();
  }
}
