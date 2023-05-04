/*
CMU MRSD Program: Course 16-681
Team Name: GetAGrip.AI
Team Members: Alec Trela, Jiyoon Park, Sridevi Kaza, Solomon Fenton, & Shri Ishwarya S V
Rev0: April. 5, 2023
Code Description: Debugging script used to factory reset and change the ID of a dynamixel motor, with UART soft-serial verification
*/

#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#endif

#define TIMEOUT 10    //default communication timeout 10ms

// WHAT YOU WANT TO CHANGE THE ID TO!!
int CHANGE_ID = 2;

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 1.0;

bool ret = false;

DynamixelShield dxl;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);   //Set debugging port baudrate to 115200bps
  while(!DEBUG_SERIAL);         //Wait until the serial port for terminal is opened
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
}

void loop() {
  // put your main code here, to run repeatedly:
  ret = false;

  int ping_success;
  int i=0;
  for(i; i<10; i++){
    ping_success = dxl.ping(i);
    String message = "Attempted Ping Id " + String(i) + ": " + String(ping_success);
    DEBUG_SERIAL.println(message);
    if(ping_success){break;}
    delay(1000);
  }

  if(ping_success){
    ret = dxl.factoryReset(i, 0xFF);
  }

  if(ret) {
    DEBUG_SERIAL.println("factory reset succeeded!");

    delay(1000);

    if(dxl.setID(i, CHANGE_ID)){
      String message = "id change succeeded, id is now: " + String(CHANGE_ID);
      DEBUG_SERIAL.println(message);
      dxl.ledOn(CHANGE_ID);
      delay(2000);
      dxl.ledOff(CHANGE_ID);
    }
    else{
      String message = "id change failed, id is still: " + String(i);
      DEBUG_SERIAL.println(message);
    }

  } else {
    DEBUG_SERIAL.println("factory reset failed!");
  }

  delay(1000);
}