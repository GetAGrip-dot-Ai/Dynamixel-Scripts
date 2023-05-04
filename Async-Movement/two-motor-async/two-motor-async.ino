/*
CMU MRSD Program: Course 16-681
Team Name: GetAGrip.AI
Team Members: Alec Trela, Jiyoon Park, Sridevi Kaza, Solomon Fenton, & Shri Ishwarya S V
Rev0: Feb. 21, 2023
Code Description: Simple Script to Actuate 2 Daisy Chained Dynamixel Motors, Asynchronously
*/



#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

// Set up the motors: got these values from the wizard
const uint8_t MX_64_ID = 2;
const uint8_t MX_28_ID = 3;
const float DX_PROTOCOL_VERSION = 1.0; // same proto for both
DynamixelShield dx1;


//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  
  // same protocol for both motors
  dx1.setPortProtocolVersion(DX_PROTOCOL_VERSION);

  // set up MX-64
  dx1.ping(MX_64_ID);
  dx1.torqueOff(MX_64_ID);
  dx1.setOperatingMode(MX_64_ID, OP_POSITION);
  dx1.torqueOn(MX_64_ID);

  // set up MX_28
  dx1.ping(MX_28_ID);
  dx1.torqueOff(MX_28_ID);
  dx1.setOperatingMode(MX_28_ID, OP_POSITION);
  dx1.torqueOn(MX_28_ID);

  dx1.begin(57600); // mx-64 baud rate

}

void loop() {
  
  dx1.setGoalPosition(MX_64_ID, 90, UNIT_DEGREE);
  delay(1000);  
  dx1.setGoalPosition(MX_64_ID, 180, UNIT_DEGREE);
  delay(1000);
  
  
  dx1.ledOn(MX_28_ID);
  dx1.setGoalPosition(MX_28_ID, 90, UNIT_DEGREE);
  delay(1000); 
  dx1.setGoalPosition(MX_28_ID, 180, UNIT_DEGREE);
  delay(1000);
  dx1.ledOff(MX_28_ID);

  dx1.ledOff(MX_64_ID);
  delay(1000);

}
