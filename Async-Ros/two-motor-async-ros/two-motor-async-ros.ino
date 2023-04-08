#include <DynamixelShield.h>
#include <ros.h>
#include <std_msgs/Int8.h>

// Set up the motors: got these values from the wizard
const uint8_t MX_64_ID = 1;
const uint8_t MX_28_ID = 2;
const float DX_PROTOCOL_VERSION = 1.0;  // same proto for both
DynamixelShield dx1;

// Positions to calibrate the motors to 
namespace MotorPos{

  float MX64_open = 100;
  float MX64_close = 100;

  float MX28_open = 100;
  float MX28_close = 100;

}

// params to tun for cutter
namespace cut_params{

  unsigned long CUTTER_DELAY = 3000;
  int CUT_ATTEMPTS = 3;

}

// make the node handle and get the message types
ros::NodeHandle  nh;

// Publisher to see if the result goes well
// 0 for error
// 1 for Success
std_msgs::Int8 harvest_rsp;
ros::Publisher harvest_pub("harvest_result", &harvest_rsp);

// Subscriber to see action to take
// 1 = Grip
// 2 = Cut 
// 3 = Release

void harvestCb( const std_msgs::Int8& command);

ros::Subscriber<std_msgs::Int8> command_sub("harvest_command", harvestCb );

// Need to track commands so we don't repeat executions
int8_t last_commmand;

// MX64 opening and closing
void open_cutter(){
  dx1.setGoalPosition(MX_64_ID, MotorPos:MX64_open, UNIT_DEGREE);
}

void close_cutter(){
  dx1.setGoalPosition(MX_64_ID, MotorPos:MX64_close, UNIT_DEGREE);
}

// mx28 opening and closing
void open_gripper(){
  dx1.setGoalPosition(MX_28_ID, MotorPos:MX28_open, UNIT_DEGREE);
}

void close_gripper(){
  dx1.setGoalPosition(MX_28_ID, MotorPos:MX28_close, UNIT_DEGREE);
}

// harvesting callback
void harvestCb(const std_msgs::Int8& command){

  if(command.data != last_commmand){

    switch(command.data){

      // Open gripper & cutter
      case 8:
        open_gripper()
        open_cutter()
        break;

      // extract: close gripper & cut 
      case 10:

        close_gripper();

        for(int i; i < cut_params::CUT_ATTEMPTS; i++){
          close_cutter();
          delay(cut_params::CUTTER_DELAY)
          open_cutter();
        }

        break;

      // open the gripper, then close both
      case 12:
        
        open_gripper();

        close_gripper();
        close_cutter();          

        harvest_rsp.data = 1;
        break;
      default:
        harvest_rsp.data = 0;
        break;       
    }

  }  

  harvest_pub.publish(&harvest_rsp);

}


void setup() {

  // set up the ros comms
  nh.initNode();
  Serial2.begin(57600);
  nh.getHardware()->setPort(&Serial2);
  nh.getHardware()->setBaud(57600);

  nh.advertise(harvest_pub);
  nh.subscribe(command_sub);

  //same protocol for both motors
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

  dx1.begin(57600);  // mx family baud rate

}

void loop()
{
  nh.spinOnce();
  delay(10);
}
