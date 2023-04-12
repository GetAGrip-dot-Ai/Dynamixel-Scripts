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

  float MX64_open = 220;
  float MX64_close = 300;

  float MX28_open = 190;
  float MX28_close = 255;

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
ros::Publisher harvest_pub("/end_effector/harvest_rsp", &harvest_rsp);

// Subscriber to see action to take
// 1 = Grip
// 2 = Cut 
// 3 = Release

void harvestCb( const std_msgs::Int8& command);

ros::Subscriber<std_msgs::Int8> command_sub("/end_effector/harvest_req", harvestCb );

// Need to track commands so we don't repeat executions
int8_t last_commmand;

// MX64 opening and closing
void openCutter(){
  dx1.setGoalPosition(MX_64_ID, MotorPos::MX64_open, UNIT_DEGREE);
}

void closeCutter(){
  dx1.setGoalPosition(MX_64_ID, MotorPos::MX64_close, UNIT_DEGREE);
}

// mx28 opening and closing
void openGripper(){
  dx1.setGoalPosition(MX_28_ID, MotorPos::MX28_open, UNIT_DEGREE);
}

void closeGripper(){
  dx1.setGoalPosition(MX_28_ID, MotorPos::MX28_close, UNIT_DEGREE);
}

// harvesting callback
void harvestCb(const std_msgs::Int8& command){

     switch(command.data){

      // Open gripper & cutter
      case 8:
        openGripper();
        openCutter();

        harvest_rsp.data = 1;
        break;

      // extract: close gripper & cut 
      case 10:

        closeGripper();

        delay(10000); // 10 second delay for debugging

        for(int i=0; i < cut_params::CUT_ATTEMPTS; i++)
        {
          openCutter();
          delay(1000);
          closeCutter();
          delay(cut_params::CUTTER_DELAY); 
        }

        harvest_rsp.data = 1;
        break;

      // open the gripper, then close both
      case 12:
        
        openGripper();

        delay(2000);

        closeGripper();
        closeCutter();          

        harvest_rsp.data = 1;
        // relevant_state = true;

        break;

      default:
        harvest_rsp.data = 0;
        break;       
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
