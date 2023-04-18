#include <DynamixelShield.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <string.h>

// Set up the motors: got these values from the wizard
const uint8_t MX_64_ID = 2;
const uint8_t MX_28_ID = 3;
const float DX_PROTOCOL_VERSION = 1.0;  // same proto for both
DynamixelShield dx1;

// Positions to calibrate the motors to 
namespace MotorPos{

  float MX64_open = 180;
  float MX64_close = 240;

  float MX28_open = 140;
  float MX28_close = 194;

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
std_msgs::Int16 harvest_rsp;
ros::Publisher harvest_pub("/end_effector/harvest_rsp", &harvest_rsp);

// create a publisher to publish the state
std_msgs::Int16 state_msg;
ros::Publisher state_pub("/end_effector/state", &state_msg);


// Subscriber to see action to take
// 1 = Grip
// 2 = Cut 
// 3 = Release

void harvestCb( const std_msgs::Int16& command);

ros::Subscriber<std_msgs::Int16> command_sub("/end_effector/harvest_req", harvestCb );

// Need to track commands so we don't repeat executions
int16_t last_commmand;

// MX64 opening and closing
int openCutter(){

  // sub 1 to the cutter closes state
  update_state(-1);

  if(dx1.setGoalPosition(MX_64_ID, MotorPos::MX64_open, UNIT_DEGREE)){
    nh.loginfo("Cutter Opening Command Sent");
    return 1;
  }
  else{
    nh.logwarn("Cutter Opening Command Not Sent");
    return 0;
   }  
}

int closeCutter(){

  // add 1 to the cutter closes state
  update_state(1);   
           
  if(dx1.setGoalPosition(MX_64_ID, MotorPos::MX64_close, UNIT_DEGREE)){
    nh.loginfo("Cuttered Closing Command Sent");
    return 1;    
  }
  else{
    nh.logwarn("Cutter Closing Command Not Sent");
    return 0;
  } 
}

// mx28 opening and closing
int openGripper(){

  // sub 10 to the gripper closes state
  update_state(-10); 

  if(dx1.setGoalPosition(MX_28_ID, MotorPos::MX28_open, UNIT_DEGREE)){
    nh.loginfo("Gripper Opening Command Sent");  
    return 1; 
  }
  else{
    nh.logwarn("Gripper Opening Command Not Sent");
    return 0;
  }
}

int closeGripper(){

  // adding ten changes from open to close
  update_state(10);

  if(dx1.setGoalPosition(MX_28_ID, MotorPos::MX28_close, UNIT_DEGREE)){
    nh.loginfo("Gripper Closing Command Sent");
    return 1;
  }
  else{
    nh.logwarn("Gripper Closing Command Not Sent");
    return 0;
  }
}

// figure out if the motors have been opened properly
int checkOpenGripper(){

  int threshold = 5;
  int current_pos = dx1.getPresentPosition(MX_28_ID, UNIT_DEGREE);

  int diff = abs(current_pos - MotorPos::MX28_open);

  if(diff>threshold){
    nh.logwarn("Gripper Not Opened");
    return 0;
    }
  else{
    nh.loginfo("Gripper Opened");
    return 1;
    }

}

// figure out if the motors have been opened properly
int checkOpenCutter(){

  int threshold = 5;
  int current_pos = dx1.getPresentPosition(MX_64_ID, UNIT_DEGREE);

  int diff = abs(current_pos - MotorPos::MX64_open);

  if(diff>threshold){
    nh.logwarn("Cutter Not Opened");
    return 0;
    }
  else{
    nh.loginfo("Cutter Opened");
    return 1;
    }

}

// motor resets
void resetMotors(int attempts)
{

  // for all motors (id 2, 3)
  for(int ID=2; ID<4; ID++)
  {
    // set the name for comms
    String motor_name;
    switch(ID){
        case MX_64_ID:
          motor_name = "MX-64";
          break;
        case MX_28_ID:
          motor_name = "MX-28";
          break;
    }
        
    // try to reset the motor 10 times and see if it works
    bool reset_success=false;
    for(int j = 0; j < attempts; j++)
      {
      reset_success = dx1.factoryReset(ID, 0xFF, 10); // 10 second timeout
      delay(50);
      
      if(reset_success){
        nh.loginfo((String("Factory Reset of Motor ID ") + motor_name + String(" Successful")).c_str());
        break;}
      nh.logwarn((String("Factory Reset of Motor ID ") + motor_name + String(" Failed")).c_str());
      }
    
    // after tring to reset, update the outcome then try to reset the id
    delay(500);

    // after trying to reset, need to reset the ID
    bool reset_id=false;
    if(reset_success)
      {
        for(int j = 0; j < attempts; j++)
          {
          reset_id = dx1.setID(1, ID);
          delay(50);
          if(reset_id){
            nh.loginfo((String("ID Reset of ") + motor_name + String(" Successful")).c_str());
            break;
          }
          nh.logwarn((String("ID Reset of ") + motor_name + String(" Failed")).c_str());
          }
      }

    }
  // Ping the motors as quick check
  ping_motors();      
}

void ping_motors(){

  if(dx1.ping(MX_64_ID)){
      nh.loginfo((String("Pinning of ID ") + String(MX_64_ID)+String(" Successful")).c_str()); 
  }
  else{
    nh.logwarn((String("Pinning of ID ") + String(MX_64_ID)+String(" Failed")).c_str());
  }
  
  if(dx1.ping(MX_28_ID)){
      nh.loginfo((String("Pinning of ID ") + String(MX_28_ID)+String(" Successful")).c_str()); 
    }
  else{
    nh.logwarn((String("Pinning of ID ") + String(MX_28_ID)+String(" Failed")).c_str());
  }  
}

void update_state(int change_val){
  // State Table:
  // 11 = Open Gripper || Open Cutter
  // 12 = Open Gripper || Close Cutter
  // 21 = Close Gripper || Open Cutter
  // 22 = Close Gripper || Close Cutter
  
  switch(state_msg.data){
    case 11: 
      // open gripper -> close gripper OR open cutter -> close cutter
      if(change_val == 10 || change_val == 1){
        state_msg.data += change_val;
      }
      break;
    case 12:
      // open gripper -> close gripper OR close cutter -> open cutter
      if(change_val == 10 || change_val == -1){
        state_msg.data += change_val;
      }
      break;
    case 21:
      // close gripper -> open gripper OR open cutter -> close cutter
      if(change_val == -10 || change_val == 1){
        state_msg.data += change_val;
      }
      break;
    case 22:
      // close gripper -> open gripper OR close cutter -> open cutter
      if(change_val == -10 || change_val == -1){
        state_msg.data += change_val;
      }  
      break;
  }
}


// harvesting callback
void harvestCb(const std_msgs::Int16& command){

     switch(command.data){
      // Open gripper & cutter
      case 4:

        // checking if the command sent is successful
        if(!openGripper()){
          harvest_rsp.data = 0;
          break;
        }  

        // seeing if it is actually open
        if(!checkOpenGripper()){
          harvest_rsp.data = 0;
          break;
        }  
        
        delay(500);

        // checking if the command sent is successful
        if(!openCutter()){
          harvest_rsp.data = 0;
          break;
        } 

        // seeing if it is actually open
        if(!checkOpenGripper()){
          harvest_rsp.data = 0;
          break;
        }  
        
        harvest_rsp.data = 1;
        break;
      // extract: close gripper & cut 
      case 6:

        if(!closeGripper()){
          harvest_rsp.data = 0;
          break;
        }

        // attempt some number of times
        for(int i=0; i < cut_params::CUT_ATTEMPTS; i++)
        {
          if(!closeCutter()){
            harvest_rsp.data = 0;
            break;
          }
          
          delay(cut_params::CUTTER_DELAY); 

          if(!openCutter()){
            harvest_rsp.data = 0;
            break;
          }

          delay(1000);
        
        }

        harvest_rsp.data = 1;
        break;

      // open the gripper, then close both
      case 8:
        
        if(!openGripper()){
          harvest_rsp.data = 0;
          break;
        } 

        // CRUCIAL: IF THE GRIPPER IS GRIPPED BUT NOT DROPPED
        if(!checkOpenGripper()){
          harvest_rsp.data = 0;
          break;
        } 
        
        delay(500);

        if(!closeGripper()){
          harvest_rsp.data = 0;
          break;
        }

        if(!closeCutter()){
            harvest_rsp.data = 0;
            break;
        }

        harvest_rsp.data = 1;

        break;

      // factory reset
      case 20:
        harvest_rsp.data = 1;
        resetMotors(10); // 10 attempts to reset
        break;                   
      
      case 21:
        ping_motors();
        harvest_rsp.data = 1;
        break;

      // other base functionalities
      case 22: // opening gripper
         if(!openGripper()){
          harvest_rsp.data = 0;
          break;
        } 
        harvest_rsp.data = 1;
        break;

      case 23: // opening cutter
        if(!openCutter()){
            harvest_rsp.data = 0;
            break;
        } 
        harvest_rsp.data = 1;
        break;
      
      case 24:
        if(!closeGripper()){ // closing gripper
          harvest_rsp.data = 0;
          break;
        } 
        harvest_rsp.data = 1;
        break;
      
      case 25: // closing cutter
        if(!closeCutter()){
          harvest_rsp.data = 0;
          break;
        } 
        harvest_rsp.data = 1;
        break;

      default:
        nh.logwarn("You have given an incorrect request to the end-effector");
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
  nh.advertise(state_pub);

  //same protocol for both motors
  dx1.setPortProtocolVersion(DX_PROTOCOL_VERSION);

  dx1.begin(57600);  // mx family baud rate

  delay(500);

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

  // close the gripper and cutter 
  int start = closeGripper();
  start = closeCutter();

  state_msg.data = 22;
      
}

void loop()
{
  nh.spinOnce();
  delay(10);
  state_pub.publish(&state_msg);
}

