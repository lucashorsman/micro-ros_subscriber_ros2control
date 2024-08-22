#include <ODriveCAN.h>

// CAN bus baudrate. Make sure this matches for every device on the bus

#define CAN_BAUDRATE 250000


// ODrive node_id for odrv0

#define ODRV0_NODE_ID 0
#define IS_TEENSY_BUILTIN // Teensy boards with built-in CAN interface (e.g. Teensy 4.1). See below to select which interface to use.
#ifdef IS_TEENSY_BUILTIN

// See https://github.com/tonton81/FlexCAN_T4

// clone https://github.com/tonton81/FlexCAN_T4.git into /src

#include <FlexCAN_T4.h>

#include "ODriveFlexCAN.hpp"

struct ODriveStatus; // hack to prevent teensy compile error

#endif // IS_TEENSY_BUILTIN

/* Teensy */


#ifdef IS_TEENSY_BUILTIN


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;


bool setupCan() {

  can_intf.begin();

  can_intf.setBaudRate(CAN_BAUDRATE);

  can_intf.setMaxMB(16);

  can_intf.enableFIFO();

  can_intf.enableFIFOInterrupt();

  can_intf.onReceive(onCanMessage);

  return true;

}


#endif // IS_TEENSY_BUILTIN


#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <control_msgs/msg/dynamic_joint_state.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/detail/string__struct.h>
#include <std_msgs/msg/float64.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
sensor_msgs__msg__JointState msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
static micro_ros_utilities_memory_conf_t conf = {0};

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}



// Instantiate ODrive objects

ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID

ODriveCAN* odrives[] = {&odrv0}; // Make sure all ODriveCAN instances are accounted for here


struct ODriveUserData {

  Heartbeat_msg_t last_heartbeat;

  bool received_heartbeat = false;

  Get_Encoder_Estimates_msg_t last_feedback;

  bool received_feedback = false;

};


// Keep some application-specific user data for every ODrive.

ODriveUserData odrv0_user_data;


// Called every time a Heartbeat message arrives from the ODrive

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {

  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);

  odrv_user_data->last_heartbeat = msg;

  odrv_user_data->received_heartbeat = true;

}


// Called every time a feedback message arrives from the ODrive

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {

  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);

  odrv_user_data->last_feedback = msg;

  odrv_user_data->received_feedback = true;

}


// Called for every message that arrives on the CAN bus

void onCanMessage(const CanMsg& msg) {

  for (auto odrive: odrives) {

    onReceive(msg, *odrive);

  }

}





void error_loop(){

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

}

//twist message cb
void subscription_callback(const void *msgin) {
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
  float vel = msg->velocity.data[0];
  
digitalWrite(LED_PIN, (vel < 0 ) ? LOW : HIGH);
Serial.println(vel);
   odrv0.setVelocity(
    vel
  ); 
}

void setup() {


  delay(200);

  Serial.begin(115200);

  Serial.println("Starting ODriveCAN demo");


  // Regiter callbacks for the heartbeat and encoder feedback messages

  odrv0.onFeedback(onFeedback, &odrv0_user_data);

  odrv0.onStatus(onHeartbeat, &odrv0_user_data);


  // Configure and initialize the CAN bus interface. This function depends on

  // your hardware and the CAN stack that you're using.

  if (!setupCan()) {

    Serial.println("CAN failed to initialize: reset required");

    while (true); // spin indefinitely

  }


  Serial.println("Waiting for ODrive...");

  while (!odrv0_user_data.received_heartbeat) {

    pumpEvents(can_intf);

    delay(100);

  }



  Serial.println("found ODrive");


  // request bus voltage and current (1sec timeout)

  Serial.println("attempting to read bus voltage and current");

  Get_Bus_Voltage_Current_msg_t vbus;

  if (!odrv0.request(vbus, 1)) {

    Serial.println("vbus request failed!");

    while (true); // spin indefinitely

  }


  Serial.print("DC voltage [V]: ");

  Serial.println(vbus.Bus_Voltage);

  Serial.print("DC current [A]: ");

  Serial.println(vbus.Bus_Current);


  Serial.println("Enabling closed loop control...");

  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {

    odrv0.clearErrors();

    delay(1);

    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
odrv0.setControllerMode(2,2);

    // Pump events for 150ms. This delay is needed for two reasons;

    // 1. If there is an error condition, such as missing DC power, the ODrive might

    //    briefly attempt to enter CLOSED_LOOP_state, so we can't rely

    //    on the first heartbeat response, so we want to receive at least two

    //    heartbeats (100ms default interval).

    // 2. If the bus is congested, the setState command won't get through

    //    immediately but can be delayed.

    for (int i = 0; i < 15; ++i) {

      delay(10);
Serial.println(i);
      pumpEvents(can_intf);

    }

  }


  Serial.println("ODrive running!");


/*------------------*/




odrv0.setVelocity(0);

  
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"));

  
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),  
      "/publisher"));





//mem allo
bool success = micro_ros_utilities_create_message_memory(
  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
  &msg,
  conf
);
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

}

void loop() {
delay(100);
RCCHECK(rclc_executor_spin(&executor));

  
}
