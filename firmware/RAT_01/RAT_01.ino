#include "MAL.h"
#include "ucPack.h"

// interface for mkr motor carrier
MAL robot;

// buffer
ucPack packeter(100);
char c;
uint8_t recv_c;
uint8_t dim=0;


unsigned long timer_motors = 0;
unsigned long timer_joints = 0;
unsigned long timer_battery = 0;
unsigned long timer_timeout = 0;
unsigned long timer_led = 0;


unsigned long deltaT = 0;

float left_speed=0.0;
float right_speed=0.0;
float tmp1,tmp2;

bool robot_enabled=false;



void setup() 
{
  //Serial port initialization
  Serial.begin(115200);
  //while (!Serial);

  robot.init();
  timer_motors = millis();
  timer_joints = millis();
  timer_battery = millis();
  timer_timeout = millis();
  timer_led = millis();
}


void loop() {
  
  //motors control updates
  deltaT=millis()-timer_motors;
  if (deltaT > 20) {
    robot.update(deltaT);
    timer_motors = millis();  
  }
  if (robot_enabled){
    //send joints
    if (millis()-timer_joints>20){
      dim = packeter.packetC4F('j', float(robot.getRadSLeft()), float(robot.getRadSRight()), 0.0, 0.0); // max four wheels
      Serial.write(packeter.msg, dim);
      timer_joints = millis();
    }

    //send battery
    if (millis()-timer_battery>1000){
      dim = packeter.packetC1F('b', robot.getVoltage());
      Serial.write(packeter.msg, dim);
      timer_battery = millis();
    }

  }
  
  //led blinking
  if ((millis()-timer_led)>500){
    robot.led_toggle();
    timer_led=millis();
  }
  

  //timeout for MKR motor carrier
  if ((millis()-timer_timeout)>5){
    robot.keep_alive();
    timer_timeout=millis();
  }

  //check data
  while(Serial.available()>0){
    packeter.buffer.push(Serial.read());
  }

  while(packeter.checkPayload()){
    c=packeter.payloadTop();

    // enable robot
    if (c=='E'){
      float timeout_tmp;
      packeter.unpacketC1F(recv_c, timeout_tmp);
      dim = packeter.packetC1F('e', timeout_tmp);
      Serial.write(packeter.msg, dim);
      robot_enabled=true;
    }
        
    // update joints references
    if (c=='J'){
      packeter.unpacketC4F(recv_c, left_speed, right_speed, tmp1, tmp2); // max 4 wheels
      dim = packeter.packetC1F('x', 0.0);
      Serial.write(packeter.msg, dim);
      robot.setRadS(left_speed, right_speed);
    }

    // stop the robot
    if (c=='S'){
      dim = packeter.packetC1F('s', 0.0);
      Serial.write(packeter.msg, dim);
      robot.stop();
      robot_enabled=false;
    }

    // set gyro scale if imu is connected
    if (c=='G'){
      float accelerometer_scale, gyro_scale;
      packeter.unpacketC2F(recv_c, accelerometer_scale, gyro_scale);
      // acknowledge ros
      dim = packeter.packetC2F('g', accelerometer_scale, gyro_scale);
      Serial.write(packeter.msg, dim);
    }
  }
}

