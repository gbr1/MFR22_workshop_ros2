/*
 * The MIT License
 *
 * Copyright (c) 2022 Giovanni di Dio Bruno https://gbr1.github.io
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

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

