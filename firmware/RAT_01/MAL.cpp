#include "api/Common.h"
#include "variant.h"
#include "ArduinoMotorCarrier.h"
#include "MAL.h"

MAL::MAL(){
  kp=25.0;
  ki=5.0;

  errorL=0.0;
  errorR=0.0;

  error_sumL=0.0;
  error_sumR=0.0;

  referenceL=0.0;
  referenceR=0.0;

  controlR=0.0;
  controlL=0.0;

  led_status=false;
  pinMode(LED_BUILTIN,OUTPUT);
  led_off();
}

void MAL::init(){
  if (!controller.begin()){
    while(1);
  }
  controller.reboot();
  delay(500);
  M1.setDuty(0);
  M2.setDuty(0);
  M3.setDuty(0);
  M4.setDuty(0);

  encoder1.resetCounter(0);
  encoder2.resetCounter(0); 
}

void MAL::stop(){
  M1.setDuty(0);
  M2.setDuty(0);
  errorL=0.0;
  errorR=0.0;

  error_sumL=0.0;
  error_sumR=0.0;

  referenceL=0.0;
  referenceR=0.0;

  controlR=0.0;
  controlL=0.0;

}

void MAL::set(float left, float right){
  M1.setDuty(left);
  M2.setDuty(right);
}

void MAL::update(float deltaT){
  countL = getRevolutionsL()*1000.0/float(deltaT);
  countR = getRevolutionsR()*1000.0/float(deltaT);
  resetEncoders();
  errorL = referenceL-countL;
  errorR = referenceR-countR;
  error_sumL +=errorL;
  error_sumR +=errorR;
  error_sumL = checkLimit(error_sumL);
  error_sumR = checkLimit(error_sumR);

  controlL = kp*(referenceL-countL)+checkLimit(ki*error_sumL);
  controlR = kp*(referenceR-countR)+checkLimit(ki*error_sumR);

  controlL=checkLimit(controlL);
  controlR=checkLimit(controlR);

  set(int(controlL),int(controlR)); 
}

void MAL::keep_alive(){
  controller.ping();
}

float MAL::getVoltage(){
  return (float)battery.getRaw()/77;
}

float MAL::getRevolutionsL(){
  return (float)(encoder1.getRawCount())/MOTOR_RATIO;
}

float MAL::getRevolutionsR(){
  return (float)(encoder2.getRawCount())/MOTOR_RATIO;
}

void MAL::setReferences(float l, float r){
  referenceL=l;
  referenceR=r;
}

void MAL::resetEncoders(){
  encoder1.resetCounter(0);
  encoder2.resetCounter(0); 
}

float MAL::checkLimit(float x){
  if (x>100){
    x=100.0;
  }
  if (x<-100){
    x=-100.0;
  }
  return x;
}

void MAL::setRadS(float l, float r){
  l=l/(2*PI);
  r=r/(2*PI);
  setReferences(l,r);
}

float MAL::getRadSLeft(){
  return countL*(2*PI);
}

float MAL::getRadSRight(){
  return countR*(2*PI);
}

float MAL::getReferenceRadSLeft(){
  return referenceL*(2*PI);
}

float MAL::getReferenceRadSRight(){
  return referenceR*(2*PI);
}