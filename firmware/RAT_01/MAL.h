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

#include "variant.h"
#ifndef __MAL_H__
#define __MAL_H__

#include <ArduinoMotorCarrier.h>
#define INTERRUPT_PIN 6


const float MOTOR_CPR = 44.0;
const float MOTOR_GR = 54.225;//56.69;//46.8;
const float MOTOR_RATIO = MOTOR_CPR * MOTOR_GR; 
const float MOTOR_LIMIT = 34.5;

class MAL{
  private:
    bool led_status;
    float countL;
    float countR;

    float referenceL;
    float referenceR;

    float errorL;
    float errorR;

    float error_sumL;
    float error_sumR;
    
    float kp;
    float ki;

    float controlL;
    float controlR;
    
    float checkLimit(float x);
    
  public:
         MAL();
    void init();
    void stop();
    void set(float left, float right);
    void keep_alive();
    float getVoltage();
    void update(float deltaT=10.0);
    float getRevolutionsL();
    float getRevolutionsR();
    void resetEncoders();
    void setReferences(float l, float r);
    float getControlLeft(){return controlL;}
    float getControlRight(){return controlR;}
    void setRadS(float l, float r);
    float getRadSLeft();
    float getRadSRight();
    float getReferenceRadSLeft();
    float getReferenceRadSRight();

    void led_on(){led_status=true; digitalWrite(LED_BUILTIN,led_status);}
    void led_off(){led_status=false; digitalWrite(LED_BUILTIN,led_status);}
    void led_toggle(){led_status=!led_status; digitalWrite(LED_BUILTIN,led_status);}


};


#endif