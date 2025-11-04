/* Follower_Arduino.ino - BILK Medical Robotics Follower */
/* Hardware: Arduino Mega + VNH5019 Shield + L298N Driver */
/* Motors: 3x Pololu + 1x Servo */

#include <Arduino.h>
#include "protocol.h"
#include <Servo.h>

static const uint8_t PRE[4]={'B','I','L','K'};
enum Mode: uint8_t { IDLE=0, FOLLOW=1, HOLD=2, SHUTDOWN=3 };
Mode mode=IDLE;

// Motor control variables
float q_meas[4]={0}, q_ref[4]={0}, q_cmd[4]={0}, e_int[4]={0}, last_e[4]={0};
float Kp[4]={5,5,5,2}, Ki[4]={0.3,0.2,0.2,0}, Kd[4]={0.05,0.05,0.05,0};

// Watchdog
const uint32_t kWatchdogUs=100000; 
uint32_t last_rx_us=0;

// Motor driver pins
// VNH5019 Shield (Motors 1 & 2)
#define VNH1_PWM 9    // Motor 1 PWM
#define VNH1_INA 8    // Motor 1 Direction A
#define VNH1_INB 7    // Motor 1 Direction B
#define VNH2_PWM 10   // Motor 2 PWM  
#define VNH2_INA 12   // Motor 2 Direction A
#define VNH2_INB 11   // Motor 2 Direction B

// L298N Driver (Motor 3 + Servo)
#define L298N_ENA 5   // Motor 3 Enable
#define L298N_IN1 4   // Motor 3 Direction 1
#define L298N_IN2 3   // Motor 3 Direction 2
#define SERVO_PIN 6   // Servo control pin

// Servo object
Servo gripper_servo;

// CRC-16 calculation
uint16_t crc16(const uint8_t* d,size_t n){ 
    uint16_t c=0xFFFF; 
    for(size_t i=0;i<n;i++){ 
        c^=(uint16_t)d[i]<<8; 
        for(uint8_t b=0;b<8;b++){ 
            c=(c&0x8000)?((c<<1)^0x1021):(c<<1);
        } 
    } 
    return c; 
}

// Motor control functions
void apply_motors(float u1, float u2, float u3, float servo_angle) {
    // Motor 1 (VNH5019) - Joint 1
    if (abs(u1) < 0.01) {
        digitalWrite(VNH1_INA, LOW);
        digitalWrite(VNH1_INB, LOW);
        analogWrite(VNH1_PWM, 0);
    } else {
        digitalWrite(VNH1_INA, u1 > 0 ? HIGH : LOW);
        digitalWrite(VNH1_INB, u1 > 0 ? LOW : HIGH);
        analogWrite(VNH1_PWM, (int)(abs(u1) * 255));
    }
    
    // Motor 2 (VNH5019) - Joint 2
    if (abs(u2) < 0.01) {
        digitalWrite(VNH2_INA, LOW);
        digitalWrite(VNH2_INB, LOW);
        analogWrite(VNH2_PWM, 0);
    } else {
        digitalWrite(VNH2_INA, u2 > 0 ? HIGH : LOW);
        digitalWrite(VNH2_INB, u2 > 0 ? LOW : HIGH);
        analogWrite(VNH2_PWM, (int)(abs(u2) * 255));
    }
    
    // Motor 3 (L298N) - Joint 3
    if (abs(u3) < 0.01) {
        digitalWrite(L298N_IN1, LOW);
        digitalWrite(L298N_IN2, LOW);
        analogWrite(L298N_ENA, 0);
    } else {
        digitalWrite(L298N_IN1, u3 > 0 ? HIGH : LOW);
        digitalWrite(L298N_IN2, u3 > 0 ? LOW : HIGH);
        analogWrite(L298N_ENA, (int)(abs(u3) * 255));
    }
    
    // Servo (Gripper) - Joint 4
    int servo_pos = map(constrain(servo_angle, -1.0, 1.0), -1.0, 1.0, 0, 180);
    gripper_servo.write(servo_pos);
}
void parse_serial(){
  static uint8_t rx[256]; static size_t n=0;
  while(Serial.available()){
    uint8_t b=Serial.read(); if(n<sizeof(rx)) rx[n++]=b;
    while(n>=4 && (rx[0]!='B'||rx[1]!='I'||rx[2]!='L'||rx[3]!='K')){ for(size_t i=1;i<n;i++) rx[i-1]=rx[i]; n--; }
    if(n>=8){
      uint8_t ver=rx[4], msg=rx[5]; uint16_t plen=(uint16_t)rx[6] | ((uint16_t)rx[7]<<8);
      if(n>=8+plen+2){
        uint16_t cr=crc16(&rx[4],2+2+plen); uint16_t rxcr=(uint16_t)rx[8+plen] | ((uint16_t)rx[8+plen+1]<<8);
        if(cr==rxcr){
          const uint8_t* p=&rx[8];
          if(msg==BILK_LeaderState){
            uint32_t t_us=(uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
            memcpy(q_ref, p+4, 16);
            last_rx_us=micros(); if(mode!=HOLD) mode=FOLLOW;
            // optional: buttons at p[36]
          }else if(msg==BILK_CmdSetMode){
            mode=(Mode)p[0]; last_rx_us=micros();
          }
        }
        size_t c=8+plen+2; for(size_t i=c;i<n;i++) rx[i-c]=rx[i]; n-=c;
      }
    }
  }
}
void setup(){ 
    Serial.begin(2000000);
    
    // Initialize motor driver pins
    pinMode(VNH1_INA, OUTPUT);
    pinMode(VNH1_INB, OUTPUT);
    pinMode(VNH1_PWM, OUTPUT);
    pinMode(VNH2_INA, OUTPUT);
    pinMode(VNH2_INB, OUTPUT);
    pinMode(VNH2_PWM, OUTPUT);
    pinMode(L298N_IN1, OUTPUT);
    pinMode(L298N_IN2, OUTPUT);
    pinMode(L298N_ENA, OUTPUT);
    
    // Initialize servo
    gripper_servo.attach(SERVO_PIN);
    gripper_servo.write(90); // Center position
    
    // Initialize motor drivers to safe state
    digitalWrite(VNH1_INA, LOW);
    digitalWrite(VNH1_INB, LOW);
    digitalWrite(VNH2_INA, LOW);
    digitalWrite(VNH2_INB, LOW);
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, LOW);
    
    Serial.println("BILK Follower Ready - VNH5019 + L298N + Servo");
}

void loop(){
    static uint32_t t0=micros(); 
    parse_serial(); 
    uint32_t t=micros(); 
    float dt=(t-t0)*1e-6f; 
    if(dt<=0) dt=1e-3f; 
    t0=t;
    
    if((uint32_t)(t-last_rx_us)>kWatchdogUs) mode=HOLD;
    
    // Simple prefilter
    float a=dt/(0.05f+dt); 
    for(int i=0;i<4;i++) q_cmd[i]=(1.0f-a)*q_cmd[i]+a*q_ref[i];
    
    if(mode==FOLLOW || mode==HOLD){
        float u[3]={0};
        float servo_angle = 0;
        
        // PID control for first 3 joints (motors)
        for(int i=0;i<3;i++){
            float e=q_cmd[i]-q_meas[i]; 
            e_int[i]+=e*dt; 
            float de=(e-last_e[i])/dt; 
            last_e[i]=e;
            float ui=Kp[i]*e + Ki[i]*e_int[i] + Kd[i]*de; 
            if(ui>1)ui=1; 
            if(ui<-1)ui=-1; 
            u[i]=ui;
        }
        
        // Servo control for joint 4 (gripper)
        servo_angle = q_cmd[3]; // Direct mapping for servo
        
        if(mode==HOLD){ 
            for(int i=0;i<3;i++) q_cmd[i]=0.99f*q_cmd[i]+0.01f*q_meas[i]; 
            servo_angle = 0; // Center servo in HOLD mode
        }
        
        apply_motors(u[0], u[1], u[2], servo_angle);
    }
}
