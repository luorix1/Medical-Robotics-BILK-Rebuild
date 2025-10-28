/* Follower_Arduino.ino */
#include <Arduino.h>
#include "protocol.h"
static const uint8_t PRE[4]={'B','I','L','K'};
enum Mode: uint8_t { IDLE=0, FOLLOW=1, HOLD=2, SHUTDOWN=3 };
Mode mode=IDLE;
float q_meas[4]={0}, q_ref[4]={0}, q_cmd[4]={0}, e_int[4]={0}, last_e[4]={0};
float Kp[4]={5,5,5,2}, Ki[4]={0.3,0.2,0.2,0}, Kd[4]={0.05,0.05,0.05,0};
const uint32_t kWatchdogUs=100000; uint32_t last_rx_us=0;
uint16_t crc16(const uint8_t* d,size_t n){ uint16_t c=0xFFFF; for(size_t i=0;i<n;i++){ c^=(uint16_t)d[i]<<8; for(uint8_t b=0;b<8;b++){ c=(c&0x8000)?((c<<1)^0x1021):(c<<1);} } return c; }
void apply_motors(float u1,float u2,float u3){ /* TODO: map to your PWM/DIR pins and motor driver */ }
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
void setup(){ Serial.begin(2000000); }
void loop(){
  static uint32_t t0=micros(); parse_serial(); uint32_t t=micros(); float dt=(t-t0)*1e-6f; if(dt<=0) dt=1e-3f; t0=t;
  if((uint32_t)(t-last_rx_us)>kWatchdogUs) mode=HOLD;
  // simple prefilter
  float a=dt/(0.05f+dt); for(int i=0;i<4;i++) q_cmd[i]=(1.0f-a)*q_cmd[i]+a*q_ref[i];
  if(mode==FOLLOW || mode==HOLD){
    float u[3]={0};
    for(int i=0;i<3;i++){
      float e=q_cmd[i]-q_meas[i]; e_int[i]+=e*dt; float de=(e-last_e[i])/dt; last_e[i]=e;
      float ui=Kp[i]*e + Ki[i]*e_int[i] + Kd[i]*de; if(ui>1)ui=1; if(ui<-1)ui=-1; u[i]=ui;
    }
    if(mode==HOLD){ for(int i=0;i<3;i++) q_cmd[i]=0.99f*q_cmd[i]+0.01f*q_meas[i]; }
    apply_motors(u[0],u[1],u[2]);
  }
}
