#include <Wire.h>
#include "soc/gpio_struct.h"
#define I2C_SLAVE_ADDR 0x20
//inizializzazione pin di flag
#define flag    21
#define flag2   18
#define poweron 19
//inizializzazione pin dei finecorsa
#define fcx     34
#define fcy     35
#define fcz1    39
#define fcz2    36
#define fcpz     4
//inizializzazione pin dei MOTORI
#define enable  13
#define stepx   17
#define dirx    16
#define stepy   33
#define diry    32
#define stepz1  26
#define dirz1   25
#define stepz2  14
#define dirz2   27
//inizializzazione pin dei motori
#define sda      23
#define scl      22
//NEGAZIONE OUTPUT (1 non negato, 0 negato)
bool  dir=0;
bool step=1;
//PARAMETRI MOTORE
float mpr=2;
float ppr=800;
//soglia massima di delay tra un passo e l'altro, in microsecondi
int soglia_max = 350;
//variabili usate dal programma
typedef struct{
  uint8_t  id;
  float    a;
  float    b;
  float    c;
  float    d;
} Packet;
typedef struct{
  int      v;
  float    d;
} Packet2;
volatile Packet  rx;
volatile Packet2 tx;
volatile bool newaction=0;
bool bkm=1, statox=0, statoy=0, statoz1=0, statoz2=0;
//FUNZIONI PRELIMINARI
inline void fastWrite(uint8_t pin, bool state){
  (pin < 32) ? fastWritelow(pin, state) : fastWritehigh(pin, state);
}
inline void fastWritelow(uint8_t pin, bool state){
  uint32_t mask = (1UL << pin);
  GPIO.out_w1ts = mask & ( -(uint32_t)state);
  GPIO.out_w1tc = mask & (-(uint32_t)!state);
}
inline void fastWritehigh(uint8_t pin, bool state){
  uint32_t mask = (1UL << (pin - 32));
  GPIO.out1_w1ts.val = mask & (-(uint32_t)state);
  GPIO.out1_w1tc.val = mask & (-(uint32_t)!state);
}
inline bool fastRead(uint8_t pin){
  return (pin < 32) ? fastReadLow(pin) : fastReadHigh(pin);
}
inline bool fastReadLow(uint8_t pin){
  return (GPIO.in & (1UL << pin)) != 0;
}
inline bool fastReadHigh(uint8_t pin){
  return (GPIO.in1.data & (1UL << (pin - 32))) != 0;
}
void exp_emulator_rx(int lunghezza) {
  if(lunghezza == sizeof(Packet)){
    Wire.readBytes((uint8_t*)&rx, sizeof(Packet));
    newaction = 1;
  }else{
    rx.id=0;
    newaction = 1;
  }
}
void exp_emulator_tx() {
  Wire.write((uint8_t*)&tx, sizeof(Packet2));
}
void blkmot(bool value) {
  fastWrite(enable, !value);
  bkm=!value;
}
//FUNZIONI CLASSICHE
void setup() {
  pinMode(flag   , OUTPUT);
  fastWrite(flag,1);
  Serial.begin(115200);
  Wire.setPins(sda, scl);
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(exp_emulator_rx);
  Wire.onRequest(exp_emulator_tx);
  pinMode(enable , OUTPUT);
  pinMode(stepx  , OUTPUT);
  pinMode(dirx   , OUTPUT);
  pinMode(stepy  , OUTPUT);
  pinMode(diry   , OUTPUT);
  pinMode(stepz1 , OUTPUT);
  pinMode(stepz2 , OUTPUT);
  pinMode(dirz1  , OUTPUT);
  pinMode(dirz2  , OUTPUT);
  pinMode(poweron,  INPUT);
  pinMode(fcx    ,  INPUT);
  pinMode(fcy    ,  INPUT);
  pinMode(fcz1   ,  INPUT);
  pinMode(fcz2   ,  INPUT);
  pinMode(fcpz   ,  INPUT);
  blkmot(0);
  tx.v = 0;
  tx.d = 0;
  fastWrite(flag,0);
}
void loop() {
  if(newaction==1){
    switch(int(rx.id)){
      case 1:
        s0zz(rx.a);
        newaction=0;
        fastWrite(flag,1);
      break;
      case 2:
        s0xy(rx.a);
        newaction=0;
        fastWrite(flag,1);
      break;
      case 3:
        s0pz(rx.a, int(rx.b));
        newaction=0;
        fastWrite(flag,1);
      break;
      case 4:
        movxyz(rx.a, rx.b, rx.c, rx.d);
        newaction=0;
        fastWrite(flag,1);
      break;
      case 5:
        blkmot(rx.a);
        newaction=0;
        fastWrite(flag,1);
      break;
      default:
        newaction=0;
        fastWrite(flag,0);
      break;
    }
  }else if(fastRead(poweron)==0){
    tx.v = 0;
    tx.d = 0;
    newaction=0;
    blkmot(0);
    fastWrite(flag,0);
    Serial.println("reset");
  }
}
//funzioni di movimento dei motori:
void s0xy(float velocita) {
  //abilitazione pin per movimento
  if(velocita <= 0){
    return;
  }
  fastWrite(enable, 0);
  fastWrite(dirx,!dir);
  fastWrite(diry,!dir);
  //calcoli
  int passix = 1;
  int passiy = 1;
  int delay = ((1000000 * mpr) / (velocita * ppr))/2;
  //movimento
  unsigned long tx = micros();
  unsigned long ty = micros();
  statox = !step;
  statoy = !step;
  while(passix > 0 || passiy > 0){
    do{
      if(fastRead(poweron)==0){
        return;
      }
    }while(fastRead(flag2)==1);
    if(fastRead(fcx) == 1 && passix > 0){
      passix = 0;
    }
    if(fastRead(fcy) == 1 && passiy > 0){
      passiy = 0;
    }
    if(micros() - tx >= delay){
      if(passix > 0){
        statox=!statox;
        fastWrite(stepx,statox);
      }
      if(passiy > 0){
        statoy=!statoy;
        fastWrite(stepy,statoy);
      }
      tx = micros();
    }
  }
  //disabilitazione pin dopo il movimento
  statox = !step;
  statoy = !step;
  fastWrite(dirx ,!dir );
  fastWrite(diry ,!dir );
  fastWrite(stepx,!step);
  fastWrite(stepy,!step);
  fastWrite(enable, bkm);
}
void s0zz(float velocita) {
  //abilitazione pin per movimento
  if(velocita <= 0){
    return;
  }
  fastWrite(enable, 0);
  fastWrite(dirz1,dir);
  fastWrite(dirz2,dir);
  //calcoli
  int passiz1 = 1;
  int passiz2 = 1;
  int delay   = ((1000000 * mpr) / (velocita * ppr))/2;
  //movimento
  unsigned long tz = micros();
  statoz1 = !step;
  statoz2 = !step;
  while(passiz1 > 0 || passiz2 > 0){
    do{
      if(fastRead(poweron)==0){
        return;
      }
    }while(fastRead(flag2)==1);
    if(fastRead(fcz1) == 1 && passiz1 > 0){
      passiz1 = 0;
    }
    if(fastRead(fcz2) == 1 && passiz2 > 0){
      passiz2 = 0;
    }
    if(micros() - tz >= delay){
      if(passiz1 > 0){
        statoz1=!statoz1;
        fastWrite(stepz1,statoz1);
      }
      if(passiz2 > 0){
        statoz2=!statoz2;
        fastWrite(stepz2,statoz2);
      }
      tz = micros();
    }
  }
  //disabilitazione pin dopo il movimento
  statoz1 = !step;
  statoz2 = !step;
  fastWrite(dirz1 ,!dir );
  fastWrite(dirz2 ,!dir );
  fastWrite(stepz1,!step);
  fastWrite(stepz2,!step);
  fastWrite(enable, bkm);
}
void s0pz(float velocita, int valore) {
  //abilitazione pin per movimento
  if(velocita <= 0){
    return;
  }
  fastWrite(enable, 0);
  fastWrite(dirz1,!dir);
  fastWrite(dirz2,!dir);
  //calcoli
  int passiz = 0;
  int delay   = ((1000000 * mpr) / (velocita * ppr))/2;
  //movimento
  unsigned long tz = micros();
  statoz1 = !step;
  statoz2 = !step;
  while(fastRead(fcpz) == 1){
    do{
      if(fastRead(poweron)==0){
        return;
      }
    }while(fastRead(flag2)==1);
    if(micros() - tz >= delay){
      statoz1=!statoz1;
      statoz2=!statoz2;
      fastWrite(stepz1,statoz1);
      fastWrite(stepz2,statoz2);
      passiz++;
      tz = micros();
    }
  }
  //disabilitazione pin dopo il movimento
  statoz1 = !step;
  statoz2 = !step;
  fastWrite(dirz1 ,!dir );
  fastWrite(dirz2 ,!dir );
  fastWrite(stepz1,!step);
  fastWrite(stepz2,!step);
  fastWrite(enable, bkm);
  tx.d = ((float(passiz)/2.0) * mpr) / ppr;
  tx.v = valore;
  return;
}
void movxyz(float velocita, float pasx, float pasy, float pasz) { //movxyz(mm/min, mm, mm, mm);
  //abilitazione pin per movimento
  if(velocita <= 0){
    return;
  }
  pasz=-pasz;
  fastWrite(enable, 0);
  if(pasx > 0){
    fastWrite(dirx,dir);
  }else{
    fastWrite(dirx,!dir);
    pasx = -pasx;
  }
  if(pasy > 0){
    fastWrite(diry,dir);
  }else{
    fastWrite(diry,!dir);
    pasy = -pasy;
  }
  if(pasz > 0){
    fastWrite(dirz1,dir);
    fastWrite(dirz2,dir);
  }else{
    fastWrite(dirz1,!dir);
    fastWrite(dirz2,!dir);
    pasz = -pasz;
  }
  //calcoli
  int passix = (pasx * ppr) / mpr;
  int passiy = (pasy * ppr) / mpr;
  int passiz = (pasz * ppr) / mpr;
  float tempo = round((sqrt((pasx*pasx)+(pasy*pasy)+(pasz*pasz))/velocita)* 1000000);
  float delayx = (tempo / float(passix))/2;
  float delayy = (tempo / float(passiy))/2;
  float delayz = (tempo / float(passiz))/2;
  passix=passix*2;
  passiy=passiy*2;
  passiz=passiz*2;
  //movimento
  unsigned long tx = micros();
  unsigned long ty = micros();
  unsigned long tz = micros();
  statox  = !step;
  statoy  = !step;
  statoz1 = !step;
  statoz2 = !step;
  while(passix > 0 || passiy > 0 || passiz > 0){
    do{
      if(fastRead(poweron)==0){
        return;
      }
    }while(fastRead(flag2)==1);
    if(passix > 0 && (micros() - tx) >= delayx){
      statox=!statox;
      fastWrite(stepx,statox);
      passix--;
      tx = micros();
    }
    if(passiy > 0 && (micros() - ty) >= delayy){
      statoy=!statoy;
      fastWrite(stepy,statoy);
      passiy--;
      ty = micros();
    }
    if(passiz > 0 && (micros() - tz) >= delayz){
      statoz1=!statoz1;
      statoz2=!statoz2;
      fastWrite(stepz1,statoz1);
      fastWrite(stepz2,statoz2);
      passiz--;
      tz = micros();
    }
  }
  //disabilitazione pin dopo il movimento
  statox  = !step;
  statoy  = !step;
  statoz1 = !step;
  statoz2 = !step;
  fastWrite(dirx  ,!dir );
  fastWrite(diry  ,!dir );
  fastWrite(dirz1 ,!dir );
  fastWrite(dirz2 ,!dir );
  fastWrite(stepx ,!step);
  fastWrite(stepy ,!step);
  fastWrite(stepz1,!step);
  fastWrite(stepz2,!step);
  fastWrite(enable, bkm);
}