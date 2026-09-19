// Reference Arduino Nano: closed switch D2, payload switch D3, servo D9.
// Switches close to GND. Calibrate angles/pins on the actual mechanism.
#include <Servo.h>
#include <EEPROM.h>
#include <string.h>

const byte CLOSED_PIN=2, PRESENT_PIN=3, SERVO_PIN=9;
const int OPEN_ANGLE=90, CLOSED_ANGLE=10;
Servo claw;
char line[161], lastId[33];
byte used=0;
bool overflowed=false, busy=false, fault=false;
unsigned long bootCounter, lastPing=0, started=0, lastReport=0;
byte stage=0, operation=0; // 1=grab, 2=release

uint16_t crc16(const char *s) {
  uint16_t crc=0xffff;
  while (*s) {
    crc^=(uint16_t)(byte)*s++ << 8;
    for (byte i=0;i<8;i++) crc=(crc&0x8000)?(crc<<1)^0x1021:crc<<1;
  }
  return crc;
}

bool closed() { return digitalRead(CLOSED_PIN)==LOW; }
bool present() { return digitalRead(PRESENT_PIN)==LOW; }
void stopMotion() { claw.detach(); busy=false; fault=true; }

void report() {
  char text[110];
  snprintf(text,sizeof(text),"STATE %lu %d %d %d %d %s",bootCounter,
           closed(),present(),busy,fault,lastId[0]?lastId:"-");
  Serial.print(text); Serial.print('*');
  char hex[5]; snprintf(hex,sizeof(hex),"%04X",crc16(text));
  Serial.println(hex);
}

void command(char *s) {
  char *star=strrchr(s,'*');
  if (!star || strlen(star+1)!=4) return;
  *star=0;
  if (crc16(s)!=(uint16_t)strtoul(star+1,NULL,16)) return;
  if (!strcmp(s,"PING")) { lastPing=millis(); return; }
  if (!strcmp(s,"STOP")) { stopMotion(); return; }
  char verb[8], counter[12], token[33], extra;
  if (sscanf(s,"%7s %11s %32s %c",verb,counter,token,&extra)!=3) return;
  if (strcmp(verb,"GRAB") && strcmp(verb,"RELEASE")) return;
  if (strtoul(counter,NULL,10)!=bootCounter || strlen(token)!=32) return;
  for (byte i=0;i<32;i++) if (!strchr("0123456789abcdef",token[i])) return;
  if (!strcmp(token,lastId)) { report(); return; }
  bool grabbing=!strcmp(verb,"GRAB");
  if (busy || fault || !closed() || (grabbing ? present() : !present()) || millis()-lastPing>500) return;
  // Persist before motion. A restart never automatically resumes an attempt.
  strcpy(lastId,token);
  for (byte i=0;i<33;i++) EEPROM.update(8+i,lastId[i]);
  claw.attach(SERVO_PIN); claw.write(OPEN_ANGLE);
  busy=true; stage=1; operation=grabbing?1:2; started=millis();
}

void setup() {
  pinMode(CLOSED_PIN,INPUT_PULLUP); pinMode(PRESENT_PIN,INPUT_PULLUP);
  Serial.begin(115200);
  EEPROM.get(0,bootCounter);
  if (bootCounter==0xffffffff) bootCounter=0;
  bootCounter++; EEPROM.put(0,bootCounter);
  for (byte i=0;i<33;i++) lastId[i]=EEPROM.read(8+i);
  lastId[32]=0;
  for (byte i=0;i<32;i++) if (!strchr("0123456789abcdef",lastId[i])) { lastId[0]=0; break; }
  // No attach/motion at boot; incomplete physical state must be recovered manually.
  fault=!closed();
}

void loop() {
  while (Serial.available()) {
    char c=Serial.read();
    if (c=='\n') {
      if (!overflowed) { line[used]=0; command(line); }
      used=0; overflowed=false;
    } else if (c!='\r') {
      if (used<160 && !overflowed) line[used++]=c;
      else overflowed=true;
    }
  }
  unsigned long now=millis();
  if (busy) {
    if (now-lastPing>500 || now-started>8000) stopMotion();
    else if (operation==2 && stage==1 && !present() && now-started>500) {
      claw.write(CLOSED_ANGLE); stage=2;
    } else if (operation==1 && stage==1 && now-started>800) {
      claw.write(CLOSED_ANGLE); stage=2;
    } else if (stage==2 && closed()) {
      if ((operation==1 && present()) || (operation==2 && !present())) {
        claw.detach(); busy=false;
      }
    }
  }
  if (now-lastReport>=100) { report(); lastReport=now; }
}
