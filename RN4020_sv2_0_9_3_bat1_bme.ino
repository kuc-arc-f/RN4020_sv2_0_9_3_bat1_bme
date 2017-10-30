
/* 
  SoftwareSerial, RN4020 BLE (Low Power version)
  with Central= raspBerryPi , pubAddr version
   battery =1 QTY
   v 0.9.03 - BME280
*/
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>
#include <BME280_MOD-1022.h>
#include <Wire.h>

SoftwareSerial mySerial(5, 6 ); /* RX:D5, TX:D6 */

String mInoFile="RN4020_sv2_0_9_3_bat1_bme.ino";
String mBuff="";
// const int mVoutPin = 0;
const int mPinBLE    =9;
const int mPinSen3V =13;

const int mOK_CODE=1;
const int mNG_CODE=0;
uint32_t mTimer_runMax= 0;
int mAdvCount=0;
// int mTemp=0;
const int mNextSec   = 1800; //Sec
const int mMax_runSec= 12; //Sec

const int mMode_RUN  = 1;
const int mMode_WAIT = 2; 
int mMode =0;
const int mMaxGap=25;

// LOW power
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
volatile int wdt_cycle;

//
long convert_Map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//
String convert_hex(String src){
  String sRet="";
//  int iLen=src.length();
  // int iLen=mMaxGap *2;
  char  sRead[mMaxGap+1 ];
  sprintf(sRead, "%s",  src.c_str() );
  for(int i=0; i< mMaxGap ; i++){
      char  cBuff[2+1];
      sprintf( cBuff, "%02x", sRead[i] );
      String sBuff=String(cBuff );
      // sRet += sBuff;
      sRet.concat(sBuff  );
//      Serial.println(cBuff );
  }
  // Serial.println ( "sRet="+ sRet );
  return sRet;
}

//
int Is_resWait(String value, uint32_t maxMsec ){
  int ret= mNG_CODE;
  uint32_t tm_maxWait=millis();
  int iLen= value.length();
  String sBuff="";
  int iBool=1;
    while(iBool ){
        while( mySerial.available() ){
            char c= mySerial.read();
            Serial.print( c );
            if( (c != 0x0a ) && (c != 0x0d ) ){
                sBuff.concat(c );
            }            
        } //end_while
        if(  (int )sBuff.length() >= iLen ){ iBool=0;  }      
        delay(100);
        uint32_t tmMax =millis() -tm_maxWait;
        if(tmMax > (uint32_t)maxMsec ){ 
          Serial.println("#Error-Is_resWait:maxSec!");
          iBool=0; 
        } 
    }//end_while_1
    Serial.println("");
    if(sBuff.length() < 1){ return ret; }
    int iLenBuff= sBuff.length();
    int iSt=iLenBuff -(iLen); 
    Serial.println("iLenBuff="+ String(iLenBuff)+",iSt="+ String(iSt) ) ;
    String sRecv = sBuff.substring(iSt   , iLen );
    Serial.println( "sBuff="+ sBuff+ " ,sRecv="+ sRecv );
    if(sRecv == value ){
      ret= mOK_CODE;
    }
  return ret;  
}
//
void proc_getSSerial(){
  while( mySerial.available() ){
    char c= mySerial.read();
Serial.print( c );
  } //end_while
}

//
void proc_sendCmd(){
  int iWaitTm= 5000;
  int iLenBuff=(mMaxGap * 2) + 3 +1;
  char cRead[25 +1];
  char cBuff[iLenBuff ];
  int iTemp = BME280_getTemperature();
  int iHumi = BME280_getHumidity();
  int iPress= BME280_getPressure();
  //int iSenNum= getTempNum();
  //Serial.println("iSenNum=" + String(iSenNum) );
  Serial.println("iSenNum=" + String(iTemp) +",mAdvCount="+String(mAdvCount ));
//  sprintf(cRead , "%05d%05d%05d%05d%05d", mTemp  ,0 ,0 , 0, 0 );
  sprintf(cRead , "%05d%05d%05d%05d%05d", iTemp  ,iHumi ,iPress , 0, 0 );
  Serial.println("cRead="+ String(cRead ));
  String sRead= convert_hex( String(cRead) );
  Serial.println("Hex=" +sRead);
  sprintf(cBuff , "N,%s\r", sRead.c_str()  );
  //N
  Serial.println( "#Start N, cBuff="+  String(cBuff)  );
   mySerial.print(String(cBuff) );
   if( Is_resWait("AOK", iWaitTm ) == mNG_CODE ){ return; }
   else{Serial.println("#OK SN" ); };
   proc_getSSerial();
   //A
   Serial.println( "#Start A" );
   mySerial.print("A,0064,07D0\r");    //100msec, total= 2000mSec
  if( Is_resWait("AOK", iWaitTm ) == mNG_CODE ){ return; }
   else{Serial.println("#OK A" ); };
   proc_getSSerial();
   wait_forSec( 2 );
   digitalWrite(mPinBLE,LOW  );
   digitalWrite(mPinSen3V , LOW);
   wait_forSec( 3 );
   mAdvCount=mAdvCount+1;
}

//
void wait_forSec(int wait){
   for(int i=0; i<wait; i++){
    delay(1000);
    Serial.println("#wait_forMsec: "+ String(i) );    
  }
}
//
void setup() {
  mMode = mMode_RUN;
  pinMode(mPinBLE ,OUTPUT);
  pinMode(mPinSen3V  ,OUTPUT);
  Serial.begin( 9600 );
  mySerial.begin( 9600 );
  Serial.println("#Start-"+  mInoFile);
  wait_forSec( 10 );  //start-time
  delay(100);
  init_BME280();
  setup_watchdog(6 );                    // WDT setting
  //wait
  delay(1000);
  proc_getSSerial(); //clear-buff
}
//
void read_bme280(){
  while (BME280.isMeasuring()) {
  }
  BME280.readMeasurements(); 
}

//
void loop() {
//  Serial.println( "mTimer="+ String(mTimer) + ",millis=" + String(millis()) );
  if(mMode ==mMode_RUN){
      if(mTimer_runMax <= 0 ){
          mTimer_runMax =  ((uint32_t)mMax_runSec * 1000) + millis();
          digitalWrite(mPinSen3V , HIGH);          
//          mTemp= getTempNum();
          read_bme280();
          print_bme280();
          delay(100);
          digitalWrite(mPinBLE, HIGH);
          delay(1000);
      }
      if(millis() < mTimer_runMax){
          if(mAdvCount <=1 ){
             digitalWrite(mPinBLE, HIGH);
             digitalWrite(mPinSen3V , HIGH);
             delay(500);
             proc_getSSerial();
             proc_sendCmd();
          }
      }else{
          mTimer_runMax=0;
          mAdvCount=0;
          digitalWrite(mPinBLE,LOW  );
          digitalWrite(mPinSen3V , LOW);
          mMode = mMode_WAIT;
      }
  }else{
    mMode = mMode_RUN;
    for(int i=0; i< mNextSec ; i++){
        system_sleep();                       // power down ,wake up WDT 
        //Serial.print("i=");
        //delay(15);
        //Serial.println(i);
        //delay(15);
    }
  }
  
}

//
void system_sleep() {
  cbi(ADCSRA,ADEN);                     // ADC power off
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // power down mode
  sleep_enable();
  sleep_mode();                         // sleep
  sleep_disable();                      // WDT time up
  sbi(ADCSRA,ADEN);                     // ADC ON
}

// WDT setting, param : time
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {           // 
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}  //setup_watchdog

ISR(WDT_vect) {                         // WDT, time Up process
  wdt_cycle++;           
}














