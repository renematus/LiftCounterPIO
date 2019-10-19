#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <Wire.h>
#include "Isigfox.h"
#include "WISOL.h"
#include <BME280.h>
#include <RTClock.h>
#include <SerialCommand.h>
#include "BendCounter.h"

extern "C"
{
  void vReadBme280SensorTask(void *pvParameters);
  void vProcessDataTask(void *pvParameters);
  void vGetDownlinkMessageTask(void *pvParameters);
  void vCheckSignalsTask(void *pvParameters);
  void vSerialCommandReadTask(void *pvParameters);
}

void GetDeviceID();
void Send_Pload(uint8_t *sendData, const uint8_t len);
void getDLMsg();
void configureTimeMinute();
void configureTimeSec();
void configureTimeHour();
void configureTimeDay();
void configureTimeMonth();
void configureTimeYear();

RTClock rt (RTCSEL_LSE);
Isigfox *isigfox = new WISOL();
BME280 bme(Wire,0x76);
SerialCommand sCmd;

BendCounter counter;

void setup() {
  // put your setup code here, to run once:
   pinMode(PC13, OUTPUT);
   Serial1.begin(115200);   //Debug console  
   //Serial2  =  Sigfox
   Serial3.begin(9600);   // BLE serial HC06


  Serial1.println("Start Sigfox Module"); // Make a clean restart
  int flagInit = -1;
  while (flagInit == -1) {
    Serial1.println(""); // Make a clean restart
    delay(1000);
    flagInit = isigfox->initSigfox();
    isigfox->testComms();
    GetDeviceID();
  }

 
  Serial1.println("Start BME Module"); 
  if (bme.begin() < 0) {
    Serial1.println("Error communicating with sensor, check wiring and I2C address");
    while(1){}
  }

  counter.init();


  //Configure serial commands
  sCmd.addCommand("tm", configureTimeMinute);
  sCmd.addCommand("ts", configureTimeSec);
  sCmd.addCommand("th", configureTimeHour);
  sCmd.addCommand("td", configureTimeDay);
  sCmd.addCommand("tM", configureTimeMonth);
  sCmd.addCommand("ty", configureTimeYear);

 xTaskCreate(vProcessDataTask,
                (const signed char * const)"Task3",
                configMINIMAL_STACK_SIZE + 500,
                NULL,
                tskIDLE_PRIORITY + 5,
                NULL);
 
xTaskCreate(vReadBme280SensorTask,
                (const signed char * const)"Task1",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);

xTaskCreate(vCheckSignalsTask,
                (const signed char * const)"Task4",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

xTaskCreate(vGetDownlinkMessageTask,
                (const signed char * const)"Task2",
                configMINIMAL_STACK_SIZE + 500,
                NULL,
                tskIDLE_PRIORITY + 3,
                NULL);

xTaskCreate(vSerialCommandReadTask,
                (const signed char * const)"Task5",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 3,
                NULL);

 // start scheduler
  vTaskStartScheduler();
  Serial1.println("Insufficient RAM");
  while(1);

}

void loop() {
  // // put your main code here, to run repeatedly:
  //Not for Free RTOS
}

void vSerialCommandReadTask(void *pvParameters) {
    for (;;) {
        sCmd.readSerial();
        vTaskDelay(100);
    }
}

void vReadBme280SensorTask(void *pvParameters) {
    for (;;) {
        bme.readSensor();

          // displaying the data
          Serial1.print(bme.getPressure_Pa(),6);
          Serial1.print("\t");
          Serial1.print(bme.getTemperature_C(),2);
          Serial1.print("\t");
          Serial1.println(bme.getHumidity_RH(),2);

        vTaskDelay(10000);
    }
}

void vProcessDataTask(void *pvParameters) {
    
    const uint8_t payloadSize = 12; //in bytes
    uint8_t buf_str[payloadSize];
    uint8_t counter = 1;
    
    for (;;) {
        buf_str[0] = counter;
        digitalWrite(PC13, LOW);
        Serial1.print("Send data:"); Serial1.println(counter);
        Send_Pload(buf_str, payloadSize);
        digitalWrite(PC13, HIGH);
        counter ++;
        vTaskDelay(110000);
    }
}

void vGetDownlinkMessageTask(void *pvParameters) {
    for (;;) {

      //Todo lock, becouse use sigfox modem like send message
        Serial1.print("Check downlink message");
        getDLMsg();
        vTaskDelay(60000);
    }
}

void vCheckSignalsTask(void *pvParameters) {
    for (;;) {

        //Serial.print("Up counter: "); Serial.println(BendCounter::getCounter());
        //Serial.print("Down counter: "); Serial.println(downCounter);

        tm_t current_time;
        rt.getTime(current_time);

        Serial1.print("time is: ");
        Serial1.print(rt.day()); Serial1.print(". ");
        Serial1.print(rt.month()); Serial1.print(". ");
        Serial1.print(rt.year()); Serial1.print(" ");
        Serial1.print(rt.hour()); Serial1.print(": ");
        Serial1.print(rt.minute()); Serial1.print(": ");
        Serial1.println(rt.second());

        Serial3.print("time is: ");
        Serial3.print(rt.day()); Serial3.print(". ");
        Serial3.print(rt.month()); Serial3.print(". ");
        Serial3.print(rt.year()); Serial3.print(" ");
        Serial3.print(rt.hour()); Serial3.print(": ");
        Serial3.print(rt.minute()); Serial3.print(": ");
        Serial3.println(rt.second());

        vTaskDelay(5000);
    }
}


void GetDeviceID(){
  recvMsg *RecvMsg;
  const char msg[] = "AT$I=10";

  RecvMsg = (recvMsg *)malloc(sizeof(recvMsg));
  isigfox->sendMessage(msg, 7, RecvMsg);

  Serial.print("Device ID: ");
  for (int i=0; i<RecvMsg->len; i++){
    Serial1.print(RecvMsg->inData[i]);
  }
  Serial1.println("");
  free(RecvMsg);
}

void getDLMsg(){
  recvMsg *RecvMsg;
  int result;

  RecvMsg = (recvMsg *)malloc(sizeof(recvMsg));
  result = isigfox->getdownlinkMsg(RecvMsg);
  for (int i=0; i<RecvMsg->len; i++){
    Serial1.print(RecvMsg->inData[i]);
  }
  Serial1.println("");
  free(RecvMsg);
}

void Send_Pload(uint8_t *sendData, const uint8_t len){
  // No downlink message require
  recvMsg *RecvMsg;

  RecvMsg = (recvMsg *)malloc(sizeof(recvMsg));
  isigfox->sendPayload(sendData, len, 0, RecvMsg);
  for (int i = 0; i < RecvMsg->len; i++) {
    Serial1.print(RecvMsg->inData[i]);
  }
  Serial1.println("");
  free(RecvMsg);


  // If want to get blocking downlink message, use the folling block instead
  /*
  recvMsg *RecvMsg;

  RecvMsg = (recvMsg *)malloc(sizeof(recvMsg));
  Isigfox->sendPayload(sendData, len, 1, RecvMsg);
  for (int i=0; i<RecvMsg->len; i++){
    Serial.print(RecvMsg->inData[i]);
  }
  Serial.println("");
  free(RecvMsg);
  */

  // If want to get non-blocking downlink message, use the folling block instead
  /*
  Isigfox->sendPayload(sendData, len, 1);
  timer.setTimeout(46000, getDLMsg);
  */
}

//Argument vlozime  command [mezera] argument [enter]
void configureTimeMinute()
{
  char * arg;
  int minute;
  arg = sCmd.next();
  if (arg != NULL) {
    minute = atoi(arg);    

    tm_t set_time;
    rt.getTime(set_time);
    set_time.minute = minute;
    rt.setTime(set_time);
    Serial1.print("Minute was set to: ");
    Serial1.println(minute);
  }
}

//Argument vlozime  command [mezera] argument [enter]
void configureTimeSec()
{
  char * arg;
  int second;
  arg = sCmd.next();
  if (arg != NULL) {
    second = atoi(arg);    

    tm_t set_time;
    rt.getTime(set_time);
    set_time.second = second;
    rt.setTime(set_time);
    Serial1.print("Sec was set to: ");
    Serial1.println(second);
  }
}

//Argument vlozime  command [mezera] argument [enter]
void configureTimeHour()
{
  char * arg;
  int hour;
  arg = sCmd.next();
  if (arg != NULL) {
    hour = atoi(arg);    

    tm_t set_time;
    rt.getTime(set_time);
    set_time.hour = hour;
    rt.setTime(set_time);
    Serial1.print("Hour was set to: ");
    Serial1.println(hour);
  }
}

//Argument vlozime  command [mezera] argument [enter]
void configureTimeDay()
{
  char * arg;
  int day;
  arg = sCmd.next();
  if (arg != NULL) {
    day = atoi(arg);    

    tm_t set_time;
    rt.getTime(set_time);
    set_time.day = day;
    rt.setTime(set_time);
    Serial1.print("Day was set to: ");
    Serial1.println(day);
  }
}

//Argument vlozime  command [mezera] argument [enter]
void configureTimeMonth()
{
  char * arg;
  int month;
  arg = sCmd.next();
  if (arg != NULL) {
    month = atoi(arg);    

    tm_t set_time;
    rt.getTime(set_time);
    set_time.month = month;
    rt.setTime(set_time);
    Serial1.print("Month was set to: ");
    Serial1.println(month);
  }
}

//Argument vlozime  command [mezera] argument [enter]
void configureTimeYear()
{
  char * arg;
  int year;
  arg = sCmd.next();
  if (arg != NULL) {
    year = atoi(arg);    

    tm_t set_time;
    rt.getTime(set_time);
    set_time.year = year;
    rt.setTime(set_time);
    Serial1.print("Year was set to: ");
    Serial1.println(year);
  }
}