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
#include "global.h"

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
void showHelp();
void showSensors();
void showTime();
void showCounter();
void clearCounter();

//Static doesnt work, I dont know why
volatile uint16 counter = 0;

RTClock rt (RTCSEL_LSE);
Isigfox *isigfox = new WISOL();
BME280 bme(Wire,0x76);
SerialCommand sCmd;

BendCounter bendCounter;

void setup() {
  // put your setup code here, to run once:
   pinMode(PC13, OUTPUT);
   Serial1.begin(115200);   //Debug console  
   //Serial2  =  Sigfox
   Serial3.begin(9600);   // BLE serial HC06


  DEBUG_CONSOLE.println("Start Sigfox Module"); // Make a clean restart
  int flagInit = -1;
  while (flagInit == -1) {
    DEBUG_CONSOLE.println(""); // Make a clean restart
    delay(1000);
    flagInit = isigfox->initSigfox();
    isigfox->testComms();
    GetDeviceID();
  }

 
  DEBUG_CONSOLE.println("Start BME Module"); 
  if (bme.begin() < 0) {
    DEBUG_CONSOLE.println("Error communicating with sensor, check wiring and I2C address");
    while(1){}
  }

  bendCounter.init();


  //Configure serial commands
  sCmd.addCommand("??", showHelp);
  sCmd.addCommand("st", showTime);
  sCmd.addCommand("ss", showSensors);
  sCmd.addCommand("sc", showCounter);
  sCmd.addCommand("cc!!", clearCounter);
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
  DEBUG_CONSOLE.println("Insufficient RAM");
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

        //   // displaying the data
        //   DEBUG_CONSOLE.print(bme.getPressure_Pa(),6);
        //   DEBUG_CONSOLE.print("\t");
        //   DEBUG_CONSOLE.print(bme.getTemperature_C(),2);
        //   DEBUG_CONSOLE.print("\t");
        //   DEBUG_CONSOLE.println(bme.getHumidity_RH(),2);

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
        DEBUG_CONSOLE.print("Send data:"); DEBUG_CONSOLE.println(counter);
        Send_Pload(buf_str, payloadSize);
        digitalWrite(PC13, HIGH);
        counter ++;
        vTaskDelay(110000);
    }
}

void vGetDownlinkMessageTask(void *pvParameters) {
    for (;;) {

      //Todo lock, becouse use sigfox modem like send message
        DEBUG_CONSOLE.print("Check downlink message");
        getDLMsg();
        vTaskDelay(60000);
    }
}

void vCheckSignalsTask(void *pvParameters) {
    for (;;) {

        //Serial.print("Up counter: "); Serial.println(BendCounter::getCounter());
        //Serial.print("Down counter: "); Serial.println(downCounter);

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
    DEBUG_CONSOLE.print(RecvMsg->inData[i]);
  }
  DEBUG_CONSOLE.println("");
  free(RecvMsg);
}

void getDLMsg(){
  recvMsg *RecvMsg;
  int result;

  RecvMsg = (recvMsg *)malloc(sizeof(recvMsg));
  result = isigfox->getdownlinkMsg(RecvMsg);
  for (int i=0; i<RecvMsg->len; i++){
    DEBUG_CONSOLE.print(RecvMsg->inData[i]);
  }
  DEBUG_CONSOLE.println("");
  free(RecvMsg);
}

void Send_Pload(uint8_t *sendData, const uint8_t len){
  // No downlink message require
  recvMsg *RecvMsg;

  RecvMsg = (recvMsg *)malloc(sizeof(recvMsg));
  isigfox->sendPayload(sendData, len, 0, RecvMsg);
  for (int i = 0; i < RecvMsg->len; i++) {
    DEBUG_CONSOLE.print(RecvMsg->inData[i]);
  }
  DEBUG_CONSOLE.println("");
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
    DEBUG_CONSOLE.print("Minute was set to: ");
    DEBUG_CONSOLE.println(minute);
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
    DEBUG_CONSOLE.print("Sec was set to: ");
    DEBUG_CONSOLE.println(second);
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
    DEBUG_CONSOLE.print("Hour was set to: ");
    DEBUG_CONSOLE.println(hour);
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
    DEBUG_CONSOLE.print("Day was set to: ");
    DEBUG_CONSOLE.println(day);
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
    DEBUG_CONSOLE.print("Month was set to: ");
    DEBUG_CONSOLE.println(month);
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
    DEBUG_CONSOLE.print("Year was set to: ");
    DEBUG_CONSOLE.println(year);
  }
}

//Argument vlozime  command [mezera] argument [enter]
void showHelp()
{
  DEBUG_CONSOLE.println("?? -  help ");
  DEBUG_CONSOLE.println("st -  time ");
  DEBUG_CONSOLE.println("ss -  sensors ");
  DEBUG_CONSOLE.println("sc -  counter ");
  DEBUG_CONSOLE.println("cc!! -  clear counter ");
  DEBUG_CONSOLE.println("ts - set seconds ");
  DEBUG_CONSOLE.println("tm - set minutes ");
  DEBUG_CONSOLE.println("th - set hours ");
  DEBUG_CONSOLE.println("td - set days ");
  DEBUG_CONSOLE.println("tM - set months ");
  DEBUG_CONSOLE.println("ty - set years ");
}

void showTime()
{
        tm_t current_time;
        rt.getTime(current_time);

        DEBUG_CONSOLE.print("time is: ");
        DEBUG_CONSOLE.print(rt.day()); DEBUG_CONSOLE.print(". ");
        DEBUG_CONSOLE.print(rt.month()); DEBUG_CONSOLE.print(". ");
        DEBUG_CONSOLE.print(20);DEBUG_CONSOLE.print(rt.year()); DEBUG_CONSOLE.print("     ");
        DEBUG_CONSOLE.print(rt.hour()); DEBUG_CONSOLE.print(": ");
        DEBUG_CONSOLE.print(rt.minute()); DEBUG_CONSOLE.print(": ");
        DEBUG_CONSOLE.println(rt.second());
}

void showSensors()
{

      // displaying the data
      DEBUG_CONSOLE.print("Pressure: ");
      DEBUG_CONSOLE.println(bme.getPressure_Pa(),0);

      DEBUG_CONSOLE.print("Temperature: ");
      DEBUG_CONSOLE.println(bme.getTemperature_C(),2);
      
      DEBUG_CONSOLE.print("Humidity: ");
      DEBUG_CONSOLE.println(bme.getHumidity_RH(),2);
}

void showCounter()
{
      DEBUG_CONSOLE.print("Counter: ");
      DEBUG_CONSOLE.println(bendCounter.getCounter());
}

void clearCounter()
{
      DEBUG_CONSOLE.println("Clear counter !!");
      bendCounter.clearCounter();
      DEBUG_CONSOLE.print("Counter: ");
      DEBUG_CONSOLE.println(bendCounter.getCounter());
}


