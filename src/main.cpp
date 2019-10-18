#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <Wire.h>
#include "Isigfox.h"
#include "WISOL.h"
#include <BME280.h>

extern "C"
{
  void vReadBme280SensorTask(void *pvParameters);
  void vProcessDataTask(void *pvParameters);
}

void GetDeviceID();
void Send_Pload(uint8_t *sendData, const uint8_t len);

Isigfox *isigfox = new WISOL();
BME280 bme(Wire,0x76);

void setup() {
  // put your setup code here, to run once:
   pinMode(PC13, OUTPUT);
   Serial1.begin(115200);   //Debug console  


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
                1,
                NULL);

 // start scheduler
  vTaskStartScheduler();
  Serial1.println("Insufficient RAM");
  while(1);

}

void loop() {
  // // put your main code here, to run repeatedly:
  // digitalWrite(PC13, HIGH);
  // Serial1.println("Text");
  // delay(1000);
  // digitalWrite(PC13, LOW);
  // delay(1000);
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

void Send_Pload(uint8_t *sendData, const uint8_t len){
  // No downlink message require
  recvMsg *RecvMsg;

  RecvMsg = (recvMsg *)malloc(sizeof(recvMsg));
  isigfox->sendPayload(sendData, len, 0, RecvMsg);
  for (int i = 0; i < RecvMsg->len; i++) {
    Serial.print(RecvMsg->inData[i]);
  }
  Serial.println("");
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
