#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include "Isigfox.h"
#include "WISOL.h"

extern "C"
{
  void vReadBme280SensorTask(void *pvParameters);
}

void GetDeviceID();

Isigfox *isigfox = new WISOL();

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



 
  xTaskCreate(vReadBme280SensorTask,
                (const signed char * const)"Task1",
                128,
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

         Serial1.println("Text");
        // bme.readSensor();

        //   // displaying the data
        //   Serial.print(bme.getPressure_Pa(),6);
        //   Serial.print("\t");
        //   Serial.print(bme.getTemperature_C(),2);
        //   Serial.print("\t");
        //   Serial.println(bme.getHumidity_RH(),2);

        vTaskDelay(10000);
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
