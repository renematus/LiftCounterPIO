#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

extern "C"
{
  void vReadBme280SensorTask(void *pvParameters);
}

void setup() {
  // put your setup code here, to run once:
   pinMode(PC13, OUTPUT);
   Serial1.begin(115200);   //Debug console  

 
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
