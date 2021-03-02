#include <Wire.h>
#include "MAX30105.h"
#include "heltec.h"
#include "heartRate.h"

MAX30105 particleSensor;
TaskHandle_t Task1;
#define MAX_BRIGHTNESS 255
#define BAND    915E6 //Banda del LoRa

float tempC; // Variable para obtener temperatura
float promtemp; //Promedio Temperatura
const byte RATE_SIZE = 8; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float Spo2;
float beatsPerMinute;
int beatAvg;

void setupADC()
{
  adcAttachPin(13); // Se usara el pin 13 para ADC
  analogReadResolution(12); // 12 bits de Resolucion
  analogSetClockDiv(255); // 1338mS
}

void setupMAX30102()
{
  // Initialize sensor
  if (!particleSensor.begin(Wire1, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

}
void leerADC()
{
    promtemp=0;
    for(int i=0;i<100;i++)
      {
      tempC = (5.0 * analogRead(13) * 100.0)/4095.0; // Sensor de Temp
      promtemp=tempC+promtemp;
      }  
      promtemp=promtemp/100;
     
}

void Spo2andHR()
{

 
  long irValue = particleSensor.getIR();


  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    
     
  }
  
  
}
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  Serial.println();
}

void pantalla()
{
Heltec.display->clear();
Heltec.display -> drawString(6,40,"BPM: "+String(beatAvg));
Heltec.display -> drawString(0,0,"ID=1");
Heltec.display -> drawString(6,50,"TEMP: "+String(promtemp));
Heltec.display -> drawString(6,30,"SPO2: "+String(Spo2));
Heltec.display ->display();
}

void sendLoRa(void *parameter)
{
  while(1)
  {
  LoRa.beginPacket();
  LoRa.setTxPower(14,RF_PACONFIG_PASELECT_PABOOST);
  LoRa.print(promtemp);
  LoRa.print("#");
  LoRa.print(beatAvg);
  LoRa.print("$");
  LoRa.print(Spo2);
  LoRa.print("/");
  LoRa.print("ID=1");
  LoRa.endPacket();
}
vTaskDelay(10);
}



void setup() {
  Wire1.begin(SDA,SCL);
  // put your setup code here, to run once:
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  setupADC();
  setupMAX30102();
  Heltec.begin(true,true,true,true,BAND);
  
  xTaskCreatePinnedToCore(
      sendLoRa, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Task1,  /* Task handle. */
      0); /* Core where the task should run */
  Heltec.display->clear();
  Heltec.display -> drawString(62,32,"HOLA");
  Heltec.display ->display();
delay(100);
}

void loop() 
{
 
leerADC();
Spo2andHR();
pantalla();
delay(10);
}
