#include <Wire.h>
#include "MAX30105.h"
#include "heltec.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;
TaskHandle_t Task1;
#define MAX_BRIGHTNESS 255
#define BAND    915E6 //Banda del LoRa

float tempC; // Variable para obtener temperatura
float promtemp; //Promedio Temperatura
const byte RATE_SIZE = 8; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
bool flag=true;

/* Variables SPO2 & HR*/
float beatsPerMinute;
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid



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

  /* Aqui va lo del SPO2*/

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

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
/* 
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
  */

/* SPo2  */

bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

 if(flag)
 {
  //lee los primeros 100 samples y determina ek rango de la sena
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
    particleSensor.check(); //Check the sensor for new data
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }
 flag=false;
 //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
 }
 
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 seconnd
  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
  for (byte i = 25; i < 100; i++)
  {
  redBuffer[i - 25] = redBuffer[i];
  irBuffer[i - 25] = irBuffer[i];
  }
   //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
  {
  while (particleSensor.available() == false) //do we have new data?
  particleSensor.check(); //Check the sensor for new data
  //digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);


}
}




void pantalla()
{
Heltec.display->clear();
Heltec.display -> drawString(6,40,"BPM: "+String(heartRate));
Heltec.display -> drawString(0,0,"ID=1");
Heltec.display -> drawString(6,50,"TEMP: "+String(promtemp));
Heltec.display -> drawString(6,30,"SPO2: "+String(spo2));
Heltec.display ->display();
}

void sendLoRa(void *parameter)
{
  delay(4000);
  while(1)
  {
  LoRa.beginPacket();
  LoRa.setTxPower(14,RF_PACONFIG_PASELECT_PABOOST);
  LoRa.print(promtemp);
  LoRa.print("#");
  LoRa.print(heartRate);
  LoRa.print("$");
  LoRa.print(spo2);
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
delay(500);
}
