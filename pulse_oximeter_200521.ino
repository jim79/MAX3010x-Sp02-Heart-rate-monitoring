/*
  ## Hardware Connections (ESP32 Arduino):
  -VIN = 3.3V
  -GND = GND
  -SDA = 21 (or SDA)
  -SCL = 22 (or SCL)

*/

#include <Wire.h>
#include "MAX30105.h" //sparkfun MAX3010X library
#include "spo2_algorithm.h"
#include "heartRate.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3C for 128x64, 0x3D for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MAX30105 particleSensor;

// #define MAX30105 //if you have Sparkfun's MAX30105 breakout board , try #define MAX30105 

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

long samplesTaken = 0; //Counter for calculating the Hz or read rate
long unblockedValue; //Average IR at power up
long startTime; //Used to calculate measurement rate

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;


#define USEFIFO
void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize sensor
  while (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
    //while (1);
  }

  // Configure the 128 x 64 OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally  
    Serial.println(F("SSD1306 allocation failed"));
    
  display.clearDisplay();

  display.setTextSize(2);             
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(28,10);             
  display.println("Pulse");
  display.setCursor(10,10);  
  display.println("\n Oximeter");
  display.setCursor(10,30); 
  display.println("\n   2.2");
  display.display();
  delay(2000);
  display.clearDisplay();
  
  //Configure the MAX 3010x sensor
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA 
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 800; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  // Set up the sensor
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these setting
  
} // setup function ends

      void loop(){
      finger_placed();
      bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 50sps
  
      //finger_placed(); // Check if pinger placed or not
  
      //read the first 100 samples, and determine the signal range
      for (byte i = 0 ; i < bufferLength ; i++)
        {
        while (particleSensor.available() == false) //do we have new data?
          particleSensor.check(); //Check the sensor for new data
    
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample(); //We're finished with this sample so move to next sample

        }
    
      //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
      maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
      
      switch (validSPO2 && validHeartRate ){
      
      case 0:
      display.clearDisplay();
      display.display();
      Serial.println("finger not steady...");
      irBuffer[100]= 0;
      redBuffer[100] = 0;  
      break;

      case 1:
      display.setCursor(0,10); 
      display.clearDisplay();
      display.setTextSize(2);
      display.print("SpO2 ");
      display.print(spo2);
      display.print(" %");
      display.setCursor(0,50);
      display.print("HR ");
      display.print(heartRate);
      display.print(" bpm");
      display.display();
      break;
     }
     

      
      //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
      while (1)
      {
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
    
          // digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
    
          redBuffer[i] = particleSensor.getRed();
          irBuffer[i] = particleSensor.getIR();
          particleSensor.nextSample(); //We're finished with this sample so move to next sample
          switch (validSPO2){
      
          case 0:
          display.clearDisplay();
          delay(500);
          display.display();
          // Serial.println("finger not steady...");
          irBuffer[100]= 0;
          redBuffer[100] = 0; 
          break;
    
          case 1:
          display.setCursor(0,10); 
          display.clearDisplay();
          display.setTextSize(2);
          display.print("SpO2 ");
          display.print(spo2);
          display.print(" %");
          display.setCursor(0,50);
          display.print("HR ");
          display.print(heartRate);
          display.print(" bpm");
          display.display();
          break;
         }
        }
    
        //After gathering 25 new samples recalculate HR and SP02
       maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
       }
    }


    // Functions 
   // Check finger placed or not 
    void finger_placed(){
     //Take an average of IR readings at power up
    unblockedValue = 0;
  for (byte x = 0 ; x < 32 ; x++)
  {
    unblockedValue += particleSensor.getIR(); //Read the IR value
  }
  unblockedValue /= 32;

  startTime = millis();

  
  samplesTaken++;
  
  long currentDelta = particleSensor.getIR() - unblockedValue;
  
  do{
    if (currentDelta > (long)100){
    display.setTextSize(1);
    display.setCursor(0,20); 
    display.println("\n  Finger detected..");
    display.display();
    delay(500);
    display.clearDisplay();
    display.setCursor(0,20);
    display.print("\n  Please wait....");
    delay(500);
    display.display();
    display.clearDisplay(); 
    break;
    }
  else
  {
    display.setTextSize(1);
    display.setCursor(0,20); 
    display.println(F("\n  No finger placed"));
    delay(500);
    display.display();
    display.clearDisplay(); 
    finger_placed();
  }
  }while(currentDelta > (long)100);
  return;
  } // finger placed function ends


    // Wake up sensor
    void sensor_wake(){
    particleSensor.wakeUp();  // Wakeup MAX3010x
    //Serial.println(F("Sensor Wake up"));
    display.println(F("Sensor wakeup"));
    display.display();
    delay(2000);
    display.clearDisplay(); 
    return;
    }

    // Shut down sensor
    void sensor_shut(){
    particleSensor.shutDown(); // Powers down the MAX3010x but retains all configuration
    //Serial.println(F("Sensor shut down"));
    display.println(F("Sensor off"));
    display.display();
    delay(2000);
    display.clearDisplay();
    return;
    }


    
   
