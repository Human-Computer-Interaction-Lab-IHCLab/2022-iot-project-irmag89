#define MC38_PIN  15 
#define MC382_PIN 13
#include "UbidotsEsp32Mqtt.h"
const char *UBIDOTS_TOKEN = "BBFF-FgvdQjSnkn00IZjWyOUIVt64K0cppM";  // Put here your Ubidots TOKEN
const char *WIFI_SSID = "iPhone de Irma";      // Put here your Wi-Fi SSID
const char *WIFI_PASS = "skip220554";      // Put here your Wi-Fi password
const char *DEVICE_LABEL = "proyecto";   // Put here your Device label to which data  will be published
#define timeSeconds 10

// Set GPIOs for LED and PIR Motion Sensor
const int led = 26;
const int motionSensor = 27;
int estado=0;

// Timer: Auxiliary variables
  unsigned long now = millis();
  unsigned long lastTrigger = 0;
  boolean startTimer = false;

// Checks if motion was detected, sets LED HIGH and starts a timer
void IRAM_ATTR detectsMovement() {
  Serial.println("MOTION DETECTED!!!");
  digitalWrite(led, HIGH);
    startTimer = true;
    lastTrigger = millis();
}


const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds
Ubidots ubidots(UBIDOTS_TOKEN);
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
int pin2LED=12;
int pinLED = 2;
void setup()
{
  pinMode(MC38_PIN, INPUT_PULLDOWN);
  pinMode(pinLED, OUTPUT);
  Serial.begin(115200);
  pinMode(MC382_PIN, INPUT_PULLDOWN);
 
  pinMode(pin2LED, OUTPUT);
  pinMode(motionSensor, INPUT);
  pinMode(led, OUTPUT);
  
  // ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi("iPhone de Irma", "skip220554");
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  
  // Serial port for debugging purposes
  
  
  // PIR Motion Sensor mode INPUT_PULLUP
  //pinMode(motionSensor, INPUT_PULLUP);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  //attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);

  // Set LED to LOW
 // pinMode(led, OUTPUT);
 // digitalWrite(led, LOW);
}

void loop()
{
     
  if (!ubidots.connected())
  {
    ubidots.reconnect();
    
  }
  int value = digitalRead(MC38_PIN);
  if (value == HIGH) {
    digitalWrite(pinLED, LOW);
  } else {
    digitalWrite(pinLED, HIGH);
  }
  int value2 = digitalRead(MC382_PIN);
  if (value2 == HIGH) {
    digitalWrite(pin2LED, LOW);
  } else {
    digitalWrite(pin2LED, HIGH);
  }
  
  delay(100);
  ubidots.add("aperturapuerta",value);
  ubidots.add("aperturaventana",value2);
  ubidots.publish(DEVICE_LABEL);
  ubidots.loop();
//
    estado = digitalRead(motionSensor);
    if (estado==HIGH){
      digitalWrite(led, HIGH);
      delay(2000);
    }else
    {
      digitalWrite(led, LOW);
    }
    int value3 = digitalRead(motionSensor);
    if (value3 == HIGH) {
       //digitalWrite(led, LOW);
    } else {
     //digitalWrite(led, HIGH);
    }
    delay(100);
   ubidots.add("presencia",value3);
  ubidots.publish(DEVICE_LABEL);
  ubidots.loop();
  now = millis();
  //Turn off the LED after the number of seconds defined in the timeSeconds variable
  //if(startTimer && (now - lastTrigger > (timeSeconds*1000))) {
    //Serial.println("Motion stopped...");
    //digitalWrite(led, LOW);
    //startTimer = false;


  //}

}
