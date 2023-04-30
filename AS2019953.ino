//AS2019953
//K.M.S.I.RATHNAYAKE

#define BLYNK_PRINT Serial

//FIRMWARE CONFIGURATION OF BLYNK
#define BLYNK_TEMPLATE_ID "TMPL-cO4N03r"
#define BLYNK_DEVICE_NAME "FINAL PROJECT"
#define BLYNK_AUTH_TOKEN "qG03sgcneyH8r-GmSCfdn3AGhtAv1BVh"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <Servo.h>
#include <SPI.h>


char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Imalsha";
char pass[] = "12345678";

//define pins
int SOUNDPIN = 34;
#define SERVOPIN 13
#define DHTPIN 14
#define FANPIN 15
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
Servo myservo;

//define variabls
int pos = 0;    // variable to store the servo position
int sensorValue = 0;  // variable to store the value coming from the sensor
int  arrayList[1000] = {}; //declare an array to store sensor values
int stop_sound_count = 0;
int start_sound_count = 0;
static int count = 0;
static int intensity = 1020;

//----------------------------------------------function which detect sound and swing the baby cradle--------------------------------
void soundSensor() {

  sensorValue = analogRead(SOUNDPIN);// read the value from the sensor
  Serial.print("Sound Sensor Value:");
  Serial.println(sensorValue);
  arrayList[count] = sensorValue;
  
  if (sensorValue >= intensity) {

    Blynk.virtualWrite(V3, 1);
   
    for (pos = 0; pos <= 180; pos++) { // goes from 0 degrees to 180 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15 ms for the servo to reach the position
    }
    
    for (pos = 180; pos >= 0; pos --) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15 ms for the servo to reach the position
    }
    
  }

  else{

    for (int i = 1; i <= 40; i++) {

      if (arrayList[count - i] < intensity) {
        stop_sound_count++;
      }
      if (stop_sound_count >39) {
        Blynk.virtualWrite(V3, 0);
        delay(5000);
      }
    }
    stop_sound_count=0;
  }
  count++;
  
}
//------------------------------------------------------------------end function--------------------------------------------------------------

//------------------------------------------------------function which detect the temperature and humidity and also on/off the fan-----------------------------------------
void sendTemperatureSensor()
{
  
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Serial.print("humidity:");
  Serial.println(h);

  Blynk.virtualWrite(V1, h);
 
  Serial.print("temperature");
  Serial.println(t);

  Blynk.virtualWrite(V0, t);
  
  if (t > 25) {
    
    digitalWrite(FANPIN, HIGH);
    Blynk.virtualWrite(V2, 1);
  }
  else {
    digitalWrite(FANPIN, LOW);
    Blynk.virtualWrite(V2, 0);
  }

}
//-------------------------------------------------------------------------end function------------------------------------------------------------------------------------

//--------------------------------------------------------------setup function---------------------------------------------------------------------------------------------
void setup() {

  pinMode(SERVOPIN, OUTPUT);
  pinMode(FANPIN, OUTPUT);

  Serial.begin(115200);

  Blynk.begin(auth,ssid,pass);

  myservo.attach(SERVOPIN);
  dht.begin();

}
//--------------------------------------------------------------end function-----------------------------------------------------------------------------------------------

//--------------------------------------------------------------loop function--------------------------------------------------------------------------------------------
void loop() {

  
  sendTemperatureSensor();
  soundSensor();
  Blynk.run();
}
//-------------------------------------------------------------end function-----------------------------------------------------------------------------------------------
