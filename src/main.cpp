#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>

#include <HTTPClient.h>
#include <WiFiClientSecure.h>

#include <ArduinoJson.h>
#include <Wire.h>
#include <BH1750.h>

const char* ssid = "ADB-EBF141";
const char* password = "60501I0004969";

String serverName = "https://greenhouse.crmsystems.sk/api/";

// certificate for https://greenhouse.crmsystems.sk
// DST Root CA X3, valid until Mon Sep 30 2024, size: 1923 bytes 
const char* rootCACertificate = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIFYDCCBEigAwIBAgIQQAF3ITfU6UK47naqPGQKtzANBgkqhkiG9w0BAQsFADA/\n" \
"MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\n" \
"DkRTVCBSb290IENBIFgzMB4XDTIxMDEyMDE5MTQwM1oXDTI0MDkzMDE4MTQwM1ow\n" \
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n" \
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwggIiMA0GCSqGSIb3DQEB\n" \
"AQUAA4ICDwAwggIKAoICAQCt6CRz9BQ385ueK1coHIe+3LffOJCMbjzmV6B493XC\n" \
"ov71am72AE8o295ohmxEk7axY/0UEmu/H9LqMZshftEzPLpI9d1537O4/xLxIZpL\n" \
"wYqGcWlKZmZsj348cL+tKSIG8+TA5oCu4kuPt5l+lAOf00eXfJlII1PoOK5PCm+D\n" \
"LtFJV4yAdLbaL9A4jXsDcCEbdfIwPPqPrt3aY6vrFk/CjhFLfs8L6P+1dy70sntK\n" \
"4EwSJQxwjQMpoOFTJOwT2e4ZvxCzSow/iaNhUd6shweU9GNx7C7ib1uYgeGJXDR5\n" \
"bHbvO5BieebbpJovJsXQEOEO3tkQjhb7t/eo98flAgeYjzYIlefiN5YNNnWe+w5y\n" \
"sR2bvAP5SQXYgd0FtCrWQemsAXaVCg/Y39W9Eh81LygXbNKYwagJZHduRze6zqxZ\n" \
"Xmidf3LWicUGQSk+WT7dJvUkyRGnWqNMQB9GoZm1pzpRboY7nn1ypxIFeFntPlF4\n" \
"FQsDj43QLwWyPntKHEtzBRL8xurgUBN8Q5N0s8p0544fAQjQMNRbcTa0B7rBMDBc\n" \
"SLeCO5imfWCKoqMpgsy6vYMEG6KDA0Gh1gXxG8K28Kh8hjtGqEgqiNx2mna/H2ql\n" \
"PRmP6zjzZN7IKw0KKP/32+IVQtQi0Cdd4Xn+GOdwiK1O5tmLOsbdJ1Fu/7xk9TND\n" \
"TwIDAQABo4IBRjCCAUIwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMCAQYw\n" \
"SwYIKwYBBQUHAQEEPzA9MDsGCCsGAQUFBzAChi9odHRwOi8vYXBwcy5pZGVudHJ1\n" \
"c3QuY29tL3Jvb3RzL2RzdHJvb3RjYXgzLnA3YzAfBgNVHSMEGDAWgBTEp7Gkeyxx\n" \
"+tvhS5B1/8QVYIWJEDBUBgNVHSAETTBLMAgGBmeBDAECATA/BgsrBgEEAYLfEwEB\n" \
"ATAwMC4GCCsGAQUFBwIBFiJodHRwOi8vY3BzLnJvb3QteDEubGV0c2VuY3J5cHQu\n" \
"b3JnMDwGA1UdHwQ1MDMwMaAvoC2GK2h0dHA6Ly9jcmwuaWRlbnRydXN0LmNvbS9E\n" \
"U1RST09UQ0FYM0NSTC5jcmwwHQYDVR0OBBYEFHm0WeZ7tuXkAXOACIjIGlj26Ztu\n" \
"MA0GCSqGSIb3DQEBCwUAA4IBAQAKcwBslm7/DlLQrt2M51oGrS+o44+/yQoDFVDC\n" \
"5WxCu2+b9LRPwkSICHXM6webFGJueN7sJ7o5XPWioW5WlHAQU7G75K/QosMrAdSW\n" \
"9MUgNTP52GE24HGNtLi1qoJFlcDyqSMo59ahy2cI2qBDLKobkx/J3vWraV0T9VuG\n" \
"WCLKTVXkcGdtwlfFRjlBz4pYg1htmf5X6DYO8A4jqv2Il9DjXA6USbW1FzXSLr9O\n" \
"he8Y4IWS6wY7bCkjCWDcRQJMEhg76fsO3txE+FiYruq9RUWhiF1myv4Q6W+CyBFC\n" \
"Dfvp7OOGAN6dEOM4+qR9sdjoSYKEBpsr6GtPAQw4dy753ec5\n" \
"-----END CERTIFICATE-----\n" \
"";


//------------------PINS---------------------------
int OPEN_VALVE = 1; 
int CLOSE_VALVE = 45; 
int POWER = 42; 

static const uint8_t SCL_PIN = 41;
static const uint8_t SDA_PIN = 40;

// PWM interface
const int pwmPin = 2;

const int pwmHum = 3;

//------------------END PINS-----------------------
#define si7021Addr 0x40
#define BH1750Addr 0x23

BH1750 lightMeter;

const int dry = 3500; // value for dry sensor
const int wet = 1300; // value for wet sensor

WiFiMulti WiFiMulti;


// Not sure if WiFiClientSecure checks the validity date of the certificate. 
// Setting clock just to be sure...
void setClock() {
  configTime(0, 0, "pool.ntp.org");

  Serial.print(F("Waiting for NTP time sync: "));
  time_t nowSecs = time(nullptr);
  while (nowSecs < 8 * 3600 * 2) {
    delay(500);
    Serial.print(F("."));
    yield();
    nowSecs = time(nullptr);
  }

  Serial.println();
  struct tm timeinfo;
  gmtime_r(&nowSecs, &timeinfo);
  Serial.print(F("Current time: "));
  Serial.print(asctime(&timeinfo));
}


void initWifi()
{
  Serial.print("Connecting to WiFi ..");
  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP(ssid, password);

  int attempts = 0; 

  while ((WiFiMulti.run() != WL_CONNECTED)) {
    Serial.print('.');
    delay(1000);

  }
  Serial.println(WiFi.localIP());
}


void openValve()
{
  digitalWrite(POWER, LOW); 
  digitalWrite(OPEN_VALVE, LOW); 
  
  delay(13000); 

  digitalWrite(POWER, HIGH); 
  digitalWrite(OPEN_VALVE, HIGH); 
}


void closeValve()
{
  digitalWrite(POWER, LOW); 
  digitalWrite(CLOSE_VALVE, LOW); 
  
  delay(13000); 

  digitalWrite(POWER, HIGH); 
  digitalWrite(CLOSE_VALVE, HIGH); 
}


float getHumidity()
{
  unsigned int data[2];
 
  Wire.beginTransmission(si7021Addr);
  //Send humidity measurement command
  Wire.write(0xF5);
  Wire.endTransmission();
  delay(500);
 
  // Request 2 bytes of data
  Wire.requestFrom(si7021Addr, 2);
  // Read 2 bytes of data to get humidity
  if(Wire.available() == 2)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
  }
 
  // Convert the data
  float humidity  = ((data[0] * 256.0) + data[1]);
  humidity = ((125 * humidity) / 65536.0) - 6;
 
  return humidity; 
}


float getTemperature(){
  unsigned int data[2];

  Wire.beginTransmission(si7021Addr);
  // Send temperature measurement command
  Wire.write(0xF3);
  Wire.endTransmission();
  delay(500);
 
  // Request 2 bytes of data
  Wire.requestFrom(si7021Addr, 2);
 
  // Read 2 bytes of data for temperature
  if(Wire.available() == 2)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
  }
 
  // Convert the data
  float temp  = ((data[0] * 256.0) + data[1]);
  float celsTemp = ((175.72 * temp) / 65536.0) - 46.85;
 
  return celsTemp;
}


int gas_concentration_PWM() {
  while (digitalRead(pwmPin) == LOW) {};
  long t0 = millis();
  while (digitalRead(pwmPin) == HIGH) {};
  long t1 = millis();
  while (digitalRead(pwmPin) == LOW) {};
  long t2 = millis();
  long tH = t1-t0;
  long tL = t2-t1;
  long ppm = 5000L * (tH - 2) / (tH + tL - 4);
  while (digitalRead(pwmPin) == HIGH) {};
  delay(10);
  
  return int(ppm);
}

int getSoilHum()
{

  int sensorVal = analogRead(pwmHum);
  int percentageHumididy = map(sensorVal, wet, dry, 100, 0);

  return percentageHumididy; 
}


void setup() 
{
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Setup start");

  pinMode(POWER, OUTPUT); 
  digitalWrite(POWER, HIGH); 

  pinMode(CLOSE_VALVE, OUTPUT); 
  digitalWrite(CLOSE_VALVE, HIGH); 

  pinMode(OPEN_VALVE, OUTPUT); 
  digitalWrite(OPEN_VALVE, HIGH); 

  pinMode(pwmPin, INPUT_PULLUP);
  
  Wire.begin(SDA_PIN, SCL_PIN); 

  initWifi(); 

  setClock();  

  Serial.println("Setup done");
}


void loop() {
    
  // uint32_t f = getCpuFrequencyMhz();
  // char string[16];

  // sprintf(string, "CPU Freq: %i", f);
  // Serial.println(string);

  if(WiFiMulti.run() != WL_CONNECTED){
    WiFi.reconnect();
  
    if(WiFiMulti.run() != WL_CONNECTED){
      initWifi();
    } 
  }


  WiFiClientSecure *client = new WiFiClientSecure;
  if(client) {
    client -> setCACert(rootCACertificate);

    {  // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is 
      HTTPClient http;

      StaticJsonDocument<200> doc;

      float temp = getTemperature(); 
      float hum =getHumidity();
      
      Serial.print("Temp: ");
      Serial.println(temp);
      
      Serial.print("Hum: ");
      Serial.println(hum);


      // float lux = lightMeter.readLightLevel();
      // Serial.print(lux);
      // Serial.println("lx");

      //int ppm_PWM = gas_concentration_PWM();

      ///int soil = getSoilHum(); 

      String serverPath = serverName + "update?temperaturea="+temp+"&humiditya="+hum; // +"&lighta="+lux+"&co2="+ppm_PWM+"&soil="+soil;
      
      // Your Domain name with URL path or IP address with path
      if(http.begin(*client, serverPath.c_str())){
      
        // If you need Node-RED/server authentication, insert user and password below
        //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");
        
        // Send HTTP GET request
        int httpResponseCode = http.GET();
        

        // httpCode will be negative on error
        if (httpResponseCode > 0) {
            // HTTP header has been send and Server response header has been handled
            Serial.printf("[HTTPS] GET... code: %d\n", httpResponseCode);

          if (httpResponseCode == HTTP_CODE_OK || httpResponseCode == HTTP_CODE_MOVED_PERMANENTLY) {

            String response = http.getString();

            Serial.println(response);

            deserializeJson(doc, response);

            if(doc["data"][0]["irrigate"]["time"]){
              long time = doc["data"][0]["irrigate"]["time"];

              Serial.println(time); 

              openValve(); 
              delay(time*1000); 
              closeValve(); 
            }
          }
        } else {
          Serial.printf("[HTTPS] GET... failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
        }
      }
    }       // End extra scoping block
    
    delete client;

   } else {
    Serial.println("Unable to create client");
  }


  delay(20000);

  Serial.println("LOoop");
}