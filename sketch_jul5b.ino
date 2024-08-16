#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 2Kb
#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <SPI.h>
#include <SD.h>
#include <Ticker.h>
#include <vector> // Include the vector library
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Update.h>



int contentLength = 0;
bool isValidContentType = false;

#define DHTPIN 4
#define DHTTYPE DHT22
#define PWRPIN 13
#define NETPIN 14
// Initialize DHT22 sensor
DHT dht(DHTPIN, DHTTYPE);
HardwareSerial GSM_AT(1);



// Initialize SHT30 sensor
Adafruit_SHT31 sht30 = Adafruit_SHT31();
#define I2C_SDA 25  // Replace with your desired SDA pin
#define I2C_SCL 27  // Replace with your desired SCL pin

Preferences preferences;
#define ONE_WIRE_BUS 12

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);


DeviceAddress sensor_address; 
String ssid = "";
String password = "";

struct SensorConfig {
    int id;
    String token;
    int timer;
    int minthreshold;
    int maxthreshold;
    int thresholdtimer;
    int index;
    unsigned long timestamp;
     unsigned long th_timestamp;
};

std::vector<SensorConfig> sensorConfigs;
std::vector<int> Numder_of_measurments; // Create a dynamic array (vector) of integers
std::vector<float> temperatures; // Create a dynamic array (vector) of integers
std::vector<float> Max_temperatures; // Create a dynamic array (vector) of integers
std::vector<float> Min_temperatures; // Create a dynamic array (vector) of integers
std::vector<float> Avg_temperatures; // Create a dynamic array (vector) of integers
std::vector<float> err_counter; // Create a dynamic array (vector) of integers
float maxsht=0;
float minsht=0;
float maxdht=0;
float mindht=0;
float avgdht=0;
float avgsht=0;
int dhtmeasureno=0;
int shtmeasureno=0;
float shthumid=0;
float dhthumid=0;
float dhttemp=0;
float shttemp=0;
unsigned long DHTprev=0;
unsigned long SHTprev=0;
unsigned long DHTprevmsg=0;
unsigned long SHTprevmsg=0;


int DHT_timer=0;
int DHT_THRESHOLD_timer=0;
int DHT_MAX_THRESHOLD=0;
int DHT_MIN_THRESHOLD=0;
int DHT_ID=0;
String DHT_TOKEN="";



int position=0;

int SHT_timer=0;
int SHT_THRESHOLD_timer=0;
int SHT_MAX_THRESHOLD=0;
int SHT_MIN_THRESHOLD=0;
int SHT_ID=0;
String SHT_TOKEN="";



int sensorID_For_Door=-1;
int stati;
bool flag= true;
bool lastflag=true;
#define GSM_PIN ""
const char apn[]  = ""; //SET TO YOUR APN
const char gprsUser[] = "";
const char gprsPass[] = "";

#define SMS_TARGET  "01220213222"
String Phonenumber="";
String AuthPhone="01097966535";
String AuthPhoneme="01094835770";
String server_url = "";
int total_sensors=0;
unsigned long prev = 0;
unsigned long prev2 = 0;
String pirority="4G";

#define SerialAT  GSM_AT

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, Serial);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif
TinyGsmClient client4G(modem);
WiFiClient client;
#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP  60

#define UART_BAUD   115200
#define PIN_DTR     34
#define PIN_TX      21
#define PIN_RX      22
#define PWR_PIN     33

// Function declarations
void GSM_SetBaudrate() 
{
  Serial.println("###GSM_SetBaudrate###");
  GSM_AT.println("AT&V "); //To see the saved settings
  //SerialMon.println("stop saved settings");
  delay(150);
  while (GSM_AT.available() > 0 ) 
  {  //todo add timeout
    String serialmsg = GSM_AT.readString();
    Serial.print("serialmsg before trim");
  Serial.println(serialmsg);
  serialmsg.trim();

    Serial.println(serialmsg);
    if (serialmsg.indexOf("+IPR: 0") > -1 ) 
    {
      Serial.println("baud rate is auto");
    } 
    else 
    {
      Serial.print("setting baud rate to auto");
      GSM_AT.println("AT+IPR=0"); //0 is autobaud rate supports upto 57600
      GSM_AT.println("AT&W"); //Then to save this (to last between boot cycles)

    }
  }

}

void setup() {
  Serial.begin(115200);
  delay(10);
//  pinMode(PWR_PIN, OUTPUT);
//  digitalWrite(PWR_PIN, HIGH);
//  delay(300);
//  digitalWrite(PWR_PIN, LOW);
 pinMode(PWRPIN, OUTPUT);
 digitalWrite(PWRPIN, HIGH);
 pinMode(NETPIN, OUTPUT);

pinMode(2,INPUT_PULLUP);



  delay(1000);

  GSM_AT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  Wire.begin(I2C_SDA, I2C_SCL);
  GSM_AT.setTimeout(70);
  delay(500);
  GSM_SetBaudrate();
  String res;
  Serial.println(F("Initializing modem..."));
  modem.init();
  delay(5000);
  String modemInfo = modem.getModemInfo();
  Serial.print("Modem: ");
  Serial.println(modemInfo);
  String IMEI = modem.getIMEI();
  Serial.println("IMEI : "+IMEI); 

  if (strlen(GSM_PIN) && modem.getSimStatus() != 3) {
    modem.simUnlock(GSM_PIN);
  }

  Serial.println("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println(F("Network connection failed"));
    ESP.restart();
  }

  if (modem.isNetworkConnected()) {
    Serial.println(F("Network connected"));
     digitalWrite(NETPIN, HIGH);
  } else {
    Serial.println("Network not connected");
  }

  
  loadSensorConfigs();   
  connectWiFi(ssid,password);
  
  dht.begin();
  sensors.begin();
  for(int i=0; i<sensors.getDeviceCount(); i++){
    reset_sensor(i);  //resizeing the measurments arrays to be = the sensors and have elements of 0
  }
   //Serial.println(String(sensors.getDeviceCount()) + " Sensors Found");
  Get_temprature_Dallas();
  if (checkDHT22()) {
    Serial.println("DHT22 sensor is connected.");
  } else {
    Serial.println("DHT22 sensor is NOT connected.");
  }

  if (checkSHT30()) {
    Serial.println("SHT30 sensor is connected.");
  } else {
    Serial.println("SHT30 sensor is NOT connected.");
  }
  // Initialize HTTP
  httpInitialize();
  GSM_AT.println(F("AT\r")); // Configuring TEXT mode
  updateSerial();
  delay(1000);
  GSM_AT.println(F("AT+CMGF=1\r")); // Configuring TEXT mode
  updateSerial();
  delay(1000);
  // GSM_AT.println(F("AT+CNMI=2,1,0,0,0"));
  // updateSerial();
  // delay(1000);
  GSM_AT.println(F("AT+CMGD=1,4"));
  delay(1000);



}
// void loop(){
//   GSM_AT.println(F("AT+CMGL=\"REC UNREAD\"\r")); // Decides how newly arrived SMS messages should be handledupdateSerial(); 
//  updateSerial();
//  delay(5000);
 
// }

void loop() {
  modem.maintain();

 if (!digitalRead(2)) {
    stati = 1;
    flag = true;
  }
  else if (digitalRead(2)) {
    stati = 0;
    flag = false;
  }

  if (flag != lastflag) {
    Serial.println(stati);
    lastflag = flag;
    if(sensorID_For_Door!=-1){
      delay(2000);
    if(senddoorstatus(sensorID_For_Door)){
      digitalWrite(NETPIN, HIGH);
      }else{
        digitalWrite(NETPIN, LOW);
      }
      
      }
  }

  GSM_AT.println(F("AT+CMGL=\"REC UNREAD\"\r")); // Decides how newly arrived SMS messages should be handledupdateSerial(); 
  updateSerial();
  unsigned long currentMillis = millis();
  if(currentMillis-prev2>=5000){ 
  if(SHT_ID!=0){
   if(!checkSHT30()){
          removesht();
        }}
  if(DHT_ID!=0){      
   if(!checkDHT22()){
          removedht();
        }}
             prev2=currentMillis;
      }
      // if(DHT_THRESHOLD_timer!=0){
      //   if(currentMillis-DHTprevmsg>=DHT_THRESHOLD_timer){
      //         bool res; 
      //         bool check=false;
      //         float avgtemp=avgdht/dhtmeasureno;
      //         Serial.println(avgtemp);
      //         if (avgtemp>DHT_MAX_THRESHOLD){
      //         check=true;
      //         delay(2500);
      //         res = modem.sendSMS(AuthPhone, "DHT Temperature above Maximum Threshold");
      //           }
      //         else if(avgtemp<DHT_MIN_THRESHOLD){
      //           check=true;
      //           delay(2500);
      //          res = modem.sendSMS(AuthPhone, "DHT Temperature below Minimum Threshold");
      //           }
      //           String status=String(res ? "Threshold msg sent to "+ AuthPhone : "fail");
      //           Serial.print(F("SMS: "));
      //           Serial.println(status);
      //           if(check==true && status=="fail"){
      //             DHTprevmsg=0;
      //           }if(isnan(avgtemp)){
      //             DHTprevmsg=0;
      //           }
      //             else{
      //             DHTprevmsg=currentMillis;
      //           }
      //             }
      // }
  if(DHT_timer!=0){
      if(currentMillis-DHTprev>=DHT_timer){
          float avgtemp=avgdht/dhtmeasureno;
          Serial.println(avgtemp);
          Serial.println(DHT_TOKEN);
          if(pirority=="4G"){
            if(!sendHTTP(server_url,DHT_ID, dhthumid,avgtemp,maxdht,mindht, DHT_TOKEN)){
             digitalWrite(NETPIN, LOW);  
            if(check_wifi_credentials()=="OK"){
            if(!sendHTTP_via_wifi(server_url,DHT_ID, dhthumid, avgtemp, maxdht,mindht, DHT_TOKEN)){
              Serial.println(F("Can't Send via Wi-Fi"));
            }
            
            }else{
              Serial.println(F("NOT Connected to Wi-Fi"));
                }   //send wifi if 4G is not available and credentials exist
            }else{
              digitalWrite(NETPIN, HIGH);
            }    
         
          }

      else if(pirority=="Wi-Fi"){

        if(check_wifi_credentials()=="OK"){
          if(!sendHTTP_via_wifi(server_url,DHT_ID, dhthumid, avgtemp, maxdht,mindht, DHT_TOKEN)){
           Serial.println(F("Can't Send via Wi-Fi"));
           if(!sendHTTP(server_url,DHT_ID, dhthumid, avgtemp, maxdht,mindht, DHT_TOKEN)){
           Serial.println(F("Can't Send via 4G"));
           digitalWrite(NETPIN, LOW);
           }else{
            digitalWrite(NETPIN, HIGH);
           }
            }
            //send wifi if 4G is not available and credentials exist
          }
            else{
            Serial.println(F("NOT Connected to Wi-Fi"));
            if(!sendHTTP(server_url,DHT_ID, dhthumid, avgtemp, maxdht,mindht, DHT_TOKEN)){
            Serial.println(F("Can't Send via 4G"));
           digitalWrite(NETPIN, LOW);
           }else{
            digitalWrite(NETPIN, HIGH);
           }
                } 
    }  

        DHTprev=currentMillis;
        resetdht();
        }

      }



      //  if(SHT_THRESHOLD_timer!=0){
      //   if(currentMillis-SHTprevmsg>=SHT_THRESHOLD_timer){
      //         bool res; 
      //         bool check=false;
      //         float avgtemp=avgsht/shtmeasureno;
      //         Serial.println(avgtemp);
      //         if (avgtemp>SHT_MAX_THRESHOLD){
      //         check=true;
      //         delay(2500);
      //         res = modem.sendSMS(AuthPhone, "SHT Temperature above Maximum Threshold");
      //           }
      //         else if(avgtemp<SHT_MIN_THRESHOLD){
      //           check=true;
      //           delay(2500);
      //          res = modem.sendSMS(AuthPhone, "SHT Temperature below Minimum Threshold");
      //           }
      //           String status=String(res ? "Threshold msg sent to "+ AuthPhone : "fail");
      //           Serial.print(F("SMS: "));
      //           Serial.println(status);
      //           if(check==true && status=="fail"){
      //             SHTprevmsg=0;
      //           }if(isnan(avgtemp)){
      //             DHTprevmsg=0;
      //           }else{
      //             SHTprevmsg=currentMillis;
      //           }
      //             }
      // }
                
      if(SHT_timer!=0){
        if(currentMillis-SHTprev>=SHT_timer){
              float avgtemp=avgsht/shtmeasureno;
              Serial.println(avgtemp);
          if(pirority=="4G"){
            if(!sendHTTP(server_url,SHT_ID, shthumid, avgtemp, maxsht,minsht, SHT_TOKEN)){
              digitalWrite(NETPIN, LOW);
            if(check_wifi_credentials()=="OK"){
            if(!sendHTTP_via_wifi(server_url,SHT_ID, shthumid, avgtemp, maxsht,minsht, SHT_TOKEN)){
              Serial.println(F("Can't Send via Wi-Fi"));
            }
            
            }else{
              Serial.println(F("NOT Connected to Wi-Fi"));
                }   //send wifi if 4G is not available and credentials exist
            }else{
              digitalWrite(NETPIN, HIGH);
            }    
         
          }

      else if(pirority=="Wi-Fi"){
        if(check_wifi_credentials()=="OK"){
          if(!sendHTTP_via_wifi(server_url,SHT_ID, shthumid, avgtemp, maxsht,minsht, SHT_TOKEN)){
           Serial.println(F("Can't Send via Wi-Fi"));
           if(!sendHTTP(server_url,SHT_ID, shthumid, avgtemp, maxsht,minsht, SHT_TOKEN)){
           Serial.println(F("Can't Send via 4G"));
           digitalWrite(NETPIN, LOW);
           }else{
            digitalWrite(NETPIN, HIGH);
           }
            }
          }
            else{
            Serial.println(F("NOT Connected to Wi-Fi"));
            if(!sendHTTP(server_url,SHT_ID, shthumid, avgtemp, maxsht,minsht, SHT_TOKEN)){
              digitalWrite(NETPIN, LOW);
            Serial.println(F("Can't Send via 4G"));
           }else{
            digitalWrite(NETPIN, HIGH);
           }
                } 
      }  

        SHTprev=currentMillis;
        resetsht();
        }

      }
        




 

  if(sensorConfigs.size()>0){
       err_counter.resize(sensorConfigs.size());
       if(currentMillis-prev>=5000){ 
        Get_temprature_Dallas();
        prev=currentMillis;
      }
   
    while(sensorConfigs.size()>total_sensors){ //incase if any sensor is disconnected, remove the last configration step on sensor
     sensorConfigs.pop_back(); 
     err_counter.resize(sensorConfigs.size());
     saveSensorConfigs();
    }

   for (size_t i = 0; i < sensorConfigs.size(); ++i) {
    if(Min_temperatures[i]==-127){
      err_counter[i]++;
      reset_sensor(i);
      if(err_counter[i]==10){
        for(int j = i+1; j < sensorConfigs.size(); j++){
          sensorConfigs[j].index=sensorConfigs[j].index-1;
        }
        err_counter[i]=0;
        sensorConfigs.erase(sensorConfigs.begin() + i);
        delay(2500);
        bool res = modem.sendSMS(AuthPhone, "Sensor "+ String(i+1) + " is not connected properly\nConfiguration is deleted");
        String status=String(res ? "OK" : "fail");
        Serial.print(F("SMS: "));
        Serial.println(status);
      }


    } else{
    if(currentMillis  - sensorConfigs[i].timestamp>=sensorConfigs[i].timer){
    int humdity = 0; 
    if(Numder_of_measurments[i]==0){Numder_of_measurments[i]=1;}
    Avg_temperatures[i]=Avg_temperatures[i]/Numder_of_measurments[i];
    if(pirority=="4G"){
    if(!sendHTTP(server_url,sensorConfigs[i].id, humdity, Avg_temperatures[i], Max_temperatures[i],Min_temperatures[i], sensorConfigs[i].token)){
      digitalWrite(NETPIN, LOW);
        if(check_wifi_credentials()=="OK"){
          if(!sendHTTP_via_wifi( server_url,sensorConfigs[i].id, humdity, Avg_temperatures[i], Max_temperatures[i],Min_temperatures[i], sensorConfigs[i].token)){
           Serial.println(F("Can't Send via Wi-Fi"));
            }
            
          }else{
            Serial.println(F("NOT Connected to Wi-Fi"));
                }   //send wifi if 4G is not available and credentials exist
    }else{
      digitalWrite(NETPIN, HIGH);
    }    
         
          }

    else if(pirority=="Wi-Fi"){
    if(check_wifi_credentials()=="OK"){
          if(!sendHTTP_via_wifi( server_url,sensorConfigs[i].id, humdity, Avg_temperatures[i], Max_temperatures[i],Min_temperatures[i], sensorConfigs[i].token)){
           Serial.println(F("Can't Send via Wi-Fi"));
           if(!sendHTTP(server_url,sensorConfigs[i].id, humdity, Avg_temperatures[i], Max_temperatures[i],Min_temperatures[i], sensorConfigs[i].token)){
           Serial.println(F("Can't Send via 4G"));
           digitalWrite(NETPIN, LOW);
           }else{
            digitalWrite(NETPIN, HIGH);
           }
            }
            //send wifi if 4G is not available and credentials exist
          }
            else{
            Serial.println(F("NOT Connected to Wi-Fi"));
            if(!sendHTTP(server_url,sensorConfigs[i].id, humdity, Avg_temperatures[i], Max_temperatures[i],Min_temperatures[i], sensorConfigs[i].token)){
            Serial.println(F("Can't Send via 4G"));
           }
                } 
    }

      reset_sensor(i);
      sensorConfigs[i].timestamp=currentMillis;
      Get_temprature_Dallas();
     //sending http 4G
    
    }
  // if(sensorConfigs[i].thresholdtimer!=0){
  // if(currentMillis  - sensorConfigs[i].th_timestamp>=sensorConfigs[i].thresholdtimer){   //send message of temprature exceed threshold
  //   bool res; 
  //   bool check=false;
    
  //   int avgtemp =Avg_temperatures[i]/Numder_of_measurments[i];
  //   Serial.println(avgtemp);
  //   if (avgtemp>sensorConfigs[i].maxthreshold){
  //    check=true;
  //    delay(2500);
  //    res = modem.sendSMS(AuthPhone, "Sensor "+ String(i+1) + " Temperature above Maximum Threshold");
  //     }
  //   else if(avgtemp<sensorConfigs[i].minthreshold){
  //     check=true;
  //     delay(2500);
  //    res = modem.sendSMS(AuthPhone, "Sensor "+ String(i+1) +" Temperature below Minimum Threshold");
  //     }
  //     String status=String(res ? "Threshold msg sent to "+ AuthPhone : "fail");
  //     Serial.print(F("SMS: "));
  //     Serial.println(status);
  //     if(check==true && status=="fail"){
  //        sensorConfigs[i].th_timestamp=0;
  //     }else{
  //       sensorConfigs[i].th_timestamp=currentMillis;
  //     }
      
      
      
  // }
  //  } 
    }
    }
  }

}

void httpInitialize() {
  GSM_AT.println(F("AT+CGATT?\r"));
  
  delay(100);
  GSM_AT.println(F("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""));
  
  delay(100);
  GSM_AT.println(F("AT+SAPBR=3,1,\"APN\",\"internet.vodafone.net\""));
  
  delay(100);
  GSM_AT.println(F("AT+SAPBR=1,1"));
  
  delay(2000);
  GSM_AT.println(F("AT+SAPBR=2,1"));
  delay(1000);
  
  Serial.println(F(" "));
}

bool sendHTTP(String url, int ID, float hum, float avgtemp, float maxth, float minth, String token) {
  checkCon();
  GSM_AT.println(F("AT+HTTPINIT"));
  delay(2000);

  // Construct the URI with parameters
  String uri = url + "/id=" + String(ID) + "/humd=" + String(hum) + "/avg=" + String(avgtemp) + "/max=" + String(maxth) + "/min=" + String(minth) + "/token=" + token;
  Serial.println(uri);
  GSM_AT.println("AT+HTTPPARA=\"URL\",\"" + uri + "\"");
  delay(1000);

  // Send HTTP POST request
  GSM_AT.println(F("AT+HTTPACTION=0"));
  delay(5000);

  // Read the response from the module
  String response = GSM_AT.readString();
  Serial.println(response);

  // Extract the status code from the response
  int startPos = response.indexOf("+HTTPACTION: 0,");
  if (startPos != -1) {
    int statusCode = response.substring(startPos + 15, startPos + 18).toInt(); // Status code is typically a 3-digit number
    Serial.print(F("HTTP Status Code: "));
    Serial.println(statusCode);

    if (statusCode == 200) {
      Serial.println(F("Packet sent successfully"));
      return true;
    } else {
      Serial.println(F("Packet not sent"));
      Serial.println(F("Wrong Credentials or weak 4G network, Trying to check Wi-Fi"));
      return false;}
      
    }
   else {
    Serial.println(F("Failed to parse HTTP response"));
   
    Serial.println(F("No or weak 4G network, Trying to check Wi-Fi"));
     return false;
  }

  // Terminate HTTP connection
  GSM_AT.println(F("AT+HTTPTERM"));
  delay(100);
}

bool senddoorstatus(int ID)
{
  checkCon();
  String ip = "http://admin.sstm-eg.com/api/updateDoorStatus/" + String(ID) + "/" + String(stati);
  GSM_AT.println("AT+HTTPINIT");
  delay(2000);
  Serial.println(ID);
  Serial.println(stati);
  Serial.println(ip);
  GSM_AT.println("AT+HTTPPARA=\"URL\",\"" + ip + "\"") ;
  delay(1000);
  GSM_AT.println(F("AT+HTTPACTION=0"));
  delay(5000);

  // Read the response from the module
  String response = GSM_AT.readString();
  Serial.println(response);

  // Extract the status code from the response
  int startPos = response.indexOf("+HTTPACTION: 0,");
  if (startPos != -1) {
    int statusCode = response.substring(startPos + 15, startPos + 18).toInt(); // Status code is typically a 3-digit number
    Serial.print(F("HTTP Status Code: "));
    Serial.println(statusCode);

    if (statusCode == 200) {
      Serial.println(F("Packet sent successfully"));
      return true;
    } else {
      Serial.println(F("Packet not sent"));
      Serial.println(F("Wrong Credentials or weak 4G network, Trying to check Wi-Fi"));
      return false;}
      
    }
   else {
    Serial.println(F("Failed to parse HTTP response"));
   
    Serial.println(F("No or weak 4G network, Trying to check Wi-Fi"));
     return false;
  }

  // Terminate HTTP connection
  GSM_AT.println(F("AT+HTTPTERM"));
  delay(100);
}



void testSendSMS() {
  String imei = modem.getIMEI();
  bool res = modem.sendSMS(SMS_TARGET, "HELLO");
 String status=String(res ? "OK" : "fail");
    Serial.print(F("SMS: "));
    Serial.println(status);
}


void checkCon() {
  GSM_AT.println(F("AT+CGATT?\r"));
  delay(100);
  
}

void resetsim() {
  GSM_AT.println(F("AT+CFUN=1,1\r"));
  delay(1000);
  
}

String smsBuffer = "";

void updateSerial()
{
  while (Serial.available())
  {
    GSM_AT.write(Serial.read()); // Forward what Serial received to Software Serial Port (SerialAT)
  }
  
  while (GSM_AT.available())    //filter messages to get the messages the have certain codes at beginning and end
  {
     //Serial.write(GSM_AT.read());
    char c = GSM_AT.read(); // Read a character from the Software Serial Port (SerialAT)
    smsBuffer += c; // Append the character to smsBuffer
   // smsBuffer=GSM_AT.read();
    //Serial.println(smsBuffer);
  
     if (smsBuffer.startsWith("+") && smsBuffer.endsWith("\"")) {
      
      int start = smsBuffer.indexOf('+') + 2;
      // Find the position of the second double quote after the first
      int end = smsBuffer.indexOf('"', start);
      
      // Extract the phone number substring between the double quotes
      Phonenumber = smsBuffer.substring(start, end);
      //Phonenumber = smsBuffer;
      Serial.println("Phone Number: " + Phonenumber);
      smsBuffer = "";
      // Reset smsBuffer after processing
     
    }

   

    if (smsBuffer.startsWith("#$") && smsBuffer.endsWith("$#")){
    smsBuffer = smsBuffer.substring(2, smsBuffer.length() - 2);
    if (smsBuffer.indexOf("SSID:") != -1 && smsBuffer.indexOf("PWD:") != -1) {
      
      
    String wifi = extractAttribute(smsBuffer, "SSID");
    for (int i = 0; i < wifi.length(); i++) {
        if (wifi[i] == '~') {
            wifi[i] = ' '; // Replace underscore with space
        }
    }
    String Pass = extractAttribute(smsBuffer, "PWD");
    ssid=wifi.c_str();
    password=Pass.c_str();
    Serial.println(wifi);
    Serial.println(Pass);
    delay(2500);
    bool res = modem.sendSMS(Phonenumber, "Credentials are Driven; Wait for 10 Seconds before resending message");
   String status=String(res ? "OK" : "fail");
    Serial.print(F("SMS: "));
    Serial.println(status);

    connectWiFi(ssid,password); 
    saveSensorConfigs();
    smsBuffer="";

    }
    else{
      Serial.println("INVALID SYNTAX");
      delay(2500);
      bool res = modem.sendSMS(Phonenumber, "INVALID SYNTAX  Please enter #$SSID:<ID>,PWD:<Password>$#" );
     String status=String(res ? "OK" : "fail");
    Serial.print(F("SMS: "));
    Serial.println(status);
    }
    }

    else if (smsBuffer.startsWith("#!") && smsBuffer.endsWith("!#")) {
    if(server_url==""){
      Serial.println("Enter the cloud API first");
      delay(2500);
      bool res = modem.sendSMS(Phonenumber, "Enter the Cloud API before defining the Credentials and Timer" );
     String status=String(res ? "OK" : "fail");
    Serial.print(F("SMS: "));
    Serial.println(status);
    }else{
    // Remove the starting and ending delimiters
    smsBuffer = smsBuffer.substring(2, smsBuffer.length() - 2);
    if (smsBuffer.indexOf("ID:") != -1 && smsBuffer.indexOf("Token:") != -1 && smsBuffer.indexOf("Timer:") != -1 && smsBuffer.indexOf("Index:") != -1 
    && smsBuffer.indexOf("Thtimer:") != -1 && smsBuffer.indexOf("Min:") != -1 && smsBuffer.indexOf("Max:") != -1)  {
    // Extract the attributes
    


    String sensorid = extractAttribute(smsBuffer, "ID");
    String sensortoken = extractAttribute(smsBuffer, "Token");
    String sensortimer = extractAttribute(smsBuffer, "Timer");
    String Max= extractAttribute(smsBuffer, "Max");
    String Min= extractAttribute(smsBuffer, "Min");
    String threshold_timer= extractAttribute(smsBuffer, "Thtimer");
    String index= extractAttribute(smsBuffer, "Index");


    if(isInteger(index)== true){

    if(index.toInt()<total_sensors && index.toInt()>-1 ){ 
  
    Serial.println("ID: " + sensorid);
    Serial.println("Token: " + sensortoken);
    Serial.println("Timer: " + sensortimer + " Minutes");
    Serial.println("Threshold: " + threshold_timer + " Minutes");
    Serial.println("Min: " + Min);
    Serial.println("Max: " + Max);
    
    bool index_exist=false;
    int position=-1;

     if(sensorConfigs.size()>0){
     for (size_t i = 0; i < sensorConfigs.size(); ++i) {
      if(sensorConfigs[i].index==index.toInt()){
      index_exist=true;
      position=i;
      break;
      }   
    }
    }
    if(index_exist==true){
      
      sensorConfigs[position].timer=sensortimer.toInt()*1000*60;
      sensorConfigs[position].id=sensorid.toInt();
      sensorConfigs[position].token=sensortoken;
      sensorConfigs[position].thresholdtimer=threshold_timer.toInt()*1000*60;
      sensorConfigs[position].minthreshold=Min.toInt();
      sensorConfigs[position].maxthreshold=Max.toInt();
      sensorConfigs[position].index=index.toInt();
      delay(2500);
      bool res = modem.sendSMS(Phonenumber, "Configrations are Edited; New Logs will be displayed each "+ sensortimer + " minute(s)" );
      saveSensorConfigs();
     String status=String(res ? "OK" : "fail");
    Serial.print(F("SMS: "));
    Serial.println(status);
     }
     
    else if(index_exist==false){

      if(sensorConfigs.size()==0 && index.toInt()==0){
        
      sensorConfigs.push_back({sensorid.toInt(),sensortoken,sensortimer.toInt()*1000*60,Min.toInt(),Max.toInt(),threshold_timer.toInt()*1000*60,index.toInt(),0,0});
      Serial.println(String(threshold_timer.toInt()*1000*60));
      delay(2500);
      bool res = modem.sendSMS(Phonenumber, "Configrations are Done; New Logs will be displayed each "+ sensortimer + " minute(s)" );
      saveSensorConfigs();
      String status=String(res ? "OK" : "fail");
      Serial.print(F("SMS: "));
      Serial.println(status);
      
      }else if(sensorConfigs.size()>0 && index.toInt()==((sensorConfigs[sensorConfigs.size()-1].index)+1)) {
      sensorConfigs.push_back({sensorid.toInt(),sensortoken,sensortimer.toInt()*1000*60,Min.toInt(),Max.toInt(),threshold_timer.toInt()*1000*60,index.toInt(),0,0});
      delay(2500);
      bool res = modem.sendSMS(Phonenumber, "Configrations are Done; New Logs will be displayed each "+ sensortimer + " minute(s)" );
      saveSensorConfigs();
     String status=String(res ? "OK" : "fail");
    Serial.print(F("SMS: "));
    Serial.println(status);
    }    else{
      bool res = modem.sendSMS(Phonenumber, "Invalid index, Please increment last index or start with 0" );
     String status=String(res ? "OK" : "fail");
    Serial.print(F("SMS: "));
    Serial.println(status);
      }
         if(SHT_ID==sensorid.toInt()&&SHT_TOKEN==sensortoken){
          delay(2500);
        bool res = modem.sendSMS(Phonenumber, "Sensor " + String(index.toInt()+1) + "replaced SHT30; New Logs will be displayed each "+ sensortimer + " minute(s)");
         SHT_ID=0;
         SHT_THRESHOLD_timer=0;
         SHT_MAX_THRESHOLD=0;
         SHT_MIN_THRESHOLD=0;
         SHT_timer=0;
         SHT_TOKEN="";
         String status=String(res ? "OK" : "fail");
    Serial.print(F("SMS: "));
    Serial.println(status);
      }
    else if(DHT_ID==sensorid.toInt()&&DHT_TOKEN==sensortoken){
      delay(2500);
        bool res = modem.sendSMS(Phonenumber, "Sensor " + String(index.toInt()+1) + "replaced DHT22; New Logs will be displayed each "+ sensortimer + " minute(s)" );
         DHT_ID=0;
         DHT_THRESHOLD_timer=0;
         DHT_MAX_THRESHOLD=0;
         DHT_MIN_THRESHOLD=0;
         DHT_timer=0;
         DHT_TOKEN="";
         String status=String(res ? "OK" : "fail");
    Serial.print(F("SMS: "));
    Serial.println(status);
      }
       

    }
    
    smsBuffer="";
    

  }else{
    bool res;
    if(total_sensors==0){
      delay(2500);
      res = modem.sendSMS(Phonenumber, "No Available Sensors");
    }else{
      delay(2500);
     res = modem.sendSMS(Phonenumber, "Index Should be from 0 to " + String(total_sensors-1) );}
   String status=String(res ? "OK" : "fail");
    Serial.print(F("SMS: "));
    Serial.println(status); 
  }
  }else{
    delay(2500);
    bool res = modem.sendSMS(Phonenumber, "Index Should be Digit" );
   String status=String(res ? "OK" : "fail");
    Serial.print(F("SMS: "));
    Serial.println(status);
  }
  
  }else{
    Serial.println("INVALID SYNTAX");
    delay(2500);
    bool res = modem.sendSMS(Phonenumber, "INVALID SYNTAX  Please enter #!ID:<ID>,Token:<Token>,Timer:<Time>,Min:<Min>,Max:<max>,Thtimer:<Threshold_time>,Index:<index>!#" );
   String status=String(res ? "OK" : "fail");
    Serial.print(F("SMS: "));
    Serial.println(status);
  } }
  }

 
    else if (smsBuffer.startsWith("&%") && smsBuffer.endsWith("%&")){
      smsBuffer = smsBuffer.substring(2, smsBuffer.length() - 2);
      if (smsBuffer.indexOf("URL:") != -1) {
      server_url = extractAttribute(smsBuffer, "URL");
      server_url.replace("[","");
      server_url.replace("]","");
      Serial.println(server_url);
      smsBuffer="";
      delay(2500);
      bool res = modem.sendSMS(Phonenumber, "Your API will be set to "+ server_url);
      String status=String(res ? "OK" : "fail");
      saveSensorConfigs();
      Serial.print(F("SMS: "));
      Serial.println(status);
      
      }
   
    }

      else if (smsBuffer.startsWith("!%&$") && smsBuffer.endsWith("$&%!")){
      smsBuffer = smsBuffer.substring(4, smsBuffer.length() - 4);
      if (smsBuffer.indexOf("4G") != -1) {
      pirority="4G";
      delay(2500);
      bool res = modem.sendSMS(Phonenumber, "Pirority 1: 4G\nPirority 2: Wi-Fi");
      String status=String(res ? "OK" : "fail");
      saveSensorConfigs();
      Serial.print(F("SMS: "));
      Serial.println(status);

      
      }else if (smsBuffer.indexOf("Wi-Fi") != -1) {
      pirority="Wi-Fi";
      delay(2500);
      bool res = modem.sendSMS(Phonenumber, "Pirority 1:  Wi-Fi\nPirority 2: 4G");
      String status=String(res ? "OK" : "fail");
      saveSensorConfigs();
      Serial.print(F("SMS: "));
      Serial.println(status);
      
      }else{
        delay(2500);
      bool res = modem.sendSMS(Phonenumber, "Invalid syntax; choose Wi-Fi or 4G");
      String status=String(res ? "OK" : "fail");
      Serial.print(F("SMS: "));
      Serial.println(status);
      }
   
    }
      else if (smsBuffer.startsWith("%D$H#T!") && smsBuffer.endsWith("!D#H$T%")){
        if(server_url==""){
      Serial.println("Enter the cloud API first");
      delay(2500);
      bool res = modem.sendSMS(Phonenumber, "Enter the Cloud API before defining the Credentials and Timer" );
     String status=String(res ? "OK" : "fail");
    Serial.print(F("SMS: "));
    Serial.println(status);
    }else{
      smsBuffer = smsBuffer.substring(7, smsBuffer.length() - 7);
      if (smsBuffer.indexOf("ID:") != -1 && smsBuffer.indexOf("Token:") != -1 && smsBuffer.indexOf("Timer:") != -1 && smsBuffer.indexOf("Thtimer:") != -1 
      && smsBuffer.indexOf("Min:") != -1 && smsBuffer.indexOf("Max:") != -1) {
      if(checkDHT22()){
      String sensorid = extractAttribute(smsBuffer, "ID");
      String sensortoken = extractAttribute(smsBuffer, "Token");
      String sensortimer = extractAttribute(smsBuffer, "Timer");
      String Max= extractAttribute(smsBuffer, "Max");
      String Min= extractAttribute(smsBuffer, "Min");
      String threshold_timer= extractAttribute(smsBuffer, "Thtimer");

      Serial.println("ID: " + sensorid);
      Serial.println("Token: " + sensortoken);
      Serial.println("Timer: " + sensortimer + " Minutes");
      Serial.println("Threshold: " + threshold_timer + " Minutes");
      Serial.println("Min: " + Min);
      Serial.println("Max: " + Max);

     bool ID_exist=false;
     if(sensorConfigs.size()>0){
     for (size_t i = 0; i < sensorConfigs.size(); ++i) {
      if(sensorConfigs[i].id==sensorid.toInt()&&sensorConfigs[i].token==sensortoken){
        delay(2500);
      bool res = modem.sendSMS(Phonenumber, "Can't Replace Dallas Sensor " + String(sensorConfigs[i].index+1));
      String status=String(res ? "OK" : "fail");
      Serial.print(F("SMS: "));
      Serial.println(status);
      Serial.println("");
      ID_exist=true;
      break;
      }   
    }
    }
      if(!ID_exist)  {
      if(DHT_ID!=0){
        delay(2500);
      bool res = modem.sendSMS(Phonenumber, "Configrations are Edited; New Logs will be displayed each "+ sensortimer + " minute(s)" );
      String status=String(res ? "OK" : "fail");
      Serial.print(F("SMS: "));
      Serial.println(status);
      }
      if(SHT_ID==sensorid.toInt()&&SHT_TOKEN==sensortoken){
        delay(2500);
         bool res = modem.sendSMS(Phonenumber, "DHT22 replaced SHT30; New Logs will be displayed each "+ sensortimer + " minute(s)" );
         SHT_ID=0;
         SHT_THRESHOLD_timer=0;
         SHT_MAX_THRESHOLD=0;
         SHT_MIN_THRESHOLD=0;
         SHT_timer=0;
         SHT_TOKEN="";
        String status=String(res ? "OK" : "fail");
      Serial.print(F("SMS: "));
      Serial.println(status);
      }
      else if(DHT_ID==0){
        delay(2500);
        bool res = modem.sendSMS(Phonenumber, "Configrations are Done; New Logs will be displayed each "+ sensortimer + " minute(s)" );
        String status=String(res ? "OK" : "fail");
        Serial.print(F("SMS: "));
        Serial.println(status);
      }
      DHT_timer=sensortimer.toInt()*60*1000;
      DHT_THRESHOLD_timer=threshold_timer.toInt()*60*1000;
      DHT_MAX_THRESHOLD=Max.toInt();
      DHT_MIN_THRESHOLD=Min.toInt();
      DHT_ID=sensorid.toInt();
      DHT_TOKEN=sensortoken;

      saveSensorConfigs();}
      }else{
        delay(2500);
        bool res = modem.sendSMS(Phonenumber, "DHT NOT CONNECTED" );
      String status=String(res ? "OK" : "fail");
      Serial.print(F("SMS: "));
      Serial.println(status);
      Serial.println("DHT NOT CONNECTED");
      }
    }}
      }

      else if (smsBuffer.startsWith("%S$H#T!") && smsBuffer.endsWith("!S#H$T%")){
      if(server_url==""){
      Serial.println("Enter the cloud API first");
      delay(2500);
      bool res = modem.sendSMS(Phonenumber, "Enter the Cloud API before defining the Credentials and Timer" );
     String status=String(res ? "OK" : "fail");
    Serial.print(F("SMS: "));
    Serial.println(status);
    }else{
      smsBuffer = smsBuffer.substring(7, smsBuffer.length() - 7);
      if (smsBuffer.indexOf("ID:") != -1 && smsBuffer.indexOf("Token:") != -1 && smsBuffer.indexOf("Timer:") != -1 && smsBuffer.indexOf("Thtimer:") != -1 
      && smsBuffer.indexOf("Min:") != -1 && smsBuffer.indexOf("Max:") != -1) {
      if(checkSHT30()){
      String sensorid = extractAttribute(smsBuffer, "ID");
      String sensortoken = extractAttribute(smsBuffer, "Token");
      String sensortimer = extractAttribute(smsBuffer, "Timer");
      String Max= extractAttribute(smsBuffer, "Max");
      String Min= extractAttribute(smsBuffer, "Min");
      String threshold_timer= extractAttribute(smsBuffer, "Thtimer");

      Serial.println("ID: " + sensorid);
      Serial.println("Token: " + sensortoken);
      Serial.println("Timer: " + sensortimer + " Minutes");
      Serial.println("Threshold: " + threshold_timer + " Minutes");
      Serial.println("Min: " + Min);
      Serial.println("Max: " + Max);

            bool ID_exist=false;
     if(sensorConfigs.size()>0){
     for (size_t i = 0; i < sensorConfigs.size(); ++i) {
      if(sensorConfigs[i].id==sensorid.toInt()&&sensorConfigs[i].token==sensortoken){
      delay(2500);
      bool res = modem.sendSMS(Phonenumber, "Can't Replace Dallas Sensor " + String(sensorConfigs[i].index+1));
      String status=String(res ? "OK" : "fail");
      Serial.print(F("SMS: "));
      Serial.println(status);
      Serial.println("");
      ID_exist=true;
      break;
      }   
    }
    }
      if(!ID_exist){
      if(SHT_ID!=0){
        delay(2500);
       bool res = modem.sendSMS(Phonenumber, "Configrations are Edited; New Logs will be displayed each "+ sensortimer + " minute(s)" );
             String status=String(res ? "OK" : "fail");
      Serial.print(F("SMS: "));
      Serial.println(status);
      }
      if(DHT_ID==sensorid.toInt()&&DHT_TOKEN==sensortoken){
        delay(2500);
         bool res = modem.sendSMS(Phonenumber, "SHT replaced DHT; New Logs will be displayed each "+ sensortimer + " minute(s)" );
         DHT_ID=0;
         DHT_THRESHOLD_timer=0;
         DHT_MAX_THRESHOLD=0;
         DHT_MIN_THRESHOLD=0;
         DHT_timer=0;
         DHT_TOKEN="";
               String status=String(res ? "OK" : "fail");
      Serial.print(F("SMS: "));
      Serial.println(status);
      }else if(SHT_ID==0){
         bool res = modem.sendSMS(Phonenumber, "Configrations are Done; New Logs will be displayed each "+ sensortimer + " minute(s)" );
               String status=String(res ? "OK" : "fail");
      Serial.print(F("SMS: "));
      Serial.println(status);
      }

      
      SHT_timer=sensortimer.toInt()*60*1000;
      SHT_THRESHOLD_timer=threshold_timer.toInt()*60*1000;
      SHT_MAX_THRESHOLD=Max.toInt();
      SHT_MIN_THRESHOLD=Min.toInt();
      SHT_ID=sensorid.toInt();
      SHT_TOKEN=sensortoken;
      
      saveSensorConfigs();
        } 
      }else{
      delay(2500);
      bool res = modem.sendSMS(Phonenumber, "SHT NOT CONNECTED" );
      String status=String(res ? "OK" : "fail");
      Serial.print(F("SMS: "));
      Serial.println(status);
      Serial.println("SHT NOT CONNECTED");
      }
    }}
      }

    else if(smsBuffer.startsWith("&#!") && smsBuffer.endsWith("!#&")){
      smsBuffer = smsBuffer.substring(3, smsBuffer.length() - 3);
      if (smsBuffer.indexOf("RESET_CONFIGS") != -1){
        preferences.begin("Configs",false);
          preferences.clear();
          preferences.end();
          delay(2500);
          bool res = modem.sendSMS(Phonenumber, "Configurations Are Reset");
          String status=String(res ? "OK" : "fail");
          Serial.print(F("SMS: "));
          Serial.println(status);
          delay(5000);
          ESP.restart();
      }

      else if(smsBuffer.indexOf("SHOW_TEMPS") != -1){
        
           for(int i=0;i<total_sensors; i++){
          temperatures[i] =  sensors.getTempCByIndex(i);
          delay(2500);
          String Message="Sensor " + String(i+1) +" Current Temprature is " + String(temperatures[i])+ " Degrees";
          bool res = modem.sendSMS(Phonenumber,Message);
          String status=String(res ? "OK" : "fail");
          Serial.print(F("SMS: "));
          Serial.println(status);
          }
          if(checkDHT22()){
            String Message="DHT Sensor Current Temprature is " + String(dhttemp)+ " Degrees\nDHT Sensor Current Humidity is "+ String(dhthumid) +"%";
          delay(2500);
          bool res = modem.sendSMS(Phonenumber,Message);
          String status=String(res ? "OK" : "fail");
          Serial.print(F("SMS: "));
          Serial.println(status);
          delay(2500);
          
          }
          if(checkSHT30()){
          String Message="SHT Sensor Current Temprature is " + String(shttemp)+ " Degrees\nDHT Sensor Current Humidity is "+ String(shthumid) +"%";
          delay(2500);
          bool res = modem.sendSMS(Phonenumber,Message);
          String status=String(res ? "OK" : "fail");
          Serial.print(F("SMS: "));
          Serial.println(status);
          delay(2500);
          
          }


        
        }
      else if(smsBuffer.indexOf("SHOW_CONFIGS") != -1){
        
        while(sensorConfigs.size()>total_sensors){
          sensorConfigs.pop_back();
          saveSensorConfigs();
        }
        delay(2500);
        bool res = modem.sendSMS(Phonenumber,"Sensors attached: " + String(total_sensors) + " Sensor(s)");
          String status=String(res ? "OK" : "fail");
          Serial.print(F("SMS: "));
          Serial.println(status);
          delay(2500);
        
         if(check_wifi_credentials()=="OK"){
          String ssid=WiFi.SSID();
          bool res = modem.sendSMS(Phonenumber,"Wi-Fi Connected To " + ssid);
          String status=String(res ? "OK" : "fail");
          Serial.print(F("SMS: "));
          Serial.println(status);
          delay(2500);

        }
        if(check_wifi_credentials()=="WiFi credentials not configured."){
          bool res = modem.sendSMS(Phonenumber,"Not Connected to Wi-Fi");
          String status=String(res ? "OK" : "fail");
          Serial.print(F("SMS: "));
          Serial.println(status);
          delay(2500);

        }

        if(pirority=="Wi-Fi"){
          bool res = modem.sendSMS(Phonenumber, "Pirority 1:  Wi-Fi\nPirority 2: 4G");
          String status=String(res ? "OK" : "fail");
          Serial.print(F("SMS: "));
          Serial.println(status);
          delay(2500);
        }
        if(pirority=="4G"){
          bool res = modem.sendSMS(Phonenumber, "Pirority 1:  4G\nPirority 2: Wi-Fi");
          String status=String(res ? "OK" : "fail");
          Serial.print(F("SMS: "));
          Serial.println(status);
          delay(2500);
        }

        if(sensorConfigs.size()==0){
          bool res = modem.sendSMS(Phonenumber,"NO CONFIGS FOR DALLAS SENSORS");
          String status=String(res ? "OK" : "fail");
          Serial.print(F("SMS: "));
          Serial.println(status);
          delay(2500);
        }
        if(DHT_ID==0){
          bool res = modem.sendSMS(Phonenumber,"NO CONFIGS FOR DHT SENSOR");
          String status=String(res ? "OK" : "fail");
          Serial.print(F("SMS: "));
          Serial.println(status);
          delay(2500);
        }else{
          String message= "DHT Sensor Configs:\nID: " + String(DHT_ID) + 
          "\nToken: " + DHT_TOKEN +"\nTimer: " + String(DHT_timer) + "\nMinumum Threshold: "
           + String(DHT_MIN_THRESHOLD) + "\nMaximum_Threshold: " + String(DHT_MAX_THRESHOLD) + 
           "\nTimer_for_threshold: " + String(DHT_THRESHOLD_timer);

          bool res = modem.sendSMS(Phonenumber,message);
          String status=String(res ? "OK" : "fail");
          Serial.print(F("SMS: "));
          Serial.println(status);

          delay(2500);  
        }
        if(SHT_ID==0){
          bool res = modem.sendSMS(Phonenumber,"NO CONFIGS FOR SHT SENSOR");
          String status=String(res ? "OK" : "fail");
          Serial.print(F("SMS: "));
          Serial.println(status);
          delay(2500);
        }else{
          String message= "SHT Sensor Configs:\nID: " + String(SHT_ID) + 
          "\nToken: " + SHT_TOKEN +"\nTimer: " + String(SHT_timer) + "\nMinumum Threshold: "
           + String(SHT_MIN_THRESHOLD) + "\nMaximum_Threshold: " + String(SHT_MAX_THRESHOLD) + 
           "\nTimer_for_threshold: " + String(SHT_THRESHOLD_timer);

          bool res = modem.sendSMS(Phonenumber,message);
          String status=String(res ? "OK" : "fail");
          Serial.print(F("SMS: "));
          Serial.println(status);

          delay(2500);  
        }

        for (int i=0; i<sensorConfigs.size();i++){
          String message= "Sensor " + String(i+1) + " Configs:\n" +"ID: " + String(sensorConfigs[i].id) + 
          "\nToken: " + sensorConfigs[i].token +"\nTimer: " + String(sensorConfigs[i].timer) + "\nMinumum Threshold: "
           + String(sensorConfigs[i].minthreshold) + "\nMaximum_Threshold: " + String(sensorConfigs[i].maxthreshold) + 
           "\nTimer_for_threshold: " + String(sensorConfigs[i].thresholdtimer) +"\nIndex: " + String(sensorConfigs[i].index);

          bool res = modem.sendSMS(Phonenumber,message);
          String status=String(res ? "OK" : "fail");
          Serial.print(F("SMS: "));
          Serial.println(status);

          delay(2500);  
        }
        

        
       
      }else{
        delay(2500);
        bool res = modem.sendSMS(Phonenumber, "Invalid Syntax; Use \"RESET_CONFIGS\" or \"SHOW_CONFIGS\"");
        String status=String(res ? "OK" : "fail");
        Serial.print(F("SMS: "));
        Serial.println(status);
      }
    }
   
    else if(smsBuffer.startsWith("&#%!$") && smsBuffer.endsWith("$!%#&")){
      smsBuffer = smsBuffer.substring(5, smsBuffer.length() - 5);
      if (smsBuffer.indexOf("doorofsensorid:") != -1) {
      String sensorid = extractAttribute(smsBuffer, "doorofsensorid");
      sensorID_For_Door=sensorid.toInt();
      saveSensorConfigs();
      delay(2500);
      bool res = modem.sendSMS(Phonenumber, "Door is Set for Sensor ID: " + sensorid);
      String status=String(res ? "OK" : "fail");
      Serial.print(F("SMS: "));
      Serial.println(status);
    }
    }
    else if (smsBuffer.startsWith("%$#!&") && smsBuffer.endsWith("&!#$%")){
      
      smsBuffer = smsBuffer.substring(5, smsBuffer.length() - 5);
      if (smsBuffer.indexOf("HOST:") != -1 && smsBuffer.indexOf("BIN:") != -1 && smsBuffer.indexOf("PORT:") != -1) {
      String host = extractAttribute(smsBuffer, "HOST");
      String bin = extractAttribute(smsBuffer, "BIN");
      String port = extractAttribute(smsBuffer, "PORT");
      Serial.println(bin);
      Serial.println(port);
      host.replace("[","");
      host.replace("]","");
      if(isInteger(port)){
        
        if(check_wifi_credentials()=="OK"){
        delay(2500);
        bool res = modem.sendSMS(Phonenumber, "Code will be updated");
        String status=String(res ? "OK" : "fail");
        Serial.print(F("SMS: "));
        Serial.println(status);
          update(host,bin,port.toInt());
        }else{
        delay(2500);
        bool res = modem.sendSMS(Phonenumber, "Connect to Wi-Fi before Sending OTA");
        String status=String(res ? "OK" : "fail");
        Serial.print(F("SMS: "));
        Serial.println(status);
         //update4G(host,bin,port.toInt());
        }
        
      }else{
        delay(2500);
        bool res = modem.sendSMS(Phonenumber, "port must be integer");
        String status=String(res ? "OK" : "fail");
        Serial.print(F("SMS: "));
        Serial.println(status);
        
      }
     
    }
      }
    
    
    
       if (c == '\n' || c == '\r' || c == ' '|| c == '\t' || c == '"') {
      smsBuffer="";
    }
    
  } 
  
}

String extractAttribute(String message, String attribute) {
  int start = message.indexOf(attribute + ":");
  if (start == -1) {
    return "";
  }
  start += attribute.length() + 1;
  int end = message.indexOf(',', start);
  if (end == -1) {
    end = message.length();
  }
  return message.substring(start, end);
}




bool sendHTTP_via_wifi(String url, int ID, float hum, float avgtemp, float maxth, float minth, String token) {
    HTTPClient http;

     String uri = url + "/id=" + String(ID) + "/humd=" + String(hum) + "/avg=" + String(avgtemp) + "/max=" + String(maxth) + "/min=" + String(minth) + "/token=" + token;

    http.begin(uri);  // Specify URL
    int httpCode = http.GET();  // Send the request

    if (httpCode > 0) {
        Serial.printf("[HTTP] GET... code: %d\n", httpCode);

        if (httpCode == HTTP_CODE_OK) {
            String payload = http.getString();   // Get the response payload
            Serial.println(payload);              // Print the response payload

            // Check if payload contains "success"
            if (payload.indexOf("success") != -1) {
                Serial.println("Success");  // Print "Success" if "success" is found in payload
                return true;
            } else {
                Serial.println("Response does not contain 'success'");
                return false;
            }
        }
    } else {
        Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
        return false;
    }

    http.end();  // Close connection
}


String check_wifi_credentials(){
  if (WiFi.SSID() == "") {

    Serial.println("WiFi credentials not configured.");
    return "WiFi credentials not configured.";
    // Optionally, you can configure WiFi here or enter setup mode.
  }
  else return "OK";
}


void connectWiFi(String ssid,String password) {
    if(ssid!=""&&password!=""){
    Serial.println("Connecting to WiFi...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    int i=0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(2000);
        Serial.println("Connecting to WiFi...");
        i++;
        if (i>20){
          Serial.println("Can't Connect to WiFi");
          delay(2500);
          bool res = modem.sendSMS(AuthPhone, "Can't Connect to Wi-Fi");
         String status=String(res ? "OK" : "fail");
    Serial.print(F("SMS: "));
    Serial.println(status);
          return;
        }
    }
    

    Serial.println("Connected to WiFi");
    bool res = modem.sendSMS(AuthPhone, "Connected Successfully");
    String status=String(res ? "OK" : "fail");
    Serial.print(F("SMS: "));
    Serial.println(status);
    return;
    }
}



bool isInteger(String str) {
  for (int i = 0; i < str.length(); i++) {
    if (!isDigit(str.charAt(i))) {
      return false;
    }
  }
  return true;
}


void loadSensorConfigs() {
    preferences.begin("Configs", true); // Open preferences in read-only mode

    ssid = preferences.getString("ssid", ssid);
    password = preferences.getString("password", password);
    server_url = preferences.getString("serverurl", server_url);
    pirority = preferences.getString("pirority", pirority);


    DHT_timer=preferences.getInt("DHTt",0);
    DHT_THRESHOLD_timer=preferences.getInt("DHTtht",0);
    DHT_MAX_THRESHOLD=preferences.getInt("dhtmx",0);
    DHT_MIN_THRESHOLD=preferences.getInt("dhtmn",0);
    DHT_ID=preferences.getInt("dhtid",0);
    DHT_TOKEN=preferences.getString("dhttoken","");


    SHT_timer=preferences.getInt("SHTt",0);
    SHT_THRESHOLD_timer=preferences.getInt("SHTtht",0);
    SHT_MAX_THRESHOLD=preferences.getInt("shtmx",0);
    SHT_MAX_THRESHOLD=preferences.getInt("shtmn",0);
    SHT_ID=preferences.getInt("shtid",0);
    SHT_TOKEN=preferences.getString("shttoken","");


    sensorID_For_Door=preferences.getInt("doorid",-1);
    
    // Read number of configurations saved
    int count = preferences.getInt("count", 0);
    if (count == 0) {
        Serial.println(F("No configs stored"));
    }

    sensorConfigs.resize(count); // Resize vector to hold the configurations

    // Load each SensorConfig
    for (int i = 0; i < count; ++i) {
        String keyPrefix = "config_" + String(i);

        sensorConfigs[i].id = preferences.getInt((keyPrefix + "_id").c_str(), 0);
        sensorConfigs[i].token = preferences.getString((keyPrefix + "_token").c_str(), "");
        sensorConfigs[i].timer = preferences.getInt((keyPrefix + "_timer").c_str(), 180000);
        sensorConfigs[i].thresholdtimer = preferences.getInt((keyPrefix + "_thTimr").c_str(), 60000);
        sensorConfigs[i].minthreshold = preferences.getInt((keyPrefix + "_minTh").c_str(), -10);
        sensorConfigs[i].maxthreshold = preferences.getInt((keyPrefix + "_maxTh").c_str(), 50);
        sensorConfigs[i].index = preferences.getInt((keyPrefix + "_index").c_str(), 100); // Check default value
        sensorConfigs[i].timestamp=preferences.getULong((keyPrefix + "_ts").c_str(), 0); // Check default value
        sensorConfigs[i].th_timestamp=preferences.getULong((keyPrefix + "_THts").c_str(), 0); // Check default value


        // Print loaded values for verification
        Serial.print("Sensor "); Serial.print(i + 1); Serial.println(":");
        Serial.print("Loaded ID: "); Serial.println(sensorConfigs[i].id);
        Serial.print("Loaded Token: "); Serial.println(sensorConfigs[i].token);
        Serial.print("Loaded Timer: "); Serial.println(sensorConfigs[i].timer);
        Serial.print("Loaded Threshold Timer: "); Serial.println(sensorConfigs[i].thresholdtimer);
        Serial.print("Loaded Min Threshold: "); Serial.println(sensorConfigs[i].minthreshold);
        Serial.print("Loaded Max Threshold: "); Serial.println(sensorConfigs[i].maxthreshold);
        Serial.print("Loaded Index: "); Serial.println(sensorConfigs[i].index);
        Serial.println();
    }

    Serial.println(F("Config Loaded"));
    preferences.end(); // End preferences session
}


void saveSensorConfigs() {
    
    preferences.begin("Configs",false);
    preferences.clear();
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.putString("serverurl", server_url);
    preferences.putString("pirority",pirority);


    preferences.putInt("DHTt",DHT_timer);
    preferences.putInt("DHTtht",DHT_THRESHOLD_timer);
    preferences.putInt("dhtmx",DHT_MAX_THRESHOLD);
    preferences.putInt("dhtmn",DHT_MIN_THRESHOLD);
    preferences.putInt("dhtid",DHT_ID);
    preferences.putString("dhttoken",DHT_TOKEN);


    preferences.putInt("SHTt",SHT_timer);
    preferences.putInt("SHTtht",SHT_THRESHOLD_timer);
    preferences.putInt("shtmx",SHT_MAX_THRESHOLD);
    preferences.putInt("shtmn",SHT_MIN_THRESHOLD);
    preferences.putInt("shtid",SHT_ID);
    preferences.putString("shttoken",SHT_TOKEN);
    // Clear existing configurations in NVS
    
   preferences.putInt("doorid",sensorID_For_Door);

    // Save number of configurations
    int count = sensorConfigs.size();
    preferences.putInt("count", count);

    // Save each SensorConfig
    for (int i = 0; i < count; ++i) {

      
        Serial.print("Sensor "); Serial.print(i + 1); Serial.println(":");
        Serial.print("Saved ID: "); Serial.println(sensorConfigs[i].id);
        Serial.print("Saved Token: "); Serial.println(sensorConfigs[i].token);
        Serial.print("Saved Timer: "); Serial.println(sensorConfigs[i].timer);
        Serial.print("Saved Threshold Timer: "); Serial.println(sensorConfigs[i].thresholdtimer);
        Serial.print("Saved Min Threshold: "); Serial.println(sensorConfigs[i].minthreshold);
        Serial.print("Saved Max Threshold: "); Serial.println(sensorConfigs[i].maxthreshold);
        Serial.print("Saved Index: "); Serial.println(sensorConfigs[i].index);
        Serial.println();

        String keyPrefix = "config_" + String(i);
        preferences.putInt((keyPrefix + "_id").c_str(), sensorConfigs[i].id);
        preferences.putString((keyPrefix + "_token").c_str(), sensorConfigs[i].token);
        preferences.putInt((keyPrefix + "_timer").c_str(), sensorConfigs[i].timer);
        preferences.putInt((keyPrefix + "_thTimr").c_str(), sensorConfigs[i].thresholdtimer);
        preferences.putInt((keyPrefix + "_minTh").c_str(), sensorConfigs[i].minthreshold);
        preferences.putInt((keyPrefix + "_maxTh").c_str(), sensorConfigs[i].maxthreshold);
        preferences.putInt((keyPrefix + "_index").c_str(), sensorConfigs[i].index);
        preferences.putULong((keyPrefix + "_th").c_str(), sensorConfigs[i].timestamp);
        preferences.putULong((keyPrefix + "_THts").c_str(), sensorConfigs[i].th_timestamp);




  
    }
    Serial.println(F("Config Saved"));
    // Commit changes
    preferences.end();
}


void Get_temprature_Dallas()
{

  
  sensors.requestTemperatures();

  total_sensors = sensors.getDeviceCount();

  Serial.println("Sensors Found = " + String(total_sensors));

  //Serial.println(total_sensors);
  for(int i=0;i<total_sensors; i++){

  temperatures[i] =  sensors.getTempCByIndex(i);  
  calctemp(i);
    
  

  
  
  }
}

void calctemp(int i) {
  if (Numder_of_measurments[i] == 0) {
    Min_temperatures[i] = temperatures[i];
    Max_temperatures[i] = temperatures[i];
    Avg_temperatures[i] = temperatures[i];
  }

  if (Numder_of_measurments[i] > 0) {
    if (Min_temperatures[i] > temperatures[i]) {
      Min_temperatures[i] = temperatures[i];
    }

    if (Max_temperatures[i] < temperatures[i]) {
      Max_temperatures[i] = temperatures[i];
    }

    Avg_temperatures[i] = (temperatures[i] + Avg_temperatures[i]);

  }
  Serial.println("Min Temprature "+String(i)+ " " + String(Min_temperatures[i]));
  Serial.println("Max Temprature "+String(i)+ " " +String(Max_temperatures[i]));
  Serial.println("Average Temprature "+String(i)+ " " +String(Avg_temperatures[i]));
  Serial.println("Current Temprature "+String(i)+ " " +String(temperatures[i]));
  Serial.println("Number of Measurments "+String(i)+ " " +String(Numder_of_measurments[i]));
  Numder_of_measurments[i]=Numder_of_measurments[i]+1;
}


void reset_sensor(int i){
total_sensors = sensors.getDeviceCount();
Numder_of_measurments.resize(total_sensors);
temperatures.resize(total_sensors);
Max_temperatures.resize(total_sensors);
Min_temperatures.resize(total_sensors);
Avg_temperatures.resize(total_sensors);

  Avg_temperatures[i]=0;
  Numder_of_measurments[i]=0;
  temperatures[i]=0;
  Max_temperatures[i]=0;
  Min_temperatures[i]=0;

}


// Function to check if DHT22 is connected
bool checkDHT22() {
    float dht22Temperature = dht.readTemperature();
    float dht22Humidity = dht.readHumidity();
    dhthumid=dht22Humidity;
    dhttemp=dht22Temperature;
    if (!isnan(dht22Temperature) && !isnan(dht22Humidity)) {
      Serial.print("DHT22 - Temperature: ");
      Serial.print(dht22Temperature);
      Serial.print(" °C, Humidity: ");
      Serial.print(dht22Humidity);
      Serial.println(" %");
      if(dhtmeasureno==0){
        mindht=dht22Temperature;
        maxdht=dht22Temperature;
      }
      if(dht22Temperature<mindht){
        mindht=dht22Temperature;
      }else if(dht22Temperature>maxdht){
        maxdht=dht22Temperature;
      }
      avgdht=avgdht+dht22Temperature;
      dhtmeasureno++;
      return true;
    } else {
      Serial.println("Failed to read from DHT22 sensor.");
      return false;
    }
}

// Function to check if SHT30 is connected
bool checkSHT30() {
    if (!sht30.begin(0x44)) { // 0x44 is the default I2C address for SHT30
    return false;
  }
  float sht30Temperature = sht30.readTemperature();
  float sht30Humidity = sht30.readHumidity();
  shthumid=sht30Humidity;
  shttemp=sht30Temperature;
  // Read temperature to see if the sensor is connected

  if (!isnan(sht30Temperature) && !isnan(sht30Humidity)) {
      avgsht=avgsht+sht30Temperature;

      Serial.print("SHT30 - Temperature: ");
      Serial.print(sht30Temperature);
      Serial.print(" °C, Humidity: ");
      Serial.print(sht30Humidity);
      Serial.print(" %, Average Temperature ");
      Serial.println(avgsht);
      if(shtmeasureno==0){
        minsht=sht30Temperature;
        maxsht=sht30Temperature;
      }
      if(sht30Temperature<minsht){
        minsht=sht30Temperature;
      }else if(sht30Temperature>maxsht){
        maxsht=sht30Temperature;
      }
      
      shtmeasureno++;
      
      
      return true;
    } else {
      Serial.println("Failed to read from SHT30 sensor.");
      return false;
    }
  
}

void removesht(){
SHT_timer=0;
SHT_THRESHOLD_timer=0;
SHT_MAX_THRESHOLD=0;
SHT_MIN_THRESHOLD=0;
SHT_ID=0;
SHT_TOKEN="";
}

void resetsht(){
maxsht=0;
minsht=0;
avgsht=0;
shtmeasureno=0;
shthumid=0;
}


void removedht(){
DHT_timer=0;
DHT_THRESHOLD_timer=0;
DHT_MAX_THRESHOLD=0;
DHT_MIN_THRESHOLD=0;
DHT_ID=0;
DHT_TOKEN="";
}

void resetdht(){
maxdht=0;
mindht=0;
avgdht=0;
dhtmeasureno=0;
dhthumid=0;
}


String getHeaderValue(String header, String headerName) {
    return header.substring(strlen(headerName.c_str()));
}

String getBinName(String url) {
    int index = 0;

    // Search for last /
    for (int i = 0; i < url.length(); i++) {
        if (url[i] == '/') {
            index = i;
        }
    }

    String binName = "";

    // Create binName
    for (int i = index; i < url.length(); i++) {
        binName += url[i];
    }

    return binName;
}

String getHostName(String url) {
     int index = 0;

    // Search for last /
    for (int i = 0; i < url.length(); i++) {
        if (url[i] == '/') {
            index = i;
        }
    }

    String hostName = "";

    // Create binName
    for (int i = 0; i < index; i++) {
        hostName += url[i];
    }

    return hostName;
}

void update(String host,String bin ,int port) {
    //String bin = "/uploads/projects/otas/47_Test_GSM_OTA/version_2/version_2.bin";
    //String host = "admin.sstm-eg.com";

    Serial.println("Connecting to: " + host);
    if (client.connect(host.c_str(), port)) {
        // Connection Succeed.
        // Fecthing the bin
        Serial.println("Fetching Bin: " + bin);

        // Get the contents of the bin file
        client.print(String("GET ") + bin + " HTTP/1.1\r\n" +
                        "Host: " + host + "\r\n" +
                        "Cache-Control: no-cache\r\n" +
                        "Connection: close\r\n\r\n");

        unsigned long timeout = millis();
       // client.setTimeout(10000);
        while (client.available() == 0) {
            if (millis() - timeout > 5000) {
                Serial.println("Client Timeout !");
                client.stop();
                return;
            }
        }
        while (client.available()) {
            // read line till /n
            String line = client.readStringUntil('\n');
            // remove space, to check if the line is end of headers
            line.trim();

            // if the the line is empty,
            // this is end of headers
            // break the while and feed the
            // remaining `client` to the
            // Update.writeStream();
            if (!line.length()) {
                //headers ended
                break; // and get the OTA started
            }

            // Check if the HTTP Response is 200
            // else break and Exit Update
            if (line.startsWith("HTTP/1.1")) {
                if (line.indexOf("200") < 0) {
                    Serial.println("Got a non 200 status code from server. Exiting OTA Update.");
                    break;
                }
            }

            // extract headers here
            // Start with content length
            if (line.startsWith("Content-Length: ")) {
                contentLength = atoi((getHeaderValue(line, "Content-Length: ")).c_str());
                Serial.println("Got " + String(contentLength) + " bytes from server");
            }

            // Next, the content type
            if (line.startsWith("Content-Type: ")) {
                String contentType = getHeaderValue(line, "Content-Type: ");
                Serial.println("Got " + contentType + " payload.");
                if (contentType == "application/octet-stream") {
                    isValidContentType = true;
                }
            }
        }
    }
    else {
        // Connect to S3 failed
        // May be try?
        // Probably a choppy network?
        Serial.println("Connection to " + host + " failed. Please check your setup");
        // retry??
    }

    // Check what is the contentLength and if content type is `application/octet-stream`
    Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

    // check contentLength and content type
    if (contentLength && isValidContentType) {
        // Check if there is enough to OTA Update
        bool canBegin = Update.begin(contentLength);
        if (canBegin) {
            Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
            size_t written = Update.writeStream(client);

            if (written == contentLength) {
                Serial.println("Written : " + String(written) + " successfully");
            }
            else {
                Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?" );
                // retry??
            }

            if (Update.end()) {
                Serial.println("OTA done!");
                if (Update.isFinished()) {
                    Serial.println("Update successfully completed. Rebooting.");
                    ESP.restart();
                }
                else {
                    Serial.println("Update not finished? Something went wrong!");
                    ESP.restart();
                }
            }
            else {
                Serial.println("Error Occurred. Error #: " + String(Update.getError()));
                //ESP.restart();
            }
        }
        else {
            // not enough space to begin OTA
            // Understand the partitions and
            // space availability
            Serial.println("Not enough space to begin OTA");
            client.flush();
            //ESP.restart();
        }
    }
    else {
        Serial.println("There was no content in the response");
        client.flush();
        //ESP.restart();
    }
}


void update4G(String host,String bin ,int port) {
    //String bin = "/uploads/projects/otas/47_Test_GSM_OTA/version_2/version_2.bin";
    //String host = "admin.sstm-eg.com";

    Serial.println("Connecting to: " + host);
    if (client4G.connect(host.c_str(), port)) {
        // Connection Succeed.
        // Fecthing the bin
        Serial.println("Fetching Bin: " + bin);

        // Get the contents of the bin file
        client4G.print(String("GET ") + bin + " HTTP/1.1\r\n" +
                        "Host: " + host + "\r\n" +
                        "Cache-Control: no-cache\r\n" +
                        "Connection: close\r\n\r\n");

        unsigned long timeout = millis();
       // client4G.setTimeout(10000);
        while (client4G.available() == 0) {
            if (millis() - timeout > 5000) {
                Serial.println("client4G Timeout !");
                client4G.stop();
                return;
            }
        }
        while (client4G.available()) {
            // read line till /n
            String line = client4G.readStringUntil('\n');
            // remove space, to check if the line is end of headers
            line.trim();

            // if the the line is empty,
            // this is end of headers
            // break the while and feed the
            // remaining `client4G` to the
            // Update.writeStream();
            if (!line.length()) {
                //headers ended
                break; // and get the OTA started
            }

            // Check if the HTTP Response is 200
            // else break and Exit Update
            if (line.startsWith("HTTP/1.1")) {
                if (line.indexOf("200") < 0) {
                    Serial.println("Got a non 200 status code from server. Exiting OTA Update.");
                    break;
                }
            }

            // extract headers here
            // Start with content length
            if (line.startsWith("Content-Length: ")) {
                contentLength = atoi((getHeaderValue(line, "Content-Length: ")).c_str());
                Serial.println("Got " + String(contentLength) + " bytes from server");
            }

            // Next, the content type
            if (line.startsWith("Content-Type: ")) {
                String contentType = getHeaderValue(line, "Content-Type: ");
                Serial.println("Got " + contentType + " payload.");
                if (contentType == "application/octet-stream") {
                    isValidContentType = true;
                }
            }
        }
    }
    else {
        // Connect to S3 failed
        // May be try?
        // Probably a choppy network?
        Serial.println("Connection to " + host + " failed. Please check your setup");
        // retry??
    }

    // Check what is the contentLength and if content type is `application/octet-stream`
    Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

    // check contentLength and content type
    if (contentLength && isValidContentType) {
        // Check if there is enough to OTA Update
        bool canBegin = Update.begin(contentLength);
        if (canBegin) {
            Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
            size_t written = Update.writeStream(client4G);

            if (written == contentLength) {
                Serial.println("Written : " + String(written) + " successfully");
            }
            else {
                Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?" );
                // retry??
            }

            if (Update.end()) {
                Serial.println("OTA done!");
                if (Update.isFinished()) {
                    Serial.println("Update successfully completed. Rebooting.");
                    ESP.restart();
                }
                else {
                    Serial.println("Update not finished? Something went wrong!");
                    ESP.restart();
                }
            }
            else {
                Serial.println("Error Occurred. Error #: " + String(Update.getError()));
                //ESP.restart();
            }
        }
        else {
            // not enough space to begin OTA
            // Understand the partitions and
            // space availability
            Serial.println("Not enough space to begin OTA");
            client4G.flush();
            //ESP.restart();
        }
    }
    else {
        Serial.println("There was no content in the response");
        client4G.flush();
        //ESP.restart();
    }
}


