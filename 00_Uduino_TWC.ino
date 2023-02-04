#include <Wire.h>
#include <U8g2lib.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <PubSubClient.h>

#include "wlan_secrets.h"
#include "mqtt_secrets.h"

//SGP30 
#include "sensirion_common.h"
#include "sgp30.h"

//DHT20
#include "DHT.h"

//Sunlight Sensor (not used with Node MCU due to library incompatibility)
//#include "Si115X.h"

//ArduinoJSON
#include "ArduinoJson.h"

//Distance Sensor
#include "Ultrasonic.h"

const char* ID = "dmt02"; 
const char* HOT_CHILI_VERSION        = "IoT dmt02 TWC"; 

const char* MQTT_TOPIC_SUB_CTRL      = "IMA20/Sensors/02/ctrl";   // Control Signal - fixed Time

const char* MQTT_TOPIC_PUB_LWT       = "IMA20/Sensors/02/lwt";         // Last Will and Testament

const char* MQTT_TOPIC_PUB_ALLSENSORS = "IMA20/Sensors/02/data";        // JSON

// -----------------------------------------------------------------------------------

//Defining the instance of ESP8266 Multi WiFi
ESP8266WiFiMulti wifi_multi;
 
// Multi WiFis 
const char* ssid1     = SECRET_SSID1;
const char* password1 = SECRET_PASS1;
const char* ssid2     = SECRET_SSID2;
const char* password2 = SECRET_PASS2;
const char* ssid3     = SECRET_SSID3;
const char* password3 = SECRET_PASS3;
const char* ssid4     = SECRET_SSID4;
const char* password4 = SECRET_PASS4;
const char* ssid5     = SECRET_SSID5;
const char* password5 = SECRET_PASS5;

String mySSID = "no SSID";
uint16_t    connectTimeOutPerAP = 5000;

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);

WiFiClientSecure espClient;
PubSubClient client(espClient);

char msg[70];
int reConnect = 0;
int mqttCnt   = 0;

const int ledCtrlPin    = D5;  
const int buttonCtrlPin = D6;  

int buttonCtrlState = 0;
int buttonCtrlPrev  = 0;

int  butCount = 0;

//AHT20 
String aht_temp = "nan";
String aht_humi = "nan";
//ATH20 ATH;

//DHT20
#define DHTTYPE DHT20 
DHT dht(DHTTYPE); 

//Sunlight
//Si115X si1151;

//Distance Sensor
//Note, when using the Node MCU Grove Base Shield plug it in at D9/10 which is also the VART0 Port
Ultrasonic ultrasonic(3);

//Light Sensor / State of the lid -> stol
bool stol = false;
int lightVal = 0; 
int oldLightVal = 0;
int lightSec = 99; 

//Buzzer
const int buzzerPin = D7;
bool buzz = false;

const int BuildInLed1 = D0; // LED_BUILTIN D4 or D0
const int BuildInLed2 = D4; // LED_BUILTIN D4 or D0 - start NOT LOW > internal LED (D4 ok for output)

// ###################################

bool   infoLed = false;

#define seconds() (millis()/1000)
#define minutes() (millis()/60000)
#define hours()   (millis()/3600000)

int run=0;
int oldSec = 99; // check if sec has changed
char   timeString[21] = "Time: init";
char   dataInfo[60]   = "Data init";
char   oledInfo[30]   = "OLED init";
char   fillInfo[30]   = "Fill init";
char   envInfo[30]   = "Env init";

float garbageCanHeight = 40.0;
long oldDist = 40; 

//Could be improved with filtering, but this is taken care of at Kafka's end
int calcFill (long dist){
  if (dist != 0){
    float div = float(dist) / garbageCanHeight;
    float fill = 100 - (div * 100);
    if (fill > 90){
      digitalWrite(ledCtrlPin, HIGH);
    }else{
      digitalWrite(ledCtrlPin, LOW);
    }
    return fill;
  } else{ 
    return 100;
  }
  
}

//Function every ten seconds in debug mode, switch to 1 Minute intervals for actual use
void tenSecond(DynamicJsonDocument mqttJSON) {
  
  //String for serialization and MQTT
  char json_string[250];
  
  float temp_hum_val[2] = {0};

  //DHT20 Readout
  if (!dht.readTempAndHumidity(temp_hum_val)) {
    mqttJSON["errm"] = 0;
    mqttJSON["humi"] = temp_hum_val[0];
    mqttJSON["temp"] = temp_hum_val[1];

  }else {
    mqttJSON["humi"] = -1;
    mqttJSON["temp"] = -1;
    Serial.println("Error with Temperature / Humidity");
    mqttJSON["errm"] = 1;
  }

  //SGP30 readout for TVOC and Co2eq
  s16 err = 0;
  u16 tvoc_ppb, co2_eq_ppm;
  err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);
  if (err == STATUS_OK) {
    //JSON for MQTT Message
    mqttJSON["tvoc"] = tvoc_ppb;
    mqttJSON["co2e"] = co2_eq_ppm;
  } else {
    //Edge Case
    mqttJSON["tvoc"] = -1;
    mqttJSON["co2e"] = -1;
    Serial.println("Error with TVOC / CO2eq");
    //String(mqttJSON["errm"]) + "E02: Error with TVOC / CO2eq"
    mqttJSON["errm"] = 2;
  }

  long DistanceInCentimeters;
  //Serial.println("The distance to obstacles in front is: ");
  DistanceInCentimeters = ultrasonic.MeasureInCentimeters();
  //Serial.print(DistanceInCentimeters);//0~400cm
 // Serial.println(" cm");

  mqttJSON["dist"] = DistanceInCentimeters;
  float fill = calcFill(DistanceInCentimeters);
  mqttJSON["fill"] = fill ;

  sprintf(fillInfo,"Fill Level:%s %%",String(fill)); 
  sprintf(envInfo,"Temp:%s|Hum:%s",String(temp_hum_val[1]),String(temp_hum_val[0])); 

  serializeJson(mqttJSON, json_string);
  client.publish(MQTT_TOPIC_PUB_ALLSENSORS, json_string);
  Serial.println("Sensor Readout;");
  Serial.println(json_string);
}

// ################################################################################
// ################################################################################
// ##
// ## SETUP
// ##
// ################################################################################
// ################################################################################

void setup() {
  s16 err;
  u16 scaled_ethanol_signal, scaled_h2_signal;
  espClient.setInsecure();

  Serial.begin(9600); // fastest communication for hottest chilis - 115200*20 SHU scoville heat units
  Serial.println("\n\n"); 
  
  Serial.println("[1/6] Starting NODE MCU Esp8266 UP..."); 
  Serial.println("[2/6] Setting up Sensors and PINs"); 

  pinMode(LED_BUILTIN,   OUTPUT);
  pinMode(BuildInLed1,  OUTPUT);
  pinMode(BuildInLed2,  OUTPUT);

  pinMode(buzzerPin,    OUTPUT);
  pinMode(ledCtrlPin,    OUTPUT);
  pinMode(buttonCtrlPin, INPUT); 
  
  Serial.println("[3/6] Setting up WiFi"); 
  setup_wifi(); // WiFi connection - see wlan_secrets.h
  delay(1000);

  Serial.println("[4a/6] Setting up WIRE"); 
  Wire.begin(); 

  Serial.println("[4b/6] Setting up Display u8g2"); 
  u8g2.begin();  // OLED 128 x 64
  showScreen();

  Serial.println("[5/6] Setting up MQTT"); 
  client.setServer(MQTT_BROKER, MQTT_PORT); // MQTT
  client.setCallback(callback);        // MQTT subscribtions
  setup_mqtt();
  delay(1000);
      
  Serial.println("[6/6] Setup finished - EOS - End of Setup");
  sprintf(dataInfo,"%s WiFi>%s MQTT>%s", ID, WiFi.SSID().c_str(), MQTT_BROKER);
  // client.publish(MQTT_TOPIC_PUB_ALL, dataInfo);
  Serial.println(dataInfo); 

  sprintf(dataInfo,"%s living", ID); 
  client.publish(MQTT_TOPIC_PUB_LWT, dataInfo, true);

  sprintf(dataInfo,"ESP-%s setup finished",ID);
  // client.publish(MQTT_TOPIC_PUB_IMPORTANT, dataInfo);
  Serial.println(dataInfo); 
  Serial.println("=NIS==================================================================EOS=");    


  //SGP30 Setup
  
  while (sgp_probe() != STATUS_OK) {
    Serial.println("SGP failed");
    while (1);
  }
    /*Read H2 and Ethanol signal in the way of blocking*/
  err = sgp_measure_signals_blocking_read(&scaled_ethanol_signal,
                                            &scaled_h2_signal);
  if (err == STATUS_OK) {
    Serial.println("get ram signal!");
  } else {
    Serial.println("error reading signals");
  }
  err = sgp_iaq_init(); 

  //Sunlight Sensor not used with NodeMCU specifically
  /*
  if (!si1151.Begin())
    Serial.println("Si1151 is not ready!");
  else
    Serial.println("Si1151 is ready!");
    */

  //DHT20 Setup
  dht.begin();
    
}

// ################################################################################
// ################################################################################
// ##
// ## MAIN LOOP
// ##
// ################################################################################
// ################################################################################
  
void loop() {

  //Allocate and declare the JSON Document
  DynamicJsonDocument mqttJSON(250);
  //String for serialization and MQTT
  char json_string[250];

  String dataSend;

  
  float temp_hum_val[2] = {0};


  //Analog Lightsensor Readout
  lightVal = analogRead(A0);
  //Distance Sensor Readout
  

  Serial.println(lightVal);
  //|| DistanceForStol < (garbageCanHeight+5.0)
  if (lightVal < 75 ) {
    stol = false;
    mqttJSON["stol"] = 0;
    mqttJSON["buzz"] = 0;
    digitalWrite(buzzerPin, LOW);
  }else{
    mqttJSON["stol"] = 1;
    mqttJSON["buzz"] = 0;
  }

  if (buzz){
      mqttJSON["buzz"] = 1;
  }

  // ###################################################
  // Timing functions 
  // ###################################################
  
  int mySec = seconds() % 60;
  int myMin = minutes() % 60;
  int myHou = hours(); 
  
  sprintf(timeString,"Time: %02d:%02d.%02d",myHou,myMin,mySec); 
  
  if (mySec != oldSec)  // only when sec changed
  {
    sprintf(dataInfo,"%s %02d:%02d.%02d",ID, myHou,myMin,mySec); 
    //mqttJSON["time"] = dataInfo;
    oldSec = mySec;

    if (mySec == lightSec-1 && stol){
      digitalWrite(buzzerPin, HIGH);
      Serial.println("Buzzer active");
      buzz = true;
      mqttJSON["buzz"] = 1;
    }

    // button contact
    //LED Button Readout
    buttonCtrlState = !digitalRead(buttonCtrlPin);
    //Serial.println(buttonCtrlState);
    if (buttonCtrlState != buttonCtrlPrev) {
      dataSend = "Pickup Request Button pressed" ; // Button (blue) 
      Serial.println(dataSend);  // send to pc/unity
      sprintf(dataInfo,"%d",buttonCtrlState);
      mqttJSON["reqp"] = 1;
      buttonCtrlPrev = buttonCtrlState;
      if (buttonCtrlState) butCount++;

      //trigger Function 
      tenSecond(mqttJSON);
    }else{
      mqttJSON["reqp"] = 0;
    }

    //|| (oldDist < (garbageCanHeight+5.0) && DistanceForStol > (garbageCanHeight+5.0))
    if ((oldLightVal < 75 && lightVal > 75) ) {
      Serial.println("Lid just opened");
      lightSec = mySec;
      stol = true;
      mqttJSON["buzz"] = 0;
      tenSecond(mqttJSON);
      //|| (oldDist > (garbageCanHeight+5.0) && DistanceForStol < (garbageCanHeight+5.0) )
    } else if ((oldLightVal > 75 && lightVal < 75) ){
      Serial.println("Lid just closed");
      digitalWrite(buzzerPin, LOW);
      stol = false;
      mqttJSON["buzz"] = 0;
      buzz = false;
      tenSecond(mqttJSON);
     
    }
    oldLightVal = lightVal;
    //oldDist = DistanceForStol; 

    if (mySec%2) // alle 2 sec wechsel
    {
      digitalWrite(BuildInLed1, HIGH); 
      digitalWrite(BuildInLed2, LOW);  
    }
    else
    {
      digitalWrite(BuildInLed1, LOW); 
      digitalWrite(BuildInLed2, HIGH);   
    }

    if ((mySec%10) == 0)  // alle 10 sec
    {
      mqttCnt++;
      sprintf(oledInfo,"MQTT:%s - Cnt:%04d",ID, mqttCnt); 
      
      //switch this to 1 minute for production
      tenSecond(mqttJSON);
    }

    if ((mySec%30) == 0)  // alle 30 sec send
    {
      
    }
  }

  if(wifi_multi.run()!=WL_CONNECTED)
  {
    Serial.print("WiFi Disconnected!!! Try new WiFi connection. ");
    setup_wifi(); 
  }

  if (!client.connected()) {  
    Serial.print("MQTT Disconnected!!! Try new MQTT connection. ");
    setup_mqtt();  // MQTT reconnection
  }
  client.loop();

  showScreen();
  delay(10);

  // Wait some time before starting a new measurement
  delay(1500); 
   
    
}

// ################################################################################
// ################################################################################

/****************************************
 * WiFi connect and reconnect
 ****************************************/

void setup_wifi() {
    delay(10);
    Serial.println();
    WiFi.mode(WIFI_STA);

    wifi_multi.addAP(ssid1,password1);
    wifi_multi.addAP(ssid2,password2);
    wifi_multi.addAP(ssid3,password3);  
    wifi_multi.addAP(ssid4,password4);
    wifi_multi.addAP(ssid5,password5);

    Serial.print("Connecting to MultiWiFis... ");
    // single WiFi
    // Serial.println(SECRET_SSID);
    // WiFi.begin(SECRET_SSID, SECRET_PASS);
 
    while(wifi_multi.run(connectTimeOutPerAP)!=WL_CONNECTED)
    {
        digitalWrite(LED_BUILTIN, LOW); // ON short
        Serial.print(".");
        delay(300);
        digitalWrite(LED_BUILTIN, HIGH); // OFF long
        Serial.print("-");
        delay(1000);
    }
 
    Serial.println();
    Serial.println("WiFi connection successful");
    Serial.print("Connected to SSID: ");
    mySSID = WiFi.SSID().c_str();
    Serial.println(mySSID);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("=NIS=WIFI=............................................................=OK=");    
}
 
void setup_mqtt() 
{
  long randNumber = random(10000000);
  char mqttCLIENT[50];
  sprintf(mqttCLIENT,MQTT_CLIENT);
  reConnect++;

    while (!client.connected()) {

        if (reConnect == 1) 
          sprintf(dataInfo,"MQTT first connection... Client: %s",mqttCLIENT);       
        else
          sprintf(dataInfo,"MQTT reconnecting... [%04d] Client: %s ",reConnect ,mqttCLIENT);
        Serial.print(dataInfo);
        
        // boolean connect (clientID, [username, password], [willTopic, willQoS, willRetain, willMessage], [cleanSession])
        sprintf(dataInfo,"MQTT died: %s",ID);
        if (!client.connect(mqttCLIENT, MQTT_USER, MQTT_PASS, MQTT_TOPIC_PUB_LWT, 2, true, dataInfo)) {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" retrying in some seconds");
            digitalWrite(LED_BUILTIN, HIGH); 
            delay(500);
            digitalWrite(LED_BUILTIN, LOW);  
            delay(1500);
          }
    }
    Serial.println();
    
    client.subscribe(MQTT_TOPIC_SUB_CTRL, 1);

    // now when WiFi and MQTT works > Info also to MQTT
    sprintf(dataInfo,"WiFi [%s]: %s @ %s", ID, WiFi.localIP().toString().c_str(), mySSID); 
    Serial.println(dataInfo); 
    
    sprintf(dataInfo,"MQTT connected OK: %s - #%d", MQTT_BROKER, reConnect); 
    Serial.println(dataInfo);  
     
    Serial.println("=NIS=MQTT=............................................................=OK=");    
}

/****************************************
 * MQTT GOT Subscribe
 ****************************************/

void callback(char* topic, byte* payload, unsigned int length) 
{ 
  Serial.println();
  Serial.println("--MQTT---------------------------------");
  // memset(msg, 0, 70);

  sprintf(dataInfo,"MQTT Got Msg: %s [%d] >> ", MQTT_BROKER, length); 
  Serial.print(dataInfo);

  char msg[length+1];
  String mqttStr;

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    msg[i] = (char)payload[i];
    mqttStr = mqttStr + msg[i];
  }
  Serial.println();

  if(strcmp(topic,MQTT_TOPIC_SUB_CTRL)==0) 
  { 
    Serial.println("CTRL command: " + mqttStr);
    int ctrlGot = mqttStr.toInt();
    
    if (ctrlGot == 1) 
    {
      digitalWrite(ledCtrlPin,HIGH); // ON 
      digitalWrite(LED_BUILTIN, LOW); // ON
    }

    if (ctrlGot == 0) 
    {
      digitalWrite(ledCtrlPin,LOW);  // OFF 
      digitalWrite(LED_BUILTIN, HIGH); // OFF
    }

    sprintf(dataInfo,"Control command: %s turn %d -> GPIO0 done", mqttStr, ctrlGot); 
    Serial.print(dataInfo);
  }

  sprintf(dataInfo,"MQTT got: >%s< >%s< ",topic, mqttStr); 
  Serial.println(dataInfo); 
  Serial.println("--MQTT----------------------------EOM--");
 }

// ################################################################################


/****************************************
 * Auxiliar Functions for OLED
 ****************************************/

void showScreen()
{
  char charRow2[25];
  char charRow3[25];
  char charRow4[25];
  char charRow5[25];
  char charRow6[25];

  sprintf(charRow2,"SSID: %s ", WiFi.SSID().c_str()); 
  IPAddress ip;  
  ip = WiFi.localIP();
  sprintf(charRow3,"IP: %s",ip.toString().c_str()); 
  sprintf(charRow4,"%s B:%04d",timeString, butCount); 
  sprintf(charRow5,"%s", fillInfo); 
  sprintf(charRow6,"%s", envInfo); 

  // u8g2.clearDisplay();
  u8g2.setDisplayRotation(U8G2_R0);
  u8g2.firstPage();
  do {
    // u8g2.setFont(u8g2_font_ncenB08_tr);
    // u8g2.setFont(u8g2_font_t0_16b_mr);
    u8g2.setFont(u8g2_font_t0_11_mr);   // choose a suitable font https://github.com/olikraus/u8g2/wiki/fntlist8
    //                 012345678901234567890
    u8g2.drawStr(0,10,HOT_CHILI_VERSION);    
    u8g2.drawStr(0,20,charRow2);   
    u8g2.drawStr(0,30,charRow3); 
    u8g2.drawStr(0,40,charRow4);    
    u8g2.drawStr(0,50,charRow5);    
    u8g2.drawStr(0,60,charRow6);    
    u8g2.drawHLine(0,0, 128);
    u8g2.drawHLine(0,63, 128);
  } while (u8g2.nextPage());
}

void showOLED(int x, int y, String str)
{
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_t0_16b_mr);
    u8g2.setCursor(x, y);
    u8g2.print(str);
  } while (u8g2.nextPage());
}