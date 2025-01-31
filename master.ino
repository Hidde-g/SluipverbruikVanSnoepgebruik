#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <LiquidCrystal_I2C.h>
#include "config.h"

#define LEDPIN 2       // Pin voor on-board LED
#define TRIG_PIN 5     // Pin voor TRIG op de ESP32
#define ECHO_PIN 18    // Pin voor ECHO op de ESP32

#define POWER_DATA_REQUEST_INTERVAL 1
#define POWER_DATA_AVERAGE_INTERVAL 60

//API adress of HomeWizard plug
const char* plugAdress = "http://192.168.4.2/api/v1/data";

//AP mode credentials
const char* ssid_ap = "SluipverbruikVanSnoepgebruik";
const char* password_ap = "SluipverbruikVanSnoepgebruik";

//LCD object
LiquidCrystal_I2C lcd(0x27, 16, 2);

//vars
unsigned long currentTime;
unsigned long lastTime;
int wifiState = 0;

const int debounceTime = 12500; // debounce timer in milliseconden
const float detectionThreshold = 20.0;   // Afstandsgrens in cm
unsigned long lastDetectionTime = 0; // Tijdstip van de laatste detectie

struct powerData {
  float activePower;  
  unsigned long powerTimestamp;
};

const int dataArrayLength = 1500;

powerData powerDataArray[dataArrayLength];
int powerDataArrayIndex = 0;

unsigned long saleDataArray[dataArrayLength];
int saleDataArrayIndex = 0;


//Initialisation functions
//Get current unix timestamp
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return 0;
  }
  time(&now);
  return now; 
}

//Blink the on-board led
void blink(){
  digitalWrite(LEDPIN, HIGH);
  delay(200);
  digitalWrite(LEDPIN, LOW);
  delay(200);
  digitalWrite(LEDPIN, HIGH);
  delay(200);
  digitalWrite(LEDPIN, LOW);
  delay(200);
  digitalWrite(LEDPIN, HIGH);
  delay(200);
  digitalWrite(LEDPIN, LOW);
  delay(200);
  digitalWrite(LEDPIN, HIGH);
  delay(200);
  digitalWrite(LEDPIN, LOW);
}

//Connect to eduroam
void connectWiFi(){
  WiFi.begin(ssid, WPA2_AUTH_PEAP, username, username, password);
  Serial.print("Connecting to eduroam");
  
  unsigned long WifiConnectTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    lcd.setCursor(0, 0);
    lcd.print("Started...");
    lcd.setCursor(0, 1);
    lcd.print("Waiting for plug");
    wifiState = 1;
    Serial.println("\nConnection established!");
    Serial.print("IP-adres: ");
    Serial.println(WiFi.localIP());
    Serial.println("---------------------------------");
  } else {
    Serial.println("\nConnection failed, try again");
    WiFi.disconnect(true);
  }
}

//Start local network for plug
void startNetwork(){
  WiFi.softAP(ssid_ap, password_ap);
  Serial.println("Network started!");
  Serial.print("AP IP adres: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("---------------------------------");
}


//loop functions
//Request current power from plug via HTTP over local network
float requestPlugData() {
  HTTPClient http;
  http.begin(plugAdress); // Initialize HTTP client
  http.setTimeout(500);
  int httpResponseCode = http.GET(); // Make a GET request

  if (httpResponseCode > 0) {
    String responseBody = http.getString();

    // Parse JSON response
    StaticJsonDocument<512> doc;  // Adjust size if needed
    DeserializationError error = deserializeJson(doc, responseBody);
    http.end();

    if (error) {
      return -1;
    }

    float activePower = doc["active_power_w"];
    return activePower;
    
  } else {
    http.end();
    return -1;
  }
}

// Check if there is a transaction
bool checkForCustomer(){
  float currentDistance = getUltrasonicDistance();
  if (currentDistance > 0 && currentDistance < detectionThreshold) {
    if (millis() - lastDetectionTime > debounceTime) {
      lastDetectionTime = millis();
      return true;
    }
    return false;
  }
  return false;
}

// Receive distance from ultrasonic sensor
float getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // read pulse length
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) {
    return -1; // No object
  }
  
  float distance = duration * 0.034 / 2; 
  return distance;
}

//Send current power usage to LCD screen
void sendDataToLCD(float currentPower) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Stroomverbruik:");
  lcd.setCursor(0, 1);
  lcd.print(currentPower);
  lcd.print(" Watt");
}

//Add power usage with timestamp to array
void addPowerDataToArray(float currentPower, unsigned long currentTime) {
  if (powerDataArrayIndex < dataArrayLength) {
    powerDataArray[powerDataArrayIndex].activePower = currentPower;
    powerDataArray[powerDataArrayIndex].powerTimestamp = currentTime;
    powerDataArrayIndex++;
  } else {
    Serial.println("Power array is full!");
  }
}

//Add sale with timestamp to array
void addSaleDataToArray(unsigned long currentTime) {
  if (saleDataArrayIndex < dataArrayLength) {
    saleDataArray[saleDataArrayIndex] = currentTime;
    saleDataArrayIndex++;
  } else {
    Serial.println("Sale array is full!");
  }
}

//Send average power to Thingsboard via HTTP over eduroam
void sendDataArraysToThingsboard(){
  // Create JSON style file
  String data = "[";

  //Add power array data to JSON file
  for(int i = 0; i < powerDataArrayIndex; i++){
    data += "{\"ts\":";
    data += powerDataArray[i].powerTimestamp;
    data += "000,\"values\":{\"power\":";
    data += powerDataArray[i].activePower;
    data += "}}";
    if(i != powerDataArrayIndex-1 || saleDataArrayIndex >= 1){
      data += ",";
    }
  }

  // Add sale array data to JSON file
  for(int i = 0; i < saleDataArrayIndex; i++){
    data += "{\"ts\":";
    data += saleDataArray[i];
    data += "000,\"values\":{\"sale\":";
    data += 1;
    data += "}}";
    if(i != saleDataArrayIndex-1){
      data += ",";
    }
  }

  // End the file
  data += "]";

  HTTPClient http;
  String url = "http://145.131.6.212/api/v1/HR/34lj8r2b7m4ed9hwmij9/telemetry";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  // Send POST request and retreive responce code
  int httpResponseCode = http.POST(data);

  // Check if request was succesful
  if (httpResponseCode <= 0) {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
    digitalWrite(LEDPIN, LOW);
  } else {
    Serial.print(saleDataArrayIndex);
    Serial.print(" sale(s) and ");
    Serial.print(powerDataArrayIndex);
    Serial.println(" Power data value(s) were sent to Thingsboard!");
    powerDataArrayIndex = 0;
    saleDataArrayIndex = 0;
  }

  http.end();
}


//setup and loop
void setup() {
  Serial.begin(115200U);
  Serial.println("\n---------------------------------");

  // Initialize lcd screen
  lcd.init(); 
  lcd.backlight();

  // Set pinmodes
  pinMode(LEDPIN, OUTPUT); 
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Connect to eduroam
  connectWiFi();
  delay(200);
  // Start local WiFi network
  startNetwork();
  delay(200);

  // Initialize unix timestamp
  configTime(0, 0, "pool.ntp.org");
  currentTime = getTime();
  Serial.print("Current Unix timestamp in seconds: ");
  Serial.println(currentTime);
  Serial.println("---------------------------------");
  delay(200);

  Serial.println("Program started!");
  Serial.println("---------------------------------");
  blink();
}

void loop() {
  currentTime = getTime(); // Get the current unix timetamp

  if(checkForCustomer()){ //Check if there is a sale
    addSaleDataToArray(currentTime); // Save data timestamp
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Transactie!");
  }
  
  if(currentTime != lastTime){ //run only once every second
    lastTime = currentTime;

    if(currentTime % POWER_DATA_REQUEST_INTERVAL == 0){
      float currentPower = requestPlugData(); // Retreive data from Homewizard energy socket
      if(currentPower >= 0.0){ // Check if data is valid
        digitalWrite(LEDPIN, HIGH);
        addPowerDataToArray(currentPower, currentTime); // Save power data and timestamo
        sendDataToLCD(currentPower); // Show power on lcd screen
      } else {
        digitalWrite(LEDPIN, LOW);
      }
    }

    if(currentTime % POWER_DATA_AVERAGE_INTERVAL == 0){ // Run every minute
      if(powerDataArrayIndex >= 1 || saleDataArrayIndex >= 1){ // Check if there is data
        if (WiFi.status() != WL_CONNECTED) { // Check if there is WiFi connection to eduroam
          if(wifiState != 0){ //C heck if ESP is already reconnecting to eduroam
            Serial.println("Lost connection, saving data locally. \ntrying to reconnect...");
            WiFi.begin(ssid, WPA2_AUTH_PEAP, username, username, password); // Reconnect to eduroam
            wifiState = 0;
            yield();
          } else {
            Serial.println("Still reconnecting...");
          }
        } else { // Send all data to Thingsboard dashboard
          wifiState = 1;
          sendDataArraysToThingsboard();
        }
      }
    }
  }
}
 