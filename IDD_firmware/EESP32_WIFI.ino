/************************************************************************************
 * This file is used for ESP32 
 * Created: June 17, 2021
 * Created by: Ricardo Bravo
 ************************************************************************************/ 
#define TIMEOUT 5000
/* ESP32 Credentials*/
const char* ssid2 = "ESP32";  // Enter SSID here
const char* password2 = "12345678";  //Enter Password here


/*******************************************************************
 * initialize_AccessPoint_ESP32()
 * Initializes the ACCESS POINT of the device
 *******************************************************************/
void initialize_AccessPoint_ESP32(){
  Serial.begin(115200);
  WiFi.softAP(ssid2, password2);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);
}


/*******************************************************************
 * initialize_wifi_ESP32() (NOT NEEDED FOR THIS VERSION)
 * Initializes ESP32 and connects it to a local network
 *******************************************************************/
void initialize_wifi_ESP32() {
    //WIFI INIT
  Serial.printf("Connecting to %s\n", ssid);
  if (String(WiFi.SSID()) != String(ssid)) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
//  Serial.println("");
 Serial.print("Connected! IP address: ");
  String ipAddress = WiFi.localIP().toString();;
  Serial.println(ipAddress);
}
