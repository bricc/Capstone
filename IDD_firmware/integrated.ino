/* MAIN CODE
   Restructured on May 2021
   JARK Industries
   Ricardo Bravo, Kyle McCaffrey, Ashley Powell, Jonathan Kiing
*/ 

/*************************************************************************************************************
   Needed libraries
**************************************************************************************************************/
#include <driver/adc.h>
#include <Wire.h>
#include <WebServer.h>
//#include <WebSocketsServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>

/*************************************************************************************************************
 * Preprocessor defined Variables 
 * FSR Pins (Note, ADC2 is used by wifi - ADC1: GPIO32-GPIO39 are the ones available)
 * NEW ASSIGNMENT OF PINS
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html 
**************************************************************************************************************/
/* ANALOG INPUT PINS */
#define FSR_IN_13 35  //IO26 - A0 - Might be useless since ADC2 is used by wifi, same as below 
#define FSR_IN_24 34 //IO25 - A1
#define FSR_IN_5  39  //SENSOR_VN/39??? - A3

//#define FSR_IN_13 26  //IO26 - A0 - Might be useless since ADC2 is used by wifi, same as below 
//#define FSR_IN_24 25 //IO25 - A1
//#define FSR_IN_5  39  //SENSOR_VN/39??? - A3

/* POWER SUPPLY PINS */
#define FSR34_V 32
#define FSR12_V 25
#define FSR5_V 33

//#define FSR34_V 13
//#define FSR12_V 32
//#define FSR5_V 33

/* IMU I2C Communication - Although not needed */ 
#define IMU_SCL 22 //A5
#define IMU_SDA 23 //A4
/**************************************************************************************************************/
/* DEBUGGING PREPROCESSED VARIABLES */
#define WIFI 1 /* READY FOR USE SET TO 1*/
/**************************************************************************************************************/

/* TCP RELATED VARIABLES */

/* WIFI CREDENTIALS: IF DEVICE IS REQUIRED TO CONNECT TO WIFI */ 
const char* ssid = "eduroam";    // <<< change this as yours
const char* password = "xx"; // <<< change this as your

/* TCP SERVER (PHONE) CREDENTIALS 
* host: Static IP (REQUIRED!)
* port: 6001 (Can change if needed. NEED TO CHANGE IN ANDROID APP TOO!)
*/
const char* host = "192.168.1.10";
const int port = 6000;

/* IDD DEVICE WIFI INFORMATION */
IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);
/**************************************************************************************************************/
/* STRUCT TU BE POPULATED */
typedef struct TCP_Packet_t {
  byte FSR_X;
  byte FSR_Y;
  byte FSR_Z;
  byte Fnet;
  byte acc_x;
  byte acc_y;
  byte acc_z;
  byte linacc_x;
  byte linacc_y;
  byte linacc_z;
  byte ZONE_LOC;
  byte theta;
  byte phi;
  byte isCalibrated; 
} dataSent_t1;
  dataSent_t1 dataSent_t;

/* LOCKS 
* tcpLOCK: If set, connection to TCPServer can be initiated
* impactSendUnlock: One time lock set to 1 at void setup(). Need to reset device in order to send again.
*/
static int tcpLOCK = 0;
int impactSendUnlock = 0;
int isCalibrated_var = 0;

/* DATA STRUCTURE #2
* msg: Data that is sent over TCP, not struct. Data is parsed from the struct above and populated in this byte array
* START/END BYTES: Number of bytes for the start and end frame, respectively
* START_DATA: Initial data in the frame, for the android app to decode
* END_DATA: Similar as START_DATA, but end of the farme
* DATA_BYTES: Total number of data parsed and sent (not including the start and end bytes)
*/
#define START_BYTE  1 
#define END_BYTES   1 
#define START_DATA  0x41
#define END_DATA    0x42
#define DATA_BYTES  14  
#define COMMAS   15
#define EOL 0x0A

/* TOTAL DATA SENT */
byte msg[START_BYTE + DATA_BYTES + END_BYTES + 1 + COMMAS]; //to be sent; 

/* CRUCIAL DATA ONLY */
byte dataSent_arr[14];

/**************************************************************************************************************/
/* Structures the data to be sent over TCP */ 
void structure_package(byte data[], byte dataFromStruct[]);


/**************************************************************************************************************/
/* SENSOR CALIBRATION VALUES */ 
float fsr1_offset, fsr2_offset, fsr3_offset, fsr4_offset, fsr5_offset; 
float temp1 = 0;  float temp2 = 0;  float temp3 = 0;  float temp4 = 0;  float temp5 = 0; 
/*************************************************************************************************************
   MAIN CODE
   SETUP() -> MAIN()
**************************************************************************************************************/
void setup()
{
  /* Serial Initialization - Baud Rate: 115200 Hz */
  Serial.begin(115200);

  /* ACTIVATE ONCE; NEED TO RESTART DEVICE TO SET THIS VARIABLE TO 1 */
  impactSendUnlock = 1;

  /* WAIT FOR SERIAL TO ACTIVATE */
  while (!Serial) { ;  }

  /* Initialize components - FSR, IMUs, WiFi (TCP) */
  initialize_FSR_matrix();
  initialize_IMU();
  initialize_AccessPoint_ESP32(); // For TCP testing (working)

  /* */
  delay(10);

  fsr1_offset = 0; 
  fsr2_offset = 0; 
  fsr3_offset = 0; 
  fsr4_offset = 0; 
  fsr5_offset = 0; 

  Serial.println("CALIBRATING SENSORS....");
  
  /* SENSOR CALIBRATION - FSR OFFSET */ 
  for(int i = 0; i < 100; i++){ 
    read_FSR_matrix(); 
    temp1 = temp1 + returnFSR_1(); 
    temp2 = temp2 + returnFSR_2(); 
    temp3 = temp3 + returnFSR_3(); 
    temp4 = temp4 + returnFSR_4(); 
    temp5 = temp5 + returnFSR_5(); 
    delay(10); 
  }
  fsr1_offset = temp1/100;
  fsr2_offset = temp2/100;
  fsr3_offset = temp3/100;
  fsr4_offset = temp4/100;
  fsr5_offset = temp5/100;
  isCalibrated_var = 1;
  Serial.println("SENSORS CALIBRATED!");
  

  
  /* END OF SETUP */

  /* Initialize struct */ 
  dataSent_t.FSR_X = 0;
  dataSent_t.FSR_Y = 0;
  dataSent_t.FSR_Z = 0;
  dataSent_t.Fnet = 0 ;
  dataSent_t.acc_x = 0;
  dataSent_t.acc_y = 0;
  dataSent_t.acc_z = 0;
  dataSent_t.linacc_x = 0;
  dataSent_t.linacc_y = 0;
  dataSent_t.linacc_z = 0;
  dataSent_t.ZONE_LOC = 0;
  dataSent_t.theta = 0;
  dataSent_t.phi = 0;
  dataSent_t.isCalibrated = 0;
}

void loop()
{
   
    //READ FSR 

//   Serial.println(return_ZoneSUM());
//   read_FSR_matrix(); 

    ////////////////////////////////////////////
    /* Fnet - Theta - Phi Debug */ 
//     double x = returnFSRSum();
//        Serial.print(returnFnet());
//        Serial.print(" "); 
//        Serial.print(returnTheta());
//        Serial.print(" "); 
//        Serial.println(returnPhi());
    ////////////////////////////////////////////

    
////    Serial.print("FSR1:");
//    Serial.print(returnFSR_1() - fsr1_offset);
//    Serial.print(" ");
////    Serial.print("FSR2:");
//    Serial.print(returnFSR_2() - fsr2_offset);
////    Serial.print("FSR3:");
//Serial.print(" ");
//    Serial.print(returnFSR_3() - fsr3_offset);
////    Serial.print("FSR4:");
//Serial.print(" ");
//    Serial.print(returnFSR_4() - fsr4_offset);
////    Serial.print("FSR5:");
//Serial.print(" ");
//    Serial.print(returnFSR_5() - fsr5_offset);
//
//
//Serial.print(" ");
//    Serial.println(returnZone());
   
//    
//      Serial.print("A_NET:");
//      Serial.println(read_IMU());

/*
      read_IMU();
//      Serial.print("A_X:");
Serial.print(" ");
      Serial.print(return_IMUX());
//      Serial.print("A_Y:");
Serial.print(" ");
      Serial.print(return_IMUY());
Serial.print(" ");
      Serial.println(return_IMUZ());
      */
      
//     Serial.print(" ");
//     Serial.print(return_LIN_IMUY());
//     Serial.print(" ");
//     Serial.println(return_LIN_IMUZ());
//     

/**
read_FSR_matrix(); 
 Serial.print(returnFSR_1() - fsr1_offset);
Serial.print(",");
 Serial.print(returnFSR_2() - fsr2_offset);
Serial.print(",");
 Serial.print(returnFSR_3() - fsr3_offset);
Serial.print(",");
 Serial.print(returnFSR_4() - fsr4_offset);
//Serial.print(",");
//read_IMU();
// Serial.print(return_IMUX());
//Serial.print(",");
// Serial.print(return_IMUY());
//Serial.print(",");
// Serial.print(return_IMUZ());
//Serial.print(",");
// Serial.print(return_LIN_IMUX());
//Serial.print(",");
// Serial.print(return_LIN_IMUY());
//Serial.print(",");
// Serial.print(return_LIN_IMUZ());
Serial.print(",");
 Serial.println(returnFSR_5() - fsr5_offset);
*/

if(angularCalib()){
  delay(1000);
//  Serial.println("LIVE CALIBRATING SENSORS.../.");
  temp1 = 0;  temp2 = 0;  temp3 = 0;  temp4 = 0;   temp5 = 0; 
  /* SENSOR CALIBRATION - FSR OFFSET */ 
  for(int i = 0; i < 100; i++){ 
    read_FSR_matrix(); 
    temp1 = temp1 + returnFSR_1(); 
    temp2 = temp2 + returnFSR_2(); 
    temp3 = temp3 + returnFSR_3(); 
    temp4 = temp4 + returnFSR_4(); 
    temp5 = temp5 + returnFSR_5(); 
//  delay(10); 
  }
  fsr1_offset = temp1/100;
  fsr2_offset = temp2/100;
  fsr3_offset = temp3/100;
  fsr4_offset = temp4/100;
  fsr5_offset = temp5/100;
//  Serial.println("LIVE CALIBRATION DONE!");
}

    
  /* MAIN CODE */ 
  #if WIFI

  /* Get FSR reading for this loop */ 
//  double fsrReading = returnFSRSum();

  /* TRIGGER SO+HOULD BE IMU */
  if(return_ZoneSUM() > 500.0){

//   if(angularTrigger()){
    /* ENABLE LOCK FOR SENDING */

    tcpLOCK = 1;

    dataSent_t.FSR_X = (char)(returnFSR_X()/100);
    dataSent_t.FSR_Y = (char)(returnFSR_Y()/100);
    dataSent_t.FSR_Z = (char)(returnFSR_Z()/100);
    dataSent_t.Fnet =  (char)(returnFnet()/100);
    dataSent_t.acc_x = (char)(return_IMUX());
    dataSent_t.acc_y = (char)(return_IMUY());
    dataSent_t.acc_z = (char)(return_IMUZ());
    dataSent_t.linacc_x = (char)return_LIN_IMUX();
    dataSent_t.linacc_y = (char)return_LIN_IMUY();
    dataSent_t.linacc_z = (char)return_LIN_IMUZ();
    dataSent_t.ZONE_LOC = (char)(returnZone());
    dataSent_t.theta = (char)returnTheta();
    dataSent_t.phi = (char)returnPhi();
    dataSent_t.isCalibrated = 1; 
    
  }else{ } // NOTHING

  delay(10);
  if(tcpLOCK){

//    Serial.print(dataSent_t.FSR_X);
//    Serial.print(" ");
//    Serial.print(dataSent_t.FSR_X);
//    Serial.print(" ");
//    Serial.print(dataSent_t.FSR_X);
//    Serial.print("  ");
    
//    Serial.print(dataSent_t.FSR_X);
//    Serial.print(" ");
//    Serial.print(dataSent_t.FSR_Y);
//    Serial.print(" ");
//    Serial.print(dataSent_t.FSR_Z);
//    Serial.print(" ");
//    Serial.print(dataSent_t.Fnet);
//    Serial.print(" | ");
//    Serial.print(dataSent_t.acc_x);
//    Serial.print(" ");
//    Serial.print(dataSent_t.acc_y);
//    Serial.print(" ");
//    Serial.print(dataSent_t.acc_z);
//    Serial.print(" | ");
//    Serial.print(dataSent_t.linacc_x);
//    Serial.print(" ");
//    Serial.print(dataSent_t.linacc_y);
//    Serial.print(" ");
//    Serial.print(dataSent_t.linacc_z);
//    Serial.print(" | ");
//    Serial.print(dataSent_t.ZONE_LOC);
//    Serial.print(" ");
//    Serial.print(dataSent_t.theta);
//    Serial.print(" ");
//    Serial.print(dataSent_t.phi);
//    Serial.print(" ");
//    Serial.println(dataSent_t.isCalibrated);

    /* Use WiFiClient class to create TCP connections */
    WiFiClient client;

    /* INITIALIZE. sendData determines if data is good for sending */
    int sendData = 0; //activates when connected to host 

    /* debug */ 
//    Serial.println("Connecting....");

    if (!client.connect(host, port)) {
//      Serial.println("CONNECTION FAILED");

      /* REMOVE LOCK. NEED NEW TRIGGER AGAIN */
      tcpLOCK = 0; 
      return;
    } else {
//      Serial.println("CONNECTED TO HOST");

      /* ENABLE DATA LOCK */
      sendData = 1; 
    }
    delay(10);
    
    #if WIFI
    /* NEED TO UNLOCKS. SECOND ONE IS ONLY SET ONCE AT SETUP */ 
    if(sendData && impactSendUnlock){
      
      /* This will send the data to the server */
//      Serial.println("DATA SENDING......");
      byte *ptr = (byte *) &dataSent_t;
      for (int i = 0; i < sizeof(dataSent_t1); i++) 
      {
        dataSent_arr[i] = *ptr;
        ptr++;
      }
      structure_package(msg, dataSent_arr);
      client.write(msg, sizeof(msg));
      delay(5);
//      impactSendUnlock = 0;
    }
//    client.stop();
    
//    while(client.connected());
    #endif 
    
    tcpLOCK = 0;
  }
    delay(100);
  #endif
} // END MAIN



/***********************************************************************
 * structure_package()
 ***********************************************************************
 * Fills up the data to be sent through TCP. 
 ***********************************************************************/
void structure_package(byte data_buf[], byte dataFromStruct[]){
  byte *buffer_ptr; 

  /* PUTS POINTER AT THE START OF THE BUFFER */ 
  buffer_ptr = data_buf;

  /* FIRST INDEX = 0x88 (START_DATA) */ 
  for(int i = 0; i < START_BYTE; i++){
    *buffer_ptr = 88;
    *buffer_ptr++;
  }
//
    *buffer_ptr = 0x2C;
    *buffer_ptr++;

  /* POPULATE THE DATA SEE STRUCT ABOVE */
  for(int j = 0; j < (DATA_BYTES); j++){
    *buffer_ptr = dataFromStruct[j];
//*buffer_ptr = 200 + j;
    *buffer_ptr++;
      *buffer_ptr = 0x2C;
      *buffer_ptr++;
  }

  /* END FRAME- 0x11 (twice! 2 bytes)*/ 
  for(int k = 0; k < END_BYTES; k++){
    *buffer_ptr = 11; 
    *buffer_ptr++;
  }
  *buffer_ptr = EOL; 
  delay(10);
}
