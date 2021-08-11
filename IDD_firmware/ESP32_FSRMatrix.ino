/************************************************************************************
 * FSR MATRIX ALGORITHM
 * Created: June 17, 2021
 * Created by: Ricardo Bravo
 ************************************************************************************/ 
// Included Libraries

/*********************************/
#include <math.h>

/* TOTAL OF 3 FSR READINGS */
static double FSR_ZONE_SUM = 0; 

/* ADC CONSTANTS */
static const float VCC = 3.3;
static const float R_DIV = 10000.0;

/* ASSUMED HEAD LENGTH CONSTANTS. WITH RESPECT TO THE MAXIMA OF THE HEAD */
static  float toptoFront = 20;
static const float toptoLateral = 16;
static const float toptoBack = 22;

/**/ 
static float Px, Py;

/* FSR PARAMETERS 
* FSR_1 to FSR_5: RAW READINGS
* Fvec: sqrt(fsrx^2 + fsry^2 + fsrz^2) - Force vector, but in RAW VALUES FOR NOW
* Theta: Polar Coordinate Angle
* Zeta: PC Angle
*/
static float FSR_1, FSR_2, FSR_3, FSR_4, FSR_5; // Force Sensitive Resistors Values 
static float Fvec, Theta, Zeta; // 3D Coordinates (Total Force, Theta, Zeta) 

/* ZONE DETECTION VARABLES */
static int ZONE_DETECTED = 0;
static float FSR_ZONE_ARR1 = 0;
static float FSR_ZONE_ARR2 = 0;
static float FSR_ZONE_ARR3 = 0;
static float FSR_ZONE_ARR4 = 0;


/*************************************************************************************************************
  initialize_FSR_matrix()
  - Called in setup()
  - Initializes the pin used (as input or output)

 FSR# - Supply - ADC Input    Location     Variable (Input) -  Variable (Output)
 FSR1 - IO32   -    A0      -   Front   |   FSR_IN_13 26    |   FSR12_V 32  // IO32
 FSR2 - IO32   -    A1      -   Left    |   FSR_IN_24 25    |   FSR34_V 13  // IO13
 FSR3 - IO13   -    A0      -   Back    |   FSR_IN_5  39    |   FSR5_V  33  // IO33
 FSR4 - IO13   -    A1      -   Right   |                   |
 FSR5 - IO33   -    A3      -   Top     |                   |
**************************************************************************************************************/
void initialize_FSR_matrix(){
  
  /* INITIALIZING INPUTS */
  pinMode(FSR_IN_13, INPUT);
  pinMode(FSR_IN_24, INPUT);
  pinMode(FSR_IN_5, INPUT);

  /* INITIALIZING OUTPUTS */
  pinMode(FSR34_V, OUTPUT);
  pinMode(FSR12_V, OUTPUT);
  pinMode(FSR5_V, OUTPUT);
  
}

/*************************************************************************************************************
   read_FSR_matrix()
   - Reads the pins and stores them to fsr1-5, respectivley
**************************************************************************************************************/
void read_FSR_matrix() {
  
  /* TOP FSR */
  digitalWrite(FSR5_V, HIGH);
  FSR_5 = (analogRead(FSR_IN_5));
  delay(10);
  
  /* FRONT AND LEFT FSR */ 
  digitalWrite(FSR12_V, HIGH);
  digitalWrite(FSR34_V, LOW);
  FSR_1 = (analogRead(FSR_IN_13));
  FSR_2 = (analogRead(FSR_IN_24));
  delay(10);

  /* BACK AND RIGHT FSR */
  digitalWrite(FSR12_V, LOW);
  digitalWrite(FSR34_V, HIGH);
  FSR_3 = (analogRead(FSR_IN_13));
  FSR_4 = (analogRead(FSR_IN_24));
  delay(10);

}

/*************************************************************************************************************
   read_impact_coordinates()
   - Uses the value from read_FSR_matrix() to calculate the impact coordinates
   - Values calculated
     - Zone (1,2,3,or 4, - depending on the FSRs triggered upon impact)
     - Magnitude - Fnet - (sqrt(FX^2+FY^2+FZ^2)

 NOTE: Algorithm only gets the x and y coordinate of the impact. 

 Release 2.0 - July 17: Will incorporate force.
**************************************************************************************************************/
String read_impact_coordinates() {
  /* READ RAW FSR VALUES */
  read_FSR_matrix();

  /* STORE RAW VALUES IN AN ARRAY */
  float FSR_ARR[5] = {FSR_1,FSR_2,FSR_3,FSR_4,FSR_5};

  /* SCALING VALUE FROM PREVIOUS GROUP; UNKNOWN*/
   /*
   for (int i = 0; i < 5 ; i++){
     if (FSR_ARR[i]!=0){
      FSR_ARR[i]=9.81*(exp((FSR_ARR[i] + 6.2982)/1.1086))/1000; //Wtf is this
     }
   }
   */

  /********************************************************************************
   * 3 - ZONE DIVISION - Zone 1, Zone 2, Zone 3, Zone 4 (Divide hemisphere)
   * Zone 1 - FSR1/2/5
   * Zone 2 - FSR2/3/5
   * Zone 3 - FSR3/4/5
   * Zone 4 - FSR1/4/5
  *********************************************************************************/
  float FSR1_ADJUSTED = FSR_1 - fsr1_offset;
  float FSR2_ADJUSTED = FSR_2 - fsr2_offset;
  float FSR3_ADJUSTED = FSR_3 - fsr3_offset;
  float FSR4_ADJUSTED = FSR_4 - fsr4_offset;
  float FSR5_ADJUSTED = FSR_5 - fsr5_offset;
  
//  float FSR_ZONE_ARR[4] = {FSR_ARR[0]+FSR_ARR[1]+FSR_ARR[4], FSR_ARR[1]+FSR_ARR[2]+FSR_ARR[4], FSR_ARR[2]+FSR_ARR[3]+FSR_ARR[4], FSR_ARR[0]+FSR_ARR[3]+FSR_ARR[4]};
  float FSR_ZONE_ARR[4] = {FSR1_ADJUSTED+FSR2_ADJUSTED+FSR5_ADJUSTED, FSR2_ADJUSTED+FSR3_ADJUSTED+FSR5_ADJUSTED, FSR3_ADJUSTED+FSR4_ADJUSTED+FSR5_ADJUSTED, FSR4_ADJUSTED+FSR1_ADJUSTED+FSR5_ADJUSTED};

  
//  Serial.print("1,2,3,4,5: "); 
//  Serial.print(FSR_1);
//  Serial.print(", ");
//  Serial.print(FSR_2);
//  Serial.print(", ");
//  Serial.print(FSR_3);
//  Serial.print(", ");
//  Serial.print(FSR_4);
//  Serial.print(", ");
//  Serial.println(FSR_5);
  FSR_ZONE_SUM = 0;
  ZONE_DETECTED = 0;
  
  /* SORT - Extract Max value to determine the Zone of Impact */ 
  for(int i = 0; i < 4; i++){

    
    /* Determine the zone with the highest impact readings */
    if (FSR_ZONE_ARR[i] > FSR_ZONE_SUM ){
      FSR_ZONE_SUM = FSR_ZONE_ARR[i];
      ZONE_DETECTED = i+1;
    }

    if(FSR_ZONE_SUM < 1000){
      ZONE_DETECTED = 0;
    }
    
  }

  // Debugging - Use for printing values of each zones. O(4) for this line.
  FSR_ZONE_ARR1 = FSR_ZONE_ARR[0]; 
  FSR_ZONE_ARR2 = FSR_ZONE_ARR[1]; 
  FSR_ZONE_ARR3 = FSR_ZONE_ARR[2]; 
  FSR_ZONE_ARR4 = FSR_ZONE_ARR[3];

  /* SIMPLIFIED X AND Y FORCES */  
  Px = 0;
  Py = 0;
  
  /* ZONE DETECTION */
  if(FSR_ZONE_SUM!=0){

   if(ZONE_DETECTED == 1){ /* +X, -Y */
       float* xy = impactLocation(FSR_ARR[0],FSR_ARR[1],toptoLateral,toptoFront);
       Px = round((abs(xy[0])));
       Py = round((abs(xy[1])));
   } else if(ZONE_DETECTED == 2){ //-x,+y
       float* xy = impactLocation(FSR_ARR[2],FSR_ARR[1],toptoLateral,toptoBack);
       Px = round((abs(xy[0])));
       Py = round((abs(xy[1])));
       if(Px!=0){  Px = -Px; }
   } else if(ZONE_DETECTED == 3){
       float* xy = impactLocation(FSR_ARR[2],FSR_ARR[3],toptoLateral,toptoBack);
       Px = round((abs(xy[0])));
       Py = round((abs(xy[1])));
       if(Px!=0){Px = -Px;}
       if(Py!=0){ Py = -Py; }
   } else if(ZONE_DETECTED == 4){
       float* xy = impactLocation(FSR_ARR[0],FSR_ARR[3],toptoLateral,toptoFront);
       Px = round((abs(xy[0])));
       Py = round((abs(xy[1])));
       if(Py!=0){ Py = -Py; }
   }
  }
  
  // Currently, there is not Z coordinate. Might need to get the average 
  return "X: "+String(Px)+",Y: "+String(Py)+",Z: "+String(FSR_ZONE_SUM);
}


/**********************************************
 * ADC2R() 
 * Converts the ADC reading to Voltage 
 **********************************************/
float ADC2R(int adc) {
  float v = adc * VCC / 4096;
  return v;
}

/**********************************************
 * impactLocation()
 * Returns the coordinates of impact 
 **********************************************/
float* impactLocation(float Zx, float Zy, float Lx, float Ly) {
  static float coordinate[2];

  // Location Scaling - Based on FSR Reading
  float Px = (Zx / FSR_ZONE_SUM) * Lx;
  float Py = (Zy / FSR_ZONE_SUM) * Ly ;
  
  coordinate[0] = Px;
  coordinate[1] = Py;
  return coordinate;
}

/**********************************************
 * impact3DLocation() - UNUSED
 * Returns the 3D coordinates of impact 
 * - Vector Magnitude of Force 
 * - Theta (angle between the X and Y components)
 * - Zeta (Angle between resultant vector and Z component of Force
 **********************************************/
float* impact3DLocation(){
  static float this_3dCoordinates[3]; 

  float Fvec, Theta, Zeta; 
  //Force 
  this_3dCoordinates[0] = Fvec;
  this_3dCoordinates[1] = Theta;
  this_3dCoordinates[2] = Zeta;

  return this_3dCoordinates;
}

// DEBUGGING FUNCTION
int returnZone(){
  return ZONE_DETECTED; // -1 FOR ERROR
}

/* DEBUGGING FUNCTION */
double returnFSRSum(){
  read_impact_coordinates();
//    if(FSR_5 > 100.0){
//    /* ENABLE LOCK FOR SENDING */
//    Serial.print("IMPACT 5 DETECTED: "); 
//    Serial.println(FSR_5);
//    } 
//  Serial.print("FSR_ZONE_SUM: "); 
//  Serial.println(FSR_ZONE_SUM);
  return FSR_ZONE_SUM;
}

double returnFSR_X() {
//  return Px > 0? Px : 69;
   float Fx = FSR_1 > FSR_3? FSR_1 - fsr1_offset : FSR_3 - fsr3_offset;
   return Fx; 
}

double returnFSR_Y() {
//  return Py > 0? Py : 69;
float Fy = FSR_2 > FSR_4? FSR_2 - fsr2_offset : FSR_4 - fsr4_offset; 
return Fy;
}

double returnFSR_Z() {
  return FSR_5 > 0? FSR_5 - fsr5_offset : 69;
}

////
float returnFSR_1() {
//  return FSR_1;
  return FSR_1 >= 0? FSR_1 : 69;
}
float returnFSR_2() {
  return FSR_2 >= 0? FSR_2 : 69;
}
float returnFSR_3() {
  return FSR_3 >= 0? FSR_3 : 69;
}
float returnFSR_4() {
  return FSR_4 >= 0? FSR_4 : 69;
}
float returnFSR_5() {
  return FSR_5 >= 0? FSR_5 : 69;
}

double return_ZoneSUM(){
   read_impact_coordinates();
  return FSR_ZONE_SUM; 
}

float returnFnet(){
   float Fx = FSR_1 > FSR_3? FSR_1 - fsr1_offset : FSR_3 - fsr3_offset; 
   float Fy = FSR_2 > FSR_4? FSR_2 - fsr2_offset : FSR_4 - fsr4_offset; 
   float Fz = FSR_5 - fsr5_offset;
   
   float Fnet = sqrt(Fx*Fx + Fy*Fy + Fz*Fz); 
   return Fnet; 
}

float returnTheta(){
  float Fx = FSR_1 > FSR_3? FSR_1 - fsr1_offset : FSR_3 - fsr3_offset; 
  float Fy = FSR_2 > FSR_4? FSR_2 - fsr2_offset : FSR_4 - fsr4_offset; 

      float theta; 
  if(Fx == 0){ 
    theta = 0;
  }else{
  theta = atan(Fy/Fx); 
  }
  return theta * (180/PI); 
}

float returnPhi(){
  float fsr5_adjusted = FSR_5 - fsr5_offset;
  float Fnet = returnFnet(); 
  float Phi;
  if(Fnet == 0){ 
    Phi = 0;
  }else{
    Serial.print(fsr5_adjusted); 
    Serial.print(" ");
    Serial.println(Fnet); 
       Phi = acos(fsr5_adjusted/Fnet);
  }
  return Phi * (180/PI);
}
 /*
 //// Zone detection - Don't delete yet, might use later
//if((FSR_ZONE_ARR[0] > 0 && (FSR_ARR[0] > 0 && FSR_ARR[1]== 0 && FSR_ARR[2]== 0 && FSR_ARR[3]==0))){
//  ZONE_DETECTED = 1;
//}else if((FSR_ZONE_ARR[1] > 0 && (FSR_ARR[1] > 0&& FSR_ARR[2]== 0 && FSR_ARR[0]== 0 && FSR_ARR[3]==0))){
//ZONE_DETECTED = 2;
//}else if((FSR_ZONE_ARR[2] > 0 && (FSR_ARR[2] > 0 && FSR_ARR[1]== 0 && FSR_ARR[0]== 0 && FSR_ARR[3]==0))){
//ZONE_DETECTED = 3;
//}else if((FSR_ZONE_ARR[3] > 0 && (FSR_ARR[3] > 0 && FSR_ARR[1]== 0 && FSR_ARR[2]== 0 && FSR_ARR[0]==0))){
//  ZONE_DETECTED = 4;
//}else{
//  ZONE_DETECTED = 0;
//}
 
 */
 
