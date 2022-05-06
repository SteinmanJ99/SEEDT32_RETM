#include <SPI.h>
#include <SD.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x3F for a 16 chars and 2 line display



float SensArea = .25 * 3.1415926535897932384626433832795 * sq(0.375);
float cfp = .024291; // caliberation factor 19.5
float cfd = 10;//calibration for distance
//float cf1 = //cf1 is distance calibration for press
float psi = (2.205 / SensArea * 3); // converts the force reading from kg to a pressure reading of psi across 3 sensors.
float in = 1 / 25.4; //converts the vertical and horizontal distance from mm to inches.

int i; //counter variable

int Potpin = A4; //sensor output pin connected to A0
int ffs1 = A0; // FlexiForce sensors connected analog pin A0 for Jaw 1
int ffs2 = A1; // FlexiForce sensors connected analog pin A1 for Jaw 2
int ffs3 = A2;// FlexiForce sensors connected analog pin A2 for Jaw 3
int ffs4 = A3; // FlexiForce sensors connected analog pin A3 for Jaw 4


int ffsdata1 = 0; //initialize data for Jaw 1
int ffsdata2 = 0; //initialize data for Jaw 2
int ffsdata3 = 0; //initialize data for Jaw 3
int ffsdata4 = 0; //initialize data for Jaw 4

float potvalue = 0;//variable for value coming from sensor

float vout; //
float vout1; //output variables for the jaws, numbered for which jaw it is sensing
float vout2;
float vout3;
float vout4;


float vout1p; //output variables for the pressure of the jaws
float vout2p;
float vout3p;
float vout4p;


float vout1f; //output variables for the force of the jaws
float vout2f;
float vout3f;
float vout4f;


// offsets
int ffs1_offset = 0;
int ffs2_offset = 0;
int ffs3_offset = 0;
int ffs4_offset = 0;

void getCalibration()
{
  ffs1_offset = analogRead(ffs1);
  ffs2_offset = analogRead(ffs2);
  ffs3_offset = analogRead(ffs3);
  ffs4_offset = analogRead(ffs4);
  //  //Following for and if statements run during start up for ____ seconds while the sensors calibrate and give the okay to begin testing.
  lcd.begin();
  lcd.clear();
  lcd.backlight();
  for (i = 1; i < 15; i++) {
    lcd.setCursor(0, 0);
    lcd.print("Hello m2 tester!");

    lcd.setCursor(0, 1);
    lcd.print("LCD Screen Test");
    Serial.println(i);
    delay(1000);

    if (i >= 14) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("You may begin");
      lcd.setCursor(0, 1);
      lcd.print("testing the RMT.");
      delay(5000);
      lcd.clear();
    }
  }
}

void logData(float vout1, float vout2, float vout3, float vout4, float totalPressure, float horizontal);

//SD card initilization variables
File myFile;
String dataString = "";

void setup()
{
  Serial.begin(9600);


  pinMode(potvalue, INPUT); //calling potvalue pin an input
  pinMode(ffs1, INPUT);
  pinMode(ffs2, INPUT);
  pinMode(ffs3, INPUT);
  pinMode(ffs4, INPUT);

  getCalibration();


  // Jeff - I need to figure out how to multithread. with that I can run this once while the sensors are initializing. for now commenting out




  //Code below will initialize the sd card on the MKR Zero Board and will create a .txt file to store the sensor data.

  Serial.print("Initializing SD card...");

  if (!SD.begin()) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");



  // open a new file and immediately close it:
  Serial.println("Creating, Opening, and Closing FrcDisp1.txt...");
  myFile = SD.open("FrcDisp1.txt", FILE_WRITE);
  myFile.close();

  //The file name has to a max of 8 characters long if not it will fail to initialize the file.
  if (SD.exists("FrcDisp1.txt")) {
    Serial.println("FrcDisp1.txt exists.");
  } else {
    Serial.println("FrcDisp1.txt doesn't exist.");
  }



}


void loop() {
  float horizontal;
  float Total_Radial_Force;
  float Total_Radial_Pressure;
  potvalue = analogRead(Potpin); //reading the analog data
  potvalue  = ((potvalue - (cfd)) / 2.6); //calibrating the data for the correct distance
  horizontal = ((potvalue * 10) / 23.462) * in;

  Serial.print("Vertical Distance (in): ");
  Serial.println(potvalue, 3); //printing the data to the serial monitor
  Serial.print("Horizontal Distance (in): ");
  Serial.println(horizontal, 3);
  delay(100); //delay for the serial monitor

  ffsdata1 = analogRead(ffs1) - ffs1_offset;
  vout1 = ffsdata1;
  vout1p = (vout1 * cfp) * psi ;
  vout1f = (vout1 * cfp);
  Serial.print("Jaw 1 Force (psi): ");
  Serial.print(vout1p, 3);
  Serial.println("");
  //  delay(100);

  ffsdata2 = analogRead(ffs2) - ffs2_offset;
  vout2 = ffsdata2;
  vout2p = vout2 * cfp * psi;
  vout2f = vout2 * cfp;
  Serial.print("Jaw 2 Force (psi): ");
  Serial.print(vout2p, 3);
  Serial.println("");
  //  delay(100);

  ffsdata3 = analogRead(ffs3) - ffs3_offset;
  ; vout3 = ffsdata3;
  vout3p = vout3 * cfp * psi ;
  vout3f = vout3 * cfp;
  Serial.print("Jaw 3 Force (psi): ");
  Serial.print(vout3p, 3);
  Serial.println("");
  //  delay(100);

  ffsdata4 = analogRead(ffs4) - ffs4_offset;
  ; vout4 = ffsdata4 ;
  vout4p = vout4 * cfp * psi ;
  vout4f = vout4 * cfp;
  Serial.print("Jaw 4 Force (psi): ");
  Serial.print(vout4p, 3);
  Serial.println("");
  //  delay(100);

  //  Total_Radial_Force = (vout1f + vout2f + vout3f + vout4f);
  //  Serial.print(" Total Radial Force (psi): ");
  //  Serial.println(Total_Radial_Force, 3);
  ////  delay(100);

  Total_Radial_Pressure = (vout1p + vout2p + vout3p + vout4p);
  Serial.print(" Total Radial Pressure (psi): ");
  Serial.println(Total_Radial_Pressure, 3);
  //delay(100);


  //Printing the current total force to the LCD screen for live results w/o need for computer.
  lcd.begin();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pressure:");
  lcd.print(Total_Radial_Pressure, 3);
  lcd.setCursor(0, 1);
  lcd.print("Expansion:");
  lcd.print(horizontal, 3);
  //delay(100);

  logData(vout1p, vout2p, vout3p, vout4p, Total_Radial_Pressure, horizontal);
}



void logData(float vout1p, float vout2p, float vout3p, float vout4p, float Total_Radial_Pressure, float horizontal)
{
  // open the file. note that only one file can be open at a time,
  myFile = SD.open("FrcDisp1.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (myFile) {
    myFile.print("Total Pressure: ");
    myFile.print(Total_Radial_Pressure);
    myFile.print(" | Pressure 1: ");
    myFile.print(vout1p);
    myFile.print(" | Pressure 2: ");
    myFile.print(vout2p);
    myFile.print(" | Pressure 3: ");
    myFile.print(vout3p);
    myFile.print(" | Pressure 4: ");
    myFile.print(vout4p);
    myFile.print(" | Horizontal: ");
    myFile.print(horizontal);
    myFile.println(" ");
    myFile.close();
  }
}
