#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <Adafruit_GFX.h>     // Library for Display
#include <Adafruit_SSD1306.h> // Library for Display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino nano every and BLE 33: A4(SDA), A5(SCL)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



BME280I2C bme;

#define SD_card_input_pin 7

File file;

static const int RXPin = 3, TXPin = 2;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;


//GPS
double gpsSpeed;
int gpsYear;
int gpsMonth;
int gpsDay;
int gpsHour;
int gpsMinute;
int gpsSecond;
double gpsLong;
double gpsLat;


//BME280
double temperature;
double humi;
double pressure;
double airDensity;
double p;
double pv;
double pd;

//Wind
float airVelocity;
const int OutPin = A0;
int windADunits;


//Final Calculation
double PKE = 0;
double PPE = 0;
double PWB;
double PRR;
double PTOT = 250;

double totalMass = 85;
double CRR = .00249;
double g = 9.81;
double driveChainEfficiency = .97;
double Coefficient;


// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup(){
  Serial.begin(9600);
  ss.begin(GPSBaud);

  
  //SD Setup
  Serial.println("Initializing SD Card...");

  if(!SD.begin(SD_card_input_pin)) {
   Serial.println("Initializing SD Failed...");
    return;
  }

  Serial.println("Initialization successful!");

  Serial.println("Creating File...");

  //check if the "testlog.csv" exists. If it does not then first use SD.open command then println header row
  //if it does just use SD.open
if(!SD.exists("testlog.csv")) {
  file=SD.open("testlog.csv", FILE_WRITE);

  //Header
  file.print("Date");
  file.print(" and ");
  file.print("Time");
  file.print(",");
  file.print("Speed (mph)");
  file.print(",");
  file.print("Speed (m/s)");
  file.print(",");
  file.print("Lattitude");
  file.print(",");
  file.print("Longitude");
  file.print(",");
  file.print("Air Density");
  file.print(",");
  file.print("Temp (Kelvin)");
  file.print(",");
  file.print("Humidity");
  file.print(",");
  file.print("Pressure (Pa)");
  file.print(",");
  file.print("Air Velocity (mph)");
  file.print(",");
  file.print("Air Velocity (m/s)");
  file.print(",");
  file.print("WindADunits");
  file.print(",");
  file.print("Volts");
  file.print(",");
  file.print("PRR");
  file.print(",");
  file.print("PWB");
  file.print(",");
  file.print("PTOT");
  file.print(",");
  file.print("CdA + Fw");
  file.println(",");

  file.close();
}
else {
  file=SD.open("testlog.csv", FILE_WRITE);
  file.close();
}

  
  

  Serial.println("File Created!");




  //BME280 Setup
  while(!Serial) {} // Wait

  Wire.begin();

  while(!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }


  //Display Setup
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.display();
  display.clearDisplay(); 
}
  


void loop(){
  
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){

      // Recard GPS values
      gpsSpeed = gps.speed.kmph() * .2778; //In m/s
      gpsYear = gps.date.year();
      gpsMonth = gps.date.month();
      gpsDay = gps.date.day();

      //Set date accounting for gps Day being in GMT (7 hours ahead)
      if(gps.time.hour() < 7) { 
        gpsDay--;
      }

       //Set time accounting for gps Day being in GMT (7 hours ahead)
       if(gps.time.hour() < 7) { 
        gpsHour = (24 + gps.time.hour()) - 7;
       }
       else {
        gpsHour = gps.time.hour() - 7;
       }
      
      gpsMinute = gps.time.minute();
      gpsSecond = gps.time.second();
      gpsLong = gps.location.lng();
      gpsLat = gps.location.lat();

      //Record Air Velocity
      windADunits = analogRead(OutPin);
      double Volts = (windADunits * 5.00)/ 1023.00;
      airVelocity = pow((((Volts - 1.3692) / (3.038517 * pow((temperature - 273), .115157)))) / .087288, 3.009364) * 0.44704; //In m/s

      //Calculate Final Number
      PRR = gpsSpeed * CRR * totalMass * g;
      PWB = gpsSpeed * (.091 + (.0087 * gpsSpeed));
      Coefficient = ((driveChainEfficiency * PTOT) - PKE - PRR - PWB - PPE) / (.5 * (pow(airVelocity, 2)) * gpsSpeed);
 
      // Write to SD Card
      if (SD.exists("testlog.csv")) {
        Serial.println("File Exists!");

        file = SD.open("testlog.csv", FILE_WRITE);
        
        if (file) {
          Serial.println("File Can Be Written To!!");

      file.print(gpsYear);
      file.print("-");
      file.print(gpsMonth); // Raw date in DDMMYY format (u32)
      file.print("-");
      file.print(gpsDay);
      file.print(" ");
      file.print(gpsHour);
      file.print(":");
      file.print(gpsMinute);
      file.print(":");
      file.print(gpsSecond);
      file.print(",");
      file.print(gpsSpeed * 2.23694);
      file.print(",");
      file.print(gpsSpeed);
      file.print(",");
      file.print(gpsLat);
      file.print(",");
      file.print(gpsLong);
      file.print(",");

      //Calling BME280 Method
      printBME280Data(&Serial);
      file.print(airDensity);
      file.print(",");
      file.print(temperature);
      file.print(",");
      file.print(humi);
      file.print(",");
      file.print(pressure);
      file.print(",");


      //Air Velocity
      file.print(airVelocity / 0.44704);
      file.print(",");
      file.print(airVelocity);
      file.print(",");
      file.print(windADunits);
      file.print(",");
      file.print(Volts);
      file.print(",");

      //Final Calculation
      file.print(PRR);
      file.print(",");
      file.print(PWB);
      file.print(",");
      file.print(PTOT);
      file.print(",");
      file.println(Coefficient);


      //Display
      display.clearDisplay();
      display.setTextSize(3);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.println(Coefficient);
      display.setTextSize(2);
      display.setCursor(0, 30);
      display.print(airVelocity / 0.44704);
      display.setCursor(0, 45);
      display.print(gps.speed.kmph() * 0.621371);
      display.display();
      
          file.close();
        } else {
          Serial.println("File Doesn't Exist!");
        }
      }

      // And write to Serial Port to make sure it is working
      Serial.print(gpsYear);
      Serial.print("-");
      Serial.print(gpsMonth); // Raw date in DDMMYY format (u32)
      Serial.print("-");
      Serial.print(gpsDay);
      Serial.print(" ");
      Serial.print(gpsHour);
      Serial.print(":");
      Serial.print(gpsMinute);
      Serial.print(":");
      Serial.print(gpsSecond);
      Serial.print(",");
      Serial.print(gpsSpeed);
      Serial.print(",");
      Serial.print(gpsLat);
      Serial.print(",");
      Serial.println(gpsLong);
      printBME280Data(&Serial);
      Serial.print(temperature);
      Serial.print(" ");
      Serial.print(humi);
      Serial.print(" ");
      Serial.println(pressure);
      Serial.print(" ");
      Serial.print(airDensity);
      Serial.print(" ");
      Serial.print(airVelocity);
      Serial.print(" ");
      Serial.print(Coefficient);

      

      
      
    }
  }
}


//BME280 Method
void printBME280Data
(
   Stream* client
)
{
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

   pressure = pres;
   temperature = temp + 273;
   humi = hum;

  p = 6.1078 * (pow(10, ((7.5 * temperature)/(temperature + 237.3))));
  pv = p * (humi/100);
  pd = pressure - pv;

  airDensity = (pd/(287.058 * temperature)) + (pv/(461.495 * temperature));
   

   
}
