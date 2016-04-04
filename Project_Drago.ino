



/*
 * Project_Drago
 * V = at + V(0)
 */

#include <SimbleeForMobile.h>
#include <SimbleeBLE.h>
#include <SPI.h> // SPI library included for SparkFunLSM9DS1
#include <Wire.h> // I2C library included for SparkFunLSM9DS1
#include <SparkFunLSM9DS1.h> // SparkFun LSM9DS1 library
LSM9DS1 imu;
#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
int pinSDA = 15;
//UI Elements
int conversion = 1;
uint8_t conversionSwitch, unitText, axDisplay,ayDisplay,azDisplay,enterWeight,setStart;
String currUnit = "LB";
uint16_t averagePower = 100;
uint16_t peakPower = 100;
uint16_t peakAcceleration = 100;
uint16_t reps = 100;
float g = 9.8;
float accSumZ,accSumX,accSumY,Vx,Vz,Vy,t,VzAvg,VzSum,Vxi,Vxf,Vyi,Vyf,Vzi,Vzf,Vpeak;
//int[] velocity;
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.


/*
 * Traditional Arduino setup routine
 *
 * Initialize the SimbleeForMobile environment.
 */
void setup() {
Serial.begin(115200);
imu.settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C
imu.settings.device.mAddress = LSM9DS1_M; // Set mag address to 0x1E
imu.settings.device.agAddress = LSM9DS1_AG; // Set ag address to 0x6B
if (!imu.begin())
{
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Looping to infinity.");
    while (1)
      ;
}
  SimbleeForMobile.deviceName = "Project Drago";
  SimbleeForMobile.advertisementData = "PD";
  SimbleeForMobile.domain = "template.simblee.com";
  // Begin Simblee UI
  SimbleeForMobile.begin();

}





/*
 * The traditional Arduino loop method
 *
 * Enable SimbleeForMobile functionality by calling the process method
 * each time through the loop. This method must be called regularly for
 * functionality to work.
 */
void loop() {
  // process must be called in the loop for Simblee UI
  SimbleeForMobile.process();
  printMag();   // Print "M: mx, my, mz"
  printGyro();  // Print "G: gx, gy, gz"
  printAccel(); // Print "A: ax, ay, az"
  t = millis()/1000;
  Vx = imu.calcAccel(imu.ax)*g*t + Vxf;
  Serial.print("Vx: ");
  Serial.print(Vx);
  accSumY += imu.calcAccel(imu.ay)*g*t;
  Vy = accSumY/t;
  Serial.print("  Vy: ");
  Serial.print(Vy);
  accSumZ += imu.calcAccel(imu.az)*g;
  Vz = accSumZ/t;
  Serial.print("  Vz: ");
  Serial.println(Vz);
  Serial.print("Time: ");
  Serial.println(t);
  // Print the heading and orientation for fun!
  // Call print attitude. The LSM9DS1's magnetometer x and y
  // axes are opposite to the accelerometer, so my and mx are
  // substituted for each other.
  printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
  Serial.println();
  if (SimbleeForMobile.updatable){
     imu.readAccel();
    SimbleeForMobile.updateValue(azDisplay, imu.calcAccel(imu.az)*980);
  }
  delay(PRINT_SPEED);

}
/*
 * SimbleeForMobile UI callback requesting the user interface
 */
void ui()
{

  SimbleeForMobile.beginScreen(GRAY);
  SimbleeForMobile.drawText(110,40,"Project Drago",BLACK);
  enterWeight = SimbleeForMobile.drawTextField(50,95,120,"","Enter weight");
  unitText = SimbleeForMobile.drawText(175, 100, "LB", WHITE);
  conversionSwitch= SimbleeForMobile.drawSwitch(220,95,BLACK);
  SimbleeForMobile.drawText(50,135,"Power Generated:",WHITE);
  SimbleeForMobile.drawText(50,160,"Reps:",WHITE);
  SimbleeForMobile.drawText(100,160,reps,BLACK);
  SimbleeForMobile.drawText(200,135,averagePower,BLACK);
  SimbleeForMobile.drawText(50, 248, "Az:", WHITE);
  azDisplay = SimbleeForMobile.drawText(225,248,"0", BLACK);
  SimbleeForMobile.drawText(50,280, "Peak Velocity:", WHITE);
  SimbleeForMobile.drawText(200,280, Vz ,BLACK);
  SimbleeForMobile.drawText(200,550,"Created by Kevin Krieg",BLACK,10);
  SimbleeForMobile.endScreen();
  setStart = SimbleeForMobile.drawButton(110,400,100,"Start Set",BLACK);
}

void ui_event(event_t &event) {
  if (event.id == conversionSwitch)
    kgPounds();
  if (event.id == setStart)
    startSet();
}

void startSet() {
    SimbleeForMobile.updateText(setStart,"Waiting...");
    SimbleeForMobile.updateColor(setStart,YELLOW);
    delay(5000);
    SimbleeForMobile.updateColor(setStart,GREEN);
    SimbleeForMobile.updateText(setStart,"Working...");
    delay(3000);
    measureRep();
    SimbleeForMobile.updateColor(setStart,BLACK);
    SimbleeForMobile.updateText(setStart,"Start Set");
    //updateResults();
 }

void kgPounds() {

  conversion++;
  if (conversion % 2 == 0) {
      currUnit = "KG";
      SimbleeForMobile.updateText(unitText, "KG");

      } else {
      currUnit = "LB";
      SimbleeForMobile.updateText(unitText, "LB");
  }
}

void measureRep() {
  boolean repIsDone = false;
  int numReadings;
  VzAvg,VzSum,numReadings,Vzi,Vpeak= 0;
  do {
  t = millis()/1000;
  Vzf = imu.calcAccel(imu.az)*g*t + Vzi;
  VzSum += Vzf;
  Vzi = Vzf;
  if (Vzf > Vpeak)
  Vpeak = Vzf;
  numReadings++;
  } while(!repIsDone);
  VzAvg = VzSum/numReadings;
  updateResults(VAvg, Vpeak);
}
void updateResults(float VAvg, float Vpeak ){

}
void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  imu.readAccel();

  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}


void printGyro()
{
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  imu.readGyro();

  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

void printMag()
{
  // To read from the magnetometer, you must first call the
  // readMag() function. When this exits, it'll update the
  // mx, my, and mz variables with the most current data.
  imu.readMag();

  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

void printAttitude(
float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}
