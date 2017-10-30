
// BME280 MOD-1022 weather multi-sensor Arduino demo
// Written originally by Embedded Adventures

#include <BME280_MOD-1022.h>
#include <Wire.h>


float mTemp, mHumidity,  mPressure;
//
void printFormattedFloat(float x, uint8_t precision) {
  char buffer[10];
  dtostrf(x, 7, precision, buffer);
  Serial.print(buffer);
}
//
int BME280_getTemperature(){ return mTemp;  }
int BME280_getHumidity(){ return mHumidity;  }
int BME280_getPressure(){ return mPressure;  }
//
void print_bme280(void) {
  float pressureMoreAccurate;
  // double tempMostAccurate, humidityMostAccurate, pressureMostAccurate;
  mTemp      = BME280.getTemperature();
  mHumidity  = BME280.getHumidity();
  mPressure  = BME280.getPressure();
  pressureMoreAccurate = BME280.getPressureMoreAccurate();  // t_fine already calculated from getTemperaure() above  
}

// print out the measurements
void printCompensatedMeasurements(void) {
float temp, humidity,  pressure, pressureMoreAccurate;
double tempMostAccurate, humidityMostAccurate, pressureMostAccurate;
char buffer[80];

  temp      = BME280.getTemperature();
  humidity  = BME280.getHumidity();
  pressure  = BME280.getPressure();
  
  pressureMoreAccurate = BME280.getPressureMoreAccurate();  // t_fine already calculated from getTemperaure() above
  
  tempMostAccurate     = BME280.getTemperatureMostAccurate();
  humidityMostAccurate = BME280.getHumidityMostAccurate();
  pressureMostAccurate = BME280.getPressureMostAccurate();
  Serial.println("                Good  Better    Best");
  Serial.print("Temperature  ");
  printFormattedFloat(temp, 2);
  Serial.print("         ");
  printFormattedFloat(tempMostAccurate, 2);
  Serial.println();
  
  Serial.print("Humidity     ");
  printFormattedFloat(humidity, 2);
  Serial.print("         ");
  printFormattedFloat(humidityMostAccurate, 2);
  Serial.println();

  Serial.print("Pressure     ");
  printFormattedFloat(pressure, 2);
  Serial.print(" ");
  printFormattedFloat(pressureMoreAccurate, 2);
  Serial.print(" ");
  printFormattedFloat(pressureMostAccurate, 2);
  Serial.println();
}

//
void init_BME280(){
  uint8_t chipID;
  
  Serial.println("Welcome to the BME280 MOD-1022 weather multi-sensor test sketch!");
  Serial.println("Embedded Adventures (www.embeddedadventures.com)");
  chipID = BME280.readChipId();
  
  // find the chip ID out just for fun
  Serial.print("ChipID = 0x");
  Serial.print(chipID, HEX);
  
 
  // need to read the NVM compensation parameters
  BME280.readCompensationParams();
  
  // Need to turn on 1x oversampling, default is os_skipped, which means it doesn't measure anything
  BME280.writeOversamplingPressure(os1x);  // 1x over sampling (ie, just one sample)
  BME280.writeOversamplingTemperature(os1x);
  BME280.writeOversamplingHumidity(os1x);
  
  // example of a forced sample.  After taking the measurement the chip goes back to sleep
  BME280.writeMode(smForced);
  while (BME280.isMeasuring()) {
    Serial.println("Measuring...");
    delay(50);
  }
  Serial.println("Done!");
  
  // read out the data - must do this before calling the getxxxxx routines
  BME280.readMeasurements();
  Serial.print("Temp=");
  Serial.println(BME280.getTemperature());  // must get temp first
  Serial.print("Humidity=");
  Serial.println(BME280.getHumidity());
  Serial.print("Pressure=");
  Serial.println(BME280.getPressure());
  Serial.print("PressureMoreAccurate=");
  Serial.println(BME280.getPressureMoreAccurate());  // use int64 calculcations
  Serial.print("TempMostAccurate=");
  Serial.println(BME280.getTemperatureMostAccurate());  // use double calculations
  Serial.print("HumidityMostAccurate=");
  Serial.println(BME280.getHumidityMostAccurate()); // use double calculations
  Serial.print("PressureMostAccurate=");
  Serial.println(BME280.getPressureMostAccurate()); // use double calculations
  
  // Example for "indoor navigation"
  // We'll switch into normal mode for regular automatic samples
  
  BME280.writeStandbyTime(tsb_0p5ms);        // tsb = 0.5ms
  BME280.writeFilterCoefficient(fc_16);      // IIR Filter coefficient 16
  BME280.writeOversamplingPressure(os16x);    // pressure x16
  BME280.writeOversamplingTemperature(os2x);  // temperature x2
  BME280.writeOversamplingHumidity(os1x);     // humidity x1
  
  BME280.writeMode(smNormal);
}

// setup wire and serial
/*
void setup()
{
  Wire.begin();
  Serial.begin(9600);
  init_BME280();
}
*/

/*
void loop()
{
  while (BME280.isMeasuring()) {
  }
  BME280.readMeasurements();
  printCompensatedMeasurements();
  delay(5000);  // do this every 5 seconds
  Serial.println();  
}
*/

