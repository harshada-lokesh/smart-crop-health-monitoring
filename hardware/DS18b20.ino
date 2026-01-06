#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire for the soil temperature is plugged into digital pin 2
#define SOIL_TEMP_PIN 2

// Setup OneWire instance for soil temperature sensor
OneWire oneWireSoil(SOIL_TEMP_PIN);

// Pass OneWire reference to DallasTemperature library
DallasTemperature soilSensors(&oneWireSoil);

void setup(void)
{
  soilSensors.begin();  // Start up the soil temperature sensor
  Serial.begin(9600);
}

void loop(void)
{ 
  // Send the command to get temperature from the soil sensor
  soilSensors.requestTemperatures();
  
  // Get the soil temperature in Celsius
  float soilTemperatureC = soilSensors.getTempCByIndex(0);
  
  // Print soil temperature in Celsius and Fahrenheit
  Serial.print("Soil Temperature: ");
  Serial.print(soilTemperatureC);
  Serial.print((char)176); // shows degrees character
  Serial.print("C  |  ");
  Serial.print((soilTemperatureC * 9.0) / 5.0 + 32.0);
  Serial.print((char)176); // shows degrees character
  Serial.println("F");

  // Add conditions for soil temperature
  if (soilTemperatureC < 15.0) {
    Serial.println("Soil temperature is too low! Consider adding a heating system.");
  } 
  else if (soilTemperatureC > 30.0) {
    Serial.println("Soil temperature is too high! Consider adding a cooling system.");
  } 
  else {
    Serial.println("Soil temperature is within optimal range.");
  }

  delay(500);
}
