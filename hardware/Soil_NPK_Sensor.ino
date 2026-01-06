#include <SoftwareSerial.h>
#include <Wire.h>

#define RE 2
#define DE 3

// Define register addresses as constants for clarity
const uint16_t NITROGEN_REG = 0x001E;
const uint16_t PHOSPHORUS_REG = 0x001F;
const uint16_t POTASSIUM_REG = 0x0020;

SoftwareSerial mod(10, 11); // Changed to pins 10 and 11, common for software serial

void setup() {
  Serial.begin(9600);
  mod.begin(9600);

  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  pinMode(10, INPUT); // Ensure RX pin is configured correctly
  pinMode(11, OUTPUT); // Ensure TX pin is configured correctly

  delay(500);
  Serial.println("Soil NPK Sensor via RS485 Initialized...");
}

void loop() {
  int nitrogen = readModbus(NITROGEN_REG, "Nitrogen");
  delay(500);
  int phosphorus = readModbus(PHOSPHORUS_REG, "Phosphorus");
  delay(500);
  int potassium = readModbus(POTASSIUM_REG, "Potassium");
  delay(500);

  Serial.print("Nitrogen: ");
  Serial.print(nitrogen);
  Serial.println(" mg/kg");

  Serial.print("Phosphorus: ");
  Serial.print(phosphorus);
  Serial.println(" mg/kg");

  Serial.print("Potassium: ");
  Serial.print(potassium);
  Serial.println(" mg/kg");

  delay(2000);
}

int readModbus(uint16_t registerAddress, const char *element) {
  byte request[8];
  byte response[7];  // Expecting 7 bytes in the response

  // Construct the Modbus request (0x01 = slave address, 0x03 = read holding registers)
  request[0] = 0x01;
  request[1] = 0x03;
  request[2] = highByte(registerAddress);
  request[3] = lowByte(registerAddress);
  request[4] = 0x00; // Number of registers to read (1)
  request[5] = 0x01;
  // Calculate CRC (Important!  This was missing in your original code)
  uint16_t crc = calculateCRC(request, 6);
  request[6] = lowByte(crc);
  request[7] = highByte(crc);

  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(10);

  mod.flush();
  Serial.print("Requesting ");
  Serial.print(element);
  Serial.println(" data...");

  mod.write(request, 8);
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  delay(500); // Increased delay significantly

  int bytesRead = 0;
  unsigned long startTime = millis(); // Add timeout
  while (mod.available() > 0 && bytesRead < 7 && (millis() - startTime) < 1000) { //1 second timeout
    response[bytesRead] = mod.read();
    bytesRead++;
    startTime=millis(); //reset timer if byte is received
  }

  Serial.print(element);
  Serial.print(" Data: ");

  if (bytesRead != 7) {
    Serial.print("Failed! (");
    Serial.print(bytesRead);
    Serial.print(" bytes received) ");
    for (int j = 0; j < bytesRead; j++) {
      Serial.print(response[j], HEX);
      Serial.print(" ");
    }
    Serial.println();
    return 255; // Indicate failure
  }

  for (int j = 0; j < 7; j++) {
    Serial.print(response[j], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Combine the two data bytes (bytes 3 and 4) to get the actual value
  int value = word(response[3], response[4]);  // Use word() for combining bytes

  return value;
}

// CRC calculation function (Crucial for Modbus)
uint16_t calculateCRC(byte *data, int length) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 8; j != 0; j--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}