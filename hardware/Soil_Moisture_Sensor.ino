int moisture = 0;

void setup() {
  pinMode(A0, INPUT);
  Serial.begin(9600);
}

void loop() {
  moisture = analogRead(A0);
  Serial.print("Soil Moisture Level: ");
  Serial.println(moisture);
  
  if (moisture < 200) {
    Serial.println("Very Dry");
  } else if (moisture < 400) {
    Serial.println("Dry");
  } else if (moisture < 600) {
    Serial.println("Moist");
  } else if (moisture < 800) {
    Serial.println("Wet");
  } else {
    Serial.println("Very Wet");
  }
  
  delay(1000); // Wait for 1 second before reading again
}