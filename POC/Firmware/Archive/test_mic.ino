const int MIC_PIN = 4;
void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println(analogRead(MIC_PIN));
  delay(20);
}