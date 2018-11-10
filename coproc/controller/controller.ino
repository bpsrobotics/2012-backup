void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(2, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  int len = pulseIn(2, HIGH);
  Serial.println((((double) (len-2263))/4096.0)*360.0);
  delay(1000.0/40.0);
}
