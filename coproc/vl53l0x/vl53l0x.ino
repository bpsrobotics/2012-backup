#include <Adafruit_VL53L0X.h>

int ADDR_1 = 0x30;
int ADDR_2 = 0x31;
int ADDR_3 = 0x32;
int ADDR_4 = 0x33;

int XSHUT_1 = 2;
int XSHUT_2 = 3;
int XSHUT_3 = 4;
int XSHUT_4 = 5;

Adafruit_VL53L0X LIDAR_1 = Adafruit_VL53L0X();
Adafruit_VL53L0X LIDAR_2 = Adafruit_VL53L0X();
Adafruit_VL53L0X LIDAR_3 = Adafruit_VL53L0X();
Adafruit_VL53L0X LIDAR_4 = Adafruit_VL53L0X();

void setup() {
  Serial.begin(9600);

  // put your setup code here, to run once:
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);
  pinMode(XSHUT_4, OUTPUT);

  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  digitalWrite(XSHUT_4, LOW);

  delay(20);

  digitalWrite(XSHUT_1, HIGH);
  delay(20);
  LIDAR_1.begin(ADDR_1);
  delay(20);

  digitalWrite(XSHUT_2, HIGH);
  delay(20);
  LIDAR_2.begin(ADDR_2);
  delay(20);

  digitalWrite(XSHUT_3, HIGH);
  delay(20);
  LIDAR_3.begin(ADDR_3);
  delay(20);

  digitalWrite(XSHUT_4, HIGH);
  delay(20);
  LIDAR_4.begin(ADDR_4);
  delay(20);
}

String json = "";

void loop() {
  String json = "";
  VL53L0X_RangingMeasurementData_t MEASURE_1, MEASURE_2, MEASURE_3, MEASURE_4;
  LIDAR_1.getSingleRangingMeasurement(&MEASURE_1, false);
  LIDAR_2.getSingleRangingMeasurement(&MEASURE_2, false);
  LIDAR_3.getSingleRangingMeasurement(&MEASURE_3, false);
  LIDAR_4.getSingleRangingMeasurement(&MEASURE_4, false);
  if (MEASURE_1.RangeStatus != 4) {
    json += String(MEASURE_1.RangeMilliMeter) + ",";
  }
  else
    json += "-1,";

  if (MEASURE_2.RangeStatus != 4) {
    json += String(MEASURE_2.RangeMilliMeter) + ",";
  }
  else json += "-1,";
  
  if (MEASURE_3.RangeStatus != 4) {
    json += String(MEASURE_3.RangeMilliMeter) + ",";
  }
  else json += "-1,";

  if (MEASURE_4.RangeStatus != 4) {
    json += String(MEASURE_4.RangeMilliMeter);
  }
  else json += "-1";

  json += "\n";
  Serial.print(json);

  delay(round(1000.0 / 50.0));
}
