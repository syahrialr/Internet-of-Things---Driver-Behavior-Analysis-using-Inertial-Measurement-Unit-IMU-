#include <Wire.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <LiquidCrystal_PCF8574.h>

LiquidCrystal_PCF8574 lcd(0x3F); 

const char* ssid = "Your SSID";
const char* password = "Your Password";
const char* mqtt_server = "172.20.10.6";
 
// I2C address of the MPU-6050 - 0x68 or 0x69 if AD0 is pulled HIGH
float elapsedTime, currentTime, previousTime;
const int MPU = 0x68;
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float gForceX, gForceY, gForceZ, rotX, rotY, rotZ;
 
WiFiClient espClient;
PubSubClient mqttClient(espClient);
 
void dataReceiver(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  processData();
}
 
void processData(){
  gForceX = (AcX / 16384.0) - 0.15;
  gForceY = AcY / 16384.0; 
  gForceZ = (AcZ / 16384.0) - 0.98;
  
  rotX = (GyX / 131.0) - (-3.0);
  rotY = GyY / 131.0; 
  rotZ = GyZ / 131.0;
}
 
void debugFunction(int16_t AcX, int16_t AcY, int16_t AcZ, int16_t GyX, int16_t GyY, int16_t GyZ){
  // Print the MPU values to the serial monitor
  Serial.print("Accelerometer: ");
  Serial.print("X="); Serial.print(gForceX);
  Serial.print("|Y="); Serial.print(gForceY);
  Serial.print("|Z="); Serial.println(gForceZ);  
  Serial.print("Gyroscope:");
  Serial.print("X="); Serial.print(rotX);
  Serial.print("|Y="); Serial.print(rotY);
  Serial.print("|Z="); Serial.println(rotZ);
}
 
void reconnect() {
  int cursorPosition = 0;
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    lcd.setCursor(00,00);
    lcd.print("Connecting MQTT");
    lcd.setCursor(00,1);
    lcd.print("---Reconnect--- ");
    // Attempt to connect
    if (mqttClient.connect("Client-MQTT-IoT")){
      lcd.clear();
      Serial.println("connected");
      lcd.setCursor(00,00);
      lcd.print("Connecting MQTT");
      lcd.setCursor(00,1);
      lcd.print("---Connected---");
      delay(2000);
      lcd.clear();
    } 
    else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      lcd.setCursor(00,00);
      lcd.print("Connecting MQTT");
      lcd.setCursor(00,1);
      lcd.print("--------------- ");
      //Wait 5 seconds before retrying
      delay(1000);
    }
  }
}
 
void setup(){
  Serial.begin(115200);
  lcd.setBacklight(HIGH);
  lcd.begin(16,2);
  lcd.clear();
  lcd.print("----MPU6050----");
  lcd.setCursor(00,1);
  lcd.print("-IoT -- Project-");
  delay(5000);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  setup_wifi();
 
  mqttClient.setServer(mqtt_server, 1883);
    
  Serial.println(mqtt_server);
  lcd.clear();
}
 
char* init(float val){
  
  char buff[100];
 
  for (int i = 0; i < 100; i++) {
      dtostrf(val, 4, 2, buff);  //4 is mininum width, 6 is precision
  }
   return buff;
 
}
 
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  int cursorPosition = 0;
  Serial.print("Connecting to ");
  Serial.println(ssid);
  lcd.setCursor(00,00);
  lcd.print("Connecting to");
  lcd.setCursor(00,1);
  lcd.print(ssid);
  lcd.clear();
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(cursorPosition,2);
    lcd.print(".");
    cursorPosition++;
  }
  lcd.clear();
  Serial.println("");
  Serial.println("WiFi connected");
  lcd.setCursor(00,00);
  lcd.print("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  lcd.setCursor(00,1);
  lcd.print(WiFi.localIP());
  delay(4000);
  lcd.clear();
}
 
void dataAcc(){
 
  char mpu6050X[100]= "";   
  strcat(mpu6050X,init(gForceX));
 
  char mpu6050Y[100]= "";   
  strcat(mpu6050Y,init(gForceY));
 
  char mpu6050Z[100]= "";   
  strcat(mpu6050Z,init(gForceZ));
 
  // accelerometer - "topic, mpu6050"
  mqttClient.publish("AccX", mpu6050X);
  mqttClient.publish("AccY", mpu6050Y);
  mqttClient.publish("AccZ", mpu6050Z);

  lcd.setCursor(00,00);
  lcd.print(gForceX);
  lcd.setCursor(5,00);
  lcd.print(gForceY);
  lcd.setCursor(10,00);
  lcd.print(gForceZ);
}
 
 
void dataGy(){
 
  char mpu6050X[100]= "";
  strcat(mpu6050X,init(rotX));
 
  char mpu6050Y[100]= "";
  strcat(mpu6050Y,init(rotY));
 
  char mpu6050Z[100]= "";
  strcat(mpu6050Z,init(rotZ));
  
  // gyroscope - "topic, mpu6050"
  mqttClient.publish("GyrX", mpu6050X);
  mqttClient.publish("GyrY", mpu6050Y);
  mqttClient.publish("GyrZ", mpu6050Z);

  lcd.setCursor(00,1);
  lcd.print(rotX);
  lcd.setCursor(5,1);
  lcd.print(rotY);
  lcd.setCursor(10,1);
  lcd.print(rotZ);
}
 
void loop(){
  dataReceiver();
  debugFunction(AcX,AcY,AcZ,GyX,GyY,GyZ);
 
  if (!mqttClient.connected()) {
    reconnect();
  }
  
 
  mqttClient.loop(); 
 
  dataAcc();
  dataGy();

  lcd.write((byte)00);
 
  delay(1000);
}
