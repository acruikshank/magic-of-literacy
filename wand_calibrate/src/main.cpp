#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "ets_sys.h"
#include "FastLED.h"

#define FASTLED_ESP8266_RAW_PIN_ORDER
#define NUM_LEDS 4
#define LED_SIGNAL_PIN 15
#define COLOR_ORDER RGB

#define STREAKS 50

#define DISRUPTPort 50060
#define BUCKETS 100
#define WIFI_TIMEOUT 30000
#define INFO_TIMEOUT 10000
#define DISRUPTIONS 15
#define ACC_RESET 14

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncWebSocketClient *wsClient;

typedef struct Streak {
  CRGB color;
  unsigned long micros;
}

CRGB leds[NUM_LEDS];
Streak streaks[NUM_STREAKS];
int nextStreak = 0;

CRGB wandColor = CRGB(0, 0, 0);

uint8_t adjRainbow[256];

unsigned long updateInterval = 100000;
unsigned long lastUpdate = micros64();

MPU6050 accelgyro;
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll, angle_yaw;
int angle_pitch_buffer, angle_roll_buffer, angle_yaw_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output, angle_yaw_output;

/* Set these to your desired AP credentials. */
const char* ssid     = "anthazoa";         // The SSID (name) of the Wi-Fi network you want to connect to
const char* password = "chaartdev";     // The password of the Wi-Fi network

typedef struct GyroMeasurement {
  float pitch;
  float roll;
  float yaw;
  float theta;
  float amplitude;
} GyroMeasurement;

WiFiUDP Udp;

uint32_t myMac = 0;

void setupLights() {
    // create ranbow
  for (int i=0; i<10; i++) adjRainbow[i] = map(i,0,10,237,255);
  for (int i=10; i<43; i++) adjRainbow[i] = map(i,10,43,0,60);
  for (int i=43; i<85; i++) adjRainbow[i] = map(i,43,85,61,78);
  for (int i=85; i<127; i++) adjRainbow[i] = map(i,85,127,79,88);
  for (int i=127; i<169; i++) adjRainbow[i] = map(i,127,169,89,100);
  for (int i=169; i<211; i++) adjRainbow[i] = map(i,169,211,101,138);
  for (int i=211; i<256; i++) adjRainbow[i] = map(i,211,255,139,255);

  FastLED.addLeds<WS2811, LED_SIGNAL_PIN>(leds, NUM_LEDS);
}

void setupAccelerometer() {
  digitalWrite(ACC_RESET, LOW);
  delay(10);
  digitalWrite(ACC_RESET, HIGH);
  delay(100);

  Wire.begin();

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  delay(100);

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  accelgyro.setFullScaleAccelRange(2);
  accelgyro.setFullScaleGyroRange(2);

  int16_t ax, ay, az, gyro_x, gyro_y, gyro_z;

  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    accelgyro.getMotion6(&ax, &ay, &az, &gyro_x, &gyro_y, &gyro_z);
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset

}

uint32_t computeMacAddress() {
  uint32_t mac = 0;

  // determine macAddress as a 32 bit int
  uint8_t macAddress[6] = {0};

  WiFi.macAddress(&macAddress[0]);
  for (int i=0; i<6; i++) {
    Serial.printf("%d:", macAddress[i]);
  }
  Serial.println();

  for(int i=3; i < 6; i++) {
    mac |= (macAddress[i] << (8*(5-i)));
  }
  Serial.printf("mac: %x\n", mac);
  return mac;
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
 
    Serial.println("Websocket client connection received");
    wsClient = client;
 
  } else if(type == WS_EVT_DISCONNECT){
    Serial.println("Client disconnected");
    wsClient = NULL;
 
  } else if(type == WS_EVT_DATA && len > 0) {
    Serial.printf("data: %d\n", data[0]);
  }
}

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

bool connectToWifi() {
  delay(3000);
  if(!SPIFFS.begin()){
    delay(1000);
    Serial.println("SPIFFS Mount Failed");
    return false;
  }

  WiFi.mode( WIFI_AP );
  IPAddress ip( 1, 2, 3, 4 );
  IPAddress gateway( 1, 2, 3, 1 );
  IPAddress subnet( 255, 255, 255, 0 );
  Serial.print("Setting soft-AP configuration ... ");
  Serial.println(WiFi.softAPConfig(ip, gateway, subnet) ? "Ready" : "Failed!");

  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAP("wand", "hermione") ? "Ready" : "Failed!");

  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());
 
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.onNotFound(notFound);

  server.begin();
  return true;
}

void setup() {

  Serial.begin(115200);

  setupLights();

  pinMode(ACC_RESET, OUTPUT);
  digitalWrite(ACC_RESET, HIGH);

  // connect to wifi
  if (!connectToWifi()) {
    Serial.println("WIFI connection failed");
    return;
  }
  Serial.println("WIFI connected");

  setupAccelerometer();
  myMac = computeMacAddress();
}

void escSeq(char *buf, char *seq) {
  sprintf(buf, "%c[%s", char(27), seq);
}

float dist(float x, float y) {
  return sqrt(x*x + y*y);
}

void loop() {
  if (!accelgyro.testConnection()) {
    setupAccelerometer();
  }

  int16_t acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
  accelgyro.getMotion6(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);
  
  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  float restore = .98;
  angle_pitch = restore*(angle_pitch + gyro_x * 0.0000611);                                   //Calculate the traveled pitch angle and add this to the angle_pitch vari)able
  angle_roll = restore*(angle_roll + gyro_y * 0.0000611);                                    //Calculate the traveled roll angle and add this to the angle_roll variabl)e
  angle_yaw = restore*(angle_yaw + gyro_z * 0.0000611);
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  // angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  // angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  // acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  // angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  // angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  // angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  // angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  // if(set_gyro_angles){                                                 //If the IMU is already started
  //   angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.00004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
  //   angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.00004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  // }
  // else{                                                                //At first start
  //   angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
  //   angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
  //   set_gyro_angles = true;                                            //Set the IMU started flag
  // }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.8 + angle_pitch * 0.2;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.8 + angle_roll * 0.2;      //Take 90% of the output roll value and add 10% of the raw roll value
  angle_yaw_output = angle_yaw_output * 0.8 + angle_yaw * 0.2;      //Take 90% of the output ywa value and add 10% of the raw yaw value

  float theta = atan2(angle_yaw_output, angle_roll_output);
  float amp = dist(angle_roll_output, angle_yaw_output);

  int hue = adjRainbow[constrain(map((theta+PI)*10000l, 0, 62831, 0, 255), 0, 255)];
  int v = map((long) amp, 0, 20, 0, 200);
  CRGB color = CHSV(hue, 200, v);

  leds[0] = color;
  leds[1] = color;
  leds[2] = color;
  leds[3] = color;
  FastLED.show();

  if (lastUpdate < micros64() - updateInterval) {
    GyroMeasurement measurment;
    measurment.pitch = angle_pitch_output;
    measurment.roll = angle_roll_output;
    measurment.yaw = angle_yaw_output;
    measurment.theta = (float) constrain(map((theta+PI)*10000l, 0, 62831, 0, 255), 0, 255);
    measurment.amplitude = (float) constrain(map((long) amp, 0, 20, 0, 200), 0, 255);

    if (wsClient != NULL) {
      wsClient->binary((uint8_t *)&measurment, sizeof(GyroMeasurement));
    }
  }
  
  delay(4);
}