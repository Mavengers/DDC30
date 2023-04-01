#include <esp_now.h>
#include <WiFi.h>

#include "ESP32_C3_ISR_Servo.h"

#define CH1_PIN 4  //接收机pwm输出CH1通道为，GPIO4
#define CH2_PIN 5  //接收机pwm输出CH2通道为，GPIO5
#define CH3_PIN 8  //接收机pwm输出CH3通道为，GPIO8
#define CH4_PIN 9  //接收机pwm输出CH4通道为，GPIO9
#define CH5_PIN 2  //接收机pwm输出CH5通道为，GPIO2
#define CH6_PIN 3  //接收机pwm输出CH6通道为，GPIO3

#define THROTTLE_PIN 5
#define STEERING_PIN 7

/*
#define MIN_MICROS      800 
#define MAX_MICROS      2450
*/
#define MIN_MICROS      1050 
#define MAX_MICROS      1950

const int ledPin = 10; // LED connected to GPIO10 on ESP32-C3


// Define a data structure
typedef struct struct_message {
  int throttle;
  int steering;
  bool mode;
} struct_message;


int User_throttle  = 0; //RC遥控器发来的用户油门值
int User_steering  = 0; //RC遥控器发来的用户转向值
int Pilot_throttle  = 0;//上位机发来的油门值
int Pilot_steering  = 0;//上位机发来的转向值
bool Drive_mode  = false;//驾驶模式，false为遥控模式，true为自动驾驶模式


const int PWM_MIN = 819; // 'minimum' pulse length count (out of 4096)
const int PWM_MAX = 1638; // 'maximum' pulse length count (out of 4096)
const int MOTOR_MID = 1229; // 需要实际测试
const int MOTOR_RANGE = 390; // Pulse range for Motor Throttle # change from 60 to 90 
const int SERVO_MID = 1270; //  需要实际测试
const int SERVO_RANGE = 390; // Pulse range for Motor Throttle # change from 60 to 90 
const int MOTOR_OFFSET = 11;
const int SERVO_OFFSET = -11;

// Create a structure object
struct_message* myData;

// callback function executed when data is received.
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
	myData = (struct_message*)incomingData;
/*	
	// Serial.print("Data received: ");
	// Serial.println(len);
  // Serial.printf("throttle: %d, steering: %d\n", myData->throttle, myData->steering);
  // ESP32_ISR_Servos.setPulseWidth(servo_throttle, map(myData->throttle, 0, 4096, MIN_MICROS, MAX_MICROS)); //myData.adc0
  // ESP32_ISR_Servos.setPulseWidth(servo_steering, map(myData->steering, 0, 4096, MIN_MICROS, MAX_MICROS));
*/
  User_throttle = myData->throttle; 
  User_steering = myData->steering;
  Drive_mode = myData->mode;

}

int adj(int v, int s)
{
    v = v + s;
    if(v > 4095) v = 4095;
    if(v < 0) v = 0;

    return v;
}

void setup() {
  pinMode(ledPin, OUTPUT);
	Serial.begin(115200);
  Serial1.begin(115200,SERIAL_8N1,/*rx =*/20,/*Tx =*/21); 
  // Serial.println("hello donkey");
/*
  ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);

  servo_throttle = ESP32_ISR_Servos.setupServo(THROTTLE_PIN, MIN_MICROS, MAX_MICROS);
  servo_steering = ESP32_ISR_Servos.setupServo(STEERING_PIN, MIN_MICROS, MAX_MICROS);
*/

  ledcSetup(0, 50, 14); // 定义通道0（Throttle）PWM输出为pin1，50Hz，14-bit resolution
  ledcSetup(1, 50, 14); // 定义通道1（steering）
	// 分配PWM输出通道到管脚	
  ledcAttachPin(THROTTLE_PIN, 0);
  ledcAttachPin(STEERING_PIN, 1);
  
	WiFi.mode(WIFI_STA); 
  for(int i = 0; i < 10; i++)
    Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  
  if (esp_now_init() != ESP_OK) {
	  Serial.println("Error initializing ESP-NOW");
	  return;
	}

	esp_now_register_recv_cb(OnDataRecv);
}


void loop() {
  //  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  int t = 0;
  int s = 0;

  if (Serial1.available()){
      String CMD = Serial1.readStringUntil('\n');
      String CMD_steering = CMD;
      String CMD_throttle = CMD;

      int CMD_gap = CMD.indexOf(':');
      int CMD_len = CMD.length();

      CMD_steering.remove(CMD_gap,-1);
      CMD_throttle.remove(0,CMD_gap+1);
    
      Pilot_steering = CMD_steering.toInt();
      Pilot_throttle = CMD_throttle.toInt();
    }

  if(Drive_mode) {
    // Controlled by Pilot
    digitalWrite(ledPin, HIGH);
    t = Pilot_throttle;
    s = Pilot_steering;
  } else {
    // Controlled by RC Controller
    digitalWrite(ledPin, LOW);
    t = User_throttle+MOTOR_OFFSET;
    s = User_steering+SERVO_OFFSET;
    Serial1.printf("T%dS%d\n", t, s);
  }

  int t1 = map(t,100,-100,MOTOR_MID-MOTOR_RANGE,MOTOR_MID+MOTOR_RANGE);
  int s1 = map(s,100,-100,SERVO_MID-SERVO_RANGE,SERVO_MID+SERVO_RANGE);

  t1 = min(max(t1,PWM_MIN),PWM_MAX);
  s1 = min(max(s1,PWM_MIN),PWM_MAX);

  Serial.printf("%d, %d\t%d, %d\n",t, t1 , s, s1);
  ledcWrite(0, t1);
  ledcWrite(1, s1);

  delay(10);
}
