#include <esp_now.h>
#include <WiFi.h>

<<<<<<< HEAD
//#include "ESP32_C3_ISR_Servo.h"
=======
#include "ESP32_C3_ISR_Servo.h"
>>>>>>> b788a231a87f010bbed330212c0ca04bb6268f10

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


// Define a data structure
typedef struct struct_message {
  int throttle;
  int steering;
} struct_message;


int servo_throttle  = -1;
int servo_steering  = -1;

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
  int t = myData->throttle;
  int s = myData->steering;

  t = adj(t, 550);
  s = adj(s, 800);
  // t = adj(t, 0);
  // s = adj(s, 0);

  Serial.printf("%d, %d\n", t, s);
 
  ledcWrite(0, t / 2.5);
  ledcWrite(1, s / 2.5);
}

int adj(int v, int s)
{
    v = v + s;
    if(v > 4095) v = 4095;
    if(v < 0) v = 0;

    return v;
}

void setup() {
	Serial.begin(115200);
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
   delay(1000);
}
