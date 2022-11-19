#include <esp_now.h>
#include "WiFi.h"
//#include <ServoInput.h>

/* Notes
2022-10-15 | Add Map Value & Add Serial Output

*/

// Variables for test data
int int_value;
float float_value;
String result = "";

int throttle_read = 0;
int steering_read = 0;
int throttle_shift = 500;
int steering_shift = 0;
int throttle_temp = 0;
int steering_temp = 0;
int throttle_min = 877;
int throttle_max = 3850;
int steering_min = 6;
int steering_max = 4095;
int throttle = 0;
int steering = 0;
//bool recvData.b = true;

// uint8_t broadcastAddress[] = {0x7c, 0xdf, 0xa1, 0xc2, 0x1a, 0x14};
// uint8_t broadcastAddress[] = {0x84, 0xf7, 0x03, 0xa8, 0x53, 0x8c};
uint8_t broadcastAddress[] = { 0x84, 0xF7, 0x03, 0xA9, 0x74, 0xB8 };

typedef struct struct_message {
  int throttle;  // 油门控制
  int steering;  // 方向控制
};

struct_message myData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  // Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

/*
float floatMap(float x, float in_min, float in_max, float out_min, float
		out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
*/

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  /* // from August
  ratio1 = (4095 - 1400) / (3950 - 1900) ;
  ratio2 = (0 - 1400) / (977 - 1900) ;
  ratio = max(ratio1, ratio2)
  shift = 1900 - 1400 * ratio 
  */
  throttle_read = 4096 - analogRead(0) + throttle_shift;
  steering_read = analogRead(1) + steering_shift;

  throttle_read = max(throttle_read, throttle_min);  // Min 877
  throttle_temp = min(throttle_read, throttle_max);  // Max 3850
  steering_read = max(steering_read, steering_min);  // Min 6
  steering_temp = min(steering_read, steering_max);  // Max 4095

  throttle = map(throttle_temp, throttle_min, throttle_max, -100, 100);  // Forward: 100, Backward: 0
  steering = map(steering_temp, steering_min, steering_max, -100, 100);  // Left: 100, Right: 0
                                                                         // float voltage = floatMap(analogValue, 0, 4095, 0, 3.3);
                                                                         // bool_value = !bool_value;
  // strcpy(myData.a, "welcome to the earth");

  myData.throttle = throttle_temp;
  myData.steering = steering_temp;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    //Serial.println("Sending confirmed");
    // result = "";
    // result += "T";
    // result += throttle;  // Range [-100,100]
    // result += "S";
    // result += steering;
    Serial.printf("T%dS%d\n", throttle, steering);


    /*
    Serial.print("throttle: ");
    // Serial.println(throttle_read);
    Serial.println(throttle);
    Serial.print("steering: ");
    // Serial.println(steering_read);
    Serial.println(steering);
    */

  } else {
    Serial.println("Sending error");
  }
  /*
  steering_val_joystick = steering.mapDeadzone(-100, 100, steering_deadzone);
  throttle_val_joystick = throttle.mapDeadzone(-100, 100, throttle_deadzone);

  result = "";
  result += "T";
  result += (throttle_val_joystick);  // Range [-100,100]
  result += "S";
  result += (steering_val_joystick);  // Range [-100,100]
  if (result_temp_TS != result) {
    Serial.println(result);    // Report to Upper Controller
    mySerial.println(result);  // Report to Debugger
    result_temp_TS = result;
  }
  */


  delay(10);
}