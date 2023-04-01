//写一个Arduino函数，实现非阻塞的按键消抖。
//这个函数的参数是：按键的引脚号。
//这个函数的原理是：当按键按下时，引脚电平会变为低电平，这时，函数会返回1，否则返回0。
int button(int pin)
{
  static int state = 0;
  static unsigned long last_time = 0;
  unsigned long now = millis();
  //如果按键按下，引脚电平变为低电平，否则为高电平。
  int reading = digitalRead(pin);
  if (reading == LOW) {
    if (state == 0) {
      //如果是刚按下，记录下按下的时间。
      last_time = now;
      state = 1;
    } else if (now - last_time >= 50) {
      //如果按键一直按下，且已经超过50ms，才返回1，否则返回0。
      state = 2;
      return 1;
    }
  } else {
    //如果按键松开，状态复位。
    state = 0;
  }
  return 0;
}
// Path: TEMP\Button-v1\Button-v1.ino
void setup()
{
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, INPUT_PULLUP);
}
// Path: TEMP\Button-v1\Button-v1.ino
void loop()
{
  //如果按键按下，就点亮LED。
  if (button(9)) {
    digitalWrite(13, HIGH);
    digitalWrite(12, HIGH);
    digitalWrite(11, HIGH);
    digitalWrite(10, HIGH);
  } else {
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    digitalWrite(10, LOW);
  }
}
