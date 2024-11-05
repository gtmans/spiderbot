I am updating an old "Nano Spiderbot" project and added an Oled display and IR remote. 

IR receiver module on A0
SSD1306 on A4 and A5
12 Servo's on D1-D13
libs Adafruit_SSD1306.h, servo.h and TinyIRReceiver.hpp
rewritten the entire code to not use Flexitimer2 which caused a lot of problems with SSD1306 and IR receiver
