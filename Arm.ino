/***************************************************
  This is an example sketch for our optical Fingerprint sensor

  Designed specifically to work with the Adafruit BMP085 Breakout
  ----> http://www.adafruit.com/products/751

  These displays use TTL Serial to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/


#include <Adafruit_Fingerprint.h>
#include <Servo.h>    // 声明调用Servo.h库
#include<Adafruit_PWMServoDriver.h>//170 0//510  180
#include<Wire.h>
#define on_off 8

Servo myservo;        // 创建一个舵机对象
int pos = 0;          // 变量pos用来存储舵机位置
const int servoDelay = 15;
const int servoStop  = 5000;
const int lowPowerRate = 1;
const int checkDelay = 1000;
const int debug = 0;

Adafruit_PWMServoDriver pwm1=Adafruit_PWMServoDriver(0x40);
void setServoPos(int n, double pos);


#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
// For UNO and others without hardware serial, we must use software serial...
// pin #2 is IN from sensor (GREEN wire)
// pin #3 is OUT from arduino  (WHITE wire)
// Set up the serial port to use softwareserial..
SoftwareSerial mySerial(2, 3);

#else
// On Leonardo/M0/etc, others with hardware serial, use hardware serial!
// #0 is green wire, #1 is white
#define mySerial Serial1

#endif


Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

void setup()
{
  myservo.attach(9);
  myservo.write(0);
  Serial.begin(9600);
  pinMode(on_off,INPUT);
  while (!Serial);  // For Yun/Leo/Micro/Zero/...
  delay(100);
  Serial.println("\n\nAdafruit finger detect test");
  pwm1.begin();
  pwm1.setPWMFreq(50);

  // set the data rate for the sensor serial port
  finger.begin(57600);
  delay(5);
  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor :(");
    while (1) { delay(1); }
  }

  Serial.println(F("Reading sensor parameters"));
  finger.getParameters();
  Serial.print(F("Status: 0x")); Serial.println(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x")); Serial.println(finger.system_id, HEX);
  Serial.print(F("Capacity: ")); Serial.println(finger.capacity);
  Serial.print(F("Security level: ")); Serial.println(finger.security_level);
  Serial.print(F("Device address: ")); Serial.println(finger.device_addr, HEX);
  Serial.print(F("Packet len: ")); Serial.println(finger.packet_len);
  Serial.print(F("Baud rate: ")); Serial.println(finger.baud_rate);

  finger.getTemplateCount();

  if (finger.templateCount == 0) {
    Serial.print("Sensor doesn't contain any fingerprint data. Please run the 'enroll' example.");
  }
  else {
    Serial.println("Waiting for valid finger...");
    Serial.print("Sensor contains "); Serial.print(finger.templateCount); Serial.println(" templates");
  }
}

void loop()                     // run over and over again
{
  Serial.println("Voltage in 4"); Serial.println(digitalRead(4));
  if(digitalRead(on_off) == HIGH){
    openAndReset();
    }
  
  if(!lowPowerRate || digitalRead(4) == HIGH) {
    getFingerprintID();
  }
  delay(checkDelay);            //don't ned to run this at full speed.
}

void openAndReset(){
  for(pos = 0; pos < 75; pos += 1){    // 舵机从0°转到180°，每次增加1°          
    myservo.write(pos);       // 给舵机写入角度   
    delay(servoDelay);                    // 延时15ms让舵机转到指定位置
  }
  Open();
  delay(servoStop);
  Close();
  for(pos = 75; pos>=1; pos-=1) {    // 舵机从180°转回到0°，每次减小1°                               
    myservo.write(pos);        // 写角度到舵机     
    delay(servoDelay);                 // 延时15ms让舵机转到指定位置
  }
}

uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println("No finger detected");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK success!

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("Found a print match!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("Did not find a match");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);

  // fuck U open the door

  openAndReset();

  return finger.fingerID;
}

// returns -1 if failed, otherwise returns ID #
int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  return -1;

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);
  return finger.fingerID;
}

void setServoPos(int n, double pos) {
   pos=170+pos*335/180;
   pwm1.setPWM(n-1, 0, pos);delay(10);
}

void Open(){
  setServoPos(1,15);delay(15);
  setServoPos(2,-45);delay(15);
  delay(2000);
  }

void Close(){
  setServoPos(1,90);delay(15);
  setServoPos(2,45);delay(15);
  delay(2000);
  }

//0.5ms————–0度：0.5/204096 = 102
//1.0ms————45度：1/204096 = 204 * 0.915 = 187
//1.5ms————90度：1.5/204096 = 306 * 0.915 = 280
//2.0ms———–135度：2/204096 = 408 * 0.915 = 373
//2.5ms———–180度：2.5/20*4096 =510
