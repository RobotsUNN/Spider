#include <Adafruit_PWMServoDriver.h>

#include <Servo.h>

#define SERVOMIN  150
#define SERVOMAX  600



Servo test;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
float sqr(float x)
{
  return x*x;
}

float rad2deg(float x)
{
  return ((180.0/PI) * x);
}


float deg2rad(float x)
{
  return ((PI/180.0)*(x));
}
float polarAngle(float x, float y)
{
    if (x > 0)
        return atan(y / x);
    
    if (x < 0 && y >= 0)
    {
        
        return atan(y / x) + PI;
    }
    
    if (x < 0 && y < 0)
        return atan(y / x) - PI;
        
   
    if (y > 0) // x = 0
        return PI / 2;
    
    if (y < 0)
        return PI / -2;
    
    // y = 0
    return 0;
}

float testMove(int x, int y)
{
  float hDist = sqrt( sqr(x) +  sqr(y) );
  float primaryAngle = rad2deg(polarAngle(x,y));
  float addAngle = rad2deg(atan(5.0/hDist));
  Serial.println(primaryAngle);
  Serial.println(addAngle);
  return (primaryAngle + addAngle);
  
}

void setAngle(int servo,float angle){
  uint16_t pulselength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo,0,pulselength);
}
void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  yield();
  //test.attach(10);
  
}

void loop() {

  setAngle(0,90);
  delay(1000);
 // test.write(90-testMove(70,70));
  //delay(3000);
  

}
