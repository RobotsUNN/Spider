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

void setAngle(int servo,float angle){
  uint16_t pulselength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo,0,pulselength);
}

void logback(char* s,float num)
{
  Serial.println(s);
  Serial.println(num);
}

void testMove(int x, int y,int z)
{//всё в милиметрах
  float startCoxaX = 31.5;
  float startCoxaY = 70; 
  float femurCoxaoffset = 30; //сдвиг от движка femur до coxa
  float femurLength = 56;
  float tibiaLength = 79;
  float hDist = sqrt( sqr(x-startCoxaX) +  sqr(y-startCoxaY) );
  logback("Dist to point: ", hDist);
  float primaryAngle = rad2deg(polarAngle(x-startCoxaX,y-startCoxaY));
  logback("Primary Coxa angle: ", primaryAngle);
  float cAngle = primaryAngle + 55 ; //55 - угол отклонения координат ноги от общей сетки 5 - погрешность
  logback("Coxa angle:", cAngle);
  float localDestX = hDist-femurCoxaoffset; 
  float localDestY = z -20; //оптимальное расстояние от пола 
  logback("localdestX:", localDestX);
  logback("localdestY:", localDestY);
  //НАЧАЛО ОПИСАНИЯ ПРЯМОЙ
  float A = -2 * localDestX;
  float B = -2 * localDestY;
  float C = sqr(localDestX) + sqr(localDestY) + sqr(femurLength) - sqr(tibiaLength); 
  //КОНЕЦ ОПИСАНИЯ ПРЯМОЙ
/*
  //ОПИСАНИЕ КОЭФФИЦИЕНТОВ КВАДРАТНОГО УРАВНЕНИЯ
  float _A = 1+ sqr(B)/A;
  float _B = 2*C*B/A;
  float _C = (sqr(C)/A) - sqr(femurLength);
  //ОКОНЧАНИЕ ОПИСАНИЯ КОЭФФИЦИЕНТОВ КВАДРАТНОГО УРАВНЕНИЯ
  float Y1 = (-_B + sqrt(sqr(_B) - 4*_A*_C)) / (2*_A); 
  float Y2 = (-_B - sqrt(sqr(_B) - 4*_A*_C)) / (2*_A); 
  float jointLocalY = (Y1 > Y2) ? Y1 : Y2;
  float jointLocalX = ((-C-B*jointLocalY) / A); //ПОЛУЧЕНА ВЕРХНЯЯ ТОЧКА ПЕРЕСЕЧЕНИЯ ПРЯМОЙ И ОКРУЖНОСТИ

  logback("JointLocalX: ", jointLocalX);
  logback("JointLocalY: ", jointLocalY);

  
  float femurPrimaryAngle = polarAngle(jointLocalX, jointLocalY);
  logback("Primary Femur Angle: ", femurPrimaryAngle);
  float fAngle = femurPrimaryAngle + 45;
  logback("Femur Angle: ", fAngle);
  float primaryTibiaAngle = polarAngle(localDestX - jointLocalX, localDestY - jointLocalY);
  logback("Primary tibia Angle: ", primaryTibiaAngle);
  float tAngle = 45 -(primaryTibiaAngle - fAngle);
  logback("Tibia Angle: ", tAngle);
*/
 // setAngle(0,cAngle);
 // setAngle(1,fAngle);
 // setAngle(2,tAngle);
  
}


void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  yield();
  //test.attach(10);
  
}

void loop() {

  testMove(130,110,0);
  //setAngle(1,43);
  //delay(5000);
  //setAngle(0,testMove(70,200,0));
  //delay(5000);
 //   setAngle(0,testMove(130,110,0));
 // delay(5000);
  //  setAngle(0,testMove(90,70,0));
 // delay(5000);
   // setAngle(0,testMove(80,25,0));
 // delay(5000);
 // test.write(90-testMove(70,70));
  //delay(3000);
  

}
