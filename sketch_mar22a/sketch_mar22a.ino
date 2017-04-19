#include <Adafruit_PWMServoDriver.h>

#include <Servo.h>

#define SERVOMIN  150
#define SERVOMAX  600




Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

struct Point{
  float x, y, z;
  Point()
  {
    
  };
  Point(float _x, float _y, float _z)
  {
    x = _x;
    y = _y;
    z = _z;
  }
};

class Leg{
	
};


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

void reach(Point& point)
{//всё в милиметрах
  float startCoxaX = 31.5;
  float startCoxaY = 70; 
  float femurCoxaoffset = 30; //сдвиг от движка femur до coxa
  float femurLength = 56;
  float tibiaLength = 78;
  float hDist = sqrt( sqr(point.x-startCoxaX) +  sqr(point.y-startCoxaY) );
  logback("Dist to point: ", hDist);
  float primaryAngle = rad2deg(polarAngle(point.x-startCoxaX,point.y-startCoxaY));
  logback("Primary Coxa angle: ", primaryAngle);
  float cAngle = primaryAngle + 55 ; //55 - угол отклонения координат ноги от общей сетки 
  logback("Coxa angle:", cAngle);
  float localDestX = hDist-femurCoxaoffset; 
  float localDestY = point.z -20; //оптимальное расстояние от пола 
  logback("localdestX:", localDestX);
  logback("localdestY:", localDestY);
  //НАЧАЛО ОПИСАНИЯ ПРЯМОЙ
  float A = -2 * localDestX;
  logback("A: ", A);
  float B = -2 * localDestY;
  logback("B: ", B);
  float C = sqr(localDestX) + sqr(localDestY) + sqr(femurLength) - sqr(tibiaLength); 
  logback("C: ", C);
  //КОНЕЦ ОПИСАНИЯ ПРЯМОЙ

  //ОПИСАНИЕ КОЭФФИЦИЕНТОВ КВАДРАТНОГО УРАВНЕНИЯ
  float _A = 1+ sqr(B)/sqr(A);
  float _B = 2*C*B/sqr(A);
  float _C = (sqr(C)/sqr(A)) - sqr(femurLength);
  //ОКОНЧАНИЕ ОПИСАНИЯ КОЭФФИЦИЕНТОВ КВАДРАТНОГО УРАВНЕНИЯ
  float Y1 = (-_B + sqrt(sqr(_B) - 4*_A*_C)) / (2*_A); 
  float Y2 = (-_B - sqrt(sqr(_B) - 4*_A*_C)) / (2*_A); 
  float jointLocalY = (Y1 > Y2) ? Y1 : Y2;
  float jointLocalX = ((-C-B*jointLocalY) / A); //ПОЛУЧЕНА ВЕРХНЯЯ ТОЧКА ПЕРЕСЕЧЕНИЯ ПРЯМОЙ И ОКРУЖНОСТИ

  logback("JointLocalX: ", jointLocalX);
  logback("JointLocalY: ", jointLocalY);

  
  float femurPrimaryAngle = rad2deg(polarAngle(jointLocalX, jointLocalY));
  logback("Primary Femur Angle: ", femurPrimaryAngle);
  float fAngle = femurPrimaryAngle + 45;
  logback("Femur Angle: ", fAngle);
  float primaryTibiaAngle = rad2deg(polarAngle(localDestX - jointLocalX, localDestY - jointLocalY));
  logback("Primary tibia Angle: ", primaryTibiaAngle);
  float tAngle = 5+femurPrimaryAngle-(primaryTibiaAngle) ; //Отклонение от оси X считается как отклонение от оси Tibia + отклонение Tibia от X
  logback("Tibia Angle: ", tAngle);

 setAngle(0,cAngle);

  setAngle(1,fAngle);
 
  setAngle(2,tAngle);
  
}


void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  yield();
  //test.attach(10);
 // testMove(50,170,0);
 //Serial.println("First point: ");
   //testMove(70,150,0);
     //Serial.println("Second point: ");
   //testMove(100,150,0);  
  // Serial.println("Third point: ");
   // testMove(100,180,0);
     //Serial.println("4 point: ");
   // testMove(70,180,0);
  //delay(2000);
}

void loop() {
  /*Serial.println("First point: ");
   testMove(70,150,5);
   delay(2000);
     Serial.println("Second point: ");
   testMove(100,150,5);  
   delay(2000);
   Serial.println("Third point: ");
    testMove(100,180,5);
    delay(2000);
     Serial.println("4 point: ");
    testMove(70,180,5);
    delay(2000);*/

  

}
