#include <Adafruit_PWMServoDriver.h>

#include <Servo.h>

#define SERVOMIN  150
#define SERVOMAX  600




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

void logback(char* s,float num)
{
  Serial.println(s);
  Serial.println(num);
}

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
private:
	Point _cPosition;
	
	Point _legPosition;
	
	float _cMeshOffset;
	float _fMeshOffset;
	float _tMeshOffset;
	
	float femurCoxaOffset;
	
	float _fLength;
	float _tLength;
	
	float _cDirectionServo;
	float _fDirectionServo;
	float _tDirectionServo;
	
	int _cPWMServoPin;
	int _fPWMServoPin;
	int _tPWMServoPin;
	
public:
Leg(int fPin, int sPin, int tPin, int cMeshOffset, int fMeshOffset, int tMeshOffset, Point& cPosition)
{
  _cPWMServoPin = fPin;
  _fPWMServoPin = sPin;
  _tPWMServoPin = tPin;
  _cMeshOffset = cMeshOffset;
  _fMeshOffset = fMeshOffset;
  _tMeshOffset = tMeshOffset;
  _cPosition = cPosition;
}
void setAngle(int servo,float angle){
  uint16_t pulselength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo,0,pulselength);
}

void reach(Point& point)
{//всё в милиметрах
  float startCoxaX = 31.5;
  float startCoxaY = 70; 
  float femurCoxaoffset = 30; //сдвиг от движка femur до coxa
  float femurLength = 56;
  float tibiaLength = 78;
  float hDist = sqrt( sqr(point.x-this->_cPosition.x) +  sqr(point.y-this->_cPosition.y) );
  logback("Dist to point: ", hDist);
  float primaryAngle = rad2deg(polarAngle(point.x-this->_cPosition.x,point.y-this->_cPosition.y));
  logback("X:", point.x-this->_cPosition.x);
  logback("Y:", point.y-this->_cPosition.y);
  logback("Primary Coxa angle: ", primaryAngle);
  float cAngle = primaryAngle + this->_cMeshOffset  ; //55 - угол отклонения координат ноги от общей сетки 
  logback("Coxa angle:", cAngle);
  float localDestX = hDist-femurCoxaoffset; 
  float localDestY = point.z -27; //оптимальное расстояние от пола 
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
  float fAngle = femurPrimaryAngle + this->_fMeshOffset;
  logback("Femur Angle: ", fAngle);
  float primaryTibiaAngle = rad2deg(polarAngle(localDestX - jointLocalX, localDestY - jointLocalY));
  logback("Primary tibia Angle: ", primaryTibiaAngle);
  float tAngle = this->_tMeshOffset+femurPrimaryAngle-(primaryTibiaAngle) ; //Отклонение от оси X считается как отклонение от оси Tibia + отклонение Tibia от X. У Tibia перевернутая сетка.
  logback("Tibia Angle: ", tAngle);

 setAngle(this->_cPWMServoPin,cAngle);

  setAngle(this->_fPWMServoPin,fAngle);
 
  setAngle(this->_tPWMServoPin,tAngle);
  
}

};

void setAngleNC(int servo,float angle){
  uint16_t pulselength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo,0,pulselength);
}
void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  yield();
}

void loop() {

// setAngleNC(2,90);
Point cPos(31.5,8,0);
Leg second(0,1,2,93,37,2,cPos);
Point f(120,10,0);
Point s(150,10,0);
Point t(150,40,0);
Point fo(120,40,0);
//Point f(70,150,0);
//Point s(100,150,0);
//Point t(100,180,0);
//Point fo(70,180,0);
Serial.println("First point: ");
second.reach(f);
delay(2000);
Serial.println("Second point: ");
second.reach(s);
delay(2000);
Serial.println("Third point: ");
second.reach(t);
delay(2000);
Serial.println("4 point: ");
second.reach(fo);
delay(2000);

 

  

}
