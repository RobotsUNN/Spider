
#include <Wire.h>

#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN  150
#define SERVOMAX  600




//#define deg2rad(x) ( (PI / 180.0) * (x) )
//#define fabs(x) ((x) >= 0 ? (x) : - (x))
//#define max(x, y) ((x) > (y) ? (x) : (y))

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
        
    // x = 0
    if (y > 0)
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
float _cStartAngle; // постоянно перезаписывается
float sx, sy;
void moveS(float _cAngle,float _fAngle,float _tAngle)
{
  float _cAngle1 = rad2deg(_cAngle);
  float _fAngle1 = 100.0+rad2deg(_fAngle);
  float _tAngle1 = rad2deg(_tAngle);
  Serial.println("");
  Serial.println(_cAngle1);
  Serial.println(_fAngle1);
  Serial.println(_tAngle1);       
  
  setAngle(0,_cAngle1);
 // delay(5000);
  setAngle(1,_fAngle1);
  //delay(5000);
  setAngle(2,_tAngle1);
}
void reach(float _x, float _y, float _z)
{
        float hDist = sqrt( sqr(_x) +  sqr(_y) );
        Serial.println("Dist: ");
        Serial.println(hDist);
        float additionalCoxaAngle = asin( 14.0 / hDist );
        Serial.println("Additional: ");
        Serial.println(rad2deg(additionalCoxaAngle));
        
        float primaryCoxaAngle = polarAngle(_x, _y);
        Serial.println("Primary: ");
        
        Serial.println(rad2deg(atan(_y/_x)));
        Serial.println(rad2deg(primaryCoxaAngle));
        float cAngle = primaryCoxaAngle + additionalCoxaAngle;
        Serial.println("Done!");
        //return cAngle;

        float localDestX = sqrt(sqr(hDist) - sqr(14.0)) - 12.0;
        Serial.println("Local X: ");
        Serial.println(localDestX);
        float localDestY = _z;
        float localDistSqr = sqr(localDestX) + sqr(localDestY);
        if (localDistSqr > sqr (60.0 + 65.0))
        {
          Serial.println("Can't reach!");
          //return;
        }
        float A = -2 * localDestX;
        float B = -2 * localDestY;
        float C = sqr(localDestX) + sqr(localDestY) + sqr(60.0) - sqr(65.0);
        float X0 = -A * C / (sqr(A) + sqr(B));
        float Y0 = -B * C / (sqr(A) + sqr(B));
        float D = sqrt( sqr(60.0) - (sqr(C) / (sqr(A) + sqr(B))) );
        float mult = sqrt ( sqr(D) / (sqr(A) + sqr(B)));
        float ax, ay, bx, by;
        ax = X0 + B * mult;
        bx = X0 - B * mult;
        ay = Y0 - A * mult;
        by = Y0 + A * mult;
        float jointLocalX = (ax > bx) ? ax : bx;
        float jointLocalY = (ax > bx) ? ay : by;
        
        float primaryFemurAngle = polarAngle(jointLocalX, jointLocalY);
        float fAngle = primaryFemurAngle; // Стоит учесть стартовый угол
 
        float primaryTibiaAngle = polarAngle(localDestX - jointLocalX, localDestY - jointLocalY);
        float tAngle = (primaryTibiaAngle);

        moveS(cAngle,fAngle,tAngle);
        
}




 void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  pwm.begin();
  pwm.setPWMFreq(50);
  yield();

}

void loop() {
  setAngle(1,0.0);
  delay(2000);
  setAngle(1, 90.0);
  delay(2000);

  
  //delay(5000);
  //reach(30.0,60.0,0.0);
 // delay(100000);

}
