
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
  float _cAngle1 = rad2deg(_cAngle) ;
  float _fAngle1 = 90+rad2deg(_fAngle)  -20;// -20
  Serial.println("CAngle: ");
  Serial.println(rad2deg(_cAngle));
  Serial.println("FAngle:");
  Serial.println(rad2deg(_fAngle));
  Serial.println("TAngle:");
  Serial.println(rad2deg(_tAngle));
  float _tAngle1 = 30  + rad2deg(_fAngle) - rad2deg(_tAngle) - 25; //- 30
  Serial.println("");
  Serial.println(_cAngle1);
  Serial.println(_fAngle1);
  Serial.println(_tAngle1);       
  
  setAngle(0,_cAngle1);
  //delay(5000);
  setAngle(1,_fAngle1);
  //delay(5000);
  setAngle(2,_tAngle1);
}
void reach(float _x, float _y, float _z)
{
        float hDist = sqrt( sqr(_x) +  sqr(_y) );
        Serial.println("Dist: ");
        Serial.println(hDist);
        float additionalCoxaAngle = asin( 5.0 / hDist );
        Serial.println("Additional: ");
        Serial.println(rad2deg(additionalCoxaAngle));
        
        float primaryCoxaAngle = polarAngle(_x, _y);
        Serial.println("Primary: ");
        Serial.println(rad2deg(primaryCoxaAngle));
        float cAngle = primaryCoxaAngle + additionalCoxaAngle;
        Serial.println("");
        //return cAngle;

        float localDestX = sqrt(sqr(hDist) - sqr(5.0)) - 14.0;
        Serial.println("New Local DestX: ");
        Serial.println(localDestX);
        float localDestY = _z - 8 ;
        Serial.println("New Local DestY: ");
        Serial.println(localDestY);
        Serial.println("");
        float localDistSqr = sqr(localDestX) + sqr(localDestY);
        if (localDistSqr > sqr (51.0 + 70.0))
        {
          Serial.println("Can't reach!");
          //return;
        }

        float A = -2 * localDestX;
        Serial.println("Line: ");
        Serial.println(A);
        float B = -2 * localDestY;
        Serial.println(B);
        float C = (sqr(localDestX) + sqr(localDestY) + sqr(51.0) - sqr(70.0));
        Serial.println(C);
        Serial.println("");
        float _A = (sqr(-A)/sqr(B)) + 1;
        float _B = -2*-A*C/sqr(B);
        float _C = ((sqr(C)) / sqr(B)) - sqr(51.0);
        Serial.println("Equation: ");
        Serial.println(_A);
        Serial.println(_B);
        Serial.println(_C);
        Serial.println("");

        float X1 = (-_B + sqrt(sqr(_B) - 4*_A*_C)) / (2*_A); //ERROR
        float X2 = (-_B - sqrt(sqr(_B) - 4*_A*_C)) / (2*_A); //ERROR

        float Y1 = ((-C-A*X1) / B);
        float Y2 = ((-C-A*X2) / B);
        
        float jointLocalY = (Y1 > Y2) ? Y1 : Y2;
        Serial.println("JointY: ");
        Serial.println(jointLocalY);
        
        float jointLocalX = ((jointLocalY * B + C) / -A)  ; //EBRROR 
        Serial.println("JointX: ");
        Serial.println(jointLocalX);

        Serial.println("");
        float primaryFemurAngle = polarAngle(jointLocalX, jointLocalY);
        float fAngle = primaryFemurAngle;

        float primaryTibiaAngle = polarAngle(localDestX - jointLocalX, localDestY - jointLocalY);
        float tAngle = primaryTibiaAngle;
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

reach(70,70, 0);
delay(500);
reach(50, 70, 0);
delay(500);
reach(50, 90, 0);
delay(500);
reach(70, 90, 0);
delay(500);

}
