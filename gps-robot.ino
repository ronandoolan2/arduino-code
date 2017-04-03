//Include libaries
#include <Scheduler.h>
#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <math.h> 
#include <LiquidCrystal.h>

//LCD SCREEN
LiquidCrystal lcd(33, 31, 29, 27, 25, 23);

//GPS pins
#define rxPin 43      //rx pin in gps connection
#define txPin 42      //tx pin in gps connection
SoftwareSerial gps = SoftwareSerial(rxPin, txPin);

//Speech Pins
#define rxPin2 53
#define txPin2 52
SoftwareSerial SpeakJet = SoftwareSerial(rxPin2, txPin2);

//Compass Pins
byte CLK_pin = 22;
byte EN_pin = 24;
byte DIO_pin = 26;

//Sonar Pins
const int anPin1 = 1;
const int anPin2 = 2;
const int anPin3 = 3;

//Motor Pins
AF_DCMotor motor(3, MOTOR12_64KHZ); // create motor #2, 64KHz pwm
AF_DCMotor rotor(2, MOTOR12_64KHZ);

//GPS integers
int Waypoint = 0;
int G = 10;
byte byteGPS = 0;
int i = 0;
int h = 0;
int b = 0;
int t = 0;
int lat = 0;
int lon = 0;
int sat = 0;
//time data
int time0 = 0;
int time1 = 0;
int time2 = 0;
int time3 = 0;
int time4 = 0;
int time5 = 0;
int hours = 0;
int minutes = 0;
int seconds = 0;
//latitude data
int latdeg0 = 0;
int latdeg1 = 0;
int latmin0 = 0;
int latmin1 = 0;
int latmin2 = 0;
int latmin3 = 0;
int latmin4 = 0;
int latmin5 = 0;
int latdeg = 0;
float latmin = 0;
int latdeg_waypoint1 = 53;
float latmin_waypoint1 = 23142;
int latdeg_waypoint2 = 53;
float latmin_waypoint2 = 23122;
int latdeg_waypoint3 = 53;
float latmin_waypoint3 = 23155;
int latdeg_waypoint4 = 53;
float latmin_waypoint4 = 23181;

//longtitude data
int londeg0 = 0;
int londeg1 = 0;
int londeg2 = 0;
int lonmin0 = 0;
int lonmin1 = 0;
int lonmin2 = 0;
int lonmin3 = 0;
int lonmin4 = 0;
int lonmin5 = 0;
int londeg = 0;
float lonmin = 0;
int londeg_waypoint1 = 6;
float lonmin_waypoint1 = 15402;
int londeg_waypoint2 = 6;
float lonmin_waypoint2 = 15422;
int londeg_waypoint3 = 6;
float lonmin_waypoint3 = 15514;
int londeg_waypoint4 = 6;
float lonmin_waypoint4 = 15510;

//Satellite data
int sat0 = 0;
int sat1 = 0;
int satellites = 0;

// Buffers for data input
char inBuffer[300] = "";
char GPS_RMC[100]="";
char GPS_GGA[100]="";
char time[100]="";
char latitude[100]="";
char longtitude[100]="";
char satellite[10]="";


//Compass integers
int X_Data = 0;
int Y_Data = 0;
int angle;
int Direction = 0;
int turn = 0;
int C = 0;
int comp = 0;

//Sonar integers
int k = 0;
int K = 0;
long anVolt1, inches1, cm1;
long anVolt2, inches2, cm2;
long anVolt3, inches3, cm3;
int sum1=0;//Create sum variable so it can be averaged
int sum2=0;
int sum3=0;
int avgrange1=60;


//========= MoveForward ====

//compass
//0 = forward
//1 = 15right
//2 = 45right
//3 = 90right
//4 = 180
//5 = 15left
//6 = 45left
//7 = 90left

//sonar
//0 = Nothing ahead
//1 right blocked
//2 left blocked
//3 blocked
//4 too close

void moveForward()
{
  G++;
  //If final destination reached stop vehicle
  if(Waypoint == 4)
  {
    Stop();
  }
  //Display which waypoint it last reached and the time it took
  lcd.setCursor(0, 1);
  lcd.print(Waypoint);
  lcd.print("         ");
  lcd.print(millis()/1000);
  lcd.print("         ");
  
  if(G > 10)
  {
    //check the GPS data
    checkGPS();
    if(Waypoint == 0)
    {
      //check if waypoint 1 has been reached
      if(latdeg == latdeg_waypoint1 && latmin >= 23139 && latmin <= 23145 && londeg == londeg_waypoint1 && lonmin >= 15398 && lonmin <= 15407)
      {
        lcd.setCursor(0, 1);
        lcd.print("Waypoint1 Reached");
        //Tell people about engineering building
        char SayIt1[] = {20, 96, 21, 114, 22, 88, 23, 5, 130, 141, 7, 165, 8, 129, 141, 149, 7, 148, 128, 143, 1, 170, 8, 128, 145, 174, 129, 143, 3, 8, 169, 8, 129, 187, 1, 7, 128, 143, 196, 7, 146, 7, 138, 174, 187, 1, 8, 169, 8, 128, 1, 187, 195, 8, 139, 146, 1, 8, 134, 166, 1, 140, 131, 196, 8, 132, 141, 129, 194, 159, 1, 8, 132, 8, 141, 177, 1, 140, 8, 132, 8, 142, 162, 186, 132, 196, 6, 182, 7, 151, 128, 143, 1, 130, 141, 7, 165, 8, 129, 141, 149, 7, 148, 128, 143, 3, 8, 132, 8, 141, 177, 1, 140, 8, 137, 187, 191, 1, 8, 134, 166, 1, 8, 169, 8, 128, 1, 187, 195, 8, 139, 146, 1, 8, 134, 166, 1, 7, 129, 145, 131, 196, 191, 7, 148, 136, 141, 129, 194, 1, 130, 141, 7, 165, 8, 129, 141, 149, 7, 148, 128, 143, 3, 8, 129, 8, 191, 1, 147, 134, 167, 1, 170, 8, 128, 145, 174, 1, 8, 129, 8, 141, 1, 8, 191, 162, 8, 190, 7, 163, 167, 7, 133, 8, 141, 8, 191, 162, 3, 8, 129, 8, 191, 1, 8, 135, 8, 146, 8, 188, 164, 1, 183, 132, 8, 167, 1, 140, 131, 141, 128, 1, 148, 128, 187, 8, 134, 152, 182, 1, 145, 132, 132, 172, 187};
        int c=0;
        while(SayIt1[c])
        c++;
        for ( int i=0;i<c;i++)
        {
          SpeakJet.print(SayIt1[i]);
          delay(70);
         }
        delay(8000);
        //update direction and waypoint
        Waypoint = 1;
        Direction = -165;
      }
      else
      {
        if(latdeg < latdeg_waypoint1 || latmin < latmin_waypoint1)
        {
          //south of engineering building set direction to north
          lcd.setCursor(0, 0);
          lcd.print("Head North");
          Direction = 0;
          
         }
        else if(latdeg > latdeg_waypoint1 || latmin > latmin_waypoint1)
        {
          //north of engineering building set direction to south
          lcd.setCursor(0, 0);
          lcd.print("Head South");
          Direction = 180;
        }
        else if(londeg < londeg_waypoint1 || lonmin < lonmin_waypoint1)
        {
          //east of engineering building set direction to west
          lcd.setCursor(0, 0);
          lcd.print("Head West");
          Direction = -90;
        }
        else if(londeg > londeg_waypoint1 || lonmin > lonmin_waypoint1)
        {
          //west of engineering building set direction to east
          lcd.setCursor(0, 0);
          lcd.print("Head East");
          Direction = 90;
        }
      }
    }
    if(Waypoint == 1)
    {
      //check has waypoint 2 (exit of windtunnel) been reached
      if(latdeg == latdeg_waypoint2 && latmin <= 23126 && latmin >= 23118 && londeg == londeg_waypoint2 && lonmin >= 15416 && lonmin <= 15428)
      {
        lcd.setCursor(0, 1);
        lcd.print("Waypoint2 Reached");
        analogWrite(50,128);
        delay(5000);
        digitalWrite(50, LOW);
        delay(500);
        //update waypoint and direction
        Direction = -60;
        Waypoint = 2;
      }
      else
      {
        lcd.setCursor(0, 1);
        lcd.print("Not Reached Waypoint2 yet continue");
        Direction = -165;
        Waypoint = 1;
      }
    }
    if(Waypoint == 2)
    {
      if(latdeg == latdeg_waypoint3 && latmin >= 23152 && latmin <= 23159 && londeg == londeg_waypoint3 && lonmin >= 15510 && lonmin <= 15519)
      {
        lcd.setCursor(0, 1);
        lcd.print("Waypoint3 Reached");
        analogWrite(50,128);
        delay(5000);
        digitalWrite(50, LOW);
        delay(500);
        Direction = 15;
        Waypoint = 3;
      }
      else
      {
        Direction = -60;
        Waypoint = 2;
      }
    }
    if(Waypoint == 3)
    {
      if(latdeg == latdeg_waypoint3 && latmin >= 23178)
      {
        lcd.setCursor(0, 1);
        lcd.print("Waypoint4 Reached");
        analogWrite(50,128);
        delay(5000);
        digitalWrite(50, LOW);
        delay(500);
        Waypoint = 4;
      }
      else
      {
        Direction = 15;
        Waypoint = 3;
      }
    }
    G = 10;
  }
  //gps takes a long time to intialise and be accurate so this is
  //necessary to stop it driving until then
  if(millis() > 120000)
  {
  sonar();
  if(k==0)
  {
    C=2;
    rotor.run(RELEASE);
    if(C > 1)
    {
      checkCompass();
      if(comp == 0)
      {
        sonar();
        if(k == 0)
        {
          motor.run(FORWARD);   // turn it on going forward
          delay(350);
          motor.run(RELEASE);
        }
        if(k == 1)
        {
          C = 0;
          lcd.setCursor(0, 0);
          lcd.print("RightBlocked");
          lcd.print("        ");
          Right45();
          forward();
          Left45();
        }
        if(k == 2)
        {
          C = 0;
          lcd.setCursor(0, 0);
          lcd.print("LeftBlocked");
          lcd.print("        ");
          Left45();
          forward();
          Right45();
        }
        if(k == 3)
        {
          C = 0;
          moveBackwards();
          Right90();
          forward();
          lcd.setCursor(0, 0);
          lcd.print("blocked");
          lcd.print("        ");
          analogWrite(50,255);
          delay(100);
          digitalWrite(50, LOW);
        }
        if(k == 4)
        {
          lcd.setCursor(0, 0);
          lcd.print("too close");
          lcd.print("        ");
          C = 0;
          moveBackwards();
        }
      }
      if(comp == 1)
      {
        lcd.setCursor(0, 1);
        lcd.print("right15 ");
        lcd.print("        ");
        Right15();
      }
      else if(comp == 2)
      {
        lcd.setCursor(0, 1);
        lcd.print("right45 ");
        lcd.print("        ");
        Right45();
      }
      else if(comp == 3)
      {
        lcd.setCursor(0, 1);
        lcd.print("right90 ");
        lcd.print("        ");
        Right90();
      }
      else if(comp == 4)
      {
        lcd.setCursor(0, 1);
        lcd.print("turn 180");
        lcd.print("        ");
        Turn180();
      }
      else if(comp == 5)
      {
        lcd.setCursor(0, 1);
        lcd.print("left 15 ");
        lcd.print("        ");
        Left15();
      }
      else if(comp == 6)
      {
        lcd.setCursor(0, 1);
        lcd.print("left 45 ");
        lcd.print("        ");
        Left45();
      }
      else if(comp == 7)
      {
        lcd.setCursor(0, 1);
        lcd.print("left 90 ");
        lcd.print("        ");
        Left90();
      }
      else{}
    }
    }
    if(k == 1)
        {
          C = 0;
          lcd.setCursor(0, 0);
          lcd.print("RightBlocked");
          lcd.print("        ");
          Right45();
          forward();
          Left45();
        }
        if(k == 2)
        {
          C = 0;
          lcd.setCursor(0, 0);
          lcd.print("LeftBlocked");
          lcd.print("        ");
          Left45();
          forward();
          Right45();
        }
        if(k == 3)
        {
          C = 0;
          moveBackwards();
          Right90();
          lcd.setCursor(0, 0);
          lcd.print("blocked");
          lcd.print("        ");
          analogWrite(50,128);
          delay(500);
          digitalWrite(50, LOW);
          forward();
        }
        if(k == 4)
        {
          lcd.setCursor(0, 0);
          lcd.print("too close");
          lcd.print("        ");
          C = 0;
          moveBackwards();
        }
  }
}
//function for robot to reverse
void moveBackwards()
{
  sonar2();
  if(K==1)
  {
  motor.run(BACKWARD);   // turn it on going forward
  analogWrite(50,128);
  delay(350);
  motor.run(RELEASE);   // turn it on going forward
  digitalWrite(50, LOW);
  delay(350);
  }
  else
  {
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
  }
}
//function for robot to turn 90 degrees left
void Left90()
{
  sonar();
  if(k!=4)
  {
  rotor.run(FORWARD);
  delay(500);
  motor.run(FORWARD);   // turn it on going forward
  delay(350);
  motor.run(RELEASE);
  delay(350);
  }
  sonar();
  if(k!=4)
  {
  motor.run(FORWARD);   // turn it on going forward
  delay(350);
  motor.run(RELEASE);
  delay(350);
  }
  sonar();
  if(k!=4)
  {
  motor.run(FORWARD);   // turn it on going forward
  delay(350);
  motor.run(RELEASE);
  delay(350);
  }
  sonar2();
  if(K==1)
  {
  motor.run(RELEASE);
  rotor.run(RELEASE);
  delay(500);
  rotor.run(BACKWARD);
  delay(500);
  motor.run(BACKWARD);
  delay(1000);
  }
  else
  {
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
  }
  motor.run(RELEASE);
  rotor.run(RELEASE);
}

//function for robot to turn 90 degrees right
void Right90()
{
  sonar();
  if(k!=4)
  {
  rotor.run(BACKWARD);
  delay(500);
  motor.run(FORWARD);   
  delay(350);
  motor.run(RELEASE);
  delay(350);
  }
  sonar();
  if(k!=4)
  {
  motor.run(FORWARD);   // turn it on going forward
  delay(350);
  motor.run(RELEASE);
  delay(350);
  }
  sonar();
  if(k!=4)
  {
  motor.run(FORWARD);
  delay(350);
  motor.run(RELEASE);
  delay(350);
  }
  motor.run(RELEASE);
  rotor.run(RELEASE);
  delay(500);
  sonar2();
  if(K==1)
  {
  rotor.run(FORWARD);
  delay(500);
  motor.run(BACKWARD);
  delay(1000);
  }
  else
  {
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
    analogWrite(50,255);
    delay(500);
    digitalWrite(50, LOW);
    delay(500);
  }
  motor.run(RELEASE);
  rotor.run(RELEASE);
}

void Left45()
{
  sonar();
  if((k==0)||(k==2))
  {
    rotor.run(FORWARD);
    delay(500);
    motor.run(FORWARD);   // turn it on going forward
    delay(350);
    motor.run(RELEASE);
  delay(350);
    //check sonar
    sonar();
     if((k==2)||(k==0))
    {
    motor.run(FORWARD);   // turn it on going forward
    delay(350);
    motor.run(RELEASE);
  delay(350);
    }
    sonar();
     if((k==2)||(k==0))
    {
    motor.run(FORWARD);   // turn it on going forward
    delay(350);
    motor.run(RELEASE);
  delay(350);
    }
    motor.run(RELEASE);
    rotor.run(RELEASE);
    delay(1000);
  }
}


int forward()
{
  int f = 0;
 for(int j=0;j<4;j++)
    {
      sonar();
      if(k==0)
      {
      rotor.run(RELEASE);
      motor.run(FORWARD);
      delay(350);
      motor.run(RELEASE);
      delay(250);
      f = 0;
      }
      else
      {
        f = 1;
      }
     
  }
  return f;
}


void Right45()
{
  sonar();
  if((k==0)||(k==1))
  {
    rotor.run(BACKWARD);
    delay(500);
    motor.run(FORWARD);   // turn it on going forward
    delay(350);
    motor.run(RELEASE);
  delay(350);
    sonar();
    if((k==1)||(k==0))
    {
    motor.run(FORWARD);   // turn it on going forward
    delay(350);
    motor.run(RELEASE);
  delay(350);
    }
    sonar();
    if((k==1)||(k==0))
    {
    motor.run(FORWARD);   // turn it on going forward
    delay(350);
    motor.run(RELEASE);
  delay(350);
    }
    motor.run(RELEASE);
    rotor.run(RELEASE);
    delay(1000);
  }
}

void Left15()
{
  sonar();
if(k==0)
{
  rotor.run(FORWARD);
  delay(500);
  motor.run(FORWARD);   // turn it on going forward
  delay(500);   
  motor.run(RELEASE);
  rotor.run(RELEASE);
}
}

void Right15()
{
  sonar();
  if(k==0)
  {
  rotor.run(BACKWARD);
  delay(500);
  motor.run(FORWARD);   // turn it on going forward
  delay(500);   
  motor.run(RELEASE);
  rotor.run(RELEASE);
  }
}
void Stop()
{
  motor.run(RELEASE);
  rotor.run(RELEASE);
  delay(1000000);
}
int checkCompass()//Display how off compass in degrees
{
  HM55B_StartMeasurementCommand(); // necessary!!
  delay(40); // the data is 40ms later ready
  HM55B_ReadCommand();
  X_Data = ShiftIn(11); // Field strength in X
  Y_Data = ShiftIn(11); // and Y direction
  digitalWrite(EN_pin, HIGH); // ok deselect chip
  angle = 180 * (atan2(-1 * Y_Data , X_Data) / M_PI); // angle is atan( -y/x) !!!
  lcd.setCursor(0, 1);
  lcd.print("Angle: ");
  lcd.print(angle);
  lcd.print("        ");
  turn = Direction  - angle ;
  if(turn < -180)
  {
    turn = (360 + turn);
  }
  if(turn > 180)
  {
    turn = -(360 - turn);
  }
  lcd.setCursor(0, 1);
  lcd.print("Error:");
  lcd.print(turn);
  lcd.print("       ");
  delay(250);
  if(turn < 10 && turn > -10)
  {
    return comp = 0;
  }
  else
  {
    if(turn > 10)
    {
      if(turn < 40)
      {
        return comp = 1;
      }
      else if(turn < 80)
      {
//        Serial.print("turn Right45");
        return comp = 2;
      }
      else if(turn < 170)
      {
//        Serial.print("turn Right90");
        return comp = 3;
      }
      else
      {
//        Serial.print("turn 180");
        return comp = 4;
      }
    }
    else
    {
      if(turn > -40)
      {
 //       Serial.print("turn Left15");
        return comp = 5;
      }
      else if(turn > -80)
      {
 //       Serial.print("turn Left45");
        return comp = 6;
      }
      else if(turn > -170)
      {
 //       Serial.print("turn Left90");
        return comp = 7;
      }
      else
      {
 //       Serial.print("turn 180");
        return comp = 4;
      }
    }
  }
}

void Turn180()
{
  Left90();
  Left90();
}
int sonar()
{
   sum1 = 0;//Reset values
   sum2 = 0;
    pinMode(anPin1, INPUT);
    pinMode(anPin2, INPUT);
    for(int i = 0; i < avgrange1 ; i++)
    {
      //Used to read in the analog voltage output that is being sent by the MaxSonar device.
      //Scale factor is (Vcc/512) per inch. A 5V supply yields ~9.8mV/in
      anVolt1 = analogRead(anPin1);
      sum1 += anVolt1;
      anVolt2 = analogRead(anPin2);
      sum2 += anVolt2;
      delay(20);
    }  
    inches1 = sum1/avgrange1;
    cm1 = inches1 * 2.54;
    inches2 = sum2/avgrange1;
    cm2 = inches2 * 2.54;
   delay(50);
   if(cm1 > 150 && cm2 >150)// 0 = nothing ahead
   {
//      Serial.print("Nothign ahead");
      return k = 0;
   }
   else if(cm1 > 150 && cm2 <= 150)
   {
//      Serial.print("Right blocked");
      return k = 1;
    }
    else if(cm1 <= 150 && cm2 > 150)
    {
//      Serial.print("Left blocked");
      return k = 2;
    }
   else if(cm1 <= 150 && cm2 <= 150) //
  {
//    Serial.print("Motor Stop!");
  
    if(cm1 <=80 || cm2 <= 80)
    {
//      Serial.print("too close");
      return k = 4;
    }
    else
    {
//      Serial.print("Blocked");
      return k = 3;
     }
  }
}
int sonar2()
{
  sum3=0;
  pinMode(anPin3, INPUT);
  for(int i = 0; i < avgrange1 ; i++)
  {
    anVolt3 = analogRead(anPin3);
    sum3 += anVolt3;
    delay(10);
  }  
  inches3 = sum3/avgrange1;
  cm3 = inches3 * 2.54;
  delay(50);
  if (cm3 < 200)
  {
    return K = 0;
  }
  else
  {
    return K = 1;
  }
  sum3 = 0;
}

void ShiftOut(int Value, int BitsCount) {
  for(int i = BitsCount; i >= 0; i--) {
    digitalWrite(CLK_pin, LOW);
    if ((Value & 1 << i) == ( 1 << i)) {
      digitalWrite(DIO_pin, HIGH);
    }
    else {
      digitalWrite(DIO_pin, LOW);
    }
    digitalWrite(CLK_pin, HIGH);
    delayMicroseconds(1);
  }
}
void checkGPS()
{
  // Read the RMC sentence from GPS  
byteGPS = 0;  
byteGPS = gps.read();  
while(byteGPS != 'R')
{    
byteGPS = gps.read();
}
GPS_RMC[0]='$';
GPS_RMC[1]='G';
GPS_RMC[2]='P';
GPS_RMC[3]='R';
i = 4;    
while(byteGPS != '*')
{                        
byteGPS = gps.read();               
inBuffer[i]=byteGPS;      
GPS_RMC[i]=byteGPS;      
i++;                        
}    
// Read GGA sentence from GPS  
byteGPS = 0;  
byteGPS = gps.read();  
while(byteGPS != 'A')
{    
byteGPS = gps.read();
}  
GPS_GGA[0]='$';
GPS_GGA[1]='G';
GPS_GGA[2]='P';
GPS_GGA[3]='G';
GPS_GGA[4]='G';
GPS_GGA[5]='A';
i = 6;    
while(byteGPS != '*')
{                        
byteGPS = gps.read();               
inBuffer[i]=byteGPS;      
GPS_GGA[i]=byteGPS;      
i++;                        
}      
// print the GGA sentence to USB  
Serial.print("GGA sentence: ");  
h = 0;  
lat = 0;
lon = 0;
sat = 0;
while(GPS_GGA[h] != 42)
{    
  if(b == 2)
  {
    latitude[lat]=GPS_GGA[h];
    lat++;
  }
  if(b == 4)
  {
    longtitude[lon]=GPS_GGA[h];
    lon++;
  }
  if(b == 7)
  {
    satellite[sat]=GPS_GGA[h];
    sat++;
  }

//Serial.print(GPS_GGA[h],BYTE);
if(GPS_GGA[h]==',')
    {
       // Serial.println();
        b++;
        if(b == 1)
        {
       //   Serial.print("time: ");
        }
        if(b == 2)
        {
       //   Serial.print("latitude: ");
        }
        if(b == 4)
        {
       //   Serial.print("longtitude: ");
        }
    }
h++;  
}  
//Serial.println();   
  int sat0 = (int)satellite[0];
  sat0 = sat0 - 48;
  int sat1 = (int)satellite[1];
  sat1 = sat1 - 48;
  satellites = sat0*10 + sat1;
//  Serial.print("No of satellites:  ");
//  Serial.print(satellites);
//  Serial.println();
    analogWrite(50,128);
    delay(50);
    digitalWrite(50, LOW);
  if(satellites > 0)
  {
  int latdeg0 = (int)latitude[0];
  latdeg0 = latdeg0 - 48;
  int latdeg1 = (int)latitude[1];
  latdeg1 = latdeg1 - 48;
  int latmin0 = (int)latitude[2];
  latmin0 = latmin0 - 48;
  int latmin1 = (int)latitude[3];
  latmin1 = latmin1 - 48;
  int latmin2 = (int)latitude[5];
  latmin2 = latmin2 - 48;
  int latmin3 = (int)latitude[6];
  latmin3 = latmin3 - 48;
  int latmin4 = (int)latitude[7];
  latmin4 = latmin4 - 48;
  int latmin5 = (int)latitude[8];
  latmin5 = latmin5 - 48;
  latdeg = latdeg0*10 + latdeg1;
  latmin = latmin0*10000 + latmin1*1000 + latmin2*100 + latmin3*10 + latmin4*1 + latmin5*0.1;
  //calculate longtitude
  int londeg0 = (int)longtitude[0];
  londeg0 = londeg0 - 48;
  int londeg1 = (int)longtitude[1];
  londeg1 = londeg1 - 48;
  int londeg2 = (int)longtitude[2];
  londeg2 = londeg2 - 48;
  int lonmin0 = (int)longtitude[3];
  lonmin0 = lonmin0 - 48;
  int lonmin1 = (int)longtitude[4];
  lonmin1 = lonmin1 - 48;
  int lonmin2 = (int)longtitude[6];
  lonmin2 = lonmin2 - 48;
  int lonmin3 = (int)longtitude[7];
  lonmin3 = lonmin3 - 48;
  int lonmin4 = (int)longtitude[8];
  lonmin4 = lonmin4 - 48;
  int lonmin5 = (int)longtitude[9];
  lonmin5 = lonmin5 - 48;
  londeg = londeg0*100 + londeg1*10 + londeg2;
  lonmin = lonmin0*10000 + lonmin1*1000 + lonmin2*100 + lonmin3*10 + lonmin4*1 + lonmin5*0.1;
  //print latitude
  lcd.setCursor(0, 0);
  lcd.print("lat");
  lcd.print(latdeg);
  lcd.print("d");
  lcd.print(latmin);
  lcd.print("                 ");
  //print longtitude
  lcd.setCursor(0, 1);
  lcd.print("long");
  lcd.print(londeg);
  lcd.print("d");
  lcd.print(lonmin);
  lcd.print("          ");
  delay(250);
  }
  else
  {
    lcd.setCursor(0, 0);
    lcd.print("NO SATELLITES");
    lcd.setCursor(0, 1);
    lcd.print("            ");
  }
// print the RMC sentence to USB  
//Serial.print("RMC sentence: ");  
h = 0;  
b = 0;
while(GPS_RMC[h] != 42)
{    

h++;  
}  
}
int ShiftIn(int BitsCount) {
  int ShiftIn_result;
    ShiftIn_result = 0;
    pinMode(DIO_pin, INPUT);
    for(int i = BitsCount; i >= 0; i--) {
      digitalWrite(CLK_pin, HIGH);
      delayMicroseconds(1);
      if (digitalRead(DIO_pin) == HIGH) {
        ShiftIn_result = (ShiftIn_result << 1) + 1;
        //Serial.print("x");
      }
      else {
        ShiftIn_result = (ShiftIn_result << 1) + 0;
        //Serial.print("_");
      }
      digitalWrite(CLK_pin, LOW);
      delayMicroseconds(1);
    }
  //Serial.print(":");

// below is difficult to understand:
// if bit 11 is Set the value is negative
// the representation of negative values you
// have to add B11111000 in the upper Byte of
// the integer.
// see: http://en.wikipedia.org/wiki/Two%27s_complement
  if ((ShiftIn_result & 1 << 11) == 1 << 11) {
    ShiftIn_result = (B11111000 << 8) | ShiftIn_result;
  }


  return ShiftIn_result;
}

void HM55B_Reset() {
  pinMode(DIO_pin, OUTPUT);
  digitalWrite(EN_pin, LOW);
  ShiftOut(B0000, 3);
  digitalWrite(EN_pin, HIGH);
}

void HM55B_StartMeasurementCommand() {
  pinMode(DIO_pin, OUTPUT);
  digitalWrite(EN_pin, LOW);
  ShiftOut(B1000, 3);
  digitalWrite(EN_pin, HIGH);
}

int HM55B_ReadCommand() {
  int result = 0;
  pinMode(DIO_pin, OUTPUT);
  digitalWrite(EN_pin, LOW);
  ShiftOut(B1100, 3);
  result = ShiftIn(3);
  return result;
}

void setup() {
  //initialise motors
  motor.setSpeed(255);
  rotor.setSpeed(255);
  //intialise GPS pins
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  gps.begin(4800);
  //intialise speech
  pinMode(txPin2, OUTPUT);
  SpeakJet.begin(9600);
  delay(1000);
  //intialise compass
  pinMode(EN_pin, OUTPUT);
  pinMode(CLK_pin, OUTPUT);
  pinMode(DIO_pin, INPUT);
  HM55B_Reset();
  //buzzer
  pinMode(50, OUTPUT);
  //intialise lcd screen
  lcd.begin(16, 2);
  lcd.print(";)");
  int G = 10;
}
void loop() {
moveForward();
}

 



