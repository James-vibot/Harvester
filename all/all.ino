#include <Servo.h>
#include <stdlib.h>
#include <math.h>

//////////
//A    B//
//C    D//
//////////
// Motor A connections
int a1 = 2; 
int a2 = 3; 
// Motor B connections
int b1 = 4;  
int b2 = 5;  
// Motor C connections
int c1 = 6;
int c2 = 7;
// Motor D connections
int d1 = 8;
int d2 = 9;
// sensors connections
int left_sensor=10;
int right_sensor=11;
//HC-SR04 Sensor connection
int trigPin = 12;
int echoPin = 13;

int left_value=0;
int right_value=0;
int prev_left_value=0;
int prev_right_value=0;
int stop_distance = 12;
long duration;
int distance;
int mode=-1;//line follower =0    auto=1
String data;
bool flagdetect;
///////////////////////////////////////
//arm
Servo servo1,servo2,servo3,servo4; 
int cur_pos1=180;
int cur_pos2=0;
int cur_pos3=90;
float x,y,z=0;
float L1 = 7;
float L2 = 10.5;
float L3 = 19;
float th1,th2,th3,k,r,alfa,beta;
float Servo1Ang[19]={0,1,1,2,3,2,3,4,5,4.5,4.5,6.5,7,6,7,7,7,7,10};//theta-this error
float Servo2Ang[19]={0,-1,2,4,7,9,12,15,17,20,22,25,28,33,25,30,20,10,0};//theta-this error
float Servo3Ang[19]={-14,-10,-10,-6,-4,-2,1,3,5,6,12,12,15,18,20,25,0,0,0};//theta-this error
///////////////////////////////////////
void setup() {
  // Set all the motor control pins to outputs
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);
  pinMode(c1, OUTPUT);
  pinMode(c2, OUTPUT);
  pinMode(d1, OUTPUT);
  pinMode(d2, OUTPUT);
  pinMode(left_sensor,INPUT);
  pinMode(right_sensor,INPUT);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
  // Turn off motors - Initial state
  // motors A&B off
  digitalWrite(a1, LOW);
  digitalWrite(a2, LOW);
  digitalWrite(b1, LOW);
  digitalWrite(b2, LOW);
  // motors C&D off
  digitalWrite(c1, LOW);
  digitalWrite(c2, LOW);
  digitalWrite(d1, LOW);
  digitalWrite(d2, LOW);
  // sensors initial state - white surface
  digitalWrite(left_sensor,HIGH);
  digitalWrite(right_sensor,HIGH);
  Serial.begin(9600);
  servo1.attach(A0);
  servo2.attach(A1);
  servo3.attach(A2);
  servo4.attach(A3);
  servo1.write(180);// 0 - 179    45 90 150
  servo2.write(0); // 0 - 179  //gear ratio error 22.5   15
  servo3.write(90); // 0 - 179//limit 150   10
  servo4.write(0);
}

void loop() 
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance= duration*0.034/2;

  left_value = digitalRead(left_sensor);
  right_value = digitalRead(right_sensor);
 
  if (Serial.available() > 0) 
  {
     data = Serial.readStringUntil('\n');
     Serial.println("mode or detect receive= "+data);
  }
  if(data=="0")//line follower mode 
  {
    mode=0;
  }
  else if(data=="1")//auto mode 
  {
    mode=1;
  }

  flagdetect=false;
  if(mode==0)//line follower  
  { 
    if (Serial.available() > 0) 
    {
       data = Serial.readStringUntil('\n');
       Serial.println("mode or detect receive= "+data);
    }
    if(data=="stop")
    {
      flagdetect=true;
      stop_pick();
    }
    //forward
    if(left_value==HIGH && right_value==HIGH)
    {
      Forward();
      delay_serial(10);
      stop_detect();
    } 
    //stop
    else if(right_value==0 && left_value==0) //black on the right and left -> stop && prev_left_value==1 && prev_right_value==1
    {
      Forward();
      delay(300);
      Stop();
      while(true)
      {
        if (Serial.available() > 0) 
        {
          data = Serial.readStringUntil('\n');
          Serial.println("finish");
        }
      }   
    } 
    //right  
    else if(right_value==LOW && left_value==HIGH) //black on the right -> go to the right
    {
      Right();
      delay_serial(250);
    }
    
    //left
    else if(right_value==HIGH && left_value==LOW) //black on the left -> go to the left
    {
      Left();
      delay_serial(250);
    }
    prev_left_value = left_value;
    prev_right_value =right_value;
    ////////////object avoidance
    if(distance < stop_distance)
    {
      Back();
      delay_serial(250);
      Left();
      delay_serial(1500);
      Forward();
      delay_serial(2500);
      Right();
      delay_serial(1500);
      Forward();
      delay_serial(2500);
      Right();
      delay_serial(1500);
      Forward();
      while(left_value == HIGH){
      left_value = digitalRead(left_sensor);
      right_value = digitalRead(right_sensor);
      }
      //Back();
      //delay(100);
      Left();
      delay_serial(700);
    }
  }
  else if(mode == 1)//auto
  {
      Forward();
      delay_serial(10);
      stop_detect();
      if (Serial.available() > 0) 
      {
        data = Serial.readStringUntil('\n');
        Serial.println("mode or detect receive= "+data);
      }
      if(data=="enter")//line follower mode 
      {
        enter();
      }
      else if (data=="Arrow")
      {
        wrap();
      }
      else if (data=="stop")
      {
        stop_pick();
      }
   
      if(distance < stop_distance)
      {
        Back();
        delay_serial(250);
        Left();
        delay_serial(1500);
        Forward();
        delay_serial(2500);
        Right();
        delay_serial(1500);
        Forward();
        delay_serial(2500);
        Right();
        delay_serial(1500);
        Forward();
      }
  }
}
//////////////////////////////////////
void enter()
{
  Forward();
  delay_serial(1500);
  Left();
  delay_serial(2000);
  Forward();
}
void wrap()
{
  Forward();
  delay_serial(1500);
  Left();
  delay_serial(2000);
  Forward();
  delay_serial(1500);
  Left();
  delay_serial(2000);
}  
void Forward()
{
  // Turn on motor A & B
  digitalWrite(a1, LOW);
  digitalWrite(a2, HIGH);
  digitalWrite(b1, LOW);
  digitalWrite(b2, HIGH);
  // Turn on motor C & D
  digitalWrite(c1, HIGH);
  digitalWrite(c2, LOW);
  digitalWrite(d1, LOW);
  digitalWrite(d2, HIGH);
  //delay to look to plant
}
void Right()
{
  //backward direction for right wheels B & D 
  //forward direction for left wheels A & C
  // B backword
  digitalWrite(b1, HIGH);
  digitalWrite(b2, LOW);
  // D backward
  digitalWrite(d1, HIGH);
  digitalWrite(d2, LOW);
  // A forward 
  digitalWrite(a1, LOW);
  digitalWrite(a2, HIGH);
  // C forward
  digitalWrite(c1, HIGH);
  digitalWrite(c2, LOW);
  //delay(250);
}
void delay_serial(int n)
{  
  for (int i=0;i<n;i++)
  {
    if (Serial.available() > 0) 
    {
      data = Serial.readStringUntil('\n');
      Serial.println("mode or detect receive delay= "+data);
    }
    if(data=="stop")//line follower mode 
    { 
      flagdetect=true;
      stop_pick();
    }
    else if(data=="enter")//line follower mode 
    {
      enter();
    }
    else if (data=="Arrow")
    {
      wrap();
    }
    else if (data=="stop_shape")
    {
      stop_shap();
    }
    delay(1);
  }
}
void Left()
{
  //backward direction for left wheels A & C
  //forward direction for right wheels B & D 
  // B forward
  digitalWrite(b1, LOW);
  digitalWrite(b2, HIGH);
  // D forward
  digitalWrite(d1, LOW);
  digitalWrite(d2, HIGH);
  // A backward 
  digitalWrite(a1, HIGH);
  digitalWrite(a2, LOW);
  // C backward
  digitalWrite(c1, LOW);
  digitalWrite(c2, HIGH);
  //delay(250);
}
void Back()
{
  //backward
  digitalWrite(a1, HIGH);
  digitalWrite(a2, LOW);
  digitalWrite(b1, HIGH);
  digitalWrite(b2, LOW);
  digitalWrite(c1, LOW);
  digitalWrite(c2, HIGH);
  digitalWrite(d1, HIGH);
  digitalWrite(d2, LOW);
}
void Stop()
{
  // Turn off motors
  digitalWrite(a1, LOW);
  digitalWrite(a2, LOW);
  digitalWrite(b1, LOW);
  digitalWrite(b2, LOW);
  digitalWrite(c1, LOW);
  digitalWrite(c2, LOW);
  digitalWrite(d1, LOW);
  digitalWrite(d2, LOW);
  while(true)
  {
    if (Serial.available() > 0) 
    {
      data = Serial.readStringUntil('\n');
      Serial.println("mode or detect receive= "+data);
      if(data=="0")//line follower mode 
      {
        mode=0;
      }
      else if(data=="1")//auto mode 
      {
        mode=1;
      }
      break;
    }
  }
}
void move_s2_s3(int pos2,int pos3)
{
  if(pos2>=cur_pos2)
  {
    for(int POS = cur_pos2 ; POS <= pos2 ; POS++)
    {
      servo2.write(POS);
      delay(20);
      if((POS > 115) && (cur_pos3 < 150))
      {
        for(int p = cur_pos3 ; p<=150 ; p++)
        {
          servo3.write(p);
          delay(20);
        }
        cur_pos3 = 150;
      }
    }
    cur_pos2 = pos2;
  } 
  else if(pos2<cur_pos2)
  {
    for(int POS = cur_pos2 ; POS >= pos2 ; POS--)
    {
      servo2.write(POS);
      delay(20);
    }
    cur_pos2 = pos2;
  }
  if(pos3>=cur_pos3)
  {
    for(int POS = cur_pos3 ; POS <= pos3 ; POS++)
    {
      servo3.write(POS);
      delay(20);
    }
    cur_pos3 = pos3;
  }
  else if(pos3<cur_pos3)
  {
    for(int POS = cur_pos3 ; POS >= pos3 ; POS--)
    {
      servo3.write(POS);
      delay(20);
    }
    cur_pos3 = pos3;
  } 
}
void return_to_init(int pos1,int pos2,int pos3)
{
  if(pos1>=cur_pos1)
  {
    for(int POS=cur_pos1;POS<=pos1;POS++)
    {
      servo1.write(POS);
      delay(20);
    }
    cur_pos1 = pos1;
  }
  else if(pos1<cur_pos1)
  {
    for(int POS = cur_pos1 ; POS >= pos1 ; POS--)
    {
      servo1.write(POS);
      delay(20);
    }
    cur_pos1 = pos1;
  }
  if(pos3>=cur_pos3)
  {
    for(int POS = cur_pos3 ; POS <= pos3 ; POS++)
    {
      servo3.write(POS);
      delay(20);
    }
    cur_pos3 = pos3;
  }
  else if(pos3<cur_pos3)
  {
    for(int POS = cur_pos3 ; POS >= pos3 ; POS--)
    {
      servo3.write(POS);
      delay(20);
    }
    cur_pos3 = pos3;
  }
  if(pos2>=cur_pos2)
  {
    for(int POS = cur_pos2 ; POS <= pos2 ; POS++)
    {
      servo2.write(POS);
      delay(20);
      if((POS > 115) && (cur_pos3 < 150))
      {
        for(int p = cur_pos3 ; p<=150 ; p++)
        {
          servo3.write(p);
          delay(20);
        }
        cur_pos3 = 150;
      }
    }
    cur_pos2 = pos2;
  } 
  else if(pos2<cur_pos2)
  {
    for(int POS = cur_pos2 ; POS >= pos2 ; POS--)
    {
      servo2.write(POS);
      delay(20);
    }
    cur_pos2 = pos2;
  }
  servo4.write(180);
  delay_serial(250);
  servo4.write(0);
}
void go_to(int pos1,int pos2,int pos3)
{
  if(pos2>=cur_pos2)
  {
    for(int POS = cur_pos2 ; POS <= pos2 ; POS++)
    {
      servo2.write(POS);
      delay(20);
      if((POS > 115) && (cur_pos3 < 150))
      {
        for(int p = cur_pos3 ; p<=150 ; p++)
        {
          servo3.write(p);
          delay(20);
        }
        cur_pos3 = 150;
      }
    }
    cur_pos2 = pos2;
  } 
  else if(pos2<cur_pos2)
  {
    for(int POS = cur_pos2 ; POS >= pos2 ; POS--)
    {
      servo2.write(POS);
      delay(20);
    }
    cur_pos2 = pos2;
  }
  for(int c=0;c<=180;c++)
  { 
    servo4.write(c);
    delay(20);
  }
  if(pos1>=cur_pos1)
  {
    for(int POS=cur_pos1;POS<=pos1;POS++)
    {
      servo1.write(POS);
      delay(20);
    }
    cur_pos1 = pos1;
  }
  else if(pos1<cur_pos1)
  {
    for(int POS = cur_pos1 ; POS >= pos1 ; POS--)
    {
      servo1.write(POS);
      delay(20);
    }
    cur_pos1 = pos1;
  }
  if(pos3>=cur_pos3)
  {
    for(int POS = cur_pos3 ; POS <= pos3 ; POS++)
    {
      servo3.write(POS);
      delay(20);
    }
    cur_pos3 = pos3;
  }
  else if(pos3<cur_pos3)
  {
    for(int POS = cur_pos3 ; POS >= pos3 ; POS--)
    {
      servo3.write(POS);
      delay(20);
    }
    cur_pos3 = pos3;
  }
  delay(1000);
  for(int c=180;c>=0;c--)
  { 
    servo4.write(c);
    delay(20);
  }
  delay(1000);
  while(cur_pos3<=150 || cur_pos2<=125  )
  {
    move_s2_s3(cur_pos2 + 2,cur_pos3 + 2);
  }
  return_to_init(180,25,100);
}
void calculateANG(int X,int Y,int Z)
{
  // Find theta 1 in rad
  th1 = atan2(Y,X);
  r = sqrt(sq(X)+sq(Y));

  // Find theta 3 in rad
  k = sqrt(sq(Z-L1) + sq(r));
  th3 = acos((sq(k)-sq(L2)-sq(L3))/(2*L2*L3));
  th3 = -th3;

  // Find th2 in rad
  beta = atan2((Z-L1),r);
  alfa = atan2((L3*sin(th3)),(L2+(L3*cos(th3))));
  th2 = beta-alfa;
  th1 = th1 * (180 / 3.14159);
  th2 = th2 * (180 / 3.14159);
  th3 = th3 * (180 / 3.14159);
  
  int pos1=(int)th1;
  int pos2=(int)th2;
  int pos3=(int)th3+90;
  float newpos1,newpos2,newpos3=0;
  int d=0;
  int index=0;
  float error=0;

  //pos1
  if(pos1%10==0)//90==>10
  {
    index=(int)(pos1/10);
    error = Servo1Ang[index];
    newpos1=pos1+error;
  }
  else if (pos1%10!=0)//93/10=9.3
  {
    d = (int)pos1%10;
    if(d<5)
    { 
        index=(int)(pos1/10);
        error = Servo1Ang[index];
        newpos1=pos1+error+(pos1%10); 
    }
    else if(d>5)
    { 
        index=(int)(pos1/10)+1;
        error = Servo1Ang[index];
        newpos1=pos1+error; 
    }
    else if(d==5)
    { 
        index=(int)(pos1/10);
        error = (Servo1Ang[index]+Servo1Ang[index+1])/2;
        newpos1=pos1+error; 
    }
  }

  //pos2
  if(pos2%10==0)//90==>10
  {
    index=(int)(pos2/10);
    error = Servo2Ang[index];
    newpos2=pos2-error;
  }
  else if (pos2%10!=0)//93/10=9.3
  {
    d = (int)pos2%10;
    if(d<5)
    { 
        index=(int)(pos2/10);
        error = Servo2Ang[index];
        newpos2=pos2-error; 
    }
    else if(d>5)
    { 
        index=(int)(pos2/10)+1;
        error = Servo2Ang[index];
        newpos2=pos2-error; 
    }
    else if(d==5)
    { 
        index=(int)(pos2/10);
        error = (Servo2Ang[index]+Servo2Ang[index+1])/2;
        newpos2=pos2-error; 
    }
  }

  //pos3
  if(pos3%10==0)//90==>10
  {
    index=(int)(pos3/10);
    error = Servo3Ang[index];
    newpos3=pos3-error;
  }
  else if (pos3%10!=0)//93/10=9.3
  {
    d = (int)pos3%10;
    if(d<5)
    { 
        index=(int)(pos3/10);
        error = Servo3Ang[index];
        newpos3=pos3-error; 
    }
    else if(d>5)
    { 
        index=(int)(pos3/10)+1;
        error = Servo3Ang[index];
        newpos3=pos3-error; 
    }
    else if(d==5)
    { 
        index=(int)(pos3/10);
        error = (Servo3Ang[index]+Servo3Ang[index+1])/2;
        newpos3=pos3-error; 
    }
  }

  go_to(newpos1,newpos2,150-newpos3);
  //when finish pick up send done to rass
  Serial.println("done");
  //if there is no other one on the frame to pick
  //rass will send Go
  //otherwise rass send the new location
}
void stop_pick()
{
  // Turn off motors
  digitalWrite(a1, LOW);
  digitalWrite(a2, LOW);
  digitalWrite(b1, LOW);
  digitalWrite(b2, LOW);
  digitalWrite(c1, LOW);
  digitalWrite(c2, LOW);
  digitalWrite(d1, LOW);
  digitalWrite(d2, LOW);
  String s[4];
  int counter = 0;
  while(true)
  {
    counter = 0;
    while(true)
    {
       if (Serial.available() > 0) 
       {
//         if (flagdetect==true)
//         {   s[0] ="detect";
//            flagdetect=false;
//            counter++;
//            continue;
//         }
         s[counter] = Serial.readStringUntil('\n');  
         Serial.println("spatial receive " +s[counter]);
         data="pick";
         counter++;
         if(s[0]=="Go")
         {
            counter=0;
            return;
            break;
         }
         if(counter==4)
         {
            counter=0;
            break;        
         }
       }       
    }
    if(s[0]=="Go")
    {
      Serial.println("go hhhh");
      return;
      break;
    }
    if(s[0]=="detect")
    {
      
     x=round(tofloat(s[1])) ;
     delay(300);
     y=round(tofloat(s[2])) ;
     delay(300);
     z=round(tofloat(s[3])) ;
     delay(300);
     Serial.println(x);
     Serial.println(y);
     Serial.println(z);
     calculateANG(x,y-19,z+11);//x y z in cm
    }  
  }
}
float  tofloat(String s)
{
  char* ch = new char[s.length()];
  for (int i = 0; i < s.length(); i++)
  {
    ch[i] = s.charAt(i);
  }  
  float x = atof(ch);
  return x;        
}
void stop_shap()
{

  digitalWrite(a1, LOW);
  digitalWrite(a2, LOW);
  digitalWrite(b1, LOW);
  digitalWrite(b2, LOW);
  digitalWrite(c1, LOW);
  digitalWrite(c2, LOW);
  digitalWrite(d1, LOW);
  digitalWrite(d2, LOW);
  while(true)
  {
    if (Serial.available() > 0) 
    {
      data = Serial.readStringUntil('\n');
      Serial.println("stop shape reicive= "+data);
    }
    if(data=="stop")//line follower mode 
    { 
      flagdetect=true;
      stop_pick();
    }
    else if(data=="enter")//line follower mode 
    {
      enter();
      break;
    }
    else if (data=="Arrow")
    {
      wrap();
      break;
    }
  }
  data=""; 
}
void stop_detect()
{
  // Turn off motors
  digitalWrite(a1, LOW);
  digitalWrite(a2, LOW);
  digitalWrite(b1, LOW);
  digitalWrite(b2, LOW);
  digitalWrite(c1, LOW);
  digitalWrite(c2, LOW);
  digitalWrite(d1, LOW);
  digitalWrite(d2, LOW);
  delay_serial(50);
}
