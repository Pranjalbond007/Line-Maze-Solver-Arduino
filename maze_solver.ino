#include <QTRSensors.h>
#define NUM_SENSORS             6 
#define NUM_SAMPLES_PER_SENSOR  4  
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  
#define SET_POSITION            2500
#define MAX_SPEED               100    
#define Kp                      1.8
#define Kd                      13
#define Kf                      0
#define THRESHOLD               500   // asuming that above this is black and below it is white 500
#define LEFT_SET_SPEED          150
#define RIGHT_SET_SPEED         150
int M1max = 150; //max motor speeds
int M2max = 150; //max motor speeds
int M1min = 0; //min motor speeds
int M2min = 0; //min motor speeds
int m1Speed = 0;
int m2Speed = 0;
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5},NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
int rag=0;
int enA = 6  ;
int in1 = 4 ;
int in2 = 7  ;
// motor two
int enB = 5   ;
int in3 = 3  ;    //0
int in4 = 2 ;   //1
char ch[100],q[100];
unsigned int sensors[NUM_SENSORS];
int position = 0;
int integral = 0;
int error = 0; 
int motor_speed = 0;
int last_error = 0;
int led=13;
void setup(){
memset(ch, 0, sizeof(ch));
memset(q, 0, sizeof(q));
  delay(100);               // delay of 100 miliseconds to get things ready
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  for (int i = 0; i < 400; i++)     // make the calibration take about 10 seconds
  {
    qtra.calibrate();             // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);
  delay(2000);
 Serial.begin(9600);
  pinMode(led,OUTPUT);
  digitalWrite(led, LOW);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

}

int state =1;             // If at any time status is 0 that means bot has reached the end .
//int mode ;               // variable used for switch case

void loop(){
delay(1000);
  maze_solve();
  delay(5000);
   digitalWrite(led, LOW);
  shortestpath();
  Serial.println(ch);
   Serial.println(q);
  delay(5000);
  digitalWrite(13, HIGH);
   // for (int i = 0; i < 400; i++)     // make the calibration take about 10 seconds
   // qtra.calibrate();             // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
 // digitalWrite(13, LOW);
  delay(2000); 
   calculate();
    state=0;
    digitalWrite(13, LOW);
    delay(10000000);
  state = 0;
}
void pid()
 {
  position = qtra.readLine(sensors,QTR_EMITTERS_ON,1) ;    ///(sensors,QTR_EMITTERS_ON,1) ;
  error = position - 2500;
  integral = integral+error;
  motor_speed = Kp*error + Kd*(error-last_error) + integral*Kf;
   last_error = error;
}
void checkpid()
{
  m1Speed = LEFT_SET_SPEED-motor_speed;
  m2Speed = RIGHT_SET_SPEED+motor_speed;
  if (m1Speed < M1min) //keep speeds to 0 or above
  m1Speed = M1min;
  if (m2Speed < M2min) 
  m2Speed = M2min;
  if (m1Speed > M1max) //maximum allowed value
  m1Speed = M1max;
  if (m2Speed > M2max) //maximum allowed value
  m2Speed = M2max; 
}
void maze_solve(){
  while(state)
  {
    pid();
    int mode=path_type(); // function to determine path type . wether intersection or not
    switch(mode)
    { 
      case 1 : // move the bot an inch ahead to check if the path is a T , Cross or End of Maze and store 'L' in array
      { 
        inch();
         pid();
        if(((sensors[0]<THRESHOLD && sensors[1]<THRESHOLD) && (sensors[2]<THRESHOLD && sensors[3]<THRESHOLD)) &&(sensors[4]<THRESHOLD && sensors[5]<THRESHOLD)) 
        {
      //    Serial.println("end of maze");
          stop_motor();
          state=0;
          digitalWrite(13,HIGH);
        //  delay(5000);
        }
        else
        left();
          break;
      }
      case 2  : // move the bot an inch ahead to check if the path is a Left or Left with straight and store 'L' in array
      { 
        inch();
        pid();
        if(sensors[2]<THRESHOLD && sensors[3]<THRESHOLD)
        left1();
        else
          left();
           break;
       }
      case 3  : // move the bot an inch ahead to check if the path is a Right or Right with straight and store in array 
      {  
        inch();
        pid();
        if(sensors[2]>THRESHOLD || sensors[3]>THRESHOLD)
          right();
        else
          straight(1);
           break;
      }
      case 4 : // stop the motor and turn back and Store 'B' in array 
       {
        around();i
        break;}
      case 5 : // just follow the line .
       {
        straight(0);
        break;}
    }
  }
}
int path_type()
{
if(((sensors[0]<THRESHOLD && sensors[1]<THRESHOLD) && (sensors[2]<THRESHOLD && sensors[3]<THRESHOLD)) &&(sensors[4]<THRESHOLD && sensors[5]<THRESHOLD)) 
    return 1 ;
  // if first sensor detect black and the last sensor white then the intersection has a left turn or left with straight path . 
  else if(((sensors[4]<THRESHOLD && sensors[5]<THRESHOLD) &&(sensors[2]<THRESHOLD && sensors[3]<THRESHOLD)) && (sensors[0]>THRESHOLD && sensors[1]>THRESHOLD)) 
       return 2 ;
  // if last sensor detect black and the first white then the intersection has a right turn or right with straight path .
  else if(((sensors[2]<THRESHOLD && sensors[3]<THRESHOLD) &&(sensors[0]<THRESHOLD && sensors[1]<THRESHOLD)) && (sensors[4]>THRESHOLD && sensors[5]>THRESHOLD)) 
    return 3 ;
  // if above cases are not valid then automatically it is NoLine condition
  else if(((sensors[2]>THRESHOLD && sensors[3]>THRESHOLD) &&(sensors[0]>THRESHOLD && sensors[1]>THRESHOLD)) && (sensors[4]>THRESHOLD && sensors[5]>THRESHOLD)) 
    return 4;
  else
    return 5;
}
void straight(int type)
{
   if(type)
  {
     ch[rag]='S';
  rag++;
  }
  pid();
  checkpid();
  digitalWrite(in1,HIGH)   ;
    digitalWrite(in2,LOW)    ;    
    digitalWrite(in3,HIGH)   ;
    digitalWrite(in4,LOW)    ;
    analogWrite(enA,m1Speed)   ;
    analogWrite(enB,m2Speed)  ;
}
void left()
{
  ch[rag]='L';
  rag++;
  while(sensors[5]>THRESHOLD)
  {
       digitalWrite(in1, LOW );
       digitalWrite(in2, HIGH)  ;
       digitalWrite(in3, HIGH);
       digitalWrite(in4, LOW);
       analogWrite(enA,100)    ;
       analogWrite(enB,100)  ;
       pid();      
   }
   while(sensors[2]>THRESHOLD &&   sensors[3]>THRESHOLD)
   {
       digitalWrite(in1,LOW)   ;
       digitalWrite(in2,HIGH)    ;    
       digitalWrite(in3,HIGH)    ;
       digitalWrite(in4,LOW)   ;
       analogWrite(enA,100)    ;
       analogWrite(enB,100)  ;
       pid()                    ;
   }
}
void right(){
   ch[rag]='R';
  rag++;
    while(sensors[2] > THRESHOLD &&   sensors[3]>THRESHOLD)
    {
        digitalWrite(in1,HIGH)   ;
        digitalWrite(in2,LOW)    ;    
        digitalWrite(in3,LOW)   ;
        digitalWrite(in4,HIGH)    ;
        analogWrite(enA,100)    ;
        analogWrite(enB,100)  ;
        pid();
     }
}
void around(){
    ch[rag]='B';
  rag++;
    inch();
    pid();
   while(sensors[2]>THRESHOLD &&   sensors[3]>THRESHOLD)
    {
        digitalWrite(in1,LOW)   ;
        digitalWrite(in2,HIGH)    ;    
        digitalWrite(in3,HIGH)   ;
        digitalWrite(in4,LOW)    ;
        analogWrite(enA,100)    ;
        analogWrite(enB,100)   ;
        pid();
    }  
}
void inch()
{
    digitalWrite(in1,HIGH)   ;
    digitalWrite(in2,LOW)    ;    
    digitalWrite(in3,HIGH)     ;
    digitalWrite(in4,LOW)    ;
    analogWrite(enA,100)     ;
    analogWrite(enB,100)     ;
    delay(200)               ;
}
void stop_motor(){
  digitalWrite(in1,LOW)      ;
  digitalWrite(in2,LOW)      ;    
  digitalWrite(in3,LOW)      ;
  digitalWrite(in4,LOW)      ;
    analogWrite(enA,0)     ;
    analogWrite(enB,0)     ;
  delay(500)                 ;
}
void left1()
{
   ch[rag]='L';
  rag++;
    while(sensors[5]>THRESHOLD)
  {
       digitalWrite(in1, LOW );
       digitalWrite(in2, HIGH)  ;
       digitalWrite(in3, HIGH);
       digitalWrite(in4, LOW);
       analogWrite(enA,150)    ;
       analogWrite(enB,150)  ;
       pid();      
   }
    while(sensors[4]>THRESHOLD)
  {
       digitalWrite(in1, LOW );
       digitalWrite(in2, HIGH)  ;
       digitalWrite(in3, HIGH);
       digitalWrite(in4, LOW);
       analogWrite(enA,150)    ;
       analogWrite(enB,150)  ;
       pid();      
   }
   while(sensors[2]>THRESHOLD)
   {
       digitalWrite(in1,LOW)   ;
       digitalWrite(in2,HIGH)    ;    
       digitalWrite(in3,HIGH)    ;
       digitalWrite(in4,LOW)   ;
       analogWrite(enA,150)    ;
       analogWrite(enB,150)  ;
       pid()                    ;
   }
    while(sensors[3]>THRESHOLD)
  {
       digitalWrite(in1, LOW );
       digitalWrite(in2, HIGH)  ;
       digitalWrite(in3, HIGH);
       digitalWrite(in4, LOW);
       analogWrite(enA,150)    ;
       analogWrite(enB,150)  ;
       pid();      
   }
}
void shortestpath()
{
  int i=0,k,pra,w=0;
  while(ch[i]!='\0'){
    q[i]=ch[i];
    i++;}
    q[i]='\0';
  while(1)
  {
    pra=w=0;
    k=strlen(q);
    for(i=0 ; i<(k-2) ; i++)
    {
      if((q[i]=='L' && q[i+1]=='B') && q[i+2]=='L'){
        q[w]='S';
        w++;
        i+=2;
        pra=1;
        continue;}
         if((q[i]=='L' && q[i+1]=='B') && q[i+2]=='S'){
        q[w]='R';
        w++;
        i+=2;
        pra=1;
          continue;}
         if((q[i]=='L' && q[i+1]=='B') && q[i+2]=='R'){
        q[w]='B';
        w++;
        i+=2;
        pra=1;
          continue;}
         if((q[i]=='R' && q[i+1]=='B') && q[i+2]=='L'){
        q[w]='B';
        w++;
        i+=2;
        pra=1;
          continue;}
         if((q[i]=='S' && q[i+1]=='B') && q[i+2]=='L'){
        q[w]='R';
        w++;
        i+=2;
        pra=1;
          continue;}
         if((q[i]=='S' && q[i+1]=='B') && q[i+2]=='S'){
        q[w]='B';
        w++;
        i+=2;
        pra=1;
          continue;}
          q[w]=q[i];
          w++;
    }
    while(q[i]!='\0')
    {
        q[w]=q[i];
        w++;
        i++;
    }
    q[w]='\0';
    if(!pra)
    break;
  }
}
void calculate()
{
  int r=(strlen(q))-1;
  int val=0;
  while(val!=r)
  {
    pid();
    int mode=path1_type(); 
    switch(mode)
    { 
      case 1 : // move the bot an inch ahead to check if the path is a T , Cross or End of Maze and store 'L' in array
      { 
          Serial.println("1");
          if(q[val]=='L')
          {
              val++;
              inch();
              pid();
              left();
          }
          else if(q[val]=='R')
          {
              val++;
              inch();
              pid();
              right();
          }
          else
          {
              val++; 
              straight(0);
          }
          break;
      }
      case 2  : // move the bot an inch ahead to check if the path is a Left or Left with straight and store 'L' in array
      { 
         Serial.println("2");
         if(q[val]=='L')
         {
             val++;
             inch();
             pid();
             left1();
         }
         else
         {
             val++; 
             straight(0); 
          }
           break;
      }
      case 3  : // move the bot an inch ahead to check if the path is a Right or Right with straight and store in array 
      {  
        Serial.println("3");
        if(q[val]=='R')
        {
          val++;
          inch();
           pid();
          right();
         }
        else
        {
           val++;
          straight(0);
         }
           break;
      }
      case 4 : // just follow the line .
       {
        Serial.println("4");
        straight(0);
        break;}
    }
  }
   
  int sec=1;
  while(sec){
  pid();
  if(((sensors[0]<THRESHOLD && sensors[1]<THRESHOLD) && (sensors[2]<THRESHOLD && sensors[3]<THRESHOLD)) &&(sensors[4]<THRESHOLD && sensors[5]<THRESHOLD)) 
  stop_motor();
  sec=0;
  }
}
int path1_type()
{ 
  if(((sensors[0]<THRESHOLD && sensors[1]<THRESHOLD) && (sensors[2]<THRESHOLD && sensors[3]<THRESHOLD)) &&(sensors[4]<THRESHOLD && sensors[5]<THRESHOLD)) 
    return 1 ;
  // if first sensor detect black and the last sensor white then the intersection has a left turn or left with straight path . 
  else if(((sensors[4]<THRESHOLD && sensors[5]<THRESHOLD) &&(sensors[2]<THRESHOLD && sensors[3]<THRESHOLD)) && (sensors[0]>THRESHOLD && sensors[1]>THRESHOLD)) 
       return 2 ;
  // if last sensor detect black and the first white then the intersection has a right turn or right with straight path .
  else if(((sensors[2]<THRESHOLD && sensors[3]<THRESHOLD) &&(sensors[0]<THRESHOLD && sensors[1]<THRESHOLD)) && (sensors[4]>THRESHOLD && sensors[5]>THRESHOLD)) 
    return 3 ;
  else
    return 4;
}
