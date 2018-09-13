
#include <LiquidCrystal.h>

/******for testing****/
#define VOLUME_D 0.25
#define K 0.8

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
int lcd_case = 0;
volatile int timeToRun = 0;
int motorPin = 5;

//#define K 0.09

#define N1 20 //# of teeths
#define N2 13 // # of teeths
#define SLOPE 0.00047 // mml
#define DEG_ML_CAM  1/SLOPE //DEGREE MML CAMSHAFT 



//RELATIONSHIP DEGREE-MML-MOTOR
#define DEG_ML_MOTOR N2/N1 * ( DEG_ML_CAM)


#define THETA_D VOLUME_D * DEG_ML_MOTOR //2765


int  PWM = 0;



#define encoderPinA 2
#define encoderPinB 3
#define CPR 12
volatile int counter = 0;
volatile boolean flag;
int var_degrees = 0;


void setup() {

   
  pinMode(5, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  Serial.begin (9600);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), isr_A, CHANGE); //INTERRUPT FOR A 
  attachInterrupt(digitalPinToInterrupt(encoderPinB), isr_B, CHANGE); //INTERRUPT FOR B
  Serial.print("THETA_D = ");
  Serial.println(THETA_D); 
}

void loop() {

   if(flag == true){     
       var_degrees = ((360.0/CPR)*counter);
       Serial.print("COUNTER = " );
       Serial.println(counter);
       Serial.print("DEGREES = ");
       Serial.println(var_degrees);
       flag = false;
  }
  
     PWM = K* (abs(THETA_D) - abs(var_degrees));

           // Serial.println(PWM); //1352

  if( abs(var_degrees) > THETA_D){
   analogWrite(motorPin, 0);
  }
  else{
  analogWrite(motorPin, PWM);
  }


}
//Interrupts 

void isr_A(){

  
flag = true;
  
  if(digitalRead(encoderPinA) == HIGH){
    
    if(digitalRead(encoderPinB) == LOW){
      counter = counter +1; 
    }
    else{
      counter = counter -1; 
    }
  }
  else{ //IF PIN A IS LOW
    if(digitalRead(encoderPinB) == HIGH){
      counter = counter +1;
    }
    else{
      counter = counter -1 ; 
    }
  }
  
}

void isr_B(){

  
flag = true;
  if(digitalRead(encoderPinB) == HIGH)
  {
    if(digitalRead(encoderPinA) == HIGH)
    {
      counter = counter + 1; 
    }
      else
    {
      counter = counter -1; 
    }
  }
  else
  { //IF PIN A IS LOW
    if(digitalRead(encoderPinA) == LOW){
      counter = counter +1; 
    }
    else{
      counter = counter -1 ; 
    }
  }
  
}

