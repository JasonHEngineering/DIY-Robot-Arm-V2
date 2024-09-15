/*

V9 - new baseline for controller V3 (purple board) + CNC machined arm
V10 - use pointers and add functions to optimize/shorten code
V11 - stepper.setSpeed() to move all linkages start/stop within fixed time
V13 - stepper 2 direction updated with a new stronger stepper motor
V14 - unknown
V15 - workaround to use original Axis 6 driver on Axis 5 (troubleshoot A5 cannot move) 

*/

// Pins for Motor Controller 

#include <Wire.h>
#include <AccelStepper.h>

// START DEFINE SECTION FOR MOTOR CONTROLLER V3
#define SLAVE_ADDRESS 0x04 // Nano IoT 2

// Define pin connections
#define stepper_1_stepPin 2
#define stepper_1_dirPin 3

#define stepper_2_stepPin 8
#define stepper_2_dirPin 9

#define stepper_3_stepPin 4
#define stepper_3_dirPin 5

#define stepper_4_stepPin 10
#define stepper_4_dirPin 11

// #define stepper_5_stepPin 6
// #define stepper_5_dirPin 7

// #define stepper_6_stepPin 12
// #define stepper_6_dirPin 13

#define stepper_5_stepPin 12
#define stepper_5_dirPin 13

#define stepper_6_stepPin 6
#define stepper_6_dirPin 7


// Define motor interface type
#define motorInterfaceType 1 //AccelStepper::DRIVER (1) means a stepper driver (with Step and Direction pins)
#define no_of_steppers 6
#define max_speed 3000
#define motor_acceleration 60000.0
#define micro_step 4 //e.g. half stepping = 2 

// #define RST_SLP_Pin 10 // do not need for controller V3
// #define enable_Pin 11 //KIV, setting high will off motor holding current

/*#define limitswitch1 14 //analog A0
#define limitswitch2 15 //analog A1
#define limitswitch3 16 //analog A2
#define limitswitch4 17 //analog A3
#define limitswitch5 20 //analog A6
#define limitswitch6 21 //analog A7*/

static int limitswitch1 = 14; //analog A0
static int limitswitch2 = 15; //analog A1
static int limitswitch3 = 16; //analog A2
static int limitswitch4 = 17; //analog A3
static int limitswitch5 = 20; //analog A6
static int limitswitch6 = 21; //analog A7

static int axis_1_start_pos = 3050*micro_step;
static int axis_2_start_pos = 1300*micro_step;
static int axis_3_start_pos = -1200*micro_step;
static int axis_4_start_pos = 2700*micro_step;
static int axis_5_start_pos = 1400*micro_step;
static int axis_6_start_pos = 0*micro_step;

/*#define axis_1_start_pos 10
#define axis_2_start_pos 10
#define axis_3_start_pos 10
#define axis_4_start_pos 10
#define axis_5_start_pos 10
#define axis_6_start_pos 10*/

//#define limit_switch_activated_steps 100 //stepper steps to take when limit switch activated

static int direction[no_of_steppers] = {-1, -1, 1, -1, -1, -1};

//int* limitswitch_vararray[no_of_steppers] = {&limitswitch1, &limitswitch2, &limitswitch3, &limitswitch4, &limitswitch5, &limitswitch6};

//define an array of stepper pointers. initialize the pointer to NULL (null pointer) for safety (see below)
AccelStepper *steppers[no_of_steppers] = {NULL, NULL, NULL, NULL, NULL, NULL};

// END DEFINE SECTION FOR MOTOR CONTROLLER V2


bool flag = false;
//static bool limit_flag = true;
static int zero_flag = 0;
  
//bool homingComplete = false;

// pointers for boolean state of stepper_x_Home
static bool stepper_1_Home, stepper_2_Home, stepper_3_Home, stepper_4_Home, stepper_5_Home, stepper_6_Home;
static bool* stepper_Home_vararray[no_of_steppers] = {&stepper_1_Home, &stepper_2_Home, &stepper_3_Home, &stepper_4_Home, &stepper_5_Home, &stepper_6_Home};

// pointers for limitswitchx pin numbers
static int* limitswitch_vararray[no_of_steppers] = {&limitswitch1, &limitswitch2, &limitswitch3, &limitswitch4, &limitswitch5, &limitswitch6};

// pointers for each axis home position (steps from limit switch activation)
static int* axis_start_pos_vararray[no_of_steppers] = {&axis_1_start_pos, &axis_2_start_pos, &axis_3_start_pos, &axis_4_start_pos, &axis_5_start_pos, &axis_6_start_pos};

// https://medium.com/@sandhan.sarma/sending-floats-over-the-i2c-bus-between-two-arduinos-part-2-486db6dc479f
union BytesToFloat {
    // 'converts' incoming bytes to long array
    byte valueBuffer[no_of_steppers*8];
    long valueReading[no_of_steppers];    
} converter;

union BytesToInt {
    // 'converts' incoming bytes to a single char
    // byte IntBuffer[1*8];
    // long IntReading[1]; 
    byte IntBuffer[8];
    long  IntReading[1];    
} convert_int;

struct dataStruct {
   long motor_1_stepper_position;          
   long motor_2_stepper_position;  
   long motor_3_stepper_position;          
   long motor_4_stepper_position;          
   long motor_5_stepper_position; 
   long motor_6_stepper_position; 
   long zero_status;
} stepper_position_data; 

void setup() {
  
  
  for (int x = 0; x <= 5; x++) {
    *stepper_Home_vararray[x] = false; // set all stepper_x_Home to false
    pinMode(*limitswitch_vararray[x], INPUT_PULLUP); // set all limitswitchx to internal pullup
  }
  
  // Create stepper instances
  steppers[0] = new AccelStepper(motorInterfaceType, stepper_1_stepPin, stepper_1_dirPin);
  steppers[1] = new AccelStepper(motorInterfaceType, stepper_2_stepPin, stepper_2_dirPin);
  steppers[2] = new AccelStepper(motorInterfaceType, stepper_3_stepPin, stepper_3_dirPin);
  steppers[3] = new AccelStepper(motorInterfaceType, stepper_4_stepPin, stepper_4_dirPin);
  steppers[4] = new AccelStepper(motorInterfaceType, stepper_5_stepPin, stepper_5_dirPin);
  steppers[5] = new AccelStepper(motorInterfaceType, stepper_6_stepPin, stepper_6_dirPin);
  
  Serial.begin(9600);          // start serial for output
  delay(100);
  Wire.begin(SLAVE_ADDRESS);   // initialize i2c as slave
  Wire.onReceive(receiveData); // receive interrupt callback (triggered by I2C-Master)
  Wire.onRequest(sendData);
  Serial.println("Ready!");
    

  for (int i = 0; i <= 5; i++) {
    steppers[i]->setMaxSpeed(max_speed); //steps per second
    steppers[i]->setAcceleration(motor_acceleration); //desired acceleration in steps per second per second
  }
  
  zero_flag = 0;
  zero_stepper();

}


void loop() {

    if(flag) run_stepper();
    
    for (int i = 0; i <= 5; i++) {
      steppers[i]->run();
    }
    
    limit_switch_checks();
    
    stepper_position_data.motor_1_stepper_position = steppers[0]->currentPosition();
    stepper_position_data.motor_2_stepper_position = steppers[1]->currentPosition();
    stepper_position_data.motor_3_stepper_position = steppers[2]->currentPosition();
    stepper_position_data.motor_4_stepper_position = steppers[3]->currentPosition();
    stepper_position_data.motor_5_stepper_position = steppers[4]->currentPosition();
    stepper_position_data.motor_6_stepper_position = steppers[5]->currentPosition();
    stepper_position_data.zero_status = zero_flag;
    
    //if ((steppers[0]->distanceToGo()!=0) || (steppers[1]->distanceToGo()!=0) || (steppers[2]->distanceToGo()!=0) || (steppers[3]->distanceToGo()!=0) || (steppers[4]->distanceToGo()!=0)) {
      
/*      for (int x = 0; x < no_of_steppers; x++) {
        Serial.println((String)"Motor "+x+" steps remaining: "+steppers[x]->distanceToGo());
      }*/
      //Serial.println((String)"Motor 0 steps remaining: "+steppers[0]->distanceToGo());
    //}



}


void zero_stepper(){
  
  // reset all stepper_x_Home to false when this function is called
  for (int x = 0; x <= 5; x++) {
    *stepper_Home_vararray[x] = false;
  }
  
  // reset to max_speed
  for (int i = 0; i <= 5; i++) {
    steppers[i]->setMaxSpeed(max_speed); //steps per second
    steppers[i]->setAcceleration(motor_acceleration); //desired acceleration in steps per second per second
  }
  
  stepper_6_Home = true; // for the moment, I am not using limit switch on axis 6
    
while((stepper_1_Home == false) || (stepper_2_Home == false) || (stepper_3_Home == false) || (stepper_4_Home == false) || (stepper_5_Home == false) || (stepper_6_Home == false)) {

  for (int x = 0; x <= 5; x++) {

    if ((digitalRead(*limitswitch_vararray[x]) == HIGH) && (*stepper_Home_vararray[x] == false)) {
      steppers[x]->move(10*direction[x]);
    }
    else {
        *stepper_Home_vararray[x] = true;
        if (digitalRead(*limitswitch_vararray[x]) == LOW) { 
          steppers[x]->move(-400*direction[x]); 
        }
     }
     
      steppers[x]->run();
    
    }
    Serial.println("Homing!");
  }
  delay(1000);
  Serial.println("Homed!");
  
  for (int x = 0; x <= 5; x++) {
    steppers[x]->move(*axis_start_pos_vararray[x]); 
  }


/*  steppers[0]->move(axis_1_start_pos); 
  steppers[1]->move(axis_2_start_pos); 
  steppers[2]->move(axis_3_start_pos); 
  steppers[3]->move(axis_4_start_pos); 
  steppers[4]->move(axis_5_start_pos); 
  steppers[5]->move(axis_6_start_pos); */

  while ((steppers[0]->distanceToGo() != 0) || (steppers[1]->distanceToGo() != 0) || (steppers[2]->distanceToGo() != 0) || (steppers[3]->distanceToGo() != 0) || (steppers[4]->distanceToGo() != 0) || (steppers[5]->distanceToGo() != 0)) {
    for (int i = 0; i <= 5; i++) {
      steppers[i]->run();
      
/*      if ((i == 0) || (i == 3)) {
        Serial.print("steppers[0]->distanceToGo(): ");
        Serial.println(steppers[0]->distanceToGo());
        Serial.print("steppers[3]->distanceToGo(): ");
        Serial.println(steppers[3]->distanceToGo());
      } */
    }
  }
  
  for (int i = 0; i <= 5; i++) {
    steppers[i]->setCurrentPosition(0);
  }
    
  zero_flag = 1;
  
}


void limit_switch_checks(){
  
for (int x = 0; x <= 5; x++) {
 
    if (digitalRead(*limitswitch_vararray[x]) == LOW) {
      steppers[x]->move(-100*direction[x]);
    }
    
    
  }

}

void run_stepper(){
  
  long motor_steps[no_of_steppers]; 
  float time_travel = 1.0;
  bool speed_flag[no_of_steppers] = {false, false, false, false, false, false};
  // sort and find the shortest time possible
  
 
  for (int x = 0; x < no_of_steppers; x++) {
    
    motor_steps[x] = converter.valueReading[x];
    
    while (speed_flag[x]!= true) {
    
    //Serial.println((String)"Motor "+x+" steps is: "+motor_steps[x]);
    //steppers[x]->move(motor_steps[x]);
    //float stepper_speed = motor_steps[x]/4.0;
    //steppers[x]->moveTo(motor_steps[x]);
      float set_speed = (motor_steps[x] - (steppers[x]->currentPosition()))/time_travel;
      if ((set_speed < 2000.0) && (set_speed > -2000.0)) {
        speed_flag[x] = true;}
      else {
        time_travel+=0.1; //add time travel allowed
      }  
    //if (set_speed < -4000.0) {set_speed = -max_speed;}
    //steppers[x]->setMaxSpeed(set_speed); // first trial, set all movement to 8 seconds

    }
  
  }
    
    Serial.println((String)"Time travel allowed is: "+time_travel);

  
  for (int x = 0; x < no_of_steppers; x++) {
    
    motor_steps[x] = converter.valueReading[x];
    Serial.println((String)"Motor "+x+" steps is: "+motor_steps[x]);
    //steppers[x]->move(motor_steps[x]);
    //float stepper_speed = motor_steps[x]/4.0;
    steppers[x]->moveTo(motor_steps[x]);
    float set_speed = (motor_steps[x] - (steppers[x]->currentPosition()))/time_travel;
    //if (set_speed > 2000.0) {set_speed = max_speed;}
    //if (set_speed < -2000.0) {set_speed = -max_speed;}
    steppers[x]->setMaxSpeed(set_speed); // first trial, set all movement to 8 seconds
    Serial.println((String)"Motor "+x+" speed is: "+set_speed);
  }
  
    flag = false;
    
}

void receiveData(int byteCount){
  
  // hint for not triggering onreceive accidentally by onrequest 
  // https://forum.arduino.cc/t/raspberrypi-i2c-arduino-onreceive-onrequest-collision/321731/3
    Serial.println("byteCount recieved");
    Serial.println(byteCount);
  
    if (byteCount>6){
      
      Serial.println("stepper triggered");
      for(uint8_t index = 0; index<byteCount; index++){
          converter.valueBuffer[index] = Wire.read();
      }
      
      flag = true;
      
    }
    
    if (byteCount<6){ // one integer from python is 4 bytes, but an additional dummy byte make it 5 in total
      
      // Serial.println(byteCount);
      for(uint8_t index = 0; index<byteCount; index++){
          convert_int.IntBuffer[index] = Wire.read();
          //Serial.println(convert_int.IntBuffer[index]);
      }
      
      //Serial.println(convert_int.IntReading[0]);

      if (convert_int.IntReading[0] == 8){
        Serial.println("stepper recalibration triggered");
        zero_flag = 0;
        zero_stepper();
      }
      

    }
    
    
}

// test reference from https://stackoverflow.com/questions/40196733/how-to-pass-arduino-struct-to-raspberry-pi-via-i2c

void sendData(){
  
  Wire.write((byte *)&stepper_position_data, sizeof(stepper_position_data));
  flag = false;

}



