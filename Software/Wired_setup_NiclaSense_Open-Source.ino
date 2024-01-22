/*Save the serial prints in case of a sudden malfunctioning!!! */

//This sketch is only for the Nicla Sense ME, not the Nicla Vision
#ifdef ARDUINO_NICLA_VISION
  #error "Run the standard Blink.ino sketch for the Nicla Vision"
#endif

#include "Filters.h"
#include <BasicLinearAlgebra.h>
#include <math.h>
#include <Servo.h>
#include "TickTwo.h" //For Arduino BLE nano 33, let's find other boards!

#define ARR_SIZE(m) (sizeof(m) / sizeof(m[0]))
#define WINDOW 50 //Windows size of 50 samples = 100ms
#define MAX_CHAR 32

//This sketch is only for the Nicla Sense ME, not the Nicla Vision
#ifdef ARDUINO_NICLA_VISION
  #error "Run the standard Blink.ino sketch for the Nicla Vision"
#endif

// Intialise library which communicates with RGB driver
// Functions accessible under 'nicla' namespace
#include "Nicla_System.h"    

/***EMG***/
Filter bandP0;
Filter notchF0;

//Emg0: Flexion and use the new one. Less gain(?)
//Emg1: Extension and ues the old one. Recognize more what is the weaker muscle. More gain(?)
int emg_inputpin = A0;
int emg_analog;
int gain;
float emg_bpf, emg_nf, emg_abs, emg_f, emg_ff; 
int threshold1, threshold2;
float array_emg[WINDOW];
bool servo_status, serial_status;
const int hand_open = 1900; //1900
const int hand_closed = 1100; //1100

//Serial read
char receivedChars[MAX_CHAR];
char command_parameter[MAX_CHAR];
int command_value = 0;

float sum;
int indexus;

float mavFilter(float in) {
    sum = sum - array_emg[indexus];
    array_emg[indexus] = in;
    sum = sum + array_emg[indexus];
    indexus++;
    if (indexus >= WINDOW) {
        indexus = 0;
    }
    return sum/WINDOW;
}

void initArrayEmg(void) {for(int i = 0; i < WINDOW; i++) {array_emg[i] = 0.0;}}

Servo hand_servo;

/***TickTwo***/
void analog_reading();
TickTwo timer1(analog_reading, 2, 0, MILLIS); //ticker 2ms, 0.002s, 500Hz. 
void servo_opening();
TickTwo timer2(servo_opening, 10, 0, MILLIS); //ticker 10ms, 0.01s, 100Hz. 
void serial_sending();
TickTwo timer3(serial_sending, 15, 0, MILLIS); //ticker 10ms, 0.01s, 100Hz. 

void setup() { // put your setup code here, to run once:

  //run this code once when Nicla Sense ME board turns on
  nicla::begin();               // initialise library
  nicla::leds.begin();          // Start I2C connection
  nicla::leds.setColor(red); 

  emg_analog = 0;
  emg_bpf = 0.0;
  emg_nf = 0.0;
  emg_abs = 0.0;
  emg_f = 0.0;
  emg_ff = 0.0;
  sum = 0.0;
  indexus = 0;
  threshold1 = 100;
  threshold2 = 100;
  gain = 1;
  initArrayEmg();
  //f0 = 55 Hz, Q = 0.42  
  //LETS, IMPROVE THIS TO: 20-150 HZ!!!
  bandP0.bandPass(84, 1.76, 500); //1.76, 0.77, 45-154. 10 difference from the high spectrum
  notchF0.notchF(50, 0.2, 500); //0.2, 7.2, 46.648-53.593
  servo_status = false;
  serial_status = false;

  hand_servo.attach(0);
  hand_servo.writeMicroseconds(hand_closed);

  /***Serial***/
  Serial.begin(230400); //115200
  while (!Serial); // This means connection to the PC, if it is connected no matter the serial port is displayed, it will be True!!!
  delay(100);
  //nicla::leds.setColor(green); 

  /***TickTwo***/
  timer1.start();
  delay(100);
  timer2.start(); //Good to see when the prosthesis is opening while setting the thresholds! 
  delay(100);
}

void loop() {
  timer1.update();
  timer2.update();  
  if ((Serial) && (serial_status == false)){ //Enter only when the microcontroller is connected to the computer
    serial_status = true;
    timer3.start();
    //nicla::leds.setColor(cyan); 
    }
  else if ((Serial) && (serial_status == true)){ //Enter only when the microcontroller is connected to the computer
    //nicla::leds.setColor(green); 
    timer3.update();
  }
  else if ((!Serial) && (serial_status == true)){
    //nicla::leds.setColor(off); 
    timer3.stop();
    serial_status = false;
  }
  else {
  }
}

void analog_reading(){
  emg_analog = analogRead(emg_inputpin);
  emg_bpf = bandP0.process(emg_analog); 
  emg_nf = notchF0.process(emg_bpf);
  emg_abs = abs(emg_nf); 
  emg_f = mavFilter(emg_abs);
  emg_ff = emg_f*gain;
}

//Open the servomotor
void servo_opening(){
  if (emg_ff > threshold1){
    servo_status = true;
    hand_servo.writeMicroseconds(hand_open);
  } 
  else if ((emg_ff > threshold2)&&(servo_status == true)){
    hand_servo.writeMicroseconds(hand_open);
  }
  else{
    servo_status = false;
    hand_servo.writeMicroseconds(hand_closed);
  }
}

void serial_sending(){
  //Plot the data all the the time that the serial port is connected.
  Serial.print(emg_ff);
  Serial.print(",");
  Serial.print(threshold1);
  Serial.print(",");
  Serial.println(threshold2);
  
  //Read the data when there a data is sent written from the serial command/plotter.
  if(Serial.available() > 0){
    for (int i = 0; i < MAX_CHAR; i++) {receivedChars[i] = 0;}
    Serial.readBytesUntil('\n', receivedChars, 32);
    //Split the data into parts.
    char* strtokIndx; 
    strtokIndx = strtok(receivedChars,"=");      // get the first part - the string
    strcpy(command_parameter, strtokIndx); // copy it to messageFromPC
    strtokIndx = strtok(NULL,"=");      // get the first part - the string
    command_value = atoi(strtokIndx); // copy it to messageFromPC
    if (strcmp(command_parameter, "t1") == 0){ //True
      threshold1 = command_value; 
    }
    else if(strcmp(command_parameter, "t2") == 0) {
      threshold2 = command_value;
    }
    else if(strcmp(command_parameter, "g") == 0) {
      gain = command_value;
    }
    else{}
  }
}
