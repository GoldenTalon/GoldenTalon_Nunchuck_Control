//Motor Data: https://www.pololu.com/product/3039

#include <SPI.h> // Not required for 
#include <Wire.h>
#include <float.h>
#include <nRF24L01.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/
#include <RF24.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/

//---- Hardcoded angle values for the PID demo ---//
#define POS1 100
#define POS2 20


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//---- I/O pins ----//
#define Amotor 5    
#define Bmotor 6
#define limit_switch_1 A0    
#define encoder_interrupt_pin 2
#define a_encoder A2
#define b_encoder A3
#define boot_indicator_LED 13

//---- Motor Feedback Parameters ----//
#define max_theta  190
#define min_theta  -5
const float GearRatio = 51.45;
const float shaftPPR = 32;
const float deg_per_pulse = 360/(GearRatio*shaftPPR); //degrees per pulse.
float max_encoder_count = 0;

//---- Incremental Encoder Variables ----//
int encoder_count = 0;
int old_encoder_state;
int new_encoder_state;
int Out;
int QEM [16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
int count = 0;

//--- PID Algorithm Vars ----//
#define kp 1.0
#define ki 0.300000000000
#define kd 0.000000000000
char pwm_output = 0;

float set_point_theta = 0;
float output_theta = 0;
float theta_error = 0;
float theta_I = 0;
float theta_d = 0;
float old_encoder_state_theta_error = 0;
char output_pwm;

byte pwm_limit = 0xFF;

//----- NRF24L01 Pinout and Configuration -----//
// Source: Forcetronic : https://forcetronic.blogspot.com/2018/09/easy-way-to-create-wireless-sensor.html
const uint8_t pinCE = 7; //This pin is used to set the nRF24 to standby (0) or active mode (1)
const uint8_t pinCSN = 8; //This pin is used for SPI comm chip select
RF24 wirelessSPI(pinCE, pinCSN); // Declare object from nRF24 library (Create your wireless SPI) 
const uint64_t rAddress = 0xB00B1E50C3LL;  //Create pipe address for the network and notice I spelled boobies because I am mature, the "LL" is for LongLong type
const uint8_t rFChan = 89; //Set channel frequency default (chan 84 is 2.484GHz to 2.489GHz)

//Create a structure to hold_encoder_state fake sensor data and channel data
struct PayLoad {
  uint8_t chan;
  int joy_x;
  int joy_y;
};

PayLoad payload; //create struct object


//----- Prototypes -----//
void encoder_change(void);

void setup() {

  Serial.begin(115200);
  //----- I/O Config -----//
  pinMode(Amotor,OUTPUT);
  pinMode(Bmotor,OUTPUT);
  pinMode(a_encoder, INPUT);
  pinMode(b_encoder, INPUT);
  pinMode(limit_switch_1, INPUT);
  pinMode(encoder_interrupt_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_interrupt_pin), encoder_change, CHANGE);
  pinMode(boot_indicator_LED,OUTPUT);
  digitalWrite(boot_indicator_LED,HIGH);

  //NRF24L01 startup
  wirelessSPI.begin();  //Start the nRF24 module
  wirelessSPI.setChannel(rFChan); //set communication frequency channel
  wirelessSPI.openReadingPipe(1,rAddress);  //This is receiver or master so we need to be ready to read data from transmitters
  wirelessSPI.startListening();    // Start listening for messages 

  //Retrieve the encoder
  new_encoder_state = digitalRead (a_encoder) * 2 + digitalRead (b_encoder); // Convert binary input to decimal valueOut = QEM [old_encoder_state * 4 + new_encoder_state]; 

  //----- PID Timer Configuration -----// 
  //Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
  //Helpful video form GreatScott! : https://www.youtube.com/watch?v=IdL0_ZJ7V2s
  //Note: Using a 3.3V pro mini -> f_clock = 8 MHz
  
  TCCR1A = 0;//Configure timer for normal mode.
  TCCR1B = 0;
  TCNT1 = 45536;// Start value of the timer, calculated by where TCCR1B is the prescaler : TCNT1 = 65536-f_clock/(TCCR1B*f_target)
                // For example if we want the PID evaluated 500 times a second 57536 = 65536-8e6/(1*500).
                // For example if we want the PID evaluated 200 times a second 45536 = 65536-8e6/(1*200).
                // NOTE: We need to leave enough time for the radio to send and recieve data! Do not make the f_target too large!!!
  TCCR1B |= (1<<CS10); //Timer prescaler set to 1.
  TIMSK1 |= (1<<TOIE1); //Enable timer interrupt in overflow mode.
}

//----- PID Time based interrupt ----///
ISR(TIMER1_OVF_vect){ 
    //Q: WHY USE A TIMER?
    //A: A timer based interrupt is needed for the PID because the "I"(Integral) and "D"(Differential) terms are timebased terms.
    //   If this code were in the main loop the time between each calculation will vary. By using a timer the evaluation will occour at exaclty the same time every time.
   
    //Ight, angle values are a float (positive or negative) and the PWM is in a unsigned int. I will need to convert at some point.  
    //Proportional - Proportional difference between the current angle and the desired angle. 
    theta_error = constrain(set_point_theta - output_theta,-max_theta,max_theta) ; 
      
    //Integral - Sum all of the errors.
    // theta_I = constrain(theta_error+theta_I,FLT_MIN,FLT_MAX);
    theta_I = constrain(theta_error+theta_I,-50,50);
     
    
    
    //Dirivative - The difference from the last error and this error.
    theta_d = constrain(theta_error - old_encoder_state_theta_error,-max_theta,max_theta);;
    old_encoder_state_theta_error = theta_error;
    
    //Voltage out calculation.
    output_pwm = constrain(abs(kp*theta_error+ki*theta_I+kd*theta_d),0,pwm_limit);
    
    //direction determiniation
    if (theta_error < 0){
      analogWrite(Amotor, 0);
      analogWrite(Bmotor, output_pwm);
    }
    else{
      analogWrite(Bmotor, 0);
      analogWrite(Amotor, output_pwm);
    }
    
      
   //This dead code is used for serial debugging the PID
    /*display.print(F("Theta_P:"));
    display.println(kp*theta_error);
    
    display.print(F("Theta_I:"));
    display.println(ki*theta_I);
    
    display.print(F("Theta_D:"));
    display.println(kd*theta_d);
    
    display.print(F("PWM:"));
    display.println(constrain(kp*theta_error+ki*theta_I+kd*theta_d,-255,255),HEX);
   */
  // Serial.print("Encoder count:");
  // Serial.println(encoder_count);
  // Serial.print("Theta:");
  // Serial.println(output_theta);
}

void loop() {
  

   if(wirelessSPI.available()){ //Check if recieved data
          wirelessSPI.read(&payload, sizeof(payload)); //read packet of data and store it in struct object
          //Serial.println(payload.theta);
   }

   set_point_theta = payload.joy_x;
   Serial.println(payload.joy_x);
   output_theta = deg_per_pulse*encoder_count;

}

//TODO - timer based home
int home_axis(void){
     analogWrite(Amotor, 0);
     analogWrite(Bmotor, 200);

     uint16_t home_timeout  = 0;
     while(digitalRead(limit_switch_1)==1){
        
     }
     encoder_count = 0;
     analogWrite(Amotor, 0);
     analogWrite(Bmotor, 0);
     set_point_theta = 0;

}

// Every encoder position change calls this interrupt
// The XOR gate on the GT board is connected to this.
// Ignored encoder error detection. This will require hardware alterations outside the scope of this project.
void encoder_change(void){
    //Figure out if the encoder went forward or backward.
    old_encoder_state = new_encoder_state; //Store current encoder state.
    new_encoder_state = digitalRead (a_encoder) * 2 + digitalRead (b_encoder); // Convert binary input to decimal valueOut = QEM [old_encoder_state * 4 + new_encoder_state];
    Out = QEM [old_encoder_state * 4 + new_encoder_state]; //Wuaturature encoder matrix determines either forward or backward rotation.

    encoder_count = encoder_count + Out;

}









  
