// Libraries for the Radio
#include <SPI.h> //Call SPI library so you can communicate with the nRF24L01+
#include <nRF24L01.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/
#include <RF24.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/
#include <NintendoExtensionCtrl.h>

Nunchuk nchuk;

// Libraries for the temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 7 //Temperature sensor data wire is plugged into port 7 on the Arduino *4.7k pullup resistor*

const uint8_t pinCE = 8; //This pin is used to set the nRF24 to standby (0) or active mode (1)
const uint8_t pinCSN = 10; //This pin is used to tell the nRF24 whether the SPI communication is a command
RF24 wirelessSPI(pinCE, pinCSN); // Declare object from nRF24 library (Create your wireless SPI) 
const uint64_t wAddress = 0xB00B1E50C3LL;  //Create pipe address to send data, the "LL" is for LongLong type
const uint8_t rFChan = 89; //Set channel default (chan 84 is 2.484GHz to 2.489GHz)
const uint8_t rDelay = 7; //this is based on 250us increments, 0 is 250us so 7 is 2 ms
const uint8_t rNum = 5; //number of retries that will be attempted 
const uint8_t chan1 = 2; //D2 pin for node channel check
const uint8_t chan2 = 3; //D3 pin for node channel check

const uint8_t chan3 = 4; //D4 pin for node channel check

//stuct of payload to send fake sensor data and node channel
struct PayLoad {
  uint8_t chan;
  int joy_x;
};
float temp;
int incomingByte = 0; // for incoming serial data

PayLoad payload; //create struct object

void setup() {
  Serial.begin(115200);
  pinMode(chan1,INPUT_PULLUP); //set channel select digital pins to input pullup
  pinMode(chan2,INPUT_PULLUP);
  pinMode(chan3,INPUT_PULLUP);
  wirelessSPI.begin();  //Start the nRF24 module
  wirelessSPI.setChannel(rFChan); 
  wirelessSPI.setRetries(rDelay,rNum); //if a transmit fails to reach receiver (no ack packet) then this sets retry attempts and delay between retries   
  wirelessSPI.openWritingPipe(wAddress); //open writing or transmit pipe

  wirelessSPI.stopListening(); //go into transmit mode
  setChannel();  //checks current channel setting for transceiver 

  payload.chan=0;


    while (!nchuk.connect()) {
      Serial.println("Nunchuk not detected!");
      delay(1000);
    }
  }

  void loop() {
  boolean success = nchuk.update();  // Get new data from the controller

  if (success == true) {  // We've got data!
    //nchuk.printDebug();  // Print all of the values!
  }
  else {  // Data is bad :(
    Serial.println("Controller Disconnected!");
    delay(1000);
    nchuk.connect();
  }

  payload.joy_x = nchuk.joyX();
//  payload.joy_y = nchuk.joyY();
  Serial.println(payload.chan);


  int zButton = nchuk.buttonZ();
  Serial.println(zButton);
  if (nchuk.buttonZ()) {
      payload.chan++;
       Serial.println(payload.chan);

      if(payload.chan >= 4){
        payload.chan = 0;
      }
      Serial.println(nchuk.buttonZ());     
      delay(250); //debounce
   }
  
 
  if (!wirelessSPI.write(&payload, sizeof(payload))){  //send data and remember it will retry if it fails
    delay(random(5,10)); //as another back up, delay for a random amount of time and try again
    if (!wirelessSPI.write(&payload, sizeof(payload))){
      //set error flag if it fails again
      Serial.println("NO RADIO");
    }
  }
}

//check for low digital pin to set node address
void setChannel() {
  if(!digitalRead(chan1)) payload.chan = 2;
  else if(!digitalRead(chan2)) payload.chan = 1;
  else if(!digitalRead(chan3)) payload.chan = 3;
  else payload.chan = 0;
}



// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
