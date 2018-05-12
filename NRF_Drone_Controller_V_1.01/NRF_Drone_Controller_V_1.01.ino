////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////NRF Drone Controller V 1.01////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////By Samannoy Ghosh/////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////Version:1.01////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This code has been obtained in bits from different other systems based on nrf and created to make it suitable for this
//purpose


////////Details: Nrf24 based Transmitter for Microquadcopter using Joystick Module and Arduino//////////////////////////

/*
Pin Assignment:
D6 Mode Switch
D7 Joystick Right Push Button
D8 NRF24 CE
D9 
D10 NRF24 CSN
D11 NRF24 MOSI
D12 NRF24 MISO
D13 NRF24 SCK
A0 Joystick Left X Axis
A1 Joystick Left Y Axis
A2 Joystick Right X Axis
A3 Joystick Right Y Axis
*/

// Joysticks Pin Definition for simplicity
#define JOYSTICK_LEFT_X A0
#define JOYSTICK_LEFT_Y A1
#define JOYSTICK_RIGHT_X A2
#define JOYSTICK_RIGHT_Y A3

// Joystick push buttons
//#define JOYSTICK_BUTTON_LEFT 5       //Remove Comment to add left joystick button
#define JOYSTICK_BUTTON_RIGHT 7
#define MODE_SWITCH 6

// Define variables
int Right_Joy_X = 0x00;
int Right_Joy_Y = 0x00;
int Left_Joy_X = 0x00;
int Left_Joy_Y = 0x00;
int Right_Joy_X_Old = 0x00;
int Right_Joy_Y_Old = 0x00;
int Left_Joy_X_Old = 0x00;
int Left_Joy_Y_Old = 0x00;
int Right_Joy_Button = LOW;
//int Left_Joy_Button = LOW;            //Remove Comment to add left joystick button
int Flight_mode_Button = LOW;           // LOW indicates Stabilise Mode, HIGH indicates AltHold Mode
int Lights = 0;

// Include Libraries
#include <SPI.h>
#include <RH_NRF24.h>

RH_NRF24 nrf24; // Create NRF24 object

uint8_t command[11]; // Create array for data to be sent

uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN]; // Create an array buffer for the NRF24 module

void setup()
{
 //comment out the button if needed
//pinMode(JOYSTICK_BUTTON_LEFT, INPUT_PULLUP);   // Left joystick button
pinMode(JOYSTICK_BUTTON_RIGHT, INPUT_PULLUP);  // Right joystick button
pinMode(JOYSTICK_LEFT_X, INPUT);
pinMode(JOYSTICK_LEFT_Y, INPUT);
pinMode(JOYSTICK_RIGHT_X, INPUT);
pinMode(JOYSTICK_RIGHT_Y, INPUT);
pinMode(MODE_SWITCH, INPUT);




// initialize the radio module
nrf24.init();
nrf24.setChannel(1);
nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm); 

command[0] = 0x4A;
command[1] = 0x44;
command[2] = 0x38;
command[3] = 0x33;
command[4] = 0x06;
//command[5] = 0x01;

}

void loop() {
Check_Vehicle_ID();
Check_Buttons();
Check_Mode_Switch();
Check_joysticks(); 
}

void Check_Vehicle_ID(){ 

command[0] = 0x4A; // J
command[1] = 0x44; // D
command[2] = 0x38; // 8
command[3] = 0x33; // 3

}

void Check_Buttons(){

// Check Right Joystick Push Button
Right_Joy_Button = digitalRead(7);

if (Right_Joy_Button==LOW)
{

if (Lights == 0){
Lights = 1;
command[10]=1;
}
else if (Lights == 1){
Lights = 0;
command[10]=0;
}

// Wait for switch to be released
while (Right_Joy_Button == LOW)
{
Right_Joy_Button = digitalRead(7);
}

Send_Data();

}

}

//Check Joystick readings and transmit the data
// check the following function and change the mapping values if required
// the data transmitted must lie between 0 and 1023
// unless if nrf is unable to transmit data beyond 0xFF
// then keep the following
void Check_joysticks(){
Right_Joy_X = analogRead(A2);
Right_Joy_X= map(Right_Joy_X, 0, 1024, 0x00, 0xFF);  // mapped values maybe 0x00 and 0xFF
Right_Joy_Y = analogRead(A3);
Right_Joy_Y= map(Right_Joy_Y, 0, 1024, 0x00, 0xFF);  // mapped values maybe 0x00 and 0xFF
Left_Joy_X = analogRead(A0);
Left_Joy_X= map(Left_Joy_X, 0, 1024, 0xFF, 0x00);    // mapped values maybe 0x00 and 0xFF
Left_Joy_Y = analogRead(A1);
Left_Joy_Y= map(Left_Joy_Y, 0, 1024, 0xFF, 0x00);    // mapped values maybe 0x00 and 0xFF

// Only send data if something has changed, to try and minimise interference
if((Right_Joy_X != Right_Joy_X_Old || Right_Joy_Y != Right_Joy_Y_Old) || (Left_Joy_X != Left_Joy_X_Old || Left_Joy_Y != Left_Joy_Y_Old))
{
command[5]=Right_Joy_X;   // Ailerons 
command[6]=Right_Joy_Y;   // Elevator
command[8]=Left_Joy_X;    // Rudder
command[7]=Left_Joy_Y;    // Throttle

// Send the data
Send_Data();

// Store the sent values
Right_Joy_X_Old=Right_Joy_X;
Right_Joy_Y_Old=Right_Joy_Y;
Left_Joy_X_Old=Left_Joy_X;
Left_Joy_Y_Old=Left_Joy_Y;
}

}

//Function to implement different flight modes, like stabilise or AltHold
void Check_Mode_Switch()
{
// Check Flight_mode Button
Flight_mode_Button = digitalRead(6);

if (Flight_mode_Button==LOW)
  {
     command[9]=0;   // 0 signifies stabilise mode
  }
else if (Flight_mode_Button == HIGH)
  {
    command[9]=1;   // 1 signifies AltHold Mode
  }

Send_Data();
}


// Function to send the data
void Send_Data(){
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
nrf24.send(command, sizeof(command));
nrf24.waitPacketSent(); 
}
