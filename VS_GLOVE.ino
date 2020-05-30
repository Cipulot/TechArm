/*
Name:		VS_GLOVE.ino
Project:	Glove part of the project
Sub_p:      Arm controller board
Author:		Luca Seva'
*/

/*BLUETOOTH
//Uses serial with the host computer and serial1 for communication with the Bluetooth module
//  Pins
//  BT VCC to Arduino 5V out. Disconnect before running the sketch
//  BT GND to Arduino GND
//  BT RX (through a voltage divider 1k & 2k) to Arduino TX1 (pin 18)
//  BT TX  to Arduino RX1 (no need voltage divider)   (pin 19)*/

/*MPU6050
A4 and A5 used
//According to the datasheet the axes are arranged depending to the chip package.
//The hand movement are not correlated as the chip one: depending on the module orientation and pitch the axes must be allocated to the "real" axes of the hand.
*/

//THIS IS AN ARDUINO NANO BOARD!!

#include <Wire.h>
#include <SoftwareSerial.h>
#include <MPU6050.h>

//Define pins and initialize a software BT
#define rxP 2
#define txP 3
SoftwareSerial BT(rxP, txP);

//Define a MPU6050 device and relative variables to store data
MPU6050 sensor;
int16_t ax, ay, az, gx, gy, gz;
int16_t moveX, moveY, moveZ, junk;

//Define of buttons pins relative to their function || sent values
#define EMERGENCY_pin 7; //EM 255;RESET 254
#define Open_Close_pin 4; //251
#define LEARN_pin 6; //253
#define LOGGED_pin 5; //252

//General emergency flag
bool EM = false;

//flags for the buttons
bool push1 = false;
bool push2 = false;
bool push3 = false;
bool push4 = false;

// Function to check button status
void check_buttons_state(pin, prev_state){
	if ((digitalRead(pin) == HIGH) && (EM == false)) {
		prev_state = true;
	}
	if ((digitalRead(pin) == LOW) && (prev_state == true)) {
		//Update button specific state for next check
		prev_state = false;

		//Send command string via BT and serial(for debug).
		//The correct string will be returned by a function
		BT.println(string_to_stream(pin));
		Serial.println(string_to_stream(pin));

		//Introduce this delay only if emergency or legged is selected
		//This avoids violent and rapid movements by the robotic arm
		if(pin == EMERGENCY_pin) or(pin == LOGGED_pin):
			delay(100)
	}
}

//Fuction that will return the correct string to send over BT according to button press
string string_to_stream(cmd){
	switch (cmd)
	{
	case EMERGENCY_pin:
		if(EM == false):
			//Update global emergency flag
			EM = true;
			return "255A255X255Y255Z";
		else
		{
			//Update global emergency flag
			EM = false;
			return "254A254X254Y254Z";
		}
		break;

	case Open_Close_pin:
		return "251A251X251Y251Z";
		break;

	case LEARN_pin:
		return "253A253X253Y253Z";
		break;

	case LOGGED_pin:
		return "252A252X252Y252Z";
		break;

	default:
		break;
	}
}

void setup() {
	//Initialize Serial(PC) and Serial1(BT)
	Wire.begin();
	BT.begin(115200);
	Serial.begin(115200);

	//Initialize sensor andcheck for device responce
	Serial.println("Initializing the sensor");
	sensor.initialize();
	Serial.println(sensor.testConnection() ? "Successfully Connected" : "Connection failed");
	delay(1000);
	Serial.println("Taking Values from the sensor");
	//delay used to allow BT modules to connect one to the other..
	delay(1000);
	//NOTE: the connection between the modules is NOT verified by the GLOVE or ARM boards/software[due to different logic voltage]
	// adding a proper voltage divider/level shifter might allow to check if a connection is established 
}

void loop() {
	if (EM == false) { //normal operation

		//get accelerator data
		sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		//junk data to prevent bugs in the BT/Serial com. in th ARM code
		junk = map(ay, -17000, 17000, 0, 180);
		//raw data mapped to servo's min/max degrees[90deg default position]
		moveX = map(ax, -17000, 17000, 0, 180);
		moveY = map(ay, -17000, 17000, 180, 0);
		moveZ = map(az, -17000, 17000, 0, 180);

		//send data with separators, used for data know
		BT.print(junk);
		BT.println("A");
		BT.print(moveX);
		BT.println("X");
		BT.print(moveY);
		BT.println("Y");
		BT.print(moveZ);
		BT.println("Z");

		//IN DEVELOPMENT: add gyro data send function
		//debug serial [mirror of the BT stream]
		Serial.print(junk);
		Serial.print("A");
		Serial.print(moveX);
		Serial.print("X");
		Serial.print(moveY);
		Serial.print("Y");
		Serial.print(moveZ);
		Serial.println("Z");//ln

		//Delay for timing the communication
		delay(115); //100~115 usual
	}

	/*Check if any button of the opCode is pressed and send command*/
	check_buttons_state(EMERGENCY_pin, push1);
	check_buttons_state(Open_Close_pin, push2);
	check_buttons_state(LEARN_pin, push3);
	check_buttons_state(LOGGED_pin, push4);
	/*
	//EMERGENCY
	if ((digitalRead(EMERGENCY) == HIGH)) {//push
		push1 = true;
	}
	if ((digitalRead(EMERGENCY) == LOW) && (push1 == true)) { //release
		push1 = false; //reset flag
					   //check of previous state
		if (EM == false) {
			EM = true;
			BT.println("255A255X255Y255Z");
			Serial.println("255A255X255Y255Z");
			delay(100); //avoid undesired reset
		}
		else {
			//RESET vector
			EM = false;
			BT.println("254A254X254Y254Z");
			Serial.println("254A254X254Y254Z");
			delay(100); //avoid undesired emergency catch
		}
	}

	//Open_Close
	if ((digitalRead(Open_Close) == HIGH) && (EM == false)) {
		push3 = true;
	}
	if ((digitalRead(Open_Close) == LOW) && (push3 == true)) {
		push3 = false;
		BT.println("251A251X251Y251Z");
		Serial.println("251A251X251Y251Z");
		//DEB: add delay???????
	}

	//LEARN
	if ((digitalRead(LEARN) == HIGH) && (EM == false)) {
		push4 = true;
	}
	if ((digitalRead(LEARN) == LOW) && (push4 == true)) {
		push4 = false;
		BT.println("253A253X253Y253Z");
		Serial.println("253A253X253Y253Z");
	}

	//LOGGED DATA
	if ((digitalRead(LOGGED) == HIGH) && (EM == false)) {
		push2 = true;
	}
	if ((digitalRead(LOGGED) == LOW) && (push2 == true)) {
		push2 = false;
		BT.println("252A252X252Y252Z");
		Serial.println("252A252X252Y252Z");
		delay(100);
	}
	*/
}