/*
 Name:      Tech Arm
 File Name: VS_ARM.ino
 Project:	Robotic Arm controlled via gyroscope
 Sub_p:     Arm controller board
 Author:	Luca Seva'
*/
/*BLUETOOTH
//Uses serial with the host computer and serial1 for communication with the Bluetooth module
//  Pins
//  BT VCC to Arduino 5V out. Disconnect before running the sketch
//  BT GND to Arduino GND
//  BT RX (through a voltage divider 1k & 2k) to Arduino TX1 (pin 18)
//  BT TX  to Arduino RX1 (no need voltage divider)   (pin 19)*/
/*MPU6050
//According to the datasheet the axes are arranged depending to the chip package.
//The hand movement are not correlated as the chip one: depending on the module orientation and pitch the axes must be allocated to the "real" axes of the hand.
*/
/*SD CARD READER
//The SD card functionality is used to show a pre made "*.bmp" file with a resolution of 320x240. The card slot is located on the display module.
//Due to differences bethween the UNO and MEGA boards (different mapping of the SPI Bus on the pins) a modification of the Adafruit SD library is needed.
//Follow this link to the mods instructions: http://tech.memoryimprintstudio.com/access-sd-cards-from-tft-lcd-shield-using-arduino-mega-2560-soft-spi/
//Note that those mods allow the software to remain untouched.
*/

//THE CODE WILL BE EXECUTED ON AN ARDUINO MEGA 2560

#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <Elegoo_GFX.h>
#include <Elegoo_TFTLCD.h>
#include <TouchScreen.h>
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include <eeprom.h>

//COLORS
#define BLACK 0x0000		//   0,   0,   0
#define NAVY 0x000F 		//   0,   0, 128
#define DARKGREEN 0x03E0	//   0, 128,   0
#define DARKCYAN 0x03EF 	//   0, 128, 128
#define MAROON 0x7800		// 128,   0,   0
#define PURPLE 0x780F		// 128,   0, 128
#define OLIVE 0x7BE0		// 128, 128,   0
#define LIGHTGREY 0xC618	// 192, 192, 192
#define DARKGREY 0x7BEF		// 128, 128, 128
#define BLUE 0x001F 		//   0,   0, 255
#define GREEN 0x07E0		//   0, 255,   0
#define CYAN 0x07FF 		//   0, 255, 255
#define RED 0xF800  		// 255,   0,   0
#define MAGENTA 0xF81F		// 255,   0, 255
#define YELLOW 0xFFE0		// 255, 255,   0
#define WHITE 0xFFFF		// 255, 255, 255
#define ORANGE 0xFD20		// 255, 165,   0
#define GREENYELLOW 0xAFE5  // 173, 255,  47
#define PINK 0xF81F

//Gui elements bound
//STATUS_BOX
#define SB_X 0
#define SB_Y 0
#define SB_W 80
#define SB_H 80
#define SB_TSIZE 2
//CONNECTION_BOX
#define CN_X 0
#define CN_Y 80
#define CN_W 80
#define CN_H 80
#define CN_TSIZE 2
//RESET_BOX
#define RS_X 0
#define RS_Y 160
#define RS_W 80
#define RS_H 80
#define RS_TSIZE 2

//DISPLAY PINS AND DATA
//PINS
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4

//DISPLAY objects declaration
Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
//Boot image propreties
#define MAX_BMP 10		// bmp file num
#define FILENAME_LEN 20 // max file name length
#define BUFFPIXEL 80	//60 DIVISOR OF 240
#define BUFFPIXEL_X3 240
const int __Gnbmp_height = 320;
const int __Gnbmp_width = 240;
unsigned char __Gnbmp_image_offset = 0;
unsigned char i = 0;
// declare files tree array for the TFT library [number of files][lenght of filename] = {file1, file2 , etc.}
char __Gsbmp_files[1][FILENAME_LEN] = {"boot.bmp"};
//Declare File type objects
File bmpFile;	//Boot image file = boot.bmp
File learndata; //File containing the learned data = learndata.txt |||||||learndata.close() at the end of the process

//SERVO PINS AND VARIABLES
#define servo1_pin 46 //Base
#define servo2_pin 44 //Motor_1
#define servo3_pin 45 //Motor_2
#define servo4_pin 10 //Motor_3
#define servo5_pin 11 //Clamp
#define torch_pin 2
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo Clamp;
int moveX, moveY, moveZ;
String receive, receiveA, receiveX, receiveY, receiveZ;

//Global flags and variables
#define BT_pin 23		//BT state pin
#define demo_pin 27		//demo mode pin
bool EM = false;		//EMERGENCY state
bool BT = false;		//Bluetooth flag (NOTE: used in prev. internal release. Not in use now)
bool learn = false;		//LEARNING flag
bool O_C_State = false; //End effector state flag (open/closed aka on/off)
int lines = 0;			//number of lines logged in the sd card file

//Function that draws bmp images to the screen
void bmpdraw(File f, int x, int y)
{
	bmpFile.seek(__Gnbmp_image_offset);
	uint32_t time = millis();
	uint8_t sdbuffer[BUFFPIXEL_X3]; // 3 * pixels to buffer
	for (int i = 0; i < 320; i++)
	{
		for (int j = 0; j < (240 / BUFFPIXEL); j++)
		{
			bmpFile.read(sdbuffer, BUFFPIXEL_X3);
			uint8_t buffidx = 0;
			int offset_x = j * BUFFPIXEL;
			unsigned int __color[BUFFPIXEL];
			for (int k = 0; k < BUFFPIXEL; k++)
			{
				__color[k] = sdbuffer[buffidx + 2] >> 3;					 // read
				__color[k] = __color[k] << 6 | (sdbuffer[buffidx + 1] >> 2); // green
				__color[k] = __color[k] << 5 | (sdbuffer[buffidx + 0] >> 3); // blue
				buffidx += 3;
			}
			for (int m = 0; m < BUFFPIXEL; m++)
			{
				tft.drawPixel(m + offset_x, i, __color[m]);
			}
		}
	}
}

//Function to read bmp file headers
boolean bmpReadHeader(File f)
{
	// read header
	uint32_t tmp;
	uint8_t bmpDepth;
	if (read16(f) != 0x4D42)
	{
		// magic bytes missing
		return false;
	}
	// read file size
	tmp = read32(f);
	// read and ignore creator bytes
	read32(f);
	__Gnbmp_image_offset = read32(f);
	// read DIB header
	tmp = read32(f);
	int bmp_width = read32(f);
	int bmp_height = read32(f);
	if (bmp_width != __Gnbmp_width || bmp_height != __Gnbmp_height)
	{ // if image is not 320x240, return false
		return false;
	}
	if (read16(f) != 1)
		return false;

	bmpDepth = read16(f);

	if (read32(f) != 0)
	{
		// compression not supported!
		return false;
	}
	return true;
}

//Read 16BPP
uint16_t read16(File f)
{
	uint16_t d;
	uint8_t b;
	b = f.read();
	d = f.read();
	d <<= 8;
	d |= b; ////d = d | b
	return d;
}

//Read 32BPP
uint32_t read32(File f)
{
	uint32_t d;
	uint16_t b;
	b = read16(f);
	d = read16(f);
	d <<= 16;
	d |= b; //d = d | b
	return d;
}

//Function that prints the boot image on the screen
void TFTBOOTSEQUENCE()
{
	bmpFile = SD.open(__Gsbmp_files[i]);
	//Check if the expected file exist otherwise
	if (!bmpReadHeader(bmpFile))
	{
		Serial.println("bad bmp");
		tft.setTextColor(WHITE);
		tft.setTextSize(1);
		tft.println("bad bmp");
		return;
	}
	bmpdraw(bmpFile, 0, 0);
	bmpFile.close();
	delay(2000);
}

//Function to draw the standard state UI for the display
void TFTNORMAL()
{
	tft.fillRect(SB_X, SB_Y, SB_W, SB_H, GREEN);
	tft.fillRect(CN_X, CN_Y, CN_W, CN_H, CYAN);
	tft.drawRect(SB_X, SB_Y, SB_W, SB_H, BLACK);
	tft.drawRect(CN_X, CN_Y, CN_W, CN_H, BLACK);
	//Text for layout
	tft.setCursor(SB_X + 2, SB_Y + 5);
	tft.setTextColor(BLACK);
	tft.setTextSize(SB_TSIZE);
	tft.print("STATE");
	tft.setCursor(CN_X + 2, CN_Y + 5);
	tft.setTextColor(BLACK);
	tft.setTextSize(CN_TSIZE);
	tft.print("BT");
	tft.setCursor(SB_X + 5, 40);
	tft.setTextColor(BLACK);
	tft.setTextSize(CN_TSIZE);
	tft.print("NORMAL");
	//text for BT connection
	if (digitalRead(BT_pin) == HIGH)
	{
		tft.setCursor(25, 120);
		tft.setTextColor(BLACK);
		tft.setTextSize(CN_TSIZE);
		tft.print("OK");
	}
	else
	{
		tft.setCursor(10, 120);
		tft.setTextColor(BLACK);
		tft.setTextSize(CN_TSIZE);
		tft.print("ERROR");
	}
}

//Function to draw the emergency state UI for the display
void TFTEMERGENCY()
{
	tft.fillRect(SB_X, SB_Y, SB_W, SB_H, RED);
	tft.drawRect(SB_X, SB_Y, SB_W, SB_H, BLACK);
	//Text for layout
	tft.setCursor(SB_X + 2, SB_Y + 5);
	tft.setTextColor(BLACK);
	tft.setTextSize(SB_TSIZE);
	tft.print("STATE");
	tft.setCursor(SB_X + 25, 40);
	tft.setTextColor(BLACK);
	tft.setTextSize(CN_TSIZE);
	tft.print("EM");
}

//Function to draw the learning state UI for the display
void TFTLEARN()
{
	tft.fillRect(SB_X, SB_Y, SB_W, SB_H, GREEN);
	tft.drawRect(SB_X, SB_Y, SB_W, SB_H, BLACK);
	//Text for layout
	tft.setCursor(SB_X + 2, SB_Y + 5);
	tft.setTextColor(BLACK);
	tft.setTextSize(SB_TSIZE);
	tft.print("STATE");
	tft.setCursor(SB_X + 10, 40);
	tft.setTextColor(BLACK);
	tft.setTextSize(CN_TSIZE);
	tft.print("LEARN");
}

//Function to draw the logged data state UI for the display
void TFTLOGGED()
{
	tft.fillRect(SB_X, SB_Y, SB_W, SB_H, GREEN);
	tft.drawRect(SB_X, SB_Y, SB_W, SB_H, BLACK);
	//Text for layout
	tft.setCursor(SB_X + 2, SB_Y + 5);
	tft.setTextColor(BLACK);
	tft.setTextSize(SB_TSIZE);
	tft.print("STATE");
	tft.setCursor(SB_X + 20, 40);
	tft.setTextColor(BLACK);
	tft.setTextSize(CN_TSIZE);
	tft.print("LOG");
}

//Function that updates the display with motor angle position
void draw_motor_angle()
{
	//base
	tft.setCursor(265, 217);
	tft.setTextColor(WHITE, BLACK);
	tft.setTextSize(1.5);
	tft.print(String(" ") + moveY + String(" ")); //the space is needed to avoid the persistence of a number when it change from 3 digit to 2 digit.

	//motor_1
	tft.setCursor(265, 125);
	tft.setTextColor(WHITE, BLACK);
	tft.setTextSize(1.5);
	tft.print(String(" ") + moveX + String(" "));

	//motor_2
	tft.setCursor(110, 35);
	tft.setTextColor(WHITE, BLACK);
	tft.setTextSize(1.5);
	tft.print(String(" ") + moveX + String(" "));

	//Serial for debug
	Serial.print(moveX);
	Serial.print(" ");
	Serial.print(moveY);
	Serial.print(" ");
	Serial.println(moveZ);
}

//Function that handles the control of the end effector of the arm
void end_effector_control()
{
	if (O_C_State == false)
	{
		O_C_State = true;
		Clamp.write(35);
	}
	else
	{
		O_C_State = false;
		Clamp.write(100);
	}
}

//Function that handles servo motion cmd
void servo_move()
{
	servo1.write(moveX); //motor_1
	servo2.write(moveY); //base
	if (moveX < 100)
	{
		servo3.write(moveX); //motor_2
		if (moveX > 70)
		{
			servo4.write(moveX); //motor_3
		}
	}
}

//The usual setup function
void setup()
{
	Serial.begin(115200);  //PC
	Serial1.begin(115200); //BT
	//ID of tft
	uint16_t ID = tft.readID();
	tft.reset();
	tft.begin(ID);
	tft.fillScreen(BLACK);

	//Init SD_Card
	pinMode(SS, OUTPUT);
	if (!SD.begin(SS))
	{
		Serial.println("Initialization failed!");
		tft.setCursor(0, 0);
		tft.setTextColor(WHITE);
		tft.setTextSize(1);
		tft.println("SD Card Init fail.");
	}
	else
		Serial.println("Initialization done.");

	//TFT Boot sequence
	TFTBOOTSEQUENCE();

	pinMode(demo_pin, INPUT);
	pinMode(BT_pin, INPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	//Attach declared motors
	servo1.attach(servo1_pin);
	servo2.attach(servo2_pin);
	servo3.attach(servo3_pin);
	servo4.attach(servo4_pin);
	Clamp.attach(servo5_pin);
	Clamp.write(100); //Put clamp in open position
	Wire.begin();
	tft.setRotation(3);
	tft.fillRect(0, 0, 120, 320, BLACK); //delete only hand draw

	//Draw ui
	TFTNORMAL();

	//Check for demo mode
	if (digitalRead(demo_pin) == HIGH)
	{ //demo mode engaged, play recorded movements on file until powerdown
		do
		{
			learndata = SD.open("demo.txt", FILE_READ); //Open demo movements file
			for (int a = 0; a < 278; a++)
			{ //write moves to servos for each line (fixed)
				receiveA = learndata.readStringUntil('A');
				receiveX = learndata.readStringUntil('X');
				receiveY = learndata.readStringUntil('Y');
				receiveZ = learndata.readStringUntil('Z');
				moveX = receiveX.toInt();
				moveY = receiveY.toInt();
				moveZ = receiveZ.toInt();
				//check clamp
				if ((moveX == 251) && (moveY == 251) && (moveZ == 251))
				{
					end_effector_control()
				}
				else if (moveX < 200)
				{
					servo_move()
				}
				delay(115);
			}
			learndata.close();
		} while (1);
	}
}

//The usual loop()
void loop()
{
	//check if any data is readable in the Serial1[buffer]
	if (Serial1.available())
	{
		//read and substring division process
		//receive = Serial1.read();
		receiveA = Serial1.readStringUntil('A');
		receiveX = Serial1.readStringUntil('X');
		receiveY = Serial1.readStringUntil('Y');
		receiveZ = Serial1.readStringUntil('Z');
		moveX = receiveX.toInt();
		moveY = receiveY.toInt();
		moveZ = receiveZ.toInt();

		//Draw updated position values to screen
		draw_motor_angle()

			//EMERGENCY STATE
			if ((EM == false) && (moveX == 255) && (moveY == 255) && (moveZ == 255))
		{
			EM = true;
			Serial.println("EMERGENCY");
			TFTEMERGENCY();
			delay(100);
		}

		//RESET [BT]
		else if ((EM == true) && (moveX == 254) && (moveY == 254) && (moveZ == 254))
		{
			EM = false;
			Serial.println("RESET");
			TFTNORMAL();
			delay(100);
		}

		//LEARN
		else if ((EM == false) && (moveX == 253) && (moveY == 253) && (moveZ == 253))
		{
			if (learn == false)
			{
				learn = true;
				lines = 0;									 //reset the number of lines
				learndata = SD.open("data.txt", FILE_WRITE); //open file for datalog
				learndata.seek(0);							 //learndata.seek(0) is important for the correct data reading
				TFTLEARN();
			}
			else
			{
				learn = false;
				learndata.close(); //close file for datalog
				TFTNORMAL();
			}
		}

		//Datalog routine
		//This will intercept the position values and write the data to file
		if (learn == true)
		{
			learndata.print(moveX);
			learndata.print("A");
			learndata.print(moveX);
			learndata.print("X");
			learndata.print(moveY);
			learndata.print("Y");
			learndata.print(moveZ);
			learndata.println("Z");
			lines++; //increment the number of lines
			//check clamp
			if ((moveX == 251) && (moveY == 251) && (moveZ == 251))
			{
				end_effector_control()
			}
			else
			{
				servo_move()
			}
		}

		//LOGGED
		else if ((EM == false) && (learn == false) && (moveX == 252) && (moveY == 252) && (moveZ == 252))
		{
			//use data from the sd card to move the motors
			//esegue tutte le opeazioni e poi esce
			//do this in a while but first chect if there are any data
			TFTLOGGED();
			learndata = SD.open("data.txt", FILE_READ);
			for (int j = 0; j < lines; j++)
			{ //write moves to servos for each line in the txt file
				receiveA = learndata.readStringUntil('A');
				receiveX = learndata.readStringUntil('X');
				receiveY = learndata.readStringUntil('Y');
				receiveZ = learndata.readStringUntil('Z');
				moveX = receiveX.toInt();
				moveY = receiveY.toInt();
				moveZ = receiveZ.toInt();
				//check clamp
				if ((moveX == 251) && (moveY == 251) && (moveZ == 251))
				{
					end_effector_control()
				}
				else if (moveX < 200)
				{
					servo_move()
				}
				delay(115);
			}
			learndata.close();
			TFTNORMAL();
		}

		//OPEN CLOSE
		else if ((EM == false) && (moveX == 251) && (moveY == 251) && (moveZ == 251))
		{
			end_effector_control()
		}

		//MOVE ROUTINE MIRROR
		//NOTE: due to speed and sync problem the move function MUST be added at the end of the code to avoid more checks in the function...
		else if ((EM == false) && (learn == false))
		{
			servo_move()
		}
	}
}