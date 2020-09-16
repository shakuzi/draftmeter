#include <WiFi.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>
#include "BluetoothSerial.h"

HardwareSerial MySerial(1);
BluetoothSerial SerialBT;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


float battery;
int sayac=0;
//************************ YENİ ADC ********************************************************************************
float aBattery,aDistance;
char bBattery[10],bDistance[10];
int distance;
uint8_t distanceBle[5] = {};
char dist_buffer[9];
char BufAci[3];
char bleReadStr[100];
int Mesafe, Mesafe0,m=1,n,t=1;

unsigned long counter;
int bleReadCount;
int say=0;

/**
 * Made with Marlin Bitmap Converter
 * https://marlinfw.org/tools/u8glib/converter.html
 *
 * This bitmap from the file 'medeci_logo-90.png'
 */
#pragma once
#define B1KE9D_BMPWIDTH 90
const unsigned char medeci_logo_full[] PROGMEM = {
  B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
  B00001111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111000,B00000000,
  B00011111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111100,B00000000,
  B00111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111110,B00000000,
  B00111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111110,B00000000,
  B00111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111110,B00000000,
  B00111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111110,B00000000,
  B00111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B00111111,B11111110,B00000000,
  B00111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B00111111,B11111110,B00000000,
  B00111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111110,B01111111,B11111111,B11111110,B00000000,
  B00111111,B11110011,B11111101,B11100000,B00111000,B10000111,B11000000,B11110001,B10001110,B00111111,B11111110,B00000000,
  B00111111,B11110011,B11111001,B11110011,B10111001,B11110001,B11001111,B11100111,B11101111,B00111111,B11111110,B00000000,
  B00111111,B11110001,B11111001,B11110011,B11111001,B11111001,B11001111,B11001111,B11111111,B00111111,B11111110,B00000000,
  B00111111,B11110001,B11110001,B11110011,B11111001,B11111100,B11001111,B11001111,B11111111,B00111111,B11111110,B00000000,
  B00111111,B11110100,B11110101,B11110011,B01111001,B11111100,B11000100,B11001111,B11111111,B00111111,B11111110,B00000000,
  B00111111,B11110100,B11101101,B11110011,B01111001,B11111100,B11001101,B11001111,B11111111,B00111111,B11111110,B00000000,
  B00111111,B11110110,B01101100,B11110011,B11111001,B11111100,B11001111,B11001111,B11111111,B00111111,B11111110,B00000000,
  B00111111,B11110110,B01001100,B11110011,B11111001,B11111101,B11001111,B11001111,B00111111,B00111111,B00111110,B00000000,
  B00111111,B11110111,B00011100,B11110011,B11111001,B11111001,B11001111,B11100111,B10111111,B00111111,B01111110,B00000000,
  B00111111,B11100111,B00011100,B11110011,B10111001,B11110011,B11001110,B11100011,B11001111,B00111110,B11111110,B00000000,
  B00111111,B11100111,B10111000,B01100000,B00110000,B00001111,B10000000,B11111000,B00100111,B00111001,B11111110,B00000000,
  B00111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111000,B00000111,B11111110,B00000000,
  B00111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111100,B00001111,B11111110,B00000000,
  B00111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B00111111,B11111100,B00000000,
  B00011111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111100,B00000000,
  B00011111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111000,B00000000,
  B00001111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11110000,B00000000,
  B00000111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11100000,B00000000,
  B00000011,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11000000,B00000000,
  B00000001,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B10000000,B00000000,
  B00000000,B01111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111110,B00000000,B00000000,
  B00000000,B00000000,B00000000,B00000000,B00000011,B11111111,B11110000,B00000000,B00000000,B00000000,B00000000,B00000000,
  B00000000,B00000000,B00000000,B00000000,B00000000,B01111111,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
  B00000000,B00000000,B00000000,B00000000,B00000000,B00011100,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
  B00000000,B00000000,B00000000,B00000000,B00000000,B00001000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,
  B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000
};

static void tft_setup(void)
{
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  delay(100); // Pause for 2 seconds
  
  // Clear the buffer
  display.clearDisplay();
  //display.clearDisplay();
  display.drawBitmap(20, 20, medeci_logo_full, 90, 35, WHITE);
  display.display();
  
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2); // Draw 2X-scale text
  display.setCursor(3, 0);
  display.println(F("e-distance"));
  display.display();
  delay(2500);
  
  
  /*display.setTextColor(SSD1306_WHITE);
  display.setCursor(25, 0);
  display.println(F("MEDECI"));
  display.display();
  delay(1000);*/
  
}

static void tft_write(void)
{
  display.clearDisplay(); // Clear the display buffer
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15, 0);
  display.println(F("DISTANCE"));
  if((distance == 9999) || (distance == 500))
  {
    display.setTextSize(2); // Draw 2X-scale text
    display.setCursor(0, 25);
    display.print("Out Of");
    display.setCursor(0, 45);
    display.print("Range");
    display.display();
  }
  else
  {
    display.setTextSize(4); // Draw 2X-scale text
    display.setCursor(0, 25);
    display.print(F(bDistance));
    display.setTextSize(2); // Draw 2X-scale text
    display.setCursor(100, 40);
    display.println(F("mm"));
    display.display();
  }
  
}

static void bluetooth_setup(void)
{
  SerialBT.begin("edistance"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

static void measurement(void)
{  
  range_measure();
  range_measure_analog();
  //battery_measure();
  /*aBattery=map(analogRead(36), 2533, 3815, 0, 100); //Serial.print(aBattery);  TODO */
  //dtostrf(aBattery,4,0,bBattery);
  dtostrf(aDistance,4,0,bDistance);
}
static void battery_measure()
{

}

static void range_measure_analog(void)
{
  int range_analog;
  int range_analog_distance;
  range_analog = analogRead(32);
  range_analog_distance = (range_analog * 10) / 3;
  //Serial.print("range_analog: ");
  Serial.println(range_analog_distance);
}
static void range_measure(void)
{
    if(MySerial.available() > 0)
    {
      MySerial.readBytes(dist_buffer, 10);
      for(int i=0; i<=9; i++)
      {
        if(dist_buffer[i]=='R')
        {
          n=i;
          break;
        }
      };
      for(int i=0; i<=3; i++) 
      {
        BufAci[i]=dist_buffer[n+i+1];
      }
      BufAci[4]=0;
      aDistance=0; 
      aDistance=atoi(BufAci);
      distance =atoi(BufAci);
      MySerial.flush();
    }
}

static void serial_write(void)
{
   Serial.print("D:"); Serial.print(bDistance); Serial.println(" ");
}

static void bluetooth_write(void)
{
  distanceBle[0] = distance/1000;
  distanceBle[1] = (distance % 1000)/100;
  distanceBle[2] = (distance % 100)/10;
  distanceBle[3] = (distance % 10);
  distanceBle[4] = 255;
  //SerialBT.write(distanceBle,5);
  if((distance == 9999) || (distance == 500))
  {
    SerialBT.println("Out Of Range");
  }
  else
  {
    SerialBT.println(distance);
  }
}

/*Bluetooth Communication 
 * First BYTE: 0xED
 * Second BYTE: Device ID
 * Third BYTE: Device ID
 * Fourth BYTE: Function Name
 * Fifth BYTE: Function/Return Value
 * Sixth BYTE: Function/Return Value
 * Seventh BYTE: Checksome Value = (2nd Byte + 3rd + 4th + 5th + 6th) % 256
 * Eighth BYTE: 'q' --> 0x71
 * 
 * Function Names                 Function Value                  Return Val
 * 0x01 Start/Stop Measurement    Start: 0x0001,                  0xA001
 *                                Stop: 0x0000,                   0xA000
 *                                Standby: 0x0101                 0xA101
 * 0x02 Range Value               0x0001                          0x0000 - 0x270F
 * 0x03 Battery Value             0x0001                          0x0000 - 0x0064
 * 0x04 Take OffSet               0x0001                          Offset Value
 * 0x05 Send Last Save Data       0x0001                          10Byte saved Data.
 * 0x05 Send Device ID            0x0001                          Device ID
 * 0x06 Send Software ID          0x0001                          Software ID
 * 0x07 Are You Alive?            0x0001                          0x0001 or 0x0000
 * 
 * 
 * 
*/
void bluetooth_read()
{
  if(SerialBT.available())
    {
      char bleRead = SerialBT.read();
      if(bleRead != 'q')
      {
        bleReadStr[bleReadCount] = bleRead;
        bleReadCount++;
      }
      else
      {
        Serial.println(bleReadCount);
        Serial.println((bleReadStr));
        for(int i = 0 ;i<100; i++)
        {
          bleReadStr[0] = '\0';
        }
        bleReadCount = 0;
      }
    }
}

void setup()
{
  Serial.begin(115200);
  MySerial.begin(9600, SERIAL_8N1, 26, 27);
  bluetooth_setup();
  tft_setup();
}
void loop()
{
    measurement(); 
    tft_write();
    bluetooth_write();
    bluetooth_read();
    
}
