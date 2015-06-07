#include <avr/wdt.h>        // Include this to handle watchdog timer

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>

#include <Plot.h>

#define NO_SERIAL_LOG

// For the breakout, you can use any 2 or 3 pins
// These pins will also work for the 1.8" TFT shield
#define TFT_CS     10
#define TFT_RST    0  // you can also connect this to the Arduino reset
// in which case, set this #define pin to 0!
#define TFT_DC     9


// Option 1 (recommended): must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
Adafruit_ST7735 adafruit = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// Option 2: use any pins but a little slower!
#define TFT_SCLK 13   // set these to be whatever pins you like!
#define TFT_MOSI 11   // set these to be whatever pins you like!
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);



// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 12
#define TEMPERATURE_UNIT 0.0625

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

DeviceAddress deviceAddress; // We'll use this variable to store a found device address


int MinTempValue = 0;
int MaxTempValue = 0x07D0;

#define numberOfDevices 3

#if defined(SERIAL_LOG)
#define maxMeasurements 50
#else
#define maxMeasurements 100
#endif
class Matrix2 : public Matrix
{
  int values[maxMeasurements + 1][numberOfDevices];
  public:
    int valueAt (int x, int y) {
      if (y > numberOfDevices)
      {
        logger("y get overflow");
        logger(y);
        delay(10000);        
      }
      if (x > maxMeasurements)
      {
        logger("x get overflow");
        logger(x);
        delay(10000);        
      }
        return values[x][y];

    }
    float labelAt (int x, int y) {
      return toC(valueAt(x,y));
    }
    
    int SetValue(int x, int y, int value) {
      if (y > numberOfDevices)
      {
        logger("y get overflow");
        logger(y);
        delay(10000);        
      }
      if (x > maxMeasurements)
      {
        logger("x get overflow");
        logger(x);
        delay(10000);
      }
      values[x][y] = value;
    }
    
};

    float toC(int16_t rawTemperature)
    {
      return (float)rawTemperature * TEMPERATURE_UNIT;
    }


Matrix2 measurements;
Plot plot(numberOfDevices, maxMeasurements, &adafruit, &measurements);

int colors[numberOfDevices + 1] = {
  ST7735_RED,
  ST7735_GREEN,
  ST7735_BLUE,
  ST7735_WHITE
};

void setup(void)
{
  MaxTempValue = 0x0550;
  MinTempValue = 0x00FF;
  pinMode(13, OUTPUT);
  initDusplay();

  SerialBegin();

  SerialPrintln("Dallas Temperature IC Control Library Demo");

  // Start up the library
  sensors.begin();
  sensors.setWaitForConversion(false);

  // locate devices on the bus
  SerialPrint("Locating devices...");

  // Grab a count of devices on the wire
  int realNumberOfDevices = sensors.getDeviceCount();
  SerialPrint("Found ");
  SerialPrint(numberOfDevices, DEC);
  SerialPrintln(" devices.");
  if (numberOfDevices != realNumberOfDevices)
  {
    SerialPrintln("Miscount. Reset.");
    reset();
  }

  // Loop through each device, print out address
  for (int i = 0; i < numberOfDevices  ; i++)
  {
    GetProbe(deviceAddress, i);
    printAddress(deviceAddress);
    // Search the wire for address
    GetProbe(deviceAddress, i);
    if (sensors.isConnected(deviceAddress))
    {
      logger("Found ");
      SerialPrint(i, DEC);
      SerialPrint(" with address: ");
      printAddress(deviceAddress);
      SerialPrintln();

      SerialPrint("Setting resolution to ");
      SerialPrintln(TEMPERATURE_PRECISION, DEC);

      // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
      sensors.setResolution(deviceAddress, TEMPERATURE_PRECISION);

      SerialPrint("Resolution actually set to: ");
      SerialPrint(sensors.getResolution(deviceAddress), DEC);
      if (sensors.getResolution(deviceAddress) != TEMPERATURE_PRECISION)
      {
        SerialPrintln("Failed. Reset.");
        reset();
      }
      SerialPrintln();
    } else {
      printAddress(deviceAddress);
      SerialPrint("Found ghost device at ");
      SerialPrint(i, DEC);
      SerialPrint(" but could not detect address. Check power and cabling");
    }
  }
  cls();
}

uint8_t scratchPad[9];
int getTemp(DeviceAddress deviceAddress)
{
  sensors.readScratchPad(deviceAddress, scratchPad);
  int16_t rawTemperature = (((int16_t)scratchPad[TEMP_MSB]) << 8) | scratchPad[TEMP_LSB];
  SerialPrintln();
  SerialPrint("readed:");
  SerialPrint(rawTemperature);
  SerialPrint(" = ");
  SerialPrint(scratchPad[TEMP_LSB]);
  SerialPrint(" + ");
  SerialPrintln(scratchPad[TEMP_MSB]);
  return rawTemperature;
}

void initDusplay() {
  adafruit.initR(INITR_BLACKTAB);
  adafruit.fillScreen(ST7735_BLACK);
  adafruit.setCursor(0, 0);
  //  testdrawtext("Dallas Temperature IC Control Library Demo ", ST7735_WHITE);
}

char buf[10];
void logger(float value) {
  dtostrf(value, 4, 3, buf);
  logger(buf);
}
void logger(unsigned long value) {
  dtostrf(value, 0, 0, buf);
  logger(buf);
}
void logger(char *text, int value) {
  dtostrf(value, 0, 0, buf);
  testdrawtext(text, ST7735_WHITE);
  logger(buf);
}
void logger(int value) {
  dtostrf(value, 0, 0, buf);
  logger(buf);
}
void logger(char *text) {
  testdrawtext(text, ST7735_WHITE);
  testdrawtext("\n", ST7735_WHITE);
}


void testdrawtext(char *text, uint16_t color) {
  //  adafruit.setCursor(0, 0);
  adafruit.setTextColor(color, ST7735_BLACK);
  adafruit.setTextWrap(true);
  adafruit.print(text);
}

void cls() {
  adafruit.fillScreen(ST7735_BLACK);
  adafruit.setCursor(0, 0);
}



void LogTemperature(byte num, int tempC)
{
  SerialPrint("Added: ");
  SerialPrintln(tempC);

  LogTemperature(num, maxMeasurements, tempC);
}

void LogTemperature(long num, int i, int tempC)
{
  if (i != 0)
  {
    LogTemperature(num, i - 1, measurements.valueAt(i,num));
  }
  else
  {
  }
  //    SerialPrintln();
  //    SerialPrint("Add: ");
  //    SerialPrint(tempC);
  measurements.SetValue(i,num,tempC);
}

void printTemperature(long i, int tempC)
{
  SerialPrint("|Temperature for device: ");
  SerialPrint(i, DEC);
  SerialPrint("Temp C: ");
  SerialPrintln(toC(tempC));
  logger(toC(tempC));
  LogTemperature(i, tempC);
}

int lastRequests = 0;
void loop(void)
{
  adafruit.setCursor(0, 0); 
  logger(millis() / 1000);
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  int mils = millis();
  //  if (lastRequests + 750 < mils)
  {
    digitalWrite(13, HIGH);
    SerialPrint("Requesting temperatures...");
    lastRequests = mils;
    sensors.requestTemperatures(); // Send the command to get temperatures
    SerialPrint("DONE in ");
    mils = millis() - mils;
    logger("DONE in ", mils);
    SerialPrintln(mils);
    //} else {
    // digitalWrite(13, LOW);
    // logger("SKIP REQ");
  }

  float tempCSum = 0.0;
  // Loop through each device, print out temperature data
  int i = 0;
  for (; i < numberOfDevices; i++)
  {
    // Search the wire for address
    GetProbe(deviceAddress, i);
    if (sensors.isConnected(deviceAddress))
    {
      printAddress(deviceAddress);
      // Output the device ID
      int mils = millis();
      int tempC = getTemp(deviceAddress);
      mils = millis() - mils;
      //      logger("READ in ", mils);

      printTemperature(i, tempC);
      tempCSum = tempCSum + tempC;
    }
    //else ghost device! Check your power requirements and cabling
  }
  plot.DrawTemperature();
  delay(250);
}

void GetProbe(uint8_t address[8], int n)
{
  int i = 0;
  if (n == 0)
  {
    address[i++] = 0x28;
    address[i++] = 0x20;
    address[i++] = 0x1B;
    address[i++] = 0x79;
    address[i++] = 0x06;
    address[i++] = 0x00;
    address[i++] = 0x00;
    address[i++] = 0x4E;
  } else if (n == 1) {
    address[i++] = 0x28;
    address[i++] = 0x64;
    address[i++] = 0x5D;
    address[i++] = 0x77;
    address[i++] = 0x06;
    address[i++] = 0x00;
    address[i++] = 0x00;
    address[i++] = 0x87;
  } else if (n == 2) {
    address[i++] = 0x28;
    address[i++] = 0x34;
    address[i++] = 0xF9;
    address[i++] = 0x76;
    address[i++] = 0x06;
    address[i++] = 0x00;
    address[i++] = 0x00;
    address[i++] = 0x0A;
  }
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) SerialPrint(0, HEX);
    SerialPrint(deviceAddress[i], HEX);
  }
}

void SerialPrint(char* text)
{
#if defined(SERIAL_LOG)
  Serial.print(text);
#endif
}

void SerialPrint(int value)
{
#if defined(SERIAL_LOG)
  Serial.print(value);
#endif
}

void SerialPrint(int value, int format)
{
#if defined(SERIAL_LOG)
  Serial.print(value, format);
#endif
}

void SerialPrintln(char* text)
{
  SerialPrint(text);
  SerialPrintln();
}

void SerialPrintln(int value, int format)
{
  SerialPrint(value, format);
  SerialPrintln();
}

void SerialPrintln(int value)
{
  SerialPrint(value);
  SerialPrintln();
}

void SerialPrintln()
{
#if defined(SERIAL_LOG)
  Serial.println();
#endif
}

void SerialBegin()
{
#if defined(SERIAL_LOG)
  Serial.begin(115200);
#endif
}

void reset() {
  //    cli();
  //    wdt_enable(WDTO_15MS);
  //    while(1);
}


//void DrawTemperature()
//{
//  int temp[numberOfDevices];
//
//  for (int i = 0; i < maxMeasurements; i++)
//  {
//    int tempcolors[numberOfDevices] = {
//      ST7735_RED,
//      ST7735_GREEN,
//      ST7735_BLUE
//    };
//    for (int d = 0; d < numberOfDevices; d++)
//    {
//      int x = map(measurements[i][d], MinTempValue, MaxTempValue, 0, tft.height() / 2);
//      temp[d] = x;
//    }
//
//    tft.fillRect(i , tft.height() / 2, 1, tft.height() / 2, ST7735_BLACK);
//    tft.drawPixel(i, tft.height() - 75, ST7735_WHITE);
//    for (int d = 0; d < numberOfDevices; d++)
//    {
//      for (int t = d + 1; t < numberOfDevices; t++)
//      {
//        if (temp[d] == temp[t])
//        {
//          temp[t] = 0;
//          unsigned int c = tempcolors[d] | tempcolors[t];
//          tempcolors[d] = c;
//          tempcolors[t] = c;
//        }
//      }
//      int x = temp[d];
//      tft.drawPixel(i, tft.height() - x, tempcolors[d]);
//
//    }
//  }
//
//  for (int i = maxMeasurements; i < 128; i++)
//  {
//    tft.fillRect(i , tft.height() / 2, 1, tft.height() / 2, ST7735_BLACK);
//  }
//
//  tft.setTextColor(ST7735_WHITE);
//  for (int d = 0; d < numberOfDevices; d++)
//  {
//    int x = temp[d];
//    if (x != 0)
//    {
//      tft.setCursor(maxMeasurements - 6, tft.height() - x - 9);
//      tft.print(toC(measurements[maxMeasurements - 1][d]));
//    }
//  }
//}

