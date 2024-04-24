#include <Arduino.h>
#include <Wire.h>
#include <AHT20.h>
#include <LTR390.h>

//Time in seconds in sleep mode
#define TIME_IN_SECONDS 10

//Stepper varibles
const int stepPin = 4;
const int dirPin = 5;
const int sleepPin = 15;
int direction = 1;
const int fullRotationSteps = 203; //Might need changing
// int steps120Degrees = map(120, 0, 360, 0, fullRotationSteps);
int steps120Degrees = map(120, 0, 360, 0, 800);
int rotationAmount = 0;

//Sensor Objects
AHT20 aht20; //Temperature and humidity
LTR390 ltr390; //UVindex


//Measurement variables, set to -1 (value that indicates no measurement)
float temperature = -1.0;
float humidity = -1.0;
// float lux = -1.0;
float uvi = -1.0;


//Move Stepper X amount of times
void moveStepperToIndex();

//Move stepper on rotation, temperalily removed and not implemented
void moveStepper120Degrees();

//Measurement caller functions
void getTemperatureData();
void getUVData();

//Send data via SIM module
void sendDataToServer();

//Enable sleep mode
void goToDeepSleep();

//In order calls -> getTemperatureData, getUVData, sendDataToServer
void captureUpdateSendProc();

//Init temperature sensor
void setAHT20();

//Init UV sensor
void setLTR390();

void goToDeepSleep()
{
  Serial.println("Sleeping for 30 minutes");
  esp_sleep_enable_timer_wakeup(TIME_IN_SECONDS * 1000000);
  esp_deep_sleep_start();
}

void setup()
{

  Serial.begin(115200);
  Serial.println("UV Guard Demo");

  //Start I2C
  Wire.begin();

  //Set Stepper driver pins as output
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(sleepPin, OUTPUT);

  //Init Sensors
  setAHT20();
  setLTR390();

  //Capture and send data. Update Stepper
  captureUpdateSendProc();
  moveStepperToIndex();

  //Go back to sleep
  goToDeepSleep();
  Serial.println("Woken"); ///Dummy Serial Print. Should never execute
}


//Empty Loop. Never reached because deep sleep
void loop() { ; }


//Tries to init UV Sensor, if it doesn't it moves on. 
void setLTR390()
{
  if (!ltr390.init())
  {
    Serial.println("LTR390 not connected!");
  }
}


//Tries to init Temperature Sensor, if it doesn't it moves on. 
void setAHT20()
{
  if (aht20.begin() == false)
  {
    Serial.println("AHT20 not detected. Please check wiring. Freezing.");
  }

  Serial.println("AHT20 acknowledged.");
}

/*
Attempts to fetch temperature data.
Note that it WAITS for the sensor to be available,
which permantently stalls otherwise. If needed, set
timeout time to move on
*/
void getTemperatureData()
{

  while (!aht20.available())
  {
    ;
  }

  temperature = aht20.getTemperature();
  humidity = aht20.getHumidity();
}


/*
Attempts to fetch UV data.
Note that it WAITS for the sensor to be available,
which permantently stalls otherwise. If needed, set
timeout time to move on
*/
void getUVData()
{

  ltr390.setGain(LTR390_GAIN_18);                // Recommended for UVI - x18
  ltr390.setResolution(LTR390_RESOLUTION_20BIT); // Recommended for UVI - 20-bit
  ltr390.setMode(LTR390_MODE_UVS);

  while (!ltr390.newDataAvailable())
  {
    ;
  }

  uvi = ltr390.getUVI();
}


//Sends data to server, doesn't check response
void sendDataToServer()
{

  //Sets parameters and sends to endpoint
  String url = "http://89.47.162.121/post_data?uvi=" + String(uvi) + "&temp=" + String(temperature) + "&hum=" + String(humidity);

  //Initial AT command
  Serial.println("AT");
  delay(200);

  //Attach to packet domain service
  Serial.println("AT+CGATT=1");
  delay(200);

  //Init HTTP client
  Serial.println("AT+HTTPINIT");
  delay(200);

  //Sets HTTP URL parameters
  Serial.println("AT+HTTPPARA=\"URL\",\"" + url + "\"");
  delay(200);

  //Sends GET request (action 0)
  Serial.println("AT+HTTPACTION=0");
  delay(200);
}


//Gets temperature data, gets UV data and sends it to server
void captureUpdateSendProc()
{

  getTemperatureData();
  getUVData();
  sendDataToServer();
}


//Determines the amount of rotations to move stepper
void moveStepperToIndex()
{

  //Awake stepper driver
  digitalWrite(sleepPin, HIGH);

  // Set motor direction clockwise
  digitalWrite(dirPin, 0);

  //Determine amount of rotations from POSITION 0
  if (uvi <= 3.0)
  {
    rotationAmount = 0;
  }
  else if (uvi <= 7.0)
  {
    rotationAmount = 1;
  }
  else if (uvi > 7.0)
  {
    rotationAmount = 2;
  }


  //Amount of 120 degree rotations to do
  for (int i = 0; i < rotationAmount; i++)
  {

    /*
    Does the amounts of steps needed to achieve 
    120 degree rotation.

    Function moveStepper120Degrees() should replace this for-loop
    */
    for (int i = 0; i < steps120Degrees; i++)
    {
      //MUST DO - Change frequency (add delay between steps?) to slow movement
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1000);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(1000);
    }

    delay(1000);
  }

  //Resets rotation amount, technically not needed
  rotationAmount = 0;
}
