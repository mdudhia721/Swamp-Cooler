#include "helpers.h"
#include <Stepper.h> // Stepper library, provided by Adafruit, for stepper motor control
#include <RTClib.h> // RTC library, provided by Adafruit, for clock module
#include <SPI.h> // Needed so RTClib.h can work!
#include <DHT.h> // DHT library, provided by Adafruit, for temp and humidity sensor
#include <Adafruit_Sensor.h> //Needed so DHT can work!

// ADC, Timer, UART, and other functions that are typically available in the Arduino library
// that we must recreate can be found in helpers.h. They have been seperated for clarity within the main file.  

// ==== DEFINITIONS AND CONSTANTS ====
// Define currentState enum
enum state{IDLE, RUNNING, DISABLED, ERROR};
state currentState = DISABLED;

// Define state LED GPIO constants
const int LED_IDLE = PB3; // D50
const int LED_RUNNING = PB2; // D51
const int LED_DISABLED = PB1; // D52
const int LED_ERROR = PB0; // D53

//Define fan motor GPIO constants
const int FAN_PIN = PC2; //D35

// Configure vent fan and control (stepper motor and potentiometer)
const int stepsPerRevolution = 2038;
Stepper vent = Stepper(stepsPerRevolution, 23, 27, 25, 29); // IN1 = 23, IN2 = 25, IN3 = 27, IN4 = 29
int currentStepperPos = 0;
const int ventPot = 0; // ADC channel 0

//Configure clock (DS1307 module)
RTC_DS1307 clock;

//Configure temp and humidity sensor (DHT11 module)
const int DHT_PIN = 45;
DHT dht(DHT_PIN, DHT11);

//Define water level sensor pin
const int WATER_LVL_PIN = 1; // ADC channel 1

// ==== FUNCTION PROTOTYPES ====
void setupStateLED();
void updateLED();
void controlVent();
void setupSensorsAndMotors();
void printCurrentTime();
void displayTempAndHumidity();
bool checkThreshold();

// ==== MAIN LOOP ====
void setup() {
    setupStateLED();
    setupSensorsAndMotors();
    adcInit();

    U0init(9600);
}

void loop() {   
    
}

// ==== FUNCTION DEFINITIONS ====
void setupStateLED() {
    //Set all LEDs to output, then all to LOW
    DDRB |= (1 << LED_IDLE) | (1 << LED_RUNNING) | (1 << LED_DISABLED) | (1 << LED_DISABLED);
    PORTB &= ~((1 << LED_IDLE) | (1 << LED_RUNNING) | (1 << LED_DISABLED) | (1 << LED_ERROR));
}

void updateLED() {
    //Set all LEDs to LOW briefly, then set corresponding state LED to high.
    PORTB &= ~((1 << LED_IDLE) | (1 << LED_RUNNING) | (1 << LED_DISABLED) | (1 << LED_ERROR));
    switch (currentState) {
        case IDLE: PORTB |= (1 << LED_IDLE); break;
        case RUNNING: PORTB |= (1 << LED_RUNNING); break;
        case DISABLED: PORTB |= (1 << LED_DISABLED); break;
        case ERROR: PORTB |= (1 << LED_ERROR); break;
    }
}

void controlVent() {
    int potValue = adc_read(ventPot);
    int targetPos = map(potValue, 0, 1023, 0, stepsPerRevolution); //Map full range of potentiometer readings to full range of steps
    int stepsToMove = targetPos - currentStepperPos;
    vent.step(stepsToMove);
    currentStepperPos = targetPos;
}

void setupSensorsAndMotors() {
    // Begin RTC module
    if (!clock.begin()) {
        Serial.println("Couldn't find clock");
        while (1);
    }

    // Begin DHT module
    dht.begin();

    // Water level sensor is simple ADC read, no setup needed

    // Setup vent speed
    vent.setSpeed(10);

    // Setup motor as output, initialize to LOW to disable fan
    DDRC |= (1 << FAN_PIN);
    PORTC &= ~(1 << FAN_PIN);
}

void printCurrentTime() {
    DateTime now = clock.now();

    U0printNumber(now.hour());
    U0putchar(':');
    U0printNumber(now.minute());
    U0putchar(':');
    U0printNumber(now.second());
    U0putchar('\n');
}

void displayTempAndHumidity() {
    U0printString("Temperature = ");
    U0printFloat(dht.readTemperature());
    U0printString(" Celsius\n");
    
    U0printString("Humidity = ");
    U0printFloat(dht.readHumidity());
    U0printString("%\n");
}

bool checkThreshold() {
    return adc_read(WATER_LVL_PIN) > 100;
}