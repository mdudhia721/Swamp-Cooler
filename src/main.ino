#include "myArduino.h" // Contains our register-level versions of common Arduino functions
#include <Stepper.h> // Stepper library, provided by Adafruit, for stepper motor control
#include <RTClib.h> // RTC library, provided by Adafruit, for clock module
#include <SPI.h> // Needed so RTClib.h can work!
#include <DHT.h> // DHT library, provided by Adafruit, for temp and humidity sensor
#include <Adafruit_Sensor.h> //Needed so DHT can work!
#include <LiquidCrystal.h> // LCD display library

// ADC, Timer, UART, and other functions that are typically available in the Arduino library
// that we must recreate can be found in myArduino.h. They have been seperated for clarity within the main file.  

// ==== DEFINITIONS AND CONSTANTS ====
// Define variable to track state.
enum state{IDLE, RUNNING, DISABLED, ERROR};
state currentState = DISABLED;
state previousState = DISABLED;

//Define variables for switching states
float currentTemp = 0;
float currentWaterLevel = 0;
const int WATER_THRESHOLD = 110; //About the level of my reservoir threshold
const float TEMP_THRESHOLD = 30.00; //My DHT sensor measures temp a bit higher than it actually is

//Configure on/off button
const int START_DIGITAL_PIN = 18; //For attachInterrupt()
const int START_PIN = PD3;
volatile bool on = false;

// Define state LED GPIO constants
const int LED_IDLE = PB3; // D50
const int LED_RUNNING = PB2; // D51
const int LED_DISABLED = PB1; // D52
const int LED_ERROR = PB0; // D53

// Configure vent fan, vent fan control, and main fan
const int stepsPerRevolution = 2038;
Stepper vent = Stepper(stepsPerRevolution, 23, 27, 25, 29); // IN1 = 23, IN2 = 25, IN3 = 27, IN4 = 29
int currentStepperPos = 0, previousVentPos = 0;
const int VENT_POT_PIN = 0; // ADC channel 0
const int FAN_PIN = PC2; //D35

//Configure clock, temp and humidity sensor, water level sensor 
RTC_DS1307 clock;
const int DHT_PIN = 45;
DHT dht(DHT_PIN, DHT11);
const int WATER_LVL_PIN = 1; // ADC channel 1

//Configure LCD pins and variables for LCD delay
const int RS = 12, EN = 11, D4 = 5, D5 = 4, D6 = 3, D7 = 2;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
unsigned long previousMillis = 0, currentMillis = 0;
const unsigned long updateDisplayInterval = 60000; //60000 ms = 1 minute

// ==== FUNCTION PROTOTYPES ====
//Setup functions
void gpioInit();
void sensorsMotorsInit();

//State control functions
void toggleOn();
void checkState();
void updateLED();

//Vent control functions
void controlVent();
void printVentPosition(int newpos);

//Event reporting functions
void printCurrentTime();
void printStateChange();

//Display temperature functions
void displayTempAndHumidity();

// ==== MAIN LOOP ====
void setup() {
    lcd.begin(16,2);
    U0Init(9600);
    adcInit();
    gpioInit();
    sensorsMotorsInit();
    
    currentState = DISABLED;
    on = false;
    lcd.clear();
}

void loop() {   
    //State control and checking
    if(!on) currentState = DISABLED;
    else checkState(); //Checks WHICH state we should be in
    printStateChange();
    previousState = currentState;

    //Controls WHAT each state should do
    switch (currentState) { // Opted not to use functions for each state (i.e. stateRunning, stateIdle, etc.) to keep functionality of each state clear in main
        case IDLE: //Display temp and humidity, control vent (basic functions)
            displayTempAndHumidity();
            controlVent();
            break;

        case RUNNING: // Basic functions, enable fan, fan turned off upon exiting RUNNING state
            displayTempAndHumidity();
            controlVent();
            PORTC |= (1 << FAN_PIN);
            break;

        case DISABLED: // Basic functions, no temp and humidity, fan off.
            lcd.clear();
            PORTC &= ~(1 << FAN_PIN);
            controlVent();
            break;

        case ERROR: // Basic functions, turn off fan and print error message
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Water level is");
            lcd.setCursor(0,1);
            lcd.print("too low!");
            PORTC &= ~(1 << FAN_PIN); 
            
            break;

        default:
            break;
    }

    updateLED();
}

// ==== FUNCTION DEFINITIONS ====
void gpioInit() {
    //Set all LEDs to output, then all to LOW
    DDRB |= (1 << LED_IDLE) | (1 << LED_RUNNING) | (1 << LED_DISABLED) | (1 << LED_DISABLED);
    PORTB &= ~((1 << LED_IDLE) | (1 << LED_RUNNING) | (1 << LED_DISABLED) | (1 << LED_ERROR));

    // Setup motor as output, initialize to LOW to disable fan
    DDRC |= (1 << FAN_PIN);
    PORTC &= ~(1 << FAN_PIN);

    // Setup on/off button as input w_pullup.
    DDRD &= ~(1 << START_PIN);
    PORTD |= (1 << START_PIN);
    attachInterrupt(digitalPinToInterrupt(START_DIGITAL_PIN), toggleOn, FALLING);
}
void sensorsMotorsInit() {
    // Begin RTC module
    if (!clock.begin()) {
        U0printString("Couldn't find clock");
        while (1);
    }

    // Begin DHT module
    dht.begin();

    // Water level sensor needs just simple ADC read, no setup needed

    // Setup vent speed
    vent.setSpeed(10);
}

void toggleOn() {
    on = !on;
}
void checkState() {
    currentTemp = dht.readTemperature();
    currentWaterLevel = adc_read(1);

    switch (currentState) {
        case IDLE:
            if (!on) currentState = DISABLED;
            else if (currentWaterLevel <= WATER_THRESHOLD) currentState = ERROR;
            else if (currentTemp > TEMP_THRESHOLD) currentState = RUNNING;
            break;

        case RUNNING:
            if (!on) currentState = DISABLED;
            else if (currentWaterLevel <= WATER_THRESHOLD) currentState = ERROR;
            else if (currentTemp <= TEMP_THRESHOLD) {
                currentState = IDLE;
                PORTC &= ~(1 << FAN_PIN); //Turn off fan upon exit
            }
            break;

        case DISABLED:
            if (on) currentState = IDLE; //Idle by default when we turn it on
            break;

        case ERROR:
            if (!on) currentState = DISABLED;
            else if (currentWaterLevel > WATER_THRESHOLD) currentState = IDLE;
            break;

        default:
            break;
    }
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
    int potValue = adc_read(VENT_POT_PIN);
    int targetPos = map(potValue, 0, 1023, 0, stepsPerRevolution); // Map full range of potentiometer readings to full range of steps

    // Calculate target position in degrees for easier comparison
    int targetDegrees = map(targetPos, 0, stepsPerRevolution, 0, 360);
    int currentDegrees = map(currentStepperPos, 0, stepsPerRevolution, 0, 360);

    // Check if movement exceeds threshold before moving vent, to reduce jitter in stepper motor
    if (abs(targetDegrees - currentDegrees) > 2) {
        int stepsToMove = targetPos - currentStepperPos;
        vent.step(stepsToMove);
        currentStepperPos = targetPos;

        // Print the new position of the vent in degrees
        printVentPosition(targetDegrees);
    }

    currentStepperPos = targetPos;
}
void printVentPosition(int newpos) {
    U0printString("Vent is now at ");
    U0printNumber(newpos);
    U0printString(" deg.\n");
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
void printStateChange() {
    if (currentState != previousState) {
        U0printString("State is now ");
        switch (currentState) {
            case IDLE: 
                U0printString("IDLE"); 
                U0printString(" (fan is off) ");
                break;
            case RUNNING: 
                U0printString("RUNNING"); 
                U0printString(" (fan is on) ");
            case DISABLED: U0printString("DISABLED"); break;
            case ERROR: U0printString("ERROR"); break;
            default: break;
        }
        previousMillis = millis() - updateDisplayInterval;
        U0printString(" as of: ");
        printCurrentTime();
    }
    
}

void displayTempAndHumidity() {
    currentMillis = millis();
    int elapsedTime = (currentMillis - previousMillis) / 1000;

    // Check if more than one minute has passed since last update
    if (currentMillis - previousMillis >= updateDisplayInterval) {
        lcd.setCursor(0,0);
        lcd.print("Temp: ");
        lcd.print(dht.readTemperature());
        lcd.print((char)223);
        lcd.print("C ");

        lcd.setCursor(0,1);
        lcd.print("Humidity: ");
        lcd.print(dht.readHumidity());
        lcd.print("%");

        previousMillis = currentMillis; // Update previous millis
    }

    lcd.setCursor(14, 0);
    lcd.print(elapsedTime);
    lcd.print("   ");
}