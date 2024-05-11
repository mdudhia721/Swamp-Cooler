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
int currentStepperPos = 0, previousVentPos = 0;
const int ventPot = 0; // ADC channel 0

//Configure clock (DS1307 module)
RTC_DS1307 clock;

//Configure temp and humidity sensor (DHT11 module)
const int DHT_PIN = 45;
DHT dht(DHT_PIN, DHT11);

//Define water level sensor pin
const int WATER_LVL_PIN = 1; // ADC channel 1

//Configure LCD pins and variables for LCD delay
const int RS = 12, EN = 11, D4 = 5, D5 = 4, D6 = 3, D7 = 2;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
unsigned long previousMillis = 0;
const unsigned long updateDisplayInterval = 60000; //60000 ms = 1 minute

//Configure on/off button
const int START_DIGITAL_PIN = 18; //For attachInterrupt()
const int START_PIN = PD3;
volatile bool on = false;

// ==== FUNCTION PROTOTYPES ====
void gpioInit();
void sensorsMotorsInit();

void toggleOn();
void checkState();
void updateLED();

void controlVent();
void printVentPosition(int newpos);

void printCurrentTime();
void printStateChange();

void displayTempAndHumidity();
bool checkThreshold();

// ==== MAIN LOOP ====
void setup() {
    U0Init(9600);
    adcInit();
    gpioInit();
    sensorsMotorsInit();
    
    currentState = DISABLED;
    on = false;
}

void loop() {   
    if(!on) currentState = DISABLED;
    else checkState(); 
    printStateChange();
    previousState = currentState;

    switch (currentState) {
        case IDLE:
            displayTempAndHumidity();
            break;
        case RUNNING:
            displayTempAndHumidity();
            break;
        case DISABLED:
            break;
        case ERROR:
            displayTempAndHumidity();
            break;
        default:
            break;
    }

    updateLED();
    controlVent();
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

    //Start LCD
    lcd.begin(16,2);
}

void toggleOn() {
    on = !on;
}
void checkState() {
    if(on) {
        currentState = IDLE;
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
    int potValue = adc_read(ventPot);
    int targetPos = map(potValue, 0, 1023, 0, stepsPerRevolution); //Map full range of potentiometer readings to full range of steps
    int stepsToMove = targetPos - currentStepperPos;
    vent.step(stepsToMove);

    int newposrees = map(targetPos, 0, stepsPerRevolution, 0, 360);
    int currentDegrees = map(currentStepperPos, 0, stepsPerRevolution, 0, 360);

    if (abs(newposrees - currentDegrees) > 4) { //Checks if moved by more than 4 degrees to mitigate noise
        printVentPosition(newposrees);
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
                break;
            case RUNNING:
                U0printString("RUNNING");
                break;
            case DISABLED:
                U0printString("DISABLED");
                break;
            case ERROR:
                U0printString("ERROR");
                break;
            default:
                U0printString("UNKNOWN");
                break;
        }
        U0printString(" as of: ");
        printCurrentTime();
    }
}

void displayTempAndHumidity() {
    unsigned long currentMillis = millis();
    
    // Check if more than one minute has passed since last update
    if (currentMillis - previousMillis >= updateDisplayInterval) {
        lcd.setCursor(0,0);
        lcd.print("Temp: ");
        lcd.print(dht.readTemperature());
        lcd.print((char)223);
        lcd.print("C");

        lcd.setCursor(0,1);
        lcd.print("Humidity: ");
        lcd.print(dht.readHumidity());
        lcd.print("%");

        previousMillis = currentMillis; // Update the last update time
    }
}
bool checkThreshold() {
    return adc_read(WATER_LVL_PIN) > 100;
}