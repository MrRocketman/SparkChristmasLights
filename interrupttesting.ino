// This #include statement was automatically added by the Spark IDE.
#include "SparkIntervalTimer/SparkIntervalTimer.h"

volatile int zeroCrossInterruptCount = 0;
volatile uint32_t dimmingInterruptCount = 0;
uint32_t loopCount = 0;

volatile uint32_t previousZeroCrossTime = 0;
volatile uint32_t currentZeroCrossTime = 0;
volatile uint32_t zeroCrossTimeDifference = 8333;
IntervalTimer dimmingTimer;

byte maxBrightness = 254;
volatile byte currentBrightnessCounter = maxBrightness;

const int latchPin = A2;
const int zeroCrossPin = A6;

// Need this! Else loop onlly happen at 200Hz!
SYSTEM_MODE(MANUAL);

// Function declarations
void zeroCrossInterruptHandler();
void dimmingTimerInterruptHandler();


void setup()
{
    // Connect spark since we are in manual mode
    Spark.connect();
    
    // Serial setup
    Serial.begin(57600);
    
    // SPI Setup
    SPI.setBitOrder(LSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV4); // = 72MHz/4 = 18MHz
    SPI.setDataMode(SPI_MODE3);
    SPI.begin(latchPin);
    
    // Zero cross interrupt setup (Should happen at 120Hz)
    pinMode(zeroCrossPin, INPUT);
    attachInterrupt(zeroCrossPin, zeroCrossInterruptHandler, FALLING);
    
    // Dimming timer setup (~30kHz)
    dimmingTimer.begin(dimmingTimerInterruptHandler, (uint16_t)(zeroCrossTimeDifference / (maxBrightness + 1)), uSec);
}

void loop()
{
    // Tick the loop counter to see how fast loop() is running
    loopCount ++;
    
    // Print any zero cross timing anomalies
    if(zeroCrossTimeDifference < 7500)
    {
        Serial.print("zc:");
        Serial.println(zeroCrossTimeDifference);
    }
    
    // Show stats once a second (since zeroCross happens at 120Hz)
    if(zeroCrossInterruptCount >= 120)
    {
        Serial.print("zc:");
        Serial.print(zeroCrossInterruptCount);
        Serial.print(" tc:");
        Serial.print(dimmingInterruptCount);
        Serial.print(" lc:");
        Serial.println(loopCount);
        
        // Reset our counters
        zeroCrossInterruptCount = 0;
        dimmingInterruptCount = 0;
        loopCount = 0;
        
        // Tell Spark to process cloud stuff since we are in manual mode
        Spark.process();
    }
}

void zeroCrossInterruptHandler()
{
    // Tick the zeroCross counter to see how fast zeroCrossInterruptHandler() is running
    zeroCrossInterruptCount ++;
    
    // Disable the dimming timer minimizes to problem but doesn't fix it
    //dimmingTimer.interrupt_SIT(INT_DISABLE);
    
    // Calculate the frequency of the zero cross signal
    previousZeroCrossTime = currentZeroCrossTime;
    currentZeroCrossTime = micros();
    zeroCrossTimeDifference = currentZeroCrossTime - previousZeroCrossTime;
    
    // Update the dimming interrupt timer to stay in phase with the zero cross
    dimmingTimer.resetPeriod_SIT((uint16_t)(zeroCrossTimeDifference / (maxBrightness + 1)), uSec);
    
    // Re-enable the dimming timer
    //dimmingTimer.interrupt_SIT(INT_ENABLE);
}

void dimmingTimerInterruptHandler()
{
    // Tick the dimmingTimer counter to see how fast dimmingTimerInterruptHandler() is running
    dimmingInterruptCount ++;
    
    // Write shift register latch clock low. If this is changed to some other pin, the problem goes away
    PIN_MAP[latchPin].gpio_peripheral->BRR = PIN_MAP[latchPin].gpio_pin;
    
    // Write shift register latch clock high. If this is changed to some other pin, the problem goes away
    PIN_MAP[latchPin].gpio_peripheral->BSRR = PIN_MAP[latchPin].gpio_pin;
}