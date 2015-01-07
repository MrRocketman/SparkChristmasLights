// This #include statement was automatically added by the Spark IDE.
#include "SparkIntervalTimer/SparkIntervalTimer.h"

// This #include statement was automatically added by the Spark IDE.
/*#include "neopixel/neopixel.h"

#include "documentation.h"

// IMPORTANT: Set pixel COUNT, PIN and TYPE
#define PIXEL_COUNT 16
#define PIXEL_PIN D2
#define PIXEL_TYPE WS2812B

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
//               note: if not specified, D2 is selected for you.
// Parameter 3 = pixel type [ WS2812, WS2812B, WS2811, TM1803 ]
//               note: if not specified, WS2812B is selected for you.
//               note: RGB order is automatically applied to WS2811,
//                     WS2812/WS2812B/TM1803 is GRB order.
//
// 800 KHz bitstream 800 KHz bitstream (most NeoPixel products ...
//                         ... WS2812 (6-pin part)/WS2812B (4-pin part) )
//
// 400 KHz bitstream (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//                   (Radio Shack Tri-Color LED Strip - TM1803 driver
//                    NOTE: RS Tri-Color LED's are grouped in sets of 3)
// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

//Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

TCPClient client;*/

#pragma mark - Variable Declarations

// Debugging defines
#define SERIAL_PRINTING // Outputs critical data
//#define DEBUG // Outputs extra logs to Serial
//#define TESTING // Uncomment to enable all channel testing

// Board specific variables! Change these per board!,
byte numberOfShiftRegisters = 4;
#define AC_LIGHTS 1 // Set to 0 for DC lights

/////////////////////////////////////////////////////////////////////////////////////////////
// Don't change any variables below here unless you really, really know what you are doing //
/////////////////////////////////////////////////////////////////////////////////////////////

#define MICROSECONDS_TO_MILLISECONDS 1 / 1000.0
#define MILLISECONDS_TO_SECONDS 1 / 1000.0
#define MAX_PACKET_LENGTH 32

// The macro below uses 3 instructions per pin to generate the byte to transfer with SPI for the shift registers. Retreive duty cycle setting from memory (ldd, 2 clockcycles). Compare with the counter (cp, 1 clockcycle) --> result is stored in carry. Use the rotate over carry right to shift the compare result into the byte. (1 clockcycle).
// TL;DR super fast way of doing the following
// if(pwmValue > counter)
//      turnOn;
// else
//      turnOff;
#define add_one_pin_to_byte(sendByte, counter, pwmPointer) \
{ \
byte pwmValue = *pwmPointer; \
asm volatile ("cmp %0, %1" : /* No outputs */ : "r" (counter), "r" (pwmValue)); \
asm volatile ("rrx %0, %1" : "=r" (sendByte) : "r" (sendByte)); 	\
}

// Phase Frequency Control (PFC) (Zero Cross)
uint32_t previousZeroCrossTime = 0; // Timestamp in micros() of the latest zero crossing interrupt
uint32_t currentZeroCrossTime = 0; // Timestamp in micros() of the previous zero crossing interrupt
uint32_t zeroCrossTimeDifference =  8333; // 120Hz The calculated micros() between the last two zero crossings
const int zeroCrossPin = A6;
IntervalTimer dimmingTimer;

// Serial Packet Variables
const byte endOfPacketByte = 0xFF; // ASCII value 255 is our end of command byte
byte packetBuffer[MAX_PACKET_LENGTH]; // buffer for serial data
byte packetBufferLength = 0;
byte currentByteFromPacket = 0;
byte currentByteIndex = 0;

// Shift PWM Pins
const int latchPin = A2;

// Shift Register
byte maxBrightness = 255;
byte minBrightness = 0; // compensates for no light at < 20% dim. 14 for AC, 0 for DC
byte numberOfChannels = 0;
byte *pwmValues = 0;
volatile byte currentBrightnessCounter = maxBrightness;

// Dimming variables
float *brightnessChangePerDimmingCycle = 0;
float *temporaryPWMValues = 0;
uint16_t *dimmingUpdatesCount = 0;
volatile byte updateDimming = 0;
#ifdef TESTING
byte *dimmingDirection; // For testing only
#endif

#pragma mark - Method Declarations

// Packet Processing
void processPacket();
byte readNextByteInPacket();
void clearPacketBuffer();

// State changing methods
void turnOnChannel(byte channelNumber);
void turnOffChannel(byte channelNumber);
void setBrightnessForChannel(byte channelNumber, byte brightness);
void fadeChannelNumberToBrightnessWithMillisecondsDuration(byte channelNumber, byte brightness, uint16_t milliseconds);
bool isChannelNumberValid(byte channelNumber);

// Zero Cross
void handleZeroCross();

// ShiftRegister
void handleDimmingTimerInterrupt();
void initDimmingTimer();
void printInterruptLoad();

// Dimming
void dimmingUpdate();

#pragma mark - Setup And Main Loop

void setup()
{
    //Spark.function("led", ledControl);
    //Spark.function("connect", connectToMyServer);
    
    // Setup our pins
    pinMode(zeroCrossPin, INPUT);
    
    // Adjust the minimumBrightness for AC lights
    if(AC_LIGHTS)
    {
        minBrightness = 14;
    }
    
    // Serial setup
    Serial.begin(57600); // Xbee doesn't like to run any faster than 57600 for sustained throughput!
    
    // SPI Setup
    SPI.setBitOrder(LSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV16);
    SPI.setDataMode(SPI_MODE3);
    SPI.begin();
    
    // Initialize Dimming Timer
    initDimmingTimer();
    
    // Setup the zero cross interrupt which uses zeroCrossPin (zeroCrossPin can't be changed though)
    attachInterrupt(zeroCrossPin, handleZeroCross, FALLING);
    
    // Init our variables for the appropriate number of shift registers
    initializeWithewShiftRegisterCount();
    
    //strip.begin();
    //strip.show(); // Initialize all pixels to 'off'
}

void loop()
{
    /*if (client.connected())
     {
     if (client.available())
     {
     
     }
     }*/
    
    // Handle dimming
    if(updateDimming)
    {
        dimmingUpdate();
        updateDimming = 0;
    }
    
    // Handle Serial data
    if(Serial.available())
    {
        // Read in a byte and store it
        packetBuffer[packetBufferLength] = Serial.read();
        packetBufferLength ++;
        
        // End of command byte, so now we'll proccess the command
        if(packetBuffer[packetBufferLength - 1] == endOfPacketByte)
        {
            processPacket();
            clearPacketBuffer();
        }
        // Update everything so the next byte can be received
        else
        {
            // Somehow the buffer has overflowed. Erase it since it's probably just garbage
            if(packetBufferLength > MAX_PACKET_LENGTH)
            {
#ifdef SERIAL_PRINTING
                Serial.println("Buffer overflow");
#endif
                
                clearPacketBuffer();
            }
        }
    }
}

/*void ipArrayFromString(byte ipArray[], String ipString)
{
    int dot1 = ipString.indexOf('.');
    ipArray[0] = ipString.substring(0, dot1).toInt();
    int dot2 = ipString.indexOf('.', dot1 + 1);
    ipArray[1] = ipString.substring(dot1 + 1, dot2).toInt();
    dot1 = ipString.indexOf('.', dot2 + 1);
    ipArray[2] = ipString.substring(dot2 + 1, dot1).toInt();
    ipArray[3] = ipString.substring(dot1 + 1).toInt();
}

int connectToMyServer(String ip)
{
    byte serverAddress[4];
    ipArrayFromString(serverAddress, ip);
    
    if (client.connect(serverAddress, 9000))
    {
        return 1; // successfully connected
    }
    else
    {
        return -1; // failed to connect
    }
}

// This function gets called whenever there is a matching API request
// the command string format is l<led number>,<state>
// for example: l1,HIGH or l1,LOW
//              l2,HIGH or l2,LOW
int ledControl(String command)
{
    // TODO: !!! Fix this and add return 1 or -1 for success or failure
    //packet = command;
    //packetLength = command.length();
    //currentByteIndex = 0;
    
    return 1;
}*/

#pragma mark - Command Processing

void processPacket()
{
    // Read in the commandID byte
    byte commandID = readNextByteInPacket();
    
    if(commandID == 0x00) // Command 0x00 (1 channel on)
    {
        turnOnChannel(readNextByteInPacket());
    }
    else if(commandID == 0x01) // Command 0x01 (1 Channel off)
    {
        turnOffChannel(readNextByteInPacket());
    }
    else if(commandID == 0x04) // Command 0x04 (1 bit state for all channels)
    {
        byte startingIndex = 0;
        // Loop through only the data bytes (skip the end of packet byte)
        while(currentByteIndex < packetBufferLength - 1)
        {
            turn8ChannelsOnOffStartingAtChannelNumber(startingIndex * 8, readNextByteInPacket());
            startingIndex ++;
        }
    }
    else if(commandID == 0x05) // Command 0x05 (1 Channel on for time in hundreths)
    {
        byte channelNumber = readNextByteInPacket();
        byte fadeTimeInHundrethsOfSeconds = readNextByteInPacket();
        channelBrightnessWithMillisecondsDuration(channelNumber, maxBrightness, fadeTimeInHundrethsOfSeconds * 10);
    }
    else if(commandID == 0x06) // Command 0x06 (1 Channel on for time in tenths)
    {
        byte channelNumber = readNextByteInPacket();
        byte fadeTimeInTenthsOfSeconds = readNextByteInPacket();
        channelBrightnessWithMillisecondsDuration(channelNumber, maxBrightness, fadeTimeInTenthsOfSeconds * 100);
    }
    else if(commandID == 0x10) // Command 0x10 (1 brightness)
    {
        byte channelNumber = readNextByteInPacket();
        byte brightness = readNextByteInPacket();
        setBrightnessForChannel(channelNumber, brightness);
    }
    else if(commandID == 0x11) // Command 0x11 (All brightness)
    {
        byte brightness = readNextByteInPacket();
        
        for(byte i = 0; i < numberOfChannels; i ++)
        {
            setBrightnessForChannel(i, brightness);
        }
    }
    else if(commandID == 0x12) // Command 0x12 (1 Channel brightness for time in hundreths)
    {
        byte channelNumber = readNextByteInPacket();
        byte brightness = readNextByteInPacket();
        byte fadeTimeInHundrethsOfSeconds = readNextByteInPacket();
        channelBrightnessWithMillisecondsDuration(channelNumber, brightness, fadeTimeInHundrethsOfSeconds * 10);
    }
    else if(commandID == 0x13) // Command 0x13 (1 Channel brightness for time in tenths)
    {
        byte channelNumber = readNextByteInPacket();
        byte brightness = readNextByteInPacket();
        byte fadeTimeInTenthsOfSeconds = readNextByteInPacket();
        channelBrightnessWithMillisecondsDuration(channelNumber, brightness, fadeTimeInTenthsOfSeconds * 100);
    }
    else if(commandID == 0x15) // Command 0x15 (All on)
    {
        for(byte i = 0; i < numberOfChannels; i ++)
        {
            turnOnChannel(i);
        }
    }
    else if(commandID == 0x16) // Command 0x16 (All off)
    {
        for(byte i = 0; i < numberOfChannels; i ++)
        {
            turnOffChannel(i);
        }
    }
    // Fade channel/all
    else if(commandID == 0x20 || commandID == 0x21 || commandID == 0x22 || commandID == 0x23 || commandID == 0x24 || commandID == 0x25 || commandID == 0x30 || commandID == 0x31 || commandID == 0x32 || commandID == 0x33 || commandID == 0x34 || commandID == 0x35)
    {
        byte channelNumber;
        byte startBrightness;
        byte endBrightness;
        byte fadeTimeInHundrethsOfSeconds;
        byte fadeTimeInTenthsOfSeconds;
        
        // Get the channel number for single channel commands
        if(commandID == 0x20 || commandID == 0x21 || commandID == 0x22 || commandID == 0x23 || commandID == 0x24 || commandID == 0x25)
        {
            channelNumber = readNextByteInPacket();
        }
        
        // Fade up
        if(commandID == 0x20 || commandID == 0x21 || commandID == 0x30 || commandID == 0x31)
        {
            // Get the end brightness
            startBrightness = 0;
            endBrightness = maxBrightness;
        }
        // Fade down
        if(commandID == 0x22 || commandID == 0x23 || commandID == 0x32 || commandID == 0x33)
        {
            // Get the end brightness
            startBrightness = maxBrightness;
            endBrightness = 0;
        }
        // Fade from x to y
        if(commandID == 0x24 || commandID == 0x25 || commandID == 0x34 || commandID == 0x35)
        {
            // Get the startBrightness
            startBrightness = readNextByteInPacket();
            if(startBrightness > maxBrightness)
            {
                startBrightness = maxBrightness;
            }
            else if(startBrightness < 0)
            {
                startBrightness = 0;
            }
            
            // Get the endBrightness
            endBrightness = readNextByteInPacket();
            if(endBrightness > maxBrightness)
            {
                endBrightness = maxBrightness;
            }
            else if(endBrightness < 0)
            {
                endBrightness = 0;
            }
        }
        
        // Fade channel in hundreths
        if(commandID == 0x20 || commandID == 0x22 || commandID == 0x24)
        {
            fadeTimeInHundrethsOfSeconds = readNextByteInPacket();
            fadeChannelNumberFromBrightnessToBrightnessWithMillisecondsDuration(channelNumber, startBrightness, endBrightness, fadeTimeInHundrethsOfSeconds * 10);
        }
        // Fade channel in tenths
        else if(commandID == 0x21 || commandID == 0x23 || commandID == 0x25)
        {
            fadeTimeInTenthsOfSeconds = readNextByteInPacket();
            // Set the fade
            fadeChannelNumberFromBrightnessToBrightnessWithMillisecondsDuration(channelNumber, startBrightness, endBrightness, fadeTimeInTenthsOfSeconds * 100);
        }
        // Fade all hundredths
        else if(commandID == 0x30 || commandID == 0x32 || commandID == 0x34)
        {
            fadeTimeInHundrethsOfSeconds = readNextByteInPacket();
            // Set the fade
            for(byte i = 0; i < numberOfChannels; i ++)
            {
                fadeChannelNumberFromBrightnessToBrightnessWithMillisecondsDuration(i, startBrightness, endBrightness, fadeTimeInHundrethsOfSeconds * 10);
            }
        }
        // Fade all tenths
        else if(commandID == 0x31 || commandID == 0x33 || commandID == 0x35)
        {
            fadeTimeInTenthsOfSeconds = readNextByteInPacket();
            // Set the fade
            for(byte i = 0; i < numberOfChannels; i ++)
            {
                fadeChannelNumberFromBrightnessToBrightnessWithMillisecondsDuration(i, startBrightness, endBrightness, fadeTimeInTenthsOfSeconds * 100);
            }
        }
    }
    else if(commandID == 0xF1) // Command 0xF1 (Request status - number of boards connected)
    {
        Serial.println(numberOfShiftRegisters);
    }
}

byte readNextByteInPacket()
{
    // If we aren't past the end of our buffer, get the next buffered byte
    if(currentByteIndex < packetBufferLength)
    {
        currentByteFromPacket = packetBuffer[currentByteIndex];
    }
    else
    {
        currentByteFromPacket = 0;
    }
    
    currentByteIndex ++;
    
    return currentByteFromPacket;
}

void clearPacketBuffer()
{
    packetBufferLength = 0;
    currentByteIndex = 0;
    
    // Set our packetBuffer to all 0's (nils)
    memset(packetBuffer, 0, MAX_PACKET_LENGTH * sizeof(byte));
}

#pragma mark - Channel Changes

void turnOnChannel(byte channelNumber)
{
    setBrightnessForChannel(channelNumber, maxBrightness);
}

void turnOffChannel(byte channelNumber)
{
    setBrightnessForChannel(channelNumber, 0);
}

void turn8ChannelsOnOffStartingAtChannelNumber(byte startingChannelNumber, byte channelStates)
{
    for(byte i = 0; i < 8; i ++)
    {
        if(channelStates & (1 << i)) // If the state for this channel is 1 (off)
        {
            setBrightnessForChannel((startingChannelNumber + i), maxBrightness);
        }
        else
        {
            setBrightnessForChannel((startingChannelNumber + i), 0);
        }
    }
}

void setBrightnessForChannel(byte channelNumber, byte brightness)
{
    // Set the brightness if the channel is valid
    if(isChannelNumberValid(channelNumber))
    {
        // Clamp to min and max brightness
        if(brightness < minBrightness)
        {
            brightness = minBrightness;
        }
        else if(brightness > maxBrightness)
        {
            brightness = maxBrightness;
        }
        pwmValues[channelNumber] = brightness;
        // Cancel a fade if it was fading
        dimmingUpdatesCount[channelNumber] = 0;
    }
    
#ifdef DEBUG
    Serial.print("ch:");
    Serial.print(channelNumber);
    Serial.print(" v:");
    Serial.println(brightness);
#endif
}

void channelBrightnessWithMillisecondsDuration(byte channelNumber, byte brightness, uint16_t milliseconds)
{
    // Set the brightnessChangePerDimmingCycle if the channel is valid
    if(isChannelNumberValid(channelNumber))
    {
        // Clamp to min and max brightness
        if(brightness < minBrightness)
        {
            brightness = minBrightness;
        }
        else if(brightness > maxBrightness)
        {
            brightness = maxBrightness;
        }
        pwmValues[channelNumber] = brightness;
        
        dimmingUpdatesCount[channelNumber] = milliseconds / (zeroCrossTimeDifference * MICROSECONDS_TO_MILLISECONDS) + 11; // 11 means fade up has 11/120 seconds to get another command before it shuts off
        brightnessChangePerDimmingCycle[channelNumber] = 0;
        temporaryPWMValues[channelNumber] = pwmValues[channelNumber];
    }
    
#ifdef DEBUG
    Serial.print("ch:");
    Serial.print(channelNumber);
    Serial.print(" p:");
    Serial.println(brightnessChangePerDimmingCycle[channelNumber]);
#endif
}

void fadeChannelNumberFromBrightnessToBrightnessWithMillisecondsDuration(byte channelNumber, byte startBrightness, byte endBrightness, uint16_t milliseconds)
{
    // Set the brightnessChangePerDimmingCycle if the channel is valid
    if(isChannelNumberValid(channelNumber))
    {
        if(startBrightness < minBrightness)
        {
            startBrightness = minBrightness;
        }
        else if(startBrightness > maxBrightness)
        {
            startBrightness = maxBrightness;
        }
        if(endBrightness < minBrightness)
        {
            endBrightness = minBrightness;
        }
        else if(endBrightness > maxBrightness)
        {
            endBrightness = maxBrightness;
        }
        pwmValues[channelNumber] = startBrightness;
        
        dimmingUpdatesCount[channelNumber] = milliseconds / (zeroCrossTimeDifference * MICROSECONDS_TO_MILLISECONDS) + 11; // 11 means fade up has 11/120 seconds to get another command before it shuts off
        brightnessChangePerDimmingCycle[channelNumber] = (float)(endBrightness - startBrightness) / dimmingUpdatesCount[channelNumber];
        temporaryPWMValues[channelNumber] = pwmValues[channelNumber];
    }
    
#ifdef DEBUG
    Serial.print("ch:");
    Serial.print(channelNumber);
    Serial.print(" p:");
    Serial.println(brightnessChangePerDimmingCycle[channelNumber]);
#endif
}

bool isChannelNumberValid(byte channelNumber)
{
    if(channelNumber < numberOfChannels)
    {
        return 1;
    }
    else
    {
        Serial.print(F("Error: Trying to change channel "));
        Serial.println(channelNumber);
        Serial.println(F(" that isn't initialized"));
        return 0;
    }
}

void initializeWithewShiftRegisterCount()
{
    // Disable interrupt
    noInterrupts();
    // Update the number of channels
    numberOfChannels = numberOfShiftRegisters * 8;
    
    // Malloc pwmValues array
    pwmValues = (byte *)malloc(numberOfChannels * sizeof(byte));
    // Initialize pwmValues array to 0
    memset(pwmValues, 0, numberOfChannels * sizeof(byte));
    
    // Malloc temporaryPWMValues array
    temporaryPWMValues = (float *)malloc(numberOfChannels * sizeof(float));
    // Initialize temporaryPWMValues array to 0
    memset(temporaryPWMValues, 0, numberOfChannels * sizeof(float));
    
    // Malloc brightnessChangePerDimmingCycle array
    brightnessChangePerDimmingCycle = (float *)malloc(numberOfChannels * sizeof(float));
    // Initialize brightnessChangePerDimmingCycle array to 0
    memset(brightnessChangePerDimmingCycle, 0, numberOfChannels * sizeof(float));
    
    // Malloc dimmingUpdatesCount array
    dimmingUpdatesCount = (uint16_t *)malloc(numberOfChannels * sizeof(uint16_t));
    // Initialize dimmingUpdatesCount array to 0
    memset(dimmingUpdatesCount, 0, numberOfChannels * sizeof(uint16_t));
    
#ifdef TESTING
    // Malloc dimmingDirection array
    dimmingDirection = (byte *)malloc(numberOfChannels * sizeof(byte));
    // Initialize dimmingDirection array to 0
    memset(dimmingDirection, 0, numberOfChannels * sizeof(byte));
#endif
    
    // Re-enable interrupt
    interrupts();
}

#pragma mark - Zero Cross

// Hardware zero cross interrupt
void handleZeroCross()
{
    // Calculate the frequency
    previousZeroCrossTime = currentZeroCrossTime;
    currentZeroCrossTime = micros();
    zeroCrossTimeDifference = currentZeroCrossTime - previousZeroCrossTime;
    
    // Update the dimming interrupt timer
    dimmingTimer.resetPeriod_SIT((uint16_t)(zeroCrossTimeDifference / (maxBrightness + 1)), uSec);
    
    // Handle AC dimming (fading over time)
    updateDimming = 1;
}

#pragma mark - Shift Register

void initDimmingTimer()
{
    dimmingTimer.begin(handleDimmingTimerInterrupt, (uint16_t)(zeroCrossTimeDifference / (maxBrightness + 1)), uSec);
}

void handleDimmingTimerInterrupt()
{
    // Define a pointer that will be used to access the values for each output. Let it point one past the last value, because it is decreased before it is used.
    byte *tempPWMValues = &pwmValues[numberOfChannels];
    
    // Write shift register latch clock low
    //digitalWrite(latchPin, LOW);
    PIN_MAP[latchPin].gpio_peripheral->BRR = PIN_MAP[latchPin].gpio_pin;  // LOW
    
    // Do a whole shift register at once. This unrolls the loop for extra speed
    for(byte i = numberOfShiftRegisters; i > 0; --i)
    {
        uint32_t byteToSend;  // no need to initialize, all bits are replaced
        
        // Build the byte. One bit for each channel in this shift register
        add_one_pin_to_byte(byteToSend, currentBrightnessCounter, --tempPWMValues);
        add_one_pin_to_byte(byteToSend, currentBrightnessCounter, --tempPWMValues);
        add_one_pin_to_byte(byteToSend, currentBrightnessCounter, --tempPWMValues);
        add_one_pin_to_byte(byteToSend, currentBrightnessCounter, --tempPWMValues);
        add_one_pin_to_byte(byteToSend, currentBrightnessCounter, --tempPWMValues);
        add_one_pin_to_byte(byteToSend, currentBrightnessCounter, --tempPWMValues);
        add_one_pin_to_byte(byteToSend, currentBrightnessCounter, --tempPWMValues);
        add_one_pin_to_byte(byteToSend, currentBrightnessCounter, --tempPWMValues);
        
        // Send the byte to the SPI
        SPI.transfer(byteToSend >> 24);
        
        // Build the byte. One bit for each channel in this shift register
        /*byte byteToSend = 0x00;
         if(*(--tempPWMValues) > currentBrightnessCounter)
         {
         byteToSend |= 0b10000000;
         }
         for(byte i2 = 1; i2 < 8; i2 ++)
         {
         byteToSend >>= 1;
         if(*(--tempPWMValues) > currentBrightnessCounter)
         {
         byteToSend |= 0b10000000;
         }
         }
         
         // Send the byte to the SPI
         SPI.transfer(byteToSend);
         */
    }
    
    // Write shift register latch clock high
    //digitalWrite(latchPin, HIGH);
    PIN_MAP[latchPin].gpio_peripheral->BSRR = PIN_MAP[latchPin].gpio_pin; // HIGH
    
    // Decrease the brightness index. This is the key to getting AC dimming since triacs stay on until the next zero cross. So we need to turn on after a delay rather than turn on immediately and turn off after a delay. AKA, this needs to count down not up. Trust me!
    if(currentBrightnessCounter > 0)
    {
        currentBrightnessCounter --;
    }
    else
    {
        // Handle DC dimming (fading over time)
        updateDimming = 1;
    }
}

#pragma mark - Dimming

void dimmingUpdate()
{
    // Reset the brightness index to be in phase with the zero cross
    currentBrightnessCounter = maxBrightness;
    
#ifdef TESTING
    dimmingTest();
#endif
    
    // Handle the dimming over time for each channel
    for(byte i = 0; i < numberOfChannels; i ++)
    {
        // Update a channel if it is still dimming
        if(dimmingUpdatesCount[i] > 0)
        {
            if(dimmingUpdatesCount[i] > 10)
            {
                temporaryPWMValues[i] += brightnessChangePerDimmingCycle[i];
                pwmValues[i] = temporaryPWMValues[i];
            }
            dimmingUpdatesCount[i] --;
            
            // Fade is complete. Turn off now
            if(dimmingUpdatesCount[i] == 0)
            {
                pwmValues[i] = 0;
            }
        }
    }
}

#ifdef TESTING

void dimmingTest()
{
    for(byte i = 0; i < numberOfChannels; i ++)
    {
        if(dimmingUpdatesCount[i] == 0)
        {
            if(dimmingDirection[i] == 1)
            {
                fadeChannelNumberFromBrightnessToBrightnessWithMillisecondsDuration(i, maxBrightness, 0, 500 + i * 100);
                dimmingDirection[i] = 0;
            }
            else
            {
                fadeChannelNumberFromBrightnessToBrightnessWithMillisecondsDuration(i, 0, maxBrightness, 500 + i * 100);
                dimmingDirection[i] = 1;
            }
        }
    }
}

#endif