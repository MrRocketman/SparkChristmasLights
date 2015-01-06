
christmaslightsv1.ino  documentation.h

673
674
675
676
677
678
679
680
681
682
683
684
685
686
687
688
689
690
691
692
693
694
695
696
697
698
699
700
701
702
703
704
705
706
707
708
709
710
711
712
713
714
715
716
717
718
719
720
721
722
723
724
725
726
727
728
729
// Define a pointer that will be used to access the values for each output. Let it point one past the last value, because it is decreased before it is used.
byte *tempPWMValues = &pwmValues[numberOfChannels];

// Write shift register latch clock low
//digitalWrite(latchPin, LOW);
PIN_MAP[latchPin].gpio_peripheral->BRR = PIN_MAP[latchPin].gpio_pin;  // LOW

// Do a whole shift register at once. This unrolls the loop for extra speed
for(byte i = numberOfShiftRegisters; i > 0; --i)
{
    /*byte byteToSend;  // no need to initialize, all bits are replaced
     
     // Build the byte. One bit for each channel in this shift register
     add_one_pin_to_byte(byteToSend, currentBrightnessCounter, --tempPWMValues);
     add_one_pin_to_byte(byteToSend, currentBrightnessCounter, --tempPWMValues);
     add_one_pin_to_byte(byteToSend, currentBrightnessCounter, --tempPWMValues);
     add_one_pin_to_byte(byteToSend, currentBrightnessCounter, --tempPWMValues);
     add_one_pin_to_byte(byteToSend, currentBrightnessCounter, --tempPWMValues);
     add_one_pin_to_byte(byteToSend, currentBrightnessCounter, --tempPWMValues);
     add_one_pin_to_byte(byteToSend, currentBrightnessCounter, --tempPWMValues);
     add_one_pin_to_byte(byteToSend, currentBrightnessCounter, --tempPWMValues);*/
    
    // Build the byte. One bit for each channel in this shift register
    byte byteToSend = 0x00;
    //if(*(--tempPWMValues) > currentBrightnessCounter)
    //{
    byteToSend |= 0b10000000;
    //}
    for(byte i2 = 1; i2 < 8; i2 ++)
    {
        byteToSend >>= 1;
        //if(*(--tempPWMValues) > currentBrightnessCounter)
        //{
        byteToSend |= 0b10000000;
        //}
    }
    
    // Send the byte to the SPI
    SPI.transfer(byteToSend);
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
Ready.