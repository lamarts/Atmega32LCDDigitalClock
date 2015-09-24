# Atmega32LCDDigitalClock
 * ****************************************************************************************************************
  Atmega32LCDDigitalClock.c
(1) This program demonstrates the use of the HD74480 LCD with an Atmega32 8-bit microcontroller. 
(2)This program utilizes the LCD in 4-bit mode - to set, reset (buttons) and display time in minutes, hours and seconds. 
(3) program utilizes the Timer interrupt feature in the Atmega32 microcontroller. 
The interrupt timer onboard the chip was used to generate the 1-second timer using the internal clock set @ 4MHz

AtmelStudio 6 was used to develop code, compile and upload firmware to the chip using a usbasp ISP programmer.

The Atmega32 datasheet (doc2503.pdf) can be found at www.atmel.com and is used throughout the code as reference material.

The UART functions will be added as part of next version of this program to demonstrate control from a PC.

 This code was tested and passed all tests

Created: 9/24/2015 6:04:59 PM
Author: lamarts
 
