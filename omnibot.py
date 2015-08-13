#!/usr/bin/python
from __future__ import print_function
from Adafruit_MCP230xx import MCP230XX_GPIO
from Adafruit_MCP230xx import Adafruit_MCP230XX
from Adafruit_MCP230xx import *
from Adafruit_I2C import Adafruit_I2C
from Adafruit_CharLCD import Adafruit_CharLCD
import smbus, sys, os, random, getopt, re
import RPi.GPIO as GPIO, time, os
import Adafruit_DHT
from Adafruit_Thermal import *
from Adafruit_IO import Client
print("starting up")

ADAFRUIT_IO_KEY = '954a29c4a56787437186d8c39c57a61d6c079867'

# Create an instance of the REST client.
aio = Client(ADAFRUIT_IO_KEY)


# Send a string value 'bar' to the feed 'Foo', again creating it if it doesn't 
# exist already.
aio.send('Omnibot', 'bar')

# Now read the most recent value from the feed 'Test'.  Notice that it comes
# back as a string and should be converted to an int if performing calculations
# on it.


# Finally read the most revent value from feed 'Foo'.
data = aio.receive('Omnibot')
print('Retrieved value from Test has attributes: {0}'.format(data))
print('Latest value from Test: {0}'.format(data.value))


GPIO.setmode(GPIO.BCM)


printer = Adafruit_Thermal("/dev/ttyAMA0", 19200, timeout=5)
#LCD 
lcd_address = 0x20
gpio_lcd = 8  # Number of GPIOs exposed by the MCP230xx chip, should be 8 or 16 depending on chip.
mcp_lcd = MCP230XX_GPIO(1, lcd_address, gpio_lcd)
lcd = Adafruit_CharLCD(pin_rs=1, pin_e=2, pins_db=[3,4,5,6], GPIO=mcp_lcd)

# MCP23017
mcp23017_address = 0x23  # I2C address of the MCP230xx chip.
mcp_gpio = Adafruit_MCP230XX(mcp23017_address, 16)

#MCP3008
SPICLK = 18
SPIMISO = 23
SPIMOSI = 24
SPICS = 25
GPIO.setup(SPICLK, GPIO.OUT)
GPIO.setup(SPIMISO, GPIO.IN)
GPIO.setup(SPIMOSI, GPIO.OUT)
GPIO.setup(SPICS, GPIO.OUT)

#analog pins
ang_but=1
ang_pot=2
ang_switch=3
ang_light=0
ang_pwr=7
#initial variables
global src
src = 0



#lcd colors
mcp_gpio.config(0, mcp_gpio.OUTPUT) #lcd red
mcp_gpio.config(1, mcp_gpio.OUTPUT) #lcd green
mcp_gpio.config(8, mcp_gpio.OUTPUT) #lcd blue
#led 
mcp_gpio.config(2, mcp_gpio.OUTPUT) #led green
mcp_gpio.config(3, mcp_gpio.OUTPUT) #led red
#bumpers
mcp_gpio.config(6, mcp_gpio.INPUT) #Bumper 1
mcp_gpio.config(7, mcp_gpio.INPUT) #Bumper 2
#power
mcp_gpio.config(11, mcp_gpio.INPUT) #chg status
mcp_gpio.config(12, mcp_gpio.INPUT) #src 
mcp_gpio.config(13, mcp_gpio.OUTPUT) #power switch
#odometer
mcp_gpio.config(4, mcp_gpio.INPUT) #input A
mcp_gpio.config(5, mcp_gpio.INPUT) #input B
#IR Sensor
mcp_gpio.config(15, mcp_gpio.INPUT) #IR Sensor

# Create LCD, passing in MCP GPIO adapter.
def readadc(adcnum):
	if ((adcnum > 7) or (adcnum < 0)):
		return -1
	GPIO.output(SPICS, True)
	GPIO.output(SPICLK, False)  # start clock low
	GPIO.output(SPICS, False)     # bring CS low
	commandout = adcnum
	commandout |= 0x18  # start bit + single-ended bit
	commandout <<= 3    # we only need to send 5 bits here
	for i in range(5):
		if (commandout & 0x80):
			GPIO.output(SPIMOSI, True)
		else:
   			GPIO.output(SPIMOSI, False)
                commandout <<= 1
                GPIO.output(SPICLK, True)
                GPIO.output(SPICLK, False)
	adcout = 0
	# read in one empty bit, one null bit and 10 ADC bits
	for i in range(12):
		GPIO.output(SPICLK, True)
		GPIO.output(SPICLK, False)
		adcout <<= 1
		if (GPIO.input(SPIMISO)):
			adcout |= 0x1
	GPIO.output(SPICS, True)
	adcout /= 2       # first bit is 'null' so drop it
	return adcout

def pwr():
	global src
	global supply
	global chg_status
	supply =(mcp_gpio.input(12))
	if supply > .5 and src !=1:
		mcp_gpio.output(13, 1)
		src = 1
	elif supply < .5 and src == 1:
		mcp_gpio.output(13, 0)
		src = 0

def temp_hum():
	global humidity
	global temperature
	humidity, temperature = Adafruit_DHT.read_retry(22, 20)

def lcd_red():
	mcp_gpio.output(0, 0)  # Pin 0 Low
	mcp_gpio.output(1, 1)  # Pin 0 Low
	mcp_gpio.output(8, 1)  # Pin 0 Low
def lcd_green():
	mcp_gpio.output(0, 1)  # Pin 0 Low
	mcp_gpio.output(1, 0)  # Pin 0 Low
	mcp_gpio.output(8, 1)  # Pin 0 Low
def lcd_blue():
	mcp_gpio.output(0, 1)  # Pin 0 Low
	mcp_gpio.output(1, 1)  # Pin 0 Low
	mcp_gpio.output(8, 0)  # Pin 0 Low

lcd_green()
temp_hum()
lcd.clear()
lcd.message("%s" %(temperature))
src = 0
while 1:
	pwr()
	time.sleep(.5)
	