#!/usr/bin/python
from __future__ import print_function
from Adafruit_MCP230xx import MCP230XX_GPIO
from Adafruit_MCP230xx import Adafruit_MCP230XX
from Adafruit_MCP230xx import *
from Adafruit_I2C import Adafruit_I2C
from Adafruit_CharLCD import Adafruit_CharLCD
import smbus, sys, os, random, getopt, re, atexit
import RPi.GPIO as GPIO, time, os
import Adafruit_DHT
from Adafruit_Thermal import *
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
from Adafruit_PWM_Servo_Driver import PWM
from Adafruit_IO import Client
import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART

print("starting up")

# Get the BLE provider for the current platform.
ble = Adafruit_BluefruitLE.get_provider()


#time
t0=time.time()
t0_climate=0
t_climate=30

#Adafruit IO
ADAFRUIT_IO_KEY = '954a29c4a56787437186d8c39c57a61d6c079867'
aio = Client(ADAFRUIT_IO_KEY)
data = aio.receive('Omnibot')

GPIO.setmode(GPIO.BCM)

#Thermal Printer
printer = Adafruit_Thermal("/dev/ttyAMA0", 19200, timeout=5)

#pwm
pwm2 = PWM(0x70)

#Motor Config
motor = Adafruit_MotorHAT(addr=0x63)
RMotor = motor.getMotor(1)
LMotor = motor.getMotor(2)
RMotor.setSpeed(50)
LMotor.setSpeed(50)


##MCP GPIO Pin Config##
#LCD 
lcd_address = 0x20
gpio_lcd = 8  # Number of GPIOs exposed by the MCP230xx chip, should be 8 or 16 depending on chip.
mcp_lcd = MCP230XX_GPIO(1, lcd_address, gpio_lcd)
lcd = Adafruit_CharLCD(pin_rs=1, pin_e=2, pins_db=[3,4,5,6], GPIO=mcp_lcd)
# MCP23017 General
mcp23017_address = 0x23  # I2C address of the MCP230xx chip.
mcp_gpio = Adafruit_MCP230XX(mcp23017_address, 16)
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
mcp_gpio.config(4, mcp_gpio.INPUT) #left wheel
mcp_gpio.config(5, mcp_gpio.INPUT) #right wheel
#IR Sensor
mcp_gpio.config(14, mcp_gpio.INPUT) #IR Sensor

#MCP3008
SPICLK = 18
SPIMISO = 23
SPIMOSI = 24
SPICS = 25
GPIO.setup(SPICLK, GPIO.OUT)
GPIO.setup(SPIMISO, GPIO.IN)
GPIO.setup(SPIMOSI, GPIO.OUT)
GPIO.setup(SPICS, GPIO.OUT)

##Analog Pin Config##
#analog pins
ang_but=1
ang_pot=2
ang_switch=3
ang_light=0
ang_pwr=7

#initial variables
global src
src = 0
global odo_state_left
global odo_count_left
odo_state_left=0
odo_count_left=0
global odo_state_right
odo_state_right=0
global odo_count_right
odo_count_right=0
global button
global switch
global potentiometer
global button_change
button =10
switch  =10
potentiometer=10


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

#Monitor and Control Charging
def pwr():
	global src
	global supply
	global chg_status
	supply =(mcp_gpio.input(12))
	if supply > .5 and src !=1:
		mcp_gpio.output(13, 1)
		src = 1
		mcp_gpio.output(2, 1)
	elif supply < .5 and src == 1:
		mcp_gpio.output(13, 0)
		src = 0

#Read Climate Sensor
def climate():
	global humidity
	global temperature
	global button
	global brightness
	humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT22, 20)
	temperature=temperature*9/5+32
	aio.send('Omnibot_Humidity', round(humidity,2))
	aio.send('Omnibot_Temperature', round(temperature,2))

##Motor Control and Sensing##
#Odometers
def odo_left():
	global odo_count_left
	global odo_state_left
	tempLeftState=mcp_gpio.input(4)
	if tempLeftState != odo_state_left and tempLeftState == 0:
		odo_count_left = odo_count_left+1
	odo_state_left = tempLeftState
def odo_right():
	global odo_count_right
	global odo_state_right
	tempRightState=mcp_gpio.input(5)
	if tempRightState != odo_state_right and tempRightState == 0:
		odo_count_right = odo_count_right+1
	odo_state_right = tempRightState
		
#LCD Color			
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

#Button Controller
def panel():
	global button
	global switch
	global potentiometer
	global button_change
	button_change=0
	temp_button=button
	temp_switch=switch
	temp_potentiometer=potentiometer
	switch=10
	temp = readadc(ang_but)
	if temp < 10 :
		button = 0
	elif temp > 240 and temp < 265:
		button = 100
	elif temp > 60 and temp < 80:
		button = 10
	elif temp >30 and temp < 40:
		button = 1
	elif temp > 295 and temp < 305:
		button = 110	 		
	elif temp > 275 and temp < 285:
		button = 101
	elif temp > 100 and temp < 110:
		button = 11
	elif temp > 310 and temp < 320:
		button = 111		
	temp=readadc(ang_switch) - readadc(ang_pot)/11
	if temp > 10 and temp < 55:
		switch = 1
	if temp > 55 and temp < 75:
		switch = 2		
	if temp > 75:
		switch = 3	
	potentiometer = readadc(ang_pot)
	if temp_button != button:
		button_change = 1

#Panel Mode		
def display_default():
	global button
	global button_change
	if button == 0 and button_change == 1:
		lcd.clear()
		lcd.message("standby mode")
	if button == 1 and button_change == 1:
		lcd.clear()
		lcd.message("T: %s" %(round(temperature, 2)))
		lcd.message("\nH: %s" %(round(humidity, 2)))

#Bluetooth Functions
def BLE():
    # Clear any cached data because both bluez and CoreBluetooth have issues with
    # caching data and it going stale.
    ble.clear_cached_data()

    # Get the first available BLE network adapter and make sure it's powered on.
    adapter = ble.get_default_adapter()
    adapter.power_on()
    print('Using adapter: {0}'.format(adapter.name))

    # Disconnect any currently connected UART devices.  Good for cleaning up and
    # starting from a fresh state.
    print('Disconnecting any connected UART devices...')
    UART.disconnect_devices()

    # Scan for UART devices.
    print('Searching for UART device...')
    try:
        adapter.start_scan()
        # Search for the first UART device found (will time out after 60 seconds
        # but you can specify an optional timeout_sec parameter to change it).
        device = UART.find_device()
        if device is None:
            raise RuntimeError('Failed to find UART device!')
    finally:
        # Make sure scanning is stopped before exiting.
        adapter.stop_scan()

    print('Connecting to device...')
    device.connect()  # Will time out after 60 seconds, specify timeout_sec parameter
                      # to change the timeout.

    # Once connected do everything else in a try/finally to make sure the device
    # is disconnected when done.
    try:
        # Wait for service discovery to complete for the UART service.  Will
        # time out after 60 seconds (specify timeout_sec parameter to override).
        print('Discovering services...')
        UART.discover(device)

        # Once service discovery is complete create an instance of the service
        # and start interacting with it.
        uart = UART(device)

        # Write a string to the TX characteristic.
        uart.write('Hello world!\r\n')
        print("Sent 'Hello world!' to the device.")

        # Now wait up to one minute to receive data from the device.
        print('Waiting up to 60 seconds to receive data from the device...')
        received = uart.read(timeout_sec=60)
        if received is not None:
            # Received data, print it out.
            print('Received: {0}'.format(received))
        else:
            # Timeout waiting for data, None is returned.
            print('Received no data!')
    finally:
        # Make sure device is disconnected on exit.
        device.disconnect()



##				
lcd_green()
lcd.clear()
lcd.message("Starting Up")
#RMotor.run(Adafruit_MotorHAT.FORWARD)
#LMotor.run(Adafruit_MotorHAT.FORWARD)
panel()
#climate()
ble.initialize()
ble.run_mainloop_with(main)
lcd.message(received)

#while (True):
#	pwr()
#	panel()
#	display_default()	
#	#if t_climate <= time.time() - t0_climate:
#	#	climate()
	#	t0_climate=time.time()
#	odo_right()
#	odo_left()
#	print(mcp_gpio.input(4))
#	print(mcp_gpio.input(5))
	print(odo_count_right)
	print(odo_count_left)		
	
		
		