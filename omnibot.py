#!/usr/bin/python

from Adafruit_MCP230xx import MCP230XX_GPIO
from Adafruit_MCP3008
import RPi.GPIO as GPIO, time, os


mcp23017_address = 0x23  # I2C address of the MCP230xx chip.
lcd_address = 0x20

gpio_mcp23017 = 16
gpio_lcd = 8  # Number of GPIOs exposed by the MCP230xx chip, should be 8 or 16 depending on chip.

# Create MCP230xx GPIO adapter.
mcp_lcd = MCP230XX_GPIO(1, lcd_address, gpio_lcd)
mcp_gpio = MCP230XX_GPIO(1, mcp23017_address, gpio_mcp23017)

    mcp_gpio.config(0, mcp.OUTPUT) #lcd red
    mcp_gpio.config(1, mcp.OUTPUT) #lcd green
    mcp_gpio.config(15, mcp.OUTPUT) #lcd blue


# Create LCD, passing in MCP GPIO adapter.
lcd = Adafruit_CharLCD(pin_rs=1, pin_e=2, pins_db=[3,4,5,6], GPIO=mcp_lcd)

def lcd_red()
	mcp.output(0, 1)  # Pin 0 Low
	mcp.output(1, 0)  # Pin 0 Low
	mcp.output(15, 0)  # Pin 0 Low
def lcd_green()
	mcp.output(0, 0)  # Pin 0 Low
	mcp.output(1, 1)  # Pin 0 Low
	mcp.output(15, 0)  # Pin 0 Low
def lcd_blue()
	mcp.output(0, 0)  # Pin 0 Low
	mcp.output(1, 0)  # Pin 0 Low
	mcp.output(15, 1)  # Pin 0 Low
	
lcd.clear()
lcd.message("Standby Mode\n")
