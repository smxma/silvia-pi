#!/usr/bin/python

# Raspberry Pi SPI Port and Device
spi_port = 1
spi_dev = 0

# Added config 
CS = 22
CLK = 23
DO = 21

# Pin # for relay connected to heating element
he_pin = 26

# Default goal temperature
set_temp = 221.

# Default sleep/wake times
sched_enabled = False
wake_time = '06:30'
sleep_time = '10:00'

# Main loop sample rate in seconds
sample_time = 0.1

# PID Proportional, Integral, and Derivative values
Pc = 3.4
Ic = 0.3
Dc = 40.0

Pw = 2.9
Iw = 0.3
Dw = 40.0

#Web/REST Server Options
port = 8082
host = "0.0.0.0"
server ="cheroot"
