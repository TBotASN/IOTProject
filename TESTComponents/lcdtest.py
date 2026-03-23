import smbus
import time

bus = smbus.SMBus(1)
addr = 0x3e

def command(cmd):
    bus.write_byte_data(addr, 0x00, cmd)
    time.sleep(0.002)

def data(val):
    bus.write_byte_data(addr, 0x40, val)
    time.sleep(0.001)

def init():
    time.sleep(0.05)
    command(0x38)
    command(0x39)
    command(0x14)
    command(0x70)
    command(0x56)
    command(0x6c)
    time.sleep(0.2)
    command(0x38)
    command(0x0C)
    command(0x01)
    time.sleep(0.01)

def set_cursor(line, pos):
    if line == 0:
        command(0x80 + pos)
    elif line == 1:
        command(0xC0 + pos)

def write(text):
    for c in text:
        data(ord(c))

init()

command(0x01)
time.sleep(0.01)
set_cursor(0, 0)
write("Fill: 75%")
set_cursor(1, 0)
write("Smart Bin")