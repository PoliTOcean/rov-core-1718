#!/usr/bin/env python

import smbus

adg_addr = 0x48

options = {
           1 : [0, 0, 0],
           2 : [1, 0, 0],
           4 : [0, 1, 0],
           8 : [1, 1, 0],
           16 : [0, 0, 1],
           32 : [1, 0, 1],
           }

def reset():
    bus.write_byte(adg_addr, 0x00)

def setBit(v, index, x):
  """Set the index:th bit of v to 1 if x is truthy, else to 0, and return the new value."""
  mask = 1 << index   # Compute mask, an integer with just bit 'index' set.
  v &= ~mask          # Clear the bit indicated by the mask (if x is False)
  if x:
    v |= mask         # If x was True, set the bit indicated by the mask.
  return v 

def ch(channel): #assume input is 1-8 -> 0-7
    channel = channel-1
    
    if channel < 0:
        channel = 0
    elif channel > 7:
        channel = 7
        
    return channel
    

def writeChannel(channel, state):
    value = bus.read_byte(adg_addr)
    value = setBit(value, ch(channel), state)
    bus.write_byte(adg_addr, value)

bus = smbus.SMBus(1)

reset()

mode = input("Scegliere modalita (1:x, x = 1, 2, 4, 8, 16, 32) ")

writeChannel(1, options[mode][0])
writeChannel(2, options[mode][1])
writeChannel(3, options[mode][2])

writeChannel(4, options[mode][0])
writeChannel(5, options[mode][1])
writeChannel(6, options[mode][2])
