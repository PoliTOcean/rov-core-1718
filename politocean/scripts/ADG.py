
import smbus

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

def micro_stepping(select, mode):
    # select: 0 - 1
    # mode: 1 - 32 (see options below)
    adg_addr = 0x48

    options = {
               1 : [0, 0, 0],
               2 : [1, 0, 0],
               4 : [0, 1, 0],
               8 : [1, 1, 0],
               16 : [0, 0, 1],
               32 : [1, 0, 1],
               }

    bus = smbus.SMBus(1)

    reset()

    if select == 0:
        writeChannel(1, options[mode][0])
        writeChannel(2, options[mode][1])
        writeChannel(3, options[mode][2])
    elif select == 1:
        writeChannel(4, options[mode][0])
        writeChannel(5, options[mode][1])
<<<<<<< HEAD
        writeChannel(6, options[mode][2])
=======
        writeChannel(6, options[mode][2])
>>>>>>> bb3dfc76ac751478d9c85647fec0a347bf5c2a35
