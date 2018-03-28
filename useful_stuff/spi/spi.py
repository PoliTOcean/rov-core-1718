import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)

comm = 2

try:
        while True:
                print(comm)
                resp = spi.xfer2([comm])
                time.sleep(1)
                print(resp)
                comm = comm + 1
                if comm > 15:
                        comm = 0

except KeyboardInterrupt:
        spi.close()
