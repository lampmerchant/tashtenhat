"""A tool to set up a pty to act as a serial port that connects through TashTenHat's UART."""


from argparse import ArgumentParser
from fcntl import ioctl
import os
import pty
from select import select
import sys
from threading import Thread, Lock, Event


BAUD_RATES = (300, 600, 1200, 1800, 2400, 4800, 9600, 19200, 28800, 38400, 57600, 78800, 115200, 500000, 1000000, 2000000)
BAUD_RATES_STR = ', '.join(str(i) for i in BAUD_RATES)

I2C_STREAM_ADDR = 0x5A
I2C_CONFIG_STATUS_ADDR = 0x5B

IOCTL_I2C_TARGET = 0x0703  # From linux/i2c-dev.h


class SerialPortEmulator:
  """Device to shuffle bytes between the TashTenHat's UART and a pty that emulates a serial port."""
  
  def __init__(self, controller, i2c_device, baud_rate):
    if baud_rate not in BAUD_RATES: raise ValueError('baud_rate must be one of: %s' % BAUD_RATES_STR)
    self._baud_rate = baud_rate
    self._controller = controller
    os.set_blocking(self._controller, False)
    self._i2c_stream_handle = os.open(i2c_device, os.O_RDWR)
    ioctl(self._i2c_stream_handle, IOCTL_I2C_TARGET, I2C_STREAM_ADDR)
    self._i2c_config_status_handle = os.open(i2c_device, os.O_RDWR)
    ioctl(self._i2c_config_status_handle, IOCTL_I2C_TARGET, I2C_CONFIG_STATUS_ADDR)
    self._i2c_lock = Lock()
    self._i2c_write_has_space = Event()
  
  def _read_thread(self):
    """Funnel bytes from the pty to TashTenHat's UART."""
    
    while True:
      select((self._controller,), (), ())  # block until controller has data available
      self._i2c_write_has_space.wait()
      data = os.read(self._controller, 112)  # this should not block
      self._i2c_write_has_space.clear()
      with self._i2c_lock:
        os.write(self._i2c_stream_handle, data)
  
  def emulate(self):
    """Set TashTenHat's UART's baud rate, start a read daemon thread, and funnel bytes from the UART to the pty."""
    
    os.write(self._i2c_config_status_handle, bytes((BAUD_RATES.index(self._baud_rate),)))
    read_thread = Thread(target=self._read_thread, daemon=True)
    read_thread.start()
    while True:
      while True:
        self._i2c_lock.acquire()
        status = os.read(self._i2c_config_status_handle, 1)[0]
        if status >> 4 > 0: break
        self._i2c_lock.release()
        if status & 0xF != 0xF: self._i2c_write_has_space.set()
      data = os.read(self._i2c_stream_handle, status >> 4)
      self._i2c_lock.release()
      os.write(self._controller, data)


def main(argv):
  parser = ArgumentParser(description="Set up a pty to act as a serial port that connects through TashTenHat's UART.",
                          epilog='The path to the pty will be written to stdout; stdout will then be closed.')
  parser.add_argument('--baud_rate', metavar='BPS', type=int, default=115200, help='baud rate for the UART, must be one of: %s' %
                                                                                   BAUD_RATES_STR)
  parser.add_argument('i2c_device', help='i2c-dev device where TashTenHat is connected')
  args = parser.parse_args(argv[1:])
  controller, emulated_serial_port = pty.openpty()
  serial_port_emulator = SerialPortEmulator(controller, args.i2c_device, args.baud_rate)
  sys.stdout.write('%s' % os.ttyname(emulated_serial_port))
  sys.stdout.flush()
  sys.stdout.close()
  serial_port_emulator.emulate()


if __name__ == '__main__': sys.exit(main(sys.argv))
