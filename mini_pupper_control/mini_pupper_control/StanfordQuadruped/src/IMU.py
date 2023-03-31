# Copyright 2023 stanfordroboticsclub
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import serial
import numpy as np
import time


class IMU:
    def __init__(self, port, baudrate=500000):
        self.serial_handle = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0,
        )
        self.last_quat = np.array([1, 0, 0, 0])
        self.start_time = time.time()

    def flush_buffer(self):
        self.serial_handle.reset_input_buffer()

    def read_orientation(self):
        """Reads quaternion measurements from the Teensy until none are left. Returns the last read quaternion.

        Parameters
        ----------
        serial_handle : Serial object
            Handle to the pyserial Serial object

        Returns
        -------
        np array (4,)
            If there was quaternion data to read on the serial port returns the quaternion as a numpy array, otherwise returns the last read quaternion.
        """

        while True:
            x = self.serial_handle.readline().decode("utf").strip()
            if x == "" or x is None:
                return self.last_quat
            else:
                parsed = x.split(",")
                if len(parsed) == 4:
                    self.last_quat = np.array(parsed, dtype=np.float64)
                else:
                    print("Did not receive 4-vector from imu")
