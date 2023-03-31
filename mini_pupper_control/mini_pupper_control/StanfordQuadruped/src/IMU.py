# Copyright 2023 MangDang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
