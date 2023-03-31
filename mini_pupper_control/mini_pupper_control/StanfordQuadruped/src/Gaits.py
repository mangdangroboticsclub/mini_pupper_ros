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


class GaitController:
    def __init__(self, config):
        self.config = config

    def phase_index(self, ticks):
        """
        Calculates which part of the gait cycle
        the robot should be in given the time in ticks.

        Parameters
        ----------
        ticks : int
            Number of timesteps since the program started
        gaitparams : GaitParams
            GaitParams object

        Returns
        -------
        Int
            The index of the gait phase that the robot should be in.
        """
        phase_time = ticks % self.config.phase_length
        phase_sum = 0
        for i in range(self.config.num_phases):
            phase_sum += self.config.phase_ticks[i]
            if phase_time < phase_sum:
                return i
        assert False

    def subphase_ticks(self, ticks):
        """
        Calculates the number of ticks (timesteps)
        since the start of the current phase.

        Parameters
        ----------
        ticks : Int
            Number of timesteps since the program started
        gaitparams : GaitParams
            GaitParams object

        Returns
        -------
        Int
            Number of ticks since the start of the current phase.
        """
        phase_time = ticks % self.config.phase_length
        phase_sum = 0
        subphase_ticks = 0
        for i in range(self.config.num_phases):
            phase_sum += self.config.phase_ticks[i]
            if phase_time < phase_sum:
                subphase_ticks = phase_time - \
                    phase_sum + self.config.phase_ticks[i]
                return subphase_ticks
        assert False

    def contacts(self, ticks):
        """
        Calculates which feet should be in contact at the given number of ticks

        Parameters
        ----------
        ticks : Int
            Number of timesteps since the program started.
        gaitparams : GaitParams
            GaitParams object

        Returns
        -------
        numpy array (4,)
            Numpy vector with 0 indicating flight and 1 indicating stance.
        """
        return self.config.contact_phases[:, self.phase_index(ticks)]
