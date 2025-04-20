class GaitController:
    def __init__(self, config):
        self.config = config

    def phase_index(self, ticks):
        """
        Calculate the current gait phase index given the elapsed ticks.

        Parameters
        ----------
        ticks : int
            Number of timesteps since the program started.
        gaitparams : GaitParams
            GaitParams object.

        Returns
        -------
        int
            Index of the current gait phase.

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
        Calculate the number of ticks elapsed since the start of the current phase.

        Parameters
        ----------
        ticks : int
            Number of timesteps since the program started.
        gaitparams : GaitParams
            GaitParams object.

        Returns
        -------
        int
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
        Calculate which feet should be in contact at the given tick.

        Parameters
        ----------
        ticks : int
            Number of timesteps since the program started.
        gaitparams : GaitParams
            GaitParams object.

        Returns
        -------
        numpy.ndarray
            Boolean vector of contact states (4,), where 1 indicates stance and 0 indicates flight.

        """
        return self.config.contact_phases[:, self.phase_index(ticks)]
