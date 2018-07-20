import time
import math
from collections import OrderedDict


class ActionBuffer(object):
    def __init__(self):
        self._timepoints = OrderedDict({0: []})

    def add_timepoint(self, ms):
        self._timepoints[list(self._timepoints.keys())[-1] + ms] = []

    def add_action(self, func, fargs):
        key = list(self._timepoints.keys())[-1]
        self._timepoints[key].append((func, fargs))

    def do_at_t(self, t):
        timepoint = min((i for i in self._timepoints.keys() if i <= t),
                        key=lambda x: t - x)
        for f, fargs in self._timepoints[timepoint]:
            f(*fargs)

        for k in list(self._timepoints.keys()):
            if k < timepoint:
                del self._timepoints[k]

        self._timepoints[timepoint] = []

    def is_empty(self):
        return len(self._timepoints) == 1


class LegacyLoader(object):
    def __init__(self, motions):
        self.motions = motions
        self.motions.delay = self.__delay
        self.motions.setServoPulse = self.__setServoPulse

    def __delay(self, ms):
        self._buffer.add_timepoint(ms)

    def __setServoPulse(self, idx, deg):
        if idx is not None:
            self._buffer.add_action(
                self.robot.set_joint_state, (idx - 8, (deg - 90) / 180 * math.pi))

    def __getattr__(self, name):
        attr = getattr(self.motions, name)
        if not callable(attr):
            return attr

        def __do(*args, **kwargs):
            self._buffer = ActionBuffer()
            attr(*args, **kwargs)
            start = time.time()
            while not self._buffer.is_empty():
                self.motions.robot.step()
                self._buffer.do_at_t((time.time() - start) * 1000)
        return __do
