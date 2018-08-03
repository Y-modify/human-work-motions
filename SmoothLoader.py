import time
import math

class SmoothLoader(object):
    def __init__(self, motions, resolution=10):
        self.motions = motions
        self.resolution = resolution
        self.motions.stand = self.__stand
        self.__default_setServoPulse = self.motions.setServoPulse
        self.motions.setServoPulse = self.__setServoPulse

        self.current_joints = [None] * 20
        for ch, idx in self.motions.portmap.items():
            if idx is not None:
                self.current_joints[idx] = self.motions.stand_positions[ch]

    def __stand(self, ms=100):
        num_iterations = ms // self.resolution
        for t in range(num_iterations):
            start = time.time()
            for name, deg in self.motions.stand_positions.items():
                idx = self.motions.portmap[name]
                if idx is not None:
                    from_deg = self.current_joints[idx]
                    deg_to_set = from_deg + ((deg - from_deg) / num_iterations) * t
                    self.__default_setServoPulse(idx, deg_to_set)
            while (time.time() - start) < self.resolution / 1000:
                pass

        for ch, idx in self.motions.portmap.items():
            if idx is not None:
                self.current_joints[idx] = self.motions.stand_positions[ch]

    def __setServoPulse(self, idx, deg):
        if idx is not None:
            self.current_joints[idx] = deg
        self.__default_setServoPulse(idx, deg)

    def __getattr__(self, name):
        return getattr(self.motions, name)
