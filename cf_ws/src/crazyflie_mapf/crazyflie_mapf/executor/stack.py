"""Small local adapter over the shared Crazyflie Python client."""

from crazyflie_py import Crazyswarm


class DroneStack:
    def __init__(self):
        self.swarm = Crazyswarm()
        self.allcfs = self.swarm.allcfs

    def names(self):
        return [cf.prefix.strip('/') for cf in self.allcfs.crazyflies]

    def get(self, name):
        for cf in self.allcfs.crazyflies:
            if cf.prefix.strip('/') == name:
                return cf
        raise KeyError(name)

    def takeoff(self, height, duration, wait=False):
        self.allcfs.takeoff(targetHeight=height, duration=duration)
        if wait:
            self.swarm.timeHelper.sleep(duration)

    def land(self, height, duration, wait=False):
        self.allcfs.land(targetHeight=height, duration=duration)
        if wait:
            self.swarm.timeHelper.sleep(duration)
