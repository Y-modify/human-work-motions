from AsyncLoader import AsyncLoader
from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket
import time
import yaml
from humanoid import Humanoid
import math
from Motions import Motions

with open('config/portmap.yml') as f:
    portmap = yaml.load(f)

with open('config/stand_positions.yml') as f:
    stand_positions = yaml.load(f)

humanoid = Humanoid("yamax.urdf", real=True)
motions = Motions(humanoid, portmap=portmap, stand_positions=stand_positions)
robot = AsyncLoader(motions)
robot.stand()

class Control(WebSocket):
    def handleMessage(self):
        print(self.data)
        token = self.data.split()
        if token[0] == 'stop':
            robot.stand()
            self.current_motion = None
        elif token[0] == 'pos':
            x = int(token[1])
            y = int(token[2])
            degree = math.atan2(x, y) * (180 / math.pi)
            distance = math.sqrt(x**2 + y**2)
            # print(distance)
            if -45 < degree <= 45:
                motion = "walk"
            elif 45 < degree <= 135:
                motion = "crabWalkingLeft"
            elif 135 < degree <=180 or -180 < degree <= -135:
                motion = "back"
            elif -135 < degree <= -45:
                motion = "crabWalkingRight"

            if self.current_motion == motion:
                return

            fake_inf = 1000000000000
            # speed = 60 - (distance / 4)
            getattr(robot, motion)(fake_inf, 20)
            self.current_motion = motion
        elif token[0] == 'sitdown':
            robot.sitDown()
            self.current_motion = 'sitdown'
        elif token[0] == 'situp':
            robot.sitUp()
            self.current_motion = 'situp'
        else:
            print('Unknown command')


    def handleConnected(self):
        robot.stand()
        self.current_motion = None
        print(self.address, 'connected')

    def handleClose(self):
        print(self.address, 'closed')

server = SimpleWebSocketServer('', 3333, Control)
print("starting")
server.serveforever()
