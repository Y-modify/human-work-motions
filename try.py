import yaml
from humanoid import Humanoid
from SequentialLoader import SequentialLoader
from AsyncLoader import AsyncLoader
from Motions import Motions
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-r', '--real', action='store_true', help="Non-simulation environment")
parser.add_argument('-a', '--async', action='store_true', help="Asynchronous execution")
parser.add_argument('-p', '--portmap', default='config/portmap.yml', type=str, help="Portmap file")
parser.add_argument('-s', '--stand-positions', default='config/stand_positions.yml', type=str, help="Stand positions file")
parser.add_argument('-u', '--urdf', default='yamax.urdf', type=str, help="Robot URDF")
args = parser.parse_args()

with open(args.portmap) as f:
    portmap = yaml.load(f)

with open(args.stand_positions) as f:
    stand_positions = yaml.load(f)

if args.real:
    robot = Humanoid(args.urdf, real=True)
else:
    import pybullet
    pybullet.connect(pybullet.GUI)
    robot = Humanoid(args.urdf, bullet_client=pybullet)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_MOUSE_PICKING, 0)
    pybullet.resetDebugVisualizerCamera(0.7 + 1, 75, -15, [0, 0, 0])

motions = Motions(robot, portmap=portmap, stand_positions=stand_positions)
if not robot.is_real:
    motions = SequentialLoader(motions)

if args.async:
    motions = AsyncLoader(motions)

motions.stand()

while True:
    print('> ', end='')
    user_input = input().split()
    action = user_input[0]
    arguments = map(int, user_input[1:])
    try:
        getattr(motions, action)(*arguments)
    except AttributeError:
        print(f'No such motion: {action}')
    except KeyboardInterrupt:
        print(f'Interrupted')
        motions.stand()
