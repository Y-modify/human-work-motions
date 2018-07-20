from humanoid import Humanoid
from SequentialLoader import SequentialLoader
from Motions import Motions

real = False

if real:
    robot = Humanoid("yamax.urdf", real=True)
else:
    import pybullet
    pybullet.connect(pybullet.GUI)
    robot = Humanoid("yamax.urdf", bullet_client=pybullet)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_MOUSE_PICKING, 0)
    pybullet.resetDebugVisualizerCamera(0.7 + 1, 75, -15, [0, 0, 0])

motions = Motions(robot)
loader = SequentialLoader(motions)
loader.stand()
loader.delay(5000)
loader.walk(10, 30)
