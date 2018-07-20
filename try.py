from SequentialLoader import SequentialLoader
import Motions

loader = SequentialLoader(Motions)
loader.stand()
loader.delay(5000)
loader.walk(10, 30)
