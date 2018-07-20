from SequentialLoader import SequentialLoader
import Motions

motions = Motions(real=False)
loader = SequentialLoader(motions)
loader.stand()
loader.delay(5000)
loader.walk(10, 30)
