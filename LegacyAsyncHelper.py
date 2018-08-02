from threading import (Event, Thread)
import legacy

class StopThread(Exception):
    pass

class LegacyAsyncHelper(object):
    def __init__(self):
        self.motions = legacy
        self.motions.delay = self.__delay
        self.stop = Event()
        self.thread = None

    def __delay(self, ms):
        if self.stop.wait(ms/1000):
            self.stop.clear()
            raise StopThread()

    def do_async(self, f, args=(), kwargs={}):
        def __f():
            try:
                f(*args, **kwargs)
            except StopThread:
                return

        if self.thread:
            while self.thread.is_alive():
                self.stop.set()

        self.stop.clear()
        self.thread = Thread(target=__f)
        self.thread.start()

    def __getattr__(self, name):
        attr = getattr(self.motions, name)
        if not callable(attr):
            return attr

        def __do(*args, **kwargs):
            self.do_async(attr, args, kwargs)
        return __do
