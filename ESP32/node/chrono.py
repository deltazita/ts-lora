import time

class ChronoException(Exception):
    pass

class Chrono:

# state can be running, stopped
    def __init__(self):
        self._state = ''

    def start(self):
        self._state = 'running'
        self.strt = time.ticks_us()

    def read(self, order=1000000):
        self.now = 0
        if self._state == 'running':
            self.now = time.ticks_us()
        elif self._state == 'stopped':
            self.now = self.end
        else:
            raise ChronoException('not started')
        return (self.now - self.strt)/order

    def read_ms(self):
        return self.read(order=1000)

    def read_us(self):
        return self.read(order=1)

    def stop(self):
        self.end = time.ticks_us()

    def reset(self):
        self.start()
