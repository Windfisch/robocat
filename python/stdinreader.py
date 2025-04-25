import sys
import select

class StdinReader:

    def __init__(self):
        self._selpoll = select.poll()
        self._selpoll.register(sys.stdin, select.POLLIN)
        self._bytes = bytes()
        self._cr = False

    def getline(self):
       while len(self._selpoll.poll(0)):
            self._bytes += sys.stdin.buffer.read(1)  # get a single byte
            if self._bytes[-1] == ord('\n') or self._bytes[-1] == ord('\r'):
                b, prev_cr = self._bytes, self._cr
                self._cr = b[-1] == ord('\r')
                self._bytes = bytes()
                if not (self._cr and b[-1] == ord('\n')):
                    string = b[:-1].decode()
                    return string
            return None
