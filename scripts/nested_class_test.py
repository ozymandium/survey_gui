#!/usr/bin/env python
class A:
    a1 = 1

    class B:
        def __init__(self):
            self.b1 = A.a1

    def __init__(self):
        A.a1 = 2
        self.B_ = A.B()

        print str(self.B_.b1)

if __name__ == '__main__':
    instance = A()