#！/usr/bin/env python
# -*- coding: UTF-8 -*-

class A:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        print("initing")
    def showPara(self):
        print(self.x)
    
class B(A):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    def showP(self):
        print(self.z)

obj1 = A(2, 3)
obj1.showPara()
obj2 = B(1, 2 ,3)
obj2.showP()
print(issubclass(B, A))
