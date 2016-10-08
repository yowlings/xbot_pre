#! /usr/bin/python
# -*- coding:utf-8 -*-

class P1():
    def foo(self):
        print 'p1-foo'

class P2():
    def foo(self):
        print 'p2-foo'
    def bar(self):
        print 'p2-bar'

class C1(P1,P2):
    pass

class C2(P1,P2):
    def bar(self):
        print 'C2-bar'

class D(C1,C2):
    pass


if __name__ =='__main__':
    d=D()
    d.foo()
    d.bar()