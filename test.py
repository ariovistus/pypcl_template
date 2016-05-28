import libpclproc
import numpy

a = numpy.zeros((3,3,3),dtype='float32')
a[0,1,2] = 5
a[1,1,2] = 7
a[2,1,2] = 9

libpclproc.process(a)
