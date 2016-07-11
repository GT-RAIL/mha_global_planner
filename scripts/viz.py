#!/usr/bin/python

import sys
import re
import matplotlib.pyplot as plt
# for visualizing a .mprim file

if len(sys.argv) == 2:
    input_file = sys.argv[1]
    f = open(input_file, 'r')
    print "writing to file ", f.name

    # skip the first 3 lines
    f.readline(); f.readline(); f.readline()


    for line in f.readlines():
      line = line.rstrip('\n')
      m = re.search('endpose_c: (-?[0-9]+) (-?[0-9]+) (-?[0-9]+)', line)
      if m:
        x = [0, m.group(1)]
        y = [0, m.group(2)]
        print m.groups()
        plt.plot(x, y)

    plt.show()

else:
  print 'please specify a file to read'
