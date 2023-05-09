#!/usr/bin/env python

import sys

if len(sys.argv) < 3:
  print('ERROR: must specify input and output files as command line arguments!\n')
  exit()

infile = sys.argv[1]
outfile = sys.argv[2]

if infile == outfile:
  print('ERROR: input and output files must be different!\n')
  exit()

padding = '                                                                   '
indent = 0

with open(infile, 'r') as inf:
  with open(outfile, 'w') as outf:
    for line in inf:
      if line.startswith('#INDENT START '):
        indent = int(line[14:-1])
        continue
      elif line.startswith('#INDENT FINISH'):
        indent = 0
        continue

      if indent == 0:
        outf.write(line)
      elif indent > 0:
        outf.write(padding[:indent] + line)
      elif indent < 0:
        outf.write(line[-indent:])

