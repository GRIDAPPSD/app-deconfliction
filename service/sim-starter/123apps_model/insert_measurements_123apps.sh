#!/bin/bash
for f in `ls -1 ieee123_app_deconfliction_*.txt`
do
  python3 src_python/cimhub/InsertMeasurementsOld.py cimhubconfig.json $f
done
