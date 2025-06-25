#!/bin/bash

if [ "$#" -lt 2 ]; then
  echo "Usage: ./run-scalability.sh <sim_id> <sim_request> <line>"
  echo
  exit
fi

SIMID=$1
SIMREQ=$2
LINENO=$3

mkdir -p log

# extract the specifed line out of app_setup.csv
LINE=`awk NR==$LINENO app_setup.csv`
# extract first column to get AppName for log filename
APPNAME=`echo $LINE | cut -f1 -d,`

python3 optimization-app-cvxpy-modular.py scalability $SIMID "$SIMREQ" "$LINE" 2>&1 | tee log/$APPNAME-app.log

