#!/bin/bash

if [ "$#" -lt 2 ]; then
  echo "Usage: ./run-scalability.sh <sim_id> <sim_request> <line>"
  echo
  exit
fi

SIMID=$1
SIMREQ=$2
LINE=$3

mkdir -p log
# hardwire to resilience for the moment
python3 optimization-app-cvxpy-modular.py scalability $SIMID "$SIMREQ" $LINE 2>&1 | tee log/scalability-app-$LINE.log

