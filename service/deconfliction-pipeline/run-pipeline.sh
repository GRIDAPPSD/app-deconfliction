#!/bin/bash

# requires at least a reference to the type of simulation to use
if [ "$#" -lt 2 ]; then
  echo "Usage: ./run-pipeline.sh <sim_id> <sim_request> [weights_basename]"
  echo
  exit
fi

SIMID=$1
SIMREQ=$2

WEIGHTS=""
if [ "$#" -gt 2 ]; then
  WEIGHTS="--weights=""$3"
fi

INTERVAL=""
if [ "$#" -gt 3 ]; then
  INTERVAL="--interval=""$4"
fi

mkdir -p log
python3 deconfliction-pipeline.py $SIMID "$SIMREQ" $WEIGHTS $INTERVAL 2>&1 | tee log/deconfliction-pipeline.log

