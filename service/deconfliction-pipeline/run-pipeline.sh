#!/bin/bash

# requires at least a reference to the type of simulation to use
if [ "$#" -lt 2 ]; then
  echo "Usage: ./run-pipeline.sh <sim_id> <sim_request> [weights_basename]"
  echo
  exit
fi

SIMID=$1
SIMREQ=$2

# special case invocation to fire off a simulation and run the pipeline standalone
if [ "$2" = "standalone" ]; then
  read -d "\n" SIMID SIMREQ <<< $(../sim-starter/sim-starter.py $1)
fi

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

