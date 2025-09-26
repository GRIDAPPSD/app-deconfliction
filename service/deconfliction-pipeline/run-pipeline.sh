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

INTERVAL=""
if [ "$#" -gt 2 ]; then
  INTERVAL="--interval=""$3"
fi

WEIGHTS=""
if [ "$#" -gt 3 ]; then
  WEIGHTS="--weights=""$4"
fi

mkdir -p log
# must remove the existing log file if it existst or otherwise it will just
# append to what is already there
rm -f log/deconfliction-pipeline.log log/deconfliction-pipeline-messages.log
python3 deconfliction-pipeline.py $SIMID "$SIMREQ" $INTERVAL $WEIGHTS

