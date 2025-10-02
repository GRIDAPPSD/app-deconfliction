#!/bin/bash

if [ "$#" -lt 2 ]; then
  echo "Usage: ./run-delay.sh <sim_id> <sim_request>"
  echo
  exit
fi

SIMID=$1
SIMREQ=$2

trap - SIGINT SIGTERM EXIT

mkdir -p log
rm -f log/delay-app-messages.log
python3 delay-app.py $SIMID "$SIMREQ" 2>&1 | tee log/delay-app.log

