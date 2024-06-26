#!/bin/bash

if [ "$#" -lt 2 ]; then
  echo "Usage: ./run-resilience.sh <sim_id> <sim_request> [opt_library]"
  echo
  exit
fi

SIMID=$1
SIMREQ=$2

OPTLIB="pulp"
if [ "$#" -gt 2 ]; then
  OPTLIB=$3
fi

mkdir -p output
if [ "$OPTLIB" = "cvxpy" ] || [ "$OPTLIB" = "CVXPY" ]; then
  python3 optimization-app-cvxpy.py resilience $SIMID "$SIMREQ" 2>&1 | tee output/resilience-app.log
else
  python3 optimization-app-pulp.py resilience $SIMID "$SIMREQ" 2>&1 | tee output/resilience-app.log
fi

