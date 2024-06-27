#!/bin/bash

if [ "$#" -lt 2 ]; then
  echo "Usage: ./run-decarbonization.sh <sim_id> <sim_request> [opt_library]"
  echo
  exit
fi

SIMID=$1
SIMREQ=$2

OPTLIB="pulp"
if [ "$#" -gt 2 ]; then
  OPTLIB=$3
fi

INTERVAL=""
if [ "$#" -gt 3 ]; then
  INTERVAL=$4
fi

mkdir -p log
if [ "$OPTLIB" = "cvxpy" ] || [ "$OPTLIB" = "CVXPY" ]; then
  python3 optimization-app-cvxpy.py decarbonization $SIMID "$SIMREQ" $INTERVAL 2>&1 | tee log/decarbonization-app.log
else
  python3 optimization-app-pulp.py decarbonization $SIMID "$SIMREQ" $INTERVAL 2>&1 | tee log/decarbonization-app.log
fi

