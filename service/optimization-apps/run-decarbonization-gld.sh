#!/bin/bash

SIMID=$1
SIMREQ=$2

mkdir -p output
if [ "$3" = "cvxpy" ] || [ "$3" = "CVXPY" ]; then
  python3 optimization-app-cvxpy-gld.py decarbonization $SIMID "$SIMREQ" 2>&1 | tee output/decarbonization-app.log
else
  python3 optimization-app-pulp-gld.py decarbonization $SIMID "$SIMREQ" 2>&1 | tee output/decarbonization-app.log
fi

