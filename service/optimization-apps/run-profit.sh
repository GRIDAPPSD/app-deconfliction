#!/bin/bash

if [ "$#" -lt 2 ]; then
  echo "Usage: ./run-profit.sh <sim_id> <sim_request> [opt_library]"
  echo
  exit
fi

SIMID=$1
SIMREQ=$2

# special case invocation to fire off a simulation and run the app standalone
if [ "$2" = "standalone" ]; then
  read -d "\n" SIMID SIMREQ <<< $(../sim-starter/sim-starter.py $1)
fi

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
  python3 optimization-app-cvxpy.py profit_cvr $SIMID "$SIMREQ" $INTERVAL 2>&1 | tee log/profit_cvr-app.log
else
  python3 optimization-app-pulp.py profit_cvr $SIMID "$SIMREQ" $INTERVAL 2>&1 | tee log/profit_cvr-app.log
fi

