#!/bin/bash

if [ "$#" -lt 2 ]; then
  echo "Usage: ./run-max_local.sh <sim_id> <sim_request> [opt_library]"
  echo
  exit
fi

SIMID=$1
SIMREQ=$2

# special case invocation to fire off a simulation and run the app standalone
if [ "$2" = "standalone" ]; then
  read -d "\n" SIMID SIMREQ <<< $(../sim-starter/sim-starter.py $1)
fi

OPTLIB="cvxpy"
if [ "$#" -gt 2 ]; then
  OPTLIB=$3
fi

INTERVAL=""
if [ "$#" -gt 3 ]; then
  INTERVAL=$4
fi

mkdir -p log
if [ "$OPTLIB" = "pulp" ] || [ "$OPTLIB" = "PuLP" ] || [ "$OPTLIB" = "PULP" ]; then
  python3 optimization-app-pulp.py max_local $SIMID "$SIMREQ" $INTERVAL 2>&1 | tee log/max_local-app.log
else
  python3 optimization-app-cvxpy-modular.py max_local $SIMID "$SIMREQ" $INTERVAL 2>&1 | tee log/max_local-app.log
  #python3 optimization-app-cvxpy-noreact.py max_local $SIMID "$SIMREQ" $INTERVAL 2>&1 | tee log/max_local-app.log
  #NO LONGER RUNNABLE WITHOUT UPDATES python3 optimization-app-cvxpy.py max_local $SIMID "$SIMREQ" $INTERVAL 2>&1 | tee log/max_local-app.log
fi

