#!/bin/bash

# ./run-deconfliction.sh <MODEL> <APPS> [--optlib <OPTLIB>]
#                                       [--weights <WEIGHTS>]
#
# where <MODEL> = 123apps (only one supported currently)
#       <APPS> = code with first letter of each competing app to run, apps are
#                (r)esilience, (d)ecarbonization, (p)rofit_cvr
#       <OPTLIB> = optional optimization library used for competing apps
#       <WEIGHTS> = optional base filename for optimization stage deconfliction
#                   application/device weighting factors. Application filename
#                   has "-app.json" appended and device filename has
#                   "-dev.json" appended. At least one of these files must
#                   exist in the deconfliction-pipeline directory if this
#                   option is specified. If both exist, device weighting factors
#                   override application weighting factors when specified.
#
# e.g.,
# ./run-deconfliction.sh 123apps rd

if [ "$#" -lt 2 ]; then
  echo "Usage: ./run-deconfliction.sh <model> <apps_code> [--optlib <opt_library>] [--weights <weights_basename>]"
  echo
  exit
fi

MODEL=$1
APPS=$2

OPTLIB="pulp"
WEIGHTS=""
# tricky logic to process optional optlib and weights arguments. This could
# be done in a more elegant way, but this gets the job done.
if [ "$#" -gt 2 ]; then
  if [ "$3" == "--optlib" ]; then
    OPTLIB=$4
  elif [ "$3" == "--weights" ]; then
    WEIGHTS=$4
  fi

  if [ "$#" -gt 4 ]; then
    if [ "$5" == "--optlib" ]; then
      OPTLIB=$6
    elif [ "$5" == "--weights" ]; then
      WEIGHTS=$6
    fi
  fi
fi

# magic that kills background jobs started in this script when the pipeline
# foreground process receives ctrl-C
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

# fire off GridLAB-D simulation
cd sim-starter
read -d "\n" SIMID SIMREQ <<< $(./sim-starter.py $MODEL)

# only optimization apps (not workflow) are supported for decon service
cd ../optimization-apps

if [[ $APPS == *"r"* || $APPS == *"R"* ]]; then
  ./run-resilience-gld.sh $SIMID "$SIMREQ" $OPTLIB >/dev/null &
fi

if [[ $APPS == *"d"* || $APPS == *"D"* ]]; then
  ./run-decarbonization-gld.sh $SIMID "$SIMREQ" $OPTLIB >/dev/null &
fi

cd ../deconfliction-pipeline
#./run-pipeline-gld.sh $SIMID "$SIMREQ" $WEIGHTS

trap - SIGINT SIGTERM EXIT
