#!/bin/bash

# ./run-deconfliction.sh <MODEL> <APPS> [--optlib <OPTLIB>]
#                                       [--interval <INTERVAL>]
#                                       [--weights <WEIGHTS>]
#
# where <MODEL> = 123apps (only one supported currently)
#       <APPS> = code with first letter of each competing app to run, apps are
#                (r)esilience, (m)ax_local, (c)vr
#       <OPTLIB> = optional optimization library used for competing apps
#       <INTERVAL> = optional interval in seconds at which competing apps
#                    perform optimizations and send setpoints requests to
#                    the pipeline process. Must be a multiple of 3 for
#                    GridLAB-D simulation compatability.
#       <WEIGHTS> = optional base filename for optimization stage deconfliction
#                   application/device weighting factors. Application filename
#                   has "-app.json" appended and device filename has
#                   "-dev.json" appended. At least one of these files must
#                   exist in the deconfliction-pipeline directory if this
#                   option is specified. If both exist, device weighting factors
#                   override application weighting factors when specified.
#
# e.g.,
# ./run-deconfliction.sh 123apps rm

if [ "$#" -lt 2 ]; then
  echo "Usage: ./run-deconfliction.sh <model> <apps_code> [--optlib <opt_library>] [--interval <interval_sec>] [--weights <weights_basename>]"
  echo
  exit
fi

MODEL=$1
APPS=$2

#OPTLIB="pulp"
# GDB 9/11/24: CVXPY will be needed if we add support for doing optimizations
# for cooperation after the FY24 deliverable
OPTLIB="cvxpy"
# GDB 3/12/25: If no interval is specified here or passed in then the apps
# default to 15 or every 5th simulation timestamp. Another good value is 9
# or every 3rd simulation timestamp.
#INTERVAL="9"
INTERVAL=""
WEIGHTS=""
# tricky logic to process optional optlib and weights arguments. This could
# be done in a more elegant way, but this gets the job done.
if [ "$#" -gt 2 ]; then
  if [ "$3" == "--optlib" ]; then
    OPTLIB=$4
  elif [ "$3" == "--interval" ]; then
    INTERVAL=$4
  elif [ "$3" == "--weights" ]; then
    WEIGHTS=$4
  fi

  if [ "$#" -gt 4 ]; then
    if [ "$5" == "--optlib" ]; then
      OPTLIB=$6
    elif [ "$5" == "--interval" ]; then
      INTERVAL=$6
    elif [ "$5" == "--weights" ]; then
      WEIGHTS=$6
    fi
  fi

  if [ "$#" -gt 6 ]; then
    if [ "$7" == "--optlib" ]; then
      OPTLIB=$8
    elif [ "$7" == "--interval" ]; then
      INTERVAL=$8
    elif [ "$7" == "--weights" ]; then
      WEIGHTS=$8
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

if [[ $APPS == "s" || $APPS == "S" ]]; then
  # scalability task invocation
  numlines=`grep -c . app_setup.csv`
  for ((line=2; line<=$numlines; line++)); do
    echo $line
    ./run-scalability.sh $SIMID "$SIMREQ" $line >/dev/null &
  done
fi

if [[ $APPS == *"r"* || $APPS == *"R"* ]]; then
  ./run-resilience.sh $SIMID "$SIMREQ" $OPTLIB $INTERVAL >/dev/null &
fi

if [[ $APPS == *"m"* || $APPS == *"M"* ]]; then
  ./run-max_local.sh $SIMID "$SIMREQ" $OPTLIB $INTERVAL >/dev/null &
fi

if [[ $APPS == *"c"* || $APPS == *"C"* ]]; then
  ./run-cvr.sh $SIMID "$SIMREQ" $OPTLIB $INTERVAL >/dev/null &
fi

cd ../deconfliction-pipeline
./run-pipeline.sh $SIMID "$SIMREQ" $INTERVAL $WEIGHTS

trap - SIGINT SIGTERM EXIT
