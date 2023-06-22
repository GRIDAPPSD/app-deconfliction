#!/bin/bash

# ./run-deconfliction.sh <MODEL> <APPS> <METHOD> <DELAY>
#
# where <MODEL> = 123 (only one supported currently)
#       <APPS> = code with first letter of each competing app to run, apps are
#                (r)esilience, (d)ecarbonization, (p)rofit_cvr
#       <METHOD> = name of DeconflictionMethod class file to use with path
#                  referenced from deconfliction-pipeline directory
#       <DELAY> = optional seconds between simulation data messages, default 8
#
# e.g.,
#./run-deconfliction.sh 123 rd deconfliction-methods/compromise-rd-partial-method.py

MODEL=$1
APPS=$2
METHOD=$3

DELAY=8
if [ "$#" -gt 3 ]; then
  DELAY=$4
fi 

# magic that kills background jobs started in this script when the pipeline
# foreground process receives ctrl-C
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

cd optimization-apps

if [[ $APPS == *"r"* ]]; then
  ./run-resilience.sh $MODEL >/dev/null &
elif [[ $APPS == *"R"* ]]; then
  ./run-resilience.sh $MODEL >/dev/null &
fi

if [[ $APPS == *"d"* ]]; then
  ./run-decarbonization.sh $MODEL >/dev/null &
elif [[ $APPS == *"D"* ]]; then
  ./run-decarbonization.sh $MODEL >/dev/null &
fi

if [[ $APPS == *"p"* ]]; then
  ./run-profit.sh $MODEL >/dev/null &
elif [[ $APPS == *"P"* ]]; then
  ./run-profit.sh $MODEL >/dev/null &
fi

cd ../sim-starter
./run-sim.sh $MODEL $DELAY --wait >/dev/null &

cd ../deconfliction-pipeline
./run-pipeline.sh $MODEL ../$METHOD

