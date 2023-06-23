#!/bin/bash

# ./run-deconfliction.sh <MODEL> <APPS> <METHOD> <DELAY>
#
# where <MODEL> = 123 (only one supported currently)
#       <APPS> = code with first letter of each competing app to run, apps are
#                (r)esilience, (d)ecarbonization, (p)rofit_cvr
#       <METHOD> = name of DeconflictionMethod class file to use with path
#                  referenced from deconfliction-pipeline directory
#       <DELAY> = optional positive value is seconds between simulation data
#                 messages, negative value is a count of apps being run,
#                 e.g., -3 for 3 apps, where counting device dispatch messages
#                 triggers simulation data messages, default=counting messages
#
# e.g.,
#./run-deconfliction.sh 123 rd deconfliction-methods/compromise-rd-partial-method.py

MODEL=$1
APPS=$2
METHOD=$3

# magic that kills background jobs started in this script when the pipeline
# foreground process receives ctrl-C
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

# choose workflow or optimization apps
if [[ $APPS == *"w"* ]]; then
  cd workflow-apps
elif [[ $APPS == *"W"* ]]; then
  cd workflow-apps
else
  cd optimization-apps
fi

delay_app_counter=0

if [[ $APPS == *"r"* ]]; then
  ((delay_app_counter--))
  ./run-resilience.sh $MODEL >/dev/null &
elif [[ $APPS == *"R"* ]]; then
  ((delay_app_counter--))
  ./run-resilience.sh $MODEL >/dev/null &
fi

if [[ $APPS == *"d"* ]]; then
  ((delay_app_counter--))
  ./run-decarbonization.sh $MODEL >/dev/null &
elif [[ $APPS == *"D"* ]]; then
  ((delay_app_counter--))
  ./run-decarbonization.sh $MODEL >/dev/null &
fi

if [[ $APPS == *"p"* ]]; then
  ((delay_app_counter--))
  ./run-profit.sh $MODEL >/dev/null &
elif [[ $APPS == *"P"* ]]; then
  ((delay_app_counter--))
  ./run-profit.sh $MODEL >/dev/null &
fi

DELAY=$delay_app_counter
if [ "$#" -gt 3 ]; then
  DELAY=$4
fi

cd ../sim-starter

# don't start sim-sim with 0 delay since that means the user will be
# running it separately and feeding data interactively
if [ "$DELAY" -lt 0 ]; then
  ./run-sim.sh $MODEL $DELAY >/dev/null &
elif [ "$DELAY" -gt 0 ]; then
  ./run-sim.sh $MODEL $DELAY --wait >/dev/null &
fi

cd ../deconfliction-pipeline
./run-pipeline.sh $MODEL ../$METHOD

