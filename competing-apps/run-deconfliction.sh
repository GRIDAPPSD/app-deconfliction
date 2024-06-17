#!/bin/bash

# ./run-deconfliction.sh <MODEL> <APPS> <METHOD> <DELAY>
#
# where <MODEL> = 123
#       <APPS> = code with first letter of each competing app to run, apps are
#                (r)esilience, (d)ecarbonization, (p)rofit_cvr
#       <METHOD> = name of DeconflictionMethod class file to use with path
#                  referenced from deconfliction-pipeline directory
#       <DELAY> = optional positive value is seconds between simulation data
#                 messages, negative value is a count of apps being run,
#                 e.g., -3 for 3 apps, where counting device dispatch messages
#                 until all apps respond triggers simulation data messages,
#                 0 value to not run sim-sim meaning it must be run separately,
#                 default=counting messages
#
# e.g.,
# ./run-deconfliction.sh 123 rd deconfliction-methods/compromise-rd-method.py

MODEL=$1
APPS=$2
METHOD=$3

if [ "$#" -gt 3 ]; then
  OPTLIB=$4
else
  OPTLIB="pulp"
fi

# magic that kills background jobs started in this script when the pipeline
# foreground process receives ctrl-C
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

# choose workflow or optimization apps
if [[ $APPS == *"w"* || $APPS == *"W"* ]]; then
  cd workflow-apps
else
  cd optimization-apps
fi

DELAY=0
SYNC=""
if [ "$#" -gt 4 ]; then
  DELAY=$5
else
  SYNC="--sync"
fi

delay_app_counter=0

if [[ $APPS == *"r"* || $APPS == *"R"* ]]; then
  ((delay_app_counter--))
  ./run-resilience.sh $MODEL $OPTLIB $SYNC >/dev/null &
fi

if [[ $APPS == *"d"* || $APPS == *"D"* ]]; then
  ((delay_app_counter--))
  ./run-decarbonization.sh $MODEL $OPTLIB $SYNC >/dev/null &
fi

if [[ $APPS == *"p"* || $APPS == *"P"* ]]; then
  ((delay_app_counter--))
  ./run-profit.sh $MODEL $OPTLIB $SYNC >/dev/null &
fi

if [[ $APPS == *"l"* || $APPS == *"L"* ]]; then
  ((delay_app_counter--))
  ./run-loadshed.sh $MODEL $OPTLIB $SYNC >/dev/null &
fi

if [[ $SYNC == "--sync" ]]; then
  DELAY=$delay_app_counter
fi

cd ../sim-starter

# don't start sim-sim with 0 delay since that means the user will be
# running it separately and feeding data interactively
if [ "$DELAY" -eq 0 ]; then
  echo "*** Not starting sim-sim due to DELAY=0 ***"
else
  ./run-sim.sh $MODEL $DELAY >/dev/null &
fi

cd ../deconfliction-pipeline
./run-pipeline.sh $MODEL ../$METHOD $SYNC

