#!/bin/bash
for arg in "$@"
do
  display $arg &
done
