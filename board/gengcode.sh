#!/bin/bash

set -e

filename=$(basename -- "$1")
filename="${filename%.*}"

kicad-cli pcb export gerbers -o output -l B.Cu $1
kicad-cli pcb export drill $1 
mv $filename.drl output/$filename.drl

mkdir -p gcode

docker run --rm -i -t -v ".:/data" ptodorov/pcb2gcode --back="output/$filename-B_Cu.gbl" --drill="output/$filename.drl"
