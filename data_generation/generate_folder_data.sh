#!/bin/bash

for file in maps/*.pgm;
do
name=${file##*/}
base=${name%.pgm}
./generate_map_data.sh $base
done
