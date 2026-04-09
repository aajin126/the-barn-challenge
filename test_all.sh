#!/bin/bash

OUT_FILE="out.txt"

for i in $(seq 0 299)
do
    echo "Running world ${i}"
    python3 run.py --world_idx ${i} --gui --out ${OUT_FILE}

    sleep 5
done
