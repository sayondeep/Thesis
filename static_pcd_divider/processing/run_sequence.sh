#!/bin/bash

# Run the programs in sequence for 100 times
for ((i=1; i<=1; i++))
do
    echo "Running iteration $i"

    python3 yamltoxy.py

    # Run pcd_probability.py
    python3 pcd_probability.py

    # Run pcd_is_updated.py
    python3 pcd_is_updated.py

    # Run pcd_validity.py
    python3 pcd_validity.py

    # Run pcd_cache_tile.py
    python3 pcd_cache_tile.py
done