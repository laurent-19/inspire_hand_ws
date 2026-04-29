#!/bin/bash
# Test predictions on multiple samples

MODEL_TYPE="${1:-pointnet}"
NUM_SAMPLES="${2:-5}"

echo "Testing $MODEL_TYPE model on $NUM_SAMPLES samples from each object"
echo "========================================================================"

# Test samples from different objects
OBJECTS=("record_250_1" "record_330_fat_1" "record_500_1" "record_small_bottle_1" "record_big_bottle_1")

for obj in "${OBJECTS[@]}"; do
    echo ""
    echo "Object: $obj"
    echo "------------------------------------------------------------------------"

    for i in $(seq 0 $((NUM_SAMPLES-1))); do
        sample_num=$(printf "%04d" $i)
        sample_path="training_data/non_deformable/$obj/sample_$sample_num"

        if [ -d "$sample_path" ]; then
            ./test_prediction.sh "$sample_path" "$MODEL_TYPE" | grep -E "(Ground Truth|Prediction|Error:)" | head -3
        fi
    done
done
