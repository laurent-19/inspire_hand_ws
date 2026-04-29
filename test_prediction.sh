#!/bin/bash
# Simple script to test radius prediction

SAMPLE_PATH="$1"
MODEL_TYPE="${2:-joint}"

if [ -z "$SAMPLE_PATH" ]; then
    echo "Usage: ./test_prediction.sh <sample_path> [model_type]"
    echo ""
    echo "Examples:"
    echo "  ./test_prediction.sh training_data/non_deformable/record_250_1/sample_0000"
    echo "  ./test_prediction.sh training_data/non_deformable/record_500_1/sample_0005 pointnet"
    exit 1
fi

# Determine checkpoint based on model type
if [ "$MODEL_TYPE" = "joint" ]; then
    CHECKPOINT="radius_prediction/checkpoints/best_joint_mid_bottle.pt"
elif [ "$MODEL_TYPE" = "pointnet" ]; then
    CHECKPOINT="radius_prediction/checkpoints/best_pointnet_mid_bottle.pt"
else
    echo "Unknown model type: $MODEL_TYPE"
    echo "Use 'joint' or 'pointnet'"
    exit 1
fi

# Check if checkpoint exists
if [ ! -f "$CHECKPOINT" ]; then
    echo "Checkpoint not found: $CHECKPOINT"
    echo "Please train the model first."
    exit 1
fi

# Run prediction
source venv/bin/activate
python -m radius_prediction.predict "$SAMPLE_PATH" --checkpoint "$CHECKPOINT" --model_type "$MODEL_TYPE"
