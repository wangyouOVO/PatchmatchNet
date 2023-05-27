#输入图片序列文件夹
IMAGES_PATH="/home/wt/Downloads/dataset/book"
#输出路径
OUTPUT_PATH="/home/wt/Projects/PatchmatchNet/output"

CHECKPOINT_FILE="./checkpoints/params_000007.ckpt"

./build/SfMTest $IMAGES_PATH $OUTPUT_PATH

#!/usr/bin/env bash

python eval.py --scan_list "./scan_list.txt" --input_folder=$OUTPUT_PATH --checkpoint_path $CHECKPOINT_FILE --num_views 3 --image_max_dim 800 --geo_mask_thres 3 --photo_thres 0.8