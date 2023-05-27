# file = ""

# with open(file) as f:
#     pass
import torch
import torch.nn as nn
import torch.nn.functional as F
batch_size = 2
patchmatch_num_sample = 3
height = 5
width = 5
max_depth = torch.tensor([10,10])
min_depth = torch.tensor([2,2])
inverse_max_depth = 1.0 / max_depth
inverse_min_depth = 1.0 / min_depth
a = torch.rand(
            size=(batch_size, patchmatch_num_sample, height, width)
        )
print(a)
print(torch.arange(start=0, end=patchmatch_num_sample, step=1).view(
            1, patchmatch_num_sample, 1, 1
        ))


depth_sample = torch.rand(
            size=(batch_size, patchmatch_num_sample, height, width)
        ) + torch.arange(start=0, end=patchmatch_num_sample, step=1).view(
            1, patchmatch_num_sample, 1, 1
        )

print(depth_sample)

depth_sample = inverse_max_depth.view(batch_size, 1, 1, 1) + depth_sample / patchmatch_num_sample * (
    inverse_min_depth.view(batch_size, 1, 1, 1) - inverse_max_depth.view(batch_size, 1, 1, 1)
)

out =  1.0 / depth_sample
print(out)
########################################
