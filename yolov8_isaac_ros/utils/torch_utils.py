from typing import List, Tuple, Union

import torch
import torch.nn.functional as F
from torch import Tensor
from torchvision.ops import batched_nms, nms


def det_postprocess(data: Tuple[Tensor, Tensor, Tensor, Tensor]):
    assert len(data) == 4
    iou_thres: float = 0.65
    num_dets, bboxes, scores, labels = data[0][0], data[1][0], data[2][
        0], data[3][0]
    nums = num_dets.item()
    if nums == 0:
        return bboxes.new_zeros((0, 4)), scores.new_zeros(
            (0, )), labels.new_zeros((0, ))
    # check score negative
    scores[scores < 0] = 1 + scores[scores < 0]
    # add nms
    idx = nms(bboxes, scores, iou_thres)
    bboxes, scores, labels = bboxes[idx], scores[idx], labels[idx]
    bboxes = bboxes[:nums]
    scores = scores[:nums]
    labels = labels[:nums]

    return bboxes, scores, labels


def crop_mask(masks: Tensor, bboxes: Tensor) -> Tensor:
    n, h, w = masks.shape
    x1, y1, x2, y2 = torch.chunk(bboxes[:, :, None], 4, 1)  # x1 shape(1,1,n)
    r = torch.arange(w, device=masks.device,
                     dtype=x1.dtype)[None, None, :]  # rows shape(1,w,1)
    c = torch.arange(h, device=masks.device,
                     dtype=x1.dtype)[None, :, None]  # cols shape(h,1,1)

    return masks * ((r >= x1) * (r < x2) * (c >= y1) * (c < y2))
