"""
3D IoU Calculation and Rotated NMS
Written by Shaoshuai Shi
All Rights Reserved 2019-2020.
"""
import torch

from ...utils import common_utils
#from . import iou3d_nms_cuda







import numpy as np
from shapely.geometry import Polygon

def box_to_polygon(box):
    """
    将3D框转为BEV多边形
    :param box: [x, y, z, dx, dy, dz, heading]
    :return: Shapely Polygon对象
    """
    center = box[:2]
    dimensions = box[3:5]
    angle = box[6]
    
    # 计算旋转矩阵
    cos_a = np.cos(angle)
    sin_a = np.sin(angle)
    rotation_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]])

    # 生成原始顶点
    half_l, half_w = dimensions[0]/2, dimensions[1]/2
    corners = np.array([
        [-half_l, -half_w],
        [half_l, -half_w],
        [half_l, half_w],
        [-half_l, half_w]
    ])

    # 应用旋转和平移
    rotated_corners = np.dot(corners, rotation_matrix.T) + center
    return Polygon(rotated_corners)
    
def custom_boxes_iou_bev_cpu(boxes_a, boxes_b):
    """
    CPU版BEV IoU计算（支持批量计算）
    :param boxes_a: (N,7) [x,y,z,dx,dy,dz,heading]
    :param boxes_b: (M,7)
    :return: IoU矩阵 (N,M)
    """
    # 转换输入为numpy数组
    boxes_a = boxes_a.cpu().numpy() if isinstance(boxes_a, torch.Tensor) else boxes_a
    boxes_b = boxes_b.cpu().numpy() if isinstance(boxes_b, torch.Tensor) else boxes_b
    
    iou_matrix = np.zeros((len(boxes_a), len(boxes_b)))
    
    # 预计算所有多边形
    polys_a = [box_to_polygon(box) for box in boxes_a]
    polys_b = [box_to_polygon(box) for box in boxes_b]

    # 并行计算IoU矩阵
    for i in range(len(polys_a)):
        for j in range(len(polys_b)):
            intersection = polys_a[i].intersection(polys_b[j]).area
            union = polys_a[i].area + polys_b[j].area - intersection
            iou_matrix[i][j] = intersection / (union + 1e-8)
            
    return torch.from_numpy(iou_matrix)

def custom_boxes_overlap_bev(boxes_a, boxes_b):
    """替代iou3d_nms_cuda.boxes_overlap_bev_gpu[1,6](@ref)"""
    overlaps = np.zeros((len(boxes_a), len(boxes_b)))
    
    # 预计算所有多边形
    polys_a = [bev_box_to_polygon(b) for b in boxes_a]
    polys_b = [bev_box_to_polygon(b) for b in boxes_b]
    
    # 双重循环计算重叠矩阵
    for i in range(len(polys_a)):
        for j in range(len(polys_b)):
            if polys_a[i].intersects(polys_b[j]):
                inter_area = polys_a[i].intersection(polys_b[j]).area
                overlaps[i][j] = inter_area
    return overlaps

def custom_nms_cpu(boxes, scores, thresh):
    """
    CPU版旋转NMS实现
    :param boxes: (N,7) [x,y,z,dx,dy,dz,heading]
    :param scores: (N)
    :param thresh: IoU阈值
    :return: 保留的索引
    """
    #order = scores.argsort()[::-1]
    order = scores.argsort(descending=True)
    keep = []
    while len(order) > 0:
        i = order[0]
        keep.append(i)
        
        # 计算当前框与剩余框的IoU
        ious = custom_boxes_iou_bev_cpu(boxes[i:i+1], boxes[order[1:]])
        
        # 筛选低重叠框
        inds = np.where(ious.numpy().ravel() <= thresh)[0]
        order = order[inds + 1]
        
    return torch.tensor(keep)

def custom_nms_normal(boxes, scores, thresh):
    """替代iou3d_nms_cuda.nms_normal_gpu[1,11](@ref)
    参数：
        boxes: (N,7) [x,y,z,dx,dy,dz,heading]
        scores: (N)
        thresh: IoU阈值
    返回：
        keep: 保留框的索引列表"""
    order = np.argsort(scores)[::-1]
    keep = []
    
    while order.size > 0:
        i = order[0]
        keep.append(i)
        
        # 计算当前框与剩余框的BEV IoU
        ious = []
        for j in order[1:]:
            poly_i = bev_box_to_polygon(boxes[i])
            poly_j = bev_box_to_polygon(boxes[j])
            
            if poly_i.intersects(poly_j):
                inter = poly_i.intersection(poly_j).area
                union = poly_i.area + poly_j.area - inter
                ious.append(inter / (union + 1e-8))
            else:
                ious.append(0)
        
        # 筛选低重叠框
        inds = np.where(np.array(ious) <= thresh)[0]
        order = order[inds + 1]
    return keep





def boxes_bev_iou_cpu(boxes_a, boxes_b):
    """
    Args:
        boxes_a: (N, 7) [x, y, z, dx, dy, dz, heading]
        boxes_b: (M, 7) [x, y, z, dx, dy, dz, heading]

    Returns:
        ans_iou: (N, M)
    """
    boxes_a, is_numpy = common_utils.check_numpy_to_torch(boxes_a)
    boxes_b, is_numpy = common_utils.check_numpy_to_torch(boxes_b)
    assert not (boxes_a.is_cuda or boxes_b.is_cuda), 'Only support CPU tensors'
    assert boxes_a.shape[1] == 7 and boxes_b.shape[1] == 7
    ans_iou = boxes_a.new_zeros(torch.Size((boxes_a.shape[0], boxes_b.shape[0])))
    #iou3d_nms_cuda.boxes_iou_bev_cpu(boxes_a.contiguous(), boxes_b.contiguous(), ans_iou)
    ans_iou = custom_boxes_iou_bev_cpu(boxes_a.contiguous(), boxes_b.contiguous())
    return ans_iou.numpy() if is_numpy else ans_iou


def boxes_iou_bev(boxes_a, boxes_b):
    """
    Args:
        boxes_a: (N, 7) [x, y, z, dx, dy, dz, heading]
        boxes_b: (M, 7) [x, y, z, dx, dy, dz, heading]

    Returns:
        ans_iou: (N, M)
    """
    assert boxes_a.shape[1] == boxes_b.shape[1] == 7
    ans_iou = torch.FloatTensor(torch.Size((boxes_a.shape[0], boxes_b.shape[0]))).zero_()

    #iou3d_nms_cuda.boxes_iou_bev_gpu(boxes_a.contiguous(), boxes_b.contiguous(), ans_iou)
    ans_iou = custom_boxes_iou_bev_cpu(boxes_a.contiguous(), boxes_b.contiguous())
    return ans_iou


def boxes_iou3d_gpu(boxes_a, boxes_b):
    """
    Args:
        boxes_a: (N, 7) [x, y, z, dx, dy, dz, heading]
        boxes_b: (M, 7) [x, y, z, dx, dy, dz, heading]

    Returns:
        ans_iou: (N, M)
    """
    assert boxes_a.shape[1] == boxes_b.shape[1] == 7

    # height overlap
    boxes_a_height_max = (boxes_a[:, 2] + boxes_a[:, 5] / 2).view(-1, 1)
    boxes_a_height_min = (boxes_a[:, 2] - boxes_a[:, 5] / 2).view(-1, 1)
    boxes_b_height_max = (boxes_b[:, 2] + boxes_b[:, 5] / 2).view(1, -1)
    boxes_b_height_min = (boxes_b[:, 2] - boxes_b[:, 5] / 2).view(1, -1)

    # bev overlap
    overlaps_bev = torch.FloatTensor(torch.Size((boxes_a.shape[0], boxes_b.shape[0]))).zero_()  # (N, M)
    #iou3d_nms_cuda.boxes_overlap_bev_gpu(boxes_a.contiguous(), boxes_b.contiguous(), overlaps_bev)
    overlaps_bev = custom_boxes_overlap_bev(boxes_a.contiguous(), boxes_b.contiguous())

    max_of_min = torch.max(boxes_a_height_min, boxes_b_height_min)
    min_of_max = torch.min(boxes_a_height_max, boxes_b_height_max)
    overlaps_h = torch.clamp(min_of_max - max_of_min, min=0)

    # 3d iou
    overlaps_3d = overlaps_bev * overlaps_h

    vol_a = (boxes_a[:, 3] * boxes_a[:, 4] * boxes_a[:, 5]).view(-1, 1)
    vol_b = (boxes_b[:, 3] * boxes_b[:, 4] * boxes_b[:, 5]).view(1, -1)

    iou3d = overlaps_3d / torch.clamp(vol_a + vol_b - overlaps_3d, min=1e-6)

    return iou3d


def nms_gpu(boxes, scores, thresh, pre_maxsize=None, **kwargs):
    """
    :param boxes: (N, 7) [x, y, z, dx, dy, dz, heading]
    :param scores: (N)
    :param thresh:
    :return:
    """
    assert boxes.shape[1] == 7
    order = scores.sort(0, descending=True)[1]
    if pre_maxsize is not None:
        order = order[:pre_maxsize]

    boxes = boxes[order].contiguous()
    keep = torch.LongTensor(boxes.size(0))
    #num_out = iou3d_nms_cuda.nms_gpu(boxes, keep, thresh)
    num_out = custom_nms_cpu(boxes, keep, thresh)
    return order[keep[:num_out]].contiguous(), None


def nms_normal_gpu(boxes, scores, thresh, **kwargs):
    """
    :param boxes: (N, 7) [x, y, z, dx, dy, dz, heading]
    :param scores: (N)
    :param thresh:
    :return:
    """
    assert boxes.shape[1] == 7
    order = scores.sort(0, descending=True)[1]

    boxes = boxes[order].contiguous()

    keep = torch.LongTensor(boxes.size(0))
    #num_out = iou3d_nms_cuda.nms_normal_gpu(boxes, keep, thresh)
    num_out = custom_nms_normal(boxes, keep, thresh)
    return order[keep[:num_out]].contiguous(), None
