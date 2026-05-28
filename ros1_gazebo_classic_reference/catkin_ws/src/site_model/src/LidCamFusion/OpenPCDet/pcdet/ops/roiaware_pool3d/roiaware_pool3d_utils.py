import torch
import torch.nn as nn
from torch.autograd import Function

from ...utils import common_utils
#from . import roiaware_pool3d_cuda

def custom_points_in_boxes_cpu(points, boxes):
    """
    CPU版点云在3D框内检测
    算法原理：分离轴定理（SAT）的3D扩展[6,7](@ref)
    """
    # 初始化输出矩阵 (N_boxes, N_points)
    point_indices = np.zeros((boxes.shape[0], points.shape[0]), dtype=np.int32)
    
    for box_idx, box in enumerate(boxes):
        # 解算框参数
        center = box[:3]
        dims = box[3:6]
        heading = box[6]
        
        # 生成旋转矩阵
        rot_mat = np.array([
            [np.cos(heading), -np.sin(heading), 0],
            [np.sin(heading), np.cos(heading), 0],
            [0, 0, 1]
        ])
        
        # 将点转换到局部坐标系
        local_points = points - center
        local_points = np.dot(local_points, rot_mat.T)
        
        # 分离轴定理检测
        in_x = np.abs(local_points[:,0]) <= dims[0]/2
        in_y = np.abs(local_points[:,1]) <= dims[1]/2 
        in_z = np.abs(local_points[:,2]) <= dims[2]/2
        
        point_indices[box_idx] = (in_x & in_y & in_z).astype(np.int32)
        
    return point_indices
def points_in_boxes_cpu(points, boxes):
    """
    Args:
        points: (num_points, 3)
        boxes: [x, y, z, dx, dy, dz, heading], (x, y, z) is the box center, each box DO NOT overlaps
    Returns:
        point_indices: (N, num_points)
    """
    assert boxes.shape[1] == 7
    assert points.shape[1] == 3
    points, is_numpy = common_utils.check_numpy_to_torch(points)
    boxes, is_numpy = common_utils.check_numpy_to_torch(boxes)

    point_indices = points.new_zeros((boxes.shape[0], points.shape[0]), dtype=torch.int)
    #roiaware_pool3d_cuda.points_in_boxes_cpu(boxes.float().contiguous(), points.float().contiguous(), point_indices)
    point_indices = custom_points_in_boxes_cpu(boxes.float().contiguous(), points.float().contiguous())
    return point_indices.numpy() if is_numpy else point_indices


def points_in_boxes_gpu(points, boxes):
    """
    :param points: (B, M, 3)
    :param boxes: (B, T, 7), num_valid_boxes <= T
    :return box_idxs_of_pts: (B, M), default background = -1
    """
    assert boxes.shape[0] == points.shape[0]
    assert boxes.shape[2] == 7 and points.shape[2] == 3
    batch_size, num_points, _ = points.shape

    box_idxs_of_pts = points.new_zeros((batch_size, num_points), dtype=torch.int).fill_(-1)
    #roiaware_pool3d_cuda.points_in_boxes_gpu(boxes.contiguous(), points.contiguous(), box_idxs_of_pts)
    point_indices = custom_points_in_boxes_cpu(boxes.float().contiguous(), points.float().contiguous())
    return box_idxs_of_pts


class RoIAwarePool3d(nn.Module):
    def __init__(self, out_size, max_pts_each_voxel=128):
        super().__init__()
        self.out_size = out_size
        self.max_pts_each_voxel = max_pts_each_voxel

    def forward(self, rois, pts, pts_feature, pool_method='max'):
        assert pool_method in ['max', 'avg']
        return RoIAwarePool3dFunction.apply(rois, pts, pts_feature, self.out_size, self.max_pts_each_voxel, pool_method)


"""class RoIAwarePool3dFunction(Function):
    @staticmethod
    def forward(ctx, rois, pts, pts_feature, out_size, max_pts_each_voxel, pool_method):
        
        #Args:
        #    ctx:
        #    rois: (N, 7) [x, y, z, dx, dy, dz, heading] (x, y, z) is the box center
        #    pts: (npoints, 3)
        #    pts_feature: (npoints, C)
        #    out_size: int or tuple, like 7 or (7, 7, 7)
        #    max_pts_each_voxel:
        #    pool_method: 'max' or 'avg'

        #Returns:
        #    pooled_features: (N, out_x, out_y, out_z, C)
        
        assert rois.shape[1] == 7 and pts.shape[1] == 3
        if isinstance(out_size, int):
            out_x = out_y = out_z = out_size
        else:
            assert len(out_size) == 3
            for k in range(3):
                assert isinstance(out_size[k], int)
            out_x, out_y, out_z = out_size

        num_rois = rois.shape[0]
        num_channels = pts_feature.shape[-1]
        num_pts = pts.shape[0]

        pooled_features = pts_feature.new_zeros((num_rois, out_x, out_y, out_z, num_channels))
        argmax = pts_feature.new_zeros((num_rois, out_x, out_y, out_z, num_channels), dtype=torch.int)
        pts_idx_of_voxels = pts_feature.new_zeros((num_rois, out_x, out_y, out_z, max_pts_each_voxel), dtype=torch.int)

        pool_method_map = {'max': 0, 'avg': 1}
        pool_method = pool_method_map[pool_method]
        roiaware_pool3d_cuda.forward(rois, pts, pts_feature, argmax, pts_idx_of_voxels, pooled_features, pool_method)

        ctx.roiaware_pool3d_for_backward = (pts_idx_of_voxels, argmax, pool_method, num_pts, num_channels)
        return pooled_features

    @staticmethod
    def backward(ctx, grad_out):
        
        #:param grad_out: (N, out_x, out_y, out_z, C)
        #:return:
        #    grad_in: (npoints, C)
        
        pts_idx_of_voxels, argmax, pool_method, num_pts, num_channels = ctx.roiaware_pool3d_for_backward

        grad_in = grad_out.new_zeros((num_pts, num_channels))
        roiaware_pool3d_cuda.backward(pts_idx_of_voxels, argmax, grad_out.contiguous(), grad_in, pool_method)

        return None, None, grad_in, None, None, None"""
        
class RoIAwarePool3dFunction(Function):
    @staticmethod
    def forward(ctx, rois, pts, pts_feature, out_size, max_pts_each_voxel, pool_method):
        """
        CPU前向传播实现
        算法原理：体素网格遍历与特征聚合[1,11](@ref)
        """
        # 初始化输出张量
        num_rois = rois.shape[0]
        num_channels = pts_feature.shape[-1]
        if isinstance(out_size, int):
            out_x = out_y = out_z = out_size
        else:
            out_x, out_y, out_z = out_size
        
        pooled_features = torch.zeros((num_rois, out_x, out_y, out_z, num_channels))
        argmax = torch.zeros_like(pooled_features, dtype=torch.int32)
        pts_idx_of_voxels = torch.zeros((num_rois, out_x, out_y, out_z, max_pts_each_voxel), dtype=torch.int32)
        
        for roi_idx in range(num_rois):
            # 解算ROI参数
            center = rois[roi_idx, :3]
            dims = rois[roi_idx, 3:6]
            heading = rois[roi_idx, 6]
            
            # 生成旋转矩阵
            rot_mat = torch.tensor([
                [np.cos(heading), -np.sin(heading), 0],
                [np.sin(heading), np.cos(heading), 0],
                [0, 0, 1]
            ], dtype=torch.float32)
            
            # 坐标转换到局部坐标系
            local_pts = pts - center
            local_pts = torch.mm(local_pts, rot_mat.t())
            
            # 计算体素尺寸
            voxel_size = dims / torch.tensor([out_x, out_y, out_z])
            
            # 遍历每个点
            for pt_idx in range(local_pts.shape[0]):
                # 计算体素坐标
                voxel_x = ((local_pts[pt_idx, 0] + dims[0]/2) // voxel_size[0]).clamp(0, out_x-1).long()
                voxel_y = ((local_pts[pt_idx, 1] + dims[1]/2) // voxel_size[1]).clamp(0, out_y-1).long()
                voxel_z = ((local_pts[pt_idx, 2] + dims[2]/2) // voxel_size[2]).clamp(0, out_z-1).long()
                
                # 记录点索引
                pos = pts_idx_of_voxels[roi_idx, voxel_x, voxel_y, voxel_z].nonzero().size(0)
                if pos < max_pts_each_voxel:
                    pts_idx_of_voxels[roi_idx, voxel_x, voxel_y, voxel_z, pos] = pt_idx
                    
            # 特征聚合
            for x in range(out_x):
                for y in range(out_y):
                    for z in range(out_z):
                        indices = pts_idx_of_voxels[roi_idx, x, y, z]
                        valid_indices = indices[indices != 0]
                        if valid_indices.size(0) == 0:
                            continue
                            
                        features = pts_feature[valid_indices]
                        if pool_method == 'max':
                            pooled, argmax_idx = torch.max(features, dim=0)
                        else:
                            pooled = torch.mean(features, dim=0)
                            argmax_idx = -1
                            
                        pooled_features[roi_idx, x, y, z] = pooled
                        argmax[roi_idx, x, y, z] = argmax_idx if argmax_idx != -1 else 0
        
        ctx.save_for_backward(pts_idx_of_voxels, argmax)
        ctx.pool_method = pool_method
        return pooled_features

    @staticmethod
    def backward(ctx, grad_out):
        """
        CPU反向传播实现
        """
        pts_idx_of_voxels, argmax = ctx.saved_tensors
        grad_in = torch.zeros_like(ctx.pts_feature)
        
        # 梯度回传逻辑
        if ctx.pool_method == 'max':
            for roi_idx in range(grad_out.shape[0]):
                for x in range(grad_out.shape[1]):
                    for y in range(grad_out.shape[2]):
                        for z in range(grad_out.shape[3]):
                            idx = argmax[roi_idx, x, y, z]
                            grad_in[idx] += grad_out[roi_idx, x, y, z]
        else:
            for roi_idx in range(grad_out.shape[0]):
                for x in range(grad_out.shape[1]):
                    for y in range(grad_out.shape[2]):
                        for z in range(grad_out.shape[3]):
                            indices = pts_idx_of_voxels[roi_idx, x, y, z]
                            valid_indices = indices[indices != 0]
                            if valid_indices.size(0) == 0:
                                continue
                            grad_in[valid_indices] += grad_out[roi_idx, x, y, z] / valid_indices.size(0)
        
        return None, None, grad_in, None, None, None

if __name__ == '__main__':
    pass
