import torch
from torch.autograd import Variable
from torch.autograd import Function
import torch.nn as nn
from typing import List

#from . import pointnet2_stack_cuda as pointnet2
from . import pointnet2_utils
class CPUVoxelQuery(Function):
    @staticmethod
    def forward(ctx, max_range, radius, nsample, xyz, new_xyz, new_coords, point_indices):
        """
        CPU体素查询实现
        原理：基于体素网格的邻域搜索与特征聚合
        """
        B, Z, Y, X = point_indices.shape
        M = new_coords.shape[0]
        device = xyz.device
        
        # 初始化输出张量
        idx = torch.full((M, nsample), -1, dtype=torch.int32, device=device)
        empty_ball_mask = torch.zeros(M, dtype=torch.bool, device=device)

        # 坐标转换参数
        z_range, y_range, x_range = max_range
        voxel_size = torch.tensor([Z, Y, X], device=device)
        
        # 主处理循环
        for m in range(M):
            batch_id, z, y, x = new_coords[m]
            batch_id = int(batch_id)
            
            # 计算查询范围
            z_start = max(0, int(z - z_range))
            z_end = min(Z, int(z + z_range) + 1)
            y_start = max(0, int(y - y_range))
            y_end = min(Y, int(y + y_range) + 1)
            x_start = max(0, int(x - x_range))
            x_end = min(X, int(x + x_range) + 1)
            
            # 收集候选点索引
            candidates = []
            for vz in range(z_start, z_end):
                for vy in range(y_start, y_end):
                    for vx in range(x_start, x_end):
                        pt_idx = point_indices[batch_id, vz, vy, vx]
                        if pt_idx >= 0:
                            candidates.append(pt_idx)
            
            if len(candidates) == 0:
                empty_ball_mask[m] = True
                continue
                
            # 距离计算与筛选
            candidate_xyz = xyz[candidates]
            dist = torch.norm(candidate_xyz - new_xyz[m], dim=1)
            valid_mask = dist < radius
            valid_candidates = torch.tensor(candidates)[valid_mask]
            
            # 采样处理
            if len(valid_candidates) >= nsample:
                selected = torch.randperm(len(valid_candidates))[:nsample]
            else:
                selected = torch.cat([torch.arange(len(valid_candidates))] * 
                                    (nsample//len(valid_candidates)+1))[:nsample]
            
            idx[m] = valid_candidates[selected]
        
        # 空查询处理
        idx[empty_ball_mask] = 0
        return idx, empty_ball_mask

    @staticmethod
    def backward(ctx, *grad_outputs):
        return None, None, None, None, None, None, None

voxel_query = CPUVoxelQuery.apply
"""class VoxelQuery(Function):

    @staticmethod
    def forward(ctx, max_range: int, radius: float, nsample: int, xyz: torch.Tensor, \
                    new_xyz: torch.Tensor, new_coords: torch.Tensor, point_indices: torch.Tensor):

        assert new_xyz.is_contiguous()
        assert xyz.is_contiguous()
        assert new_coords.is_contiguous()
        assert point_indices.is_contiguous()

        M = new_coords.shape[0]
        B, Z, Y, X = point_indices.shape
        idx = torch.cuda.IntTensor(M, nsample).zero_()

        z_range, y_range, x_range = max_range
        pointnet2.voxel_query_wrapper(M, Z, Y, X, nsample, radius, z_range, y_range, x_range, \
                    new_xyz, xyz, new_coords, point_indices, idx)

        empty_ball_mask = (idx[:, 0] == -1)
        idx[empty_ball_mask] = 0

        return idx, empty_ball_mask

    @staticmethod
    def backward(ctx, a=None):
        return None, None, None, None

voxel_query = VoxelQuery.apply"""


class VoxelQueryAndGrouping(nn.Module):
    def __init__(self, max_range: int, radius: float, nsample: int):
        """
        Args:
            radius: float, radius of ball
            nsample: int, maximum number of features to gather in the ball
        """
        super().__init__()
        self.max_range, self.radius, self.nsample = max_range, radius, nsample

    def forward(self, new_coords: torch.Tensor, xyz: torch.Tensor, xyz_batch_cnt: torch.Tensor,
                new_xyz: torch.Tensor, new_xyz_batch_cnt: torch.Tensor,
                features: torch.Tensor, voxel2point_indices: torch.Tensor):
        """
        Args:
            new_coords: (M1 + M2 ..., 3) centers voxel indices of the ball query
            xyz: (N1 + N2 ..., 3) xyz coordinates of the features
            xyz_batch_cnt: (batch_size), [N1, N2, ...]
            new_xyz: (M1 + M2 ..., 3) centers of the ball query
            new_xyz_batch_cnt: (batch_size), [M1, M2, ...]
            features: (N1 + N2 ..., C) tensor of features to group
            voxel2point_indices: (B, Z, Y, X) tensor of points indices of voxels

        Returns:
            new_features: (M1 + M2, C, nsample) tensor
        """
        assert xyz.shape[0] == xyz_batch_cnt.sum(), 'xyz: %s, xyz_batch_cnt: %s' % (str(xyz.shape), str(new_xyz_batch_cnt))
        assert new_coords.shape[0] == new_xyz_batch_cnt.sum(), \
            'new_coords: %s, new_xyz_batch_cnt: %s' % (str(new_coords.shape), str(new_xyz_batch_cnt))
        batch_size = xyz_batch_cnt.shape[0]
        
        # idx: (M1 + M2 ..., nsample), empty_ball_mask: (M1 + M2 ...)
        idx1, empty_ball_mask1 = voxel_query(self.max_range, self.radius, self.nsample, xyz, new_xyz, new_coords, voxel2point_indices)

        idx1 = idx1.view(batch_size, -1, self.nsample)
        count = 0
        for bs_idx in range(batch_size):
            idx1[bs_idx] -= count
            count += xyz_batch_cnt[bs_idx]
        idx1 = idx1.view(-1, self.nsample)
        idx1[empty_ball_mask1] = 0

        idx = idx1
        empty_ball_mask = empty_ball_mask1
        
        grouped_xyz = pointnet2_utils.grouping_operation(xyz, xyz_batch_cnt, idx, new_xyz_batch_cnt)
        # grouped_features: (M1 + M2, C, nsample)
        grouped_features = pointnet2_utils.grouping_operation(features, xyz_batch_cnt, idx, new_xyz_batch_cnt)  
        
        return grouped_features, grouped_xyz, empty_ball_mask
