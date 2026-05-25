import torch
import torch.nn as nn
from torch.autograd import Function

from ...utils import box_utils
#from . import roipoint_pool3d_cuda
def cpu_roipoint_pool3d_forward(points, boxes3d, point_features, num_sampled_points=512):
    """
    CPU版本3D ROI点池化实现
    原理：基于向量化计算实现空间查询与特征聚合
    """
    B, M, _ = boxes3d.shape
    C = point_features.shape[-1]
    device = points.device

    pooled_features = torch.zeros((B, M, num_sampled_points, 3 + C), device=device)
    pooled_empty_flag = torch.zeros((B, M), dtype=torch.int, device=device)

    # 扩展三维提案框坐标转换
    boxes_xyz = boxes3d[..., :3]
    boxes_lwh = boxes3d[..., 3:6]
    boxes_angle = boxes3d[..., 6]

    for b in range(B):
        # 当前批次数据
        cur_points = points[b]  # [N, 3]
        cur_boxes = boxes3d[b]  # [M, 7]
        cur_features = point_features[b]  # [N, C]

        for m in range(M):
            # 提取当前提案框参数
            center = cur_boxes[m, :3]
            size = cur_boxes[m, 3:6]
            angle = cur_boxes[m, 6]

            # 创建旋转矩阵
            cos_a = torch.cos(angle)
            sin_a = torch.sin(angle)
            rot_mat = torch.tensor([
                [cos_a, -sin_a, 0],
                [sin_a, cos_a, 0],
                [0, 0, 1]
            ], device=device)

            # 坐标转换到提案框局部坐标系
            local_points = cur_points - center
            rotated_points = local_points @ rot_mat.T

            # 边界检查（向量化计算）
            in_x = (rotated_points[:, 0].abs() <= size[0]/2)
            in_y = (rotated_points[:, 1].abs() <= size[1]/2)
            in_z = (rotated_points[:, 2].abs() <= size[2]/2)
            inside_mask = in_x & in_y & in_z
            candidate_idx = torch.where(inside_mask)[0]

            # 采样处理
            if len(candidate_idx) > 0:
                # 随机采样
                if len(candidate_idx) >= num_sampled_points:
                    selected_idx = torch.randperm(len(candidate_idx))[:num_sampled_points]
                else:
                    # 重复采样填充
                    repeat_times = num_sampled_points // len(candidate_idx) + 1
                    selected_idx = torch.cat([torch.arange(len(candidate_idx))] * repeat_times)[:num_sampled_points]
                
                # 收集特征
                sampled_points = cur_points[candidate_idx[selected_idx]]
                sampled_features = cur_features[candidate_idx[selected_idx]]
                pooled_features[b, m] = torch.cat([sampled_points, sampled_features], dim=-1)
            else:
                # 空提案框处理
                pooled_empty_flag[b, m] = 1
                pooled_features[b, m] = torch.zeros((num_sampled_points, 3 + C), device=device)

    return pooled_features, pooled_empty_flag

class RoIPointPool3d(nn.Module):
    def __init__(self, num_sampled_points=512, pool_extra_width=1.0):
        super().__init__()
        self.num_sampled_points = num_sampled_points
        self.pool_extra_width = pool_extra_width

    def forward(self, points, point_features, boxes3d):
        """
        Args:
            points: (B, N, 3)
            point_features: (B, N, C)
            boxes3d: (B, M, 7), [x, y, z, dx, dy, dz, heading]

        Returns:
            pooled_features: (B, M, 512, 3 + C)
            pooled_empty_flag: (B, M)
        """
        return RoIPointPool3dFunction.apply(
            points, point_features, boxes3d, self.pool_extra_width, self.num_sampled_points
        )


class RoIPointPool3dFunction(Function):
    @staticmethod
    def forward(ctx, points, point_features, boxes3d, pool_extra_width, num_sampled_points=512):
        """
        Args:
            ctx:
            points: (B, N, 3)
            point_features: (B, N, C)
            boxes3d: (B, num_boxes, 7), [x, y, z, dx, dy, dz, heading]
            pool_extra_width:
            num_sampled_points:

        Returns:
            pooled_features: (B, num_boxes, 512, 3 + C)
            pooled_empty_flag: (B, num_boxes)
        """
        assert points.shape.__len__() == 3 and points.shape[2] == 3
        batch_size, boxes_num, feature_len = points.shape[0], boxes3d.shape[1], point_features.shape[2]
        pooled_boxes3d = box_utils.enlarge_box3d(boxes3d.view(-1, 7), pool_extra_width).view(batch_size, -1, 7)

        pooled_features = point_features.new_zeros((batch_size, boxes_num, num_sampled_points, 3 + feature_len))
        pooled_empty_flag = point_features.new_zeros((batch_size, boxes_num)).int()

        #roipoint_pool3d_cuda.forward(
        #    points.contiguous(), pooled_boxes3d.contiguous(),
        #    point_features.contiguous(), pooled_features, pooled_empty_flag
        #)
        # 执行CPU版前向计算
        pooled_features, pooled_empty_flag = cpu_roipoint_pool3d_forward(
            points, pooled_boxes3d, point_features, num_sampled_points
        )
        # 保存反向传播所需参数
        ctx.save_for_backward(points, point_features, boxes3d)
        ctx.pool_extra_width = pool_extra_width

        return pooled_features, pooled_empty_flag

    """@staticmethod
    def backward(ctx, grad_out):
        raise NotImplementedError"""
    @staticmethod
    def backward(ctx, grad_out, grad_empty):
        # 实现反向传播（示例实现）
        points, point_features, boxes3d = ctx.saved_tensors
        grad_points = grad_features = grad_boxes = None

        if ctx.needs_input_grad[1]:
            # 特征梯度计算（需要根据实际采样索引实现）
            grad_features = torch.zeros_like(point_features)
            # ... 具体实现需要记录前向传播的采样索引 ...

        return grad_points, grad_features, grad_boxes, None, None


if __name__ == '__main__':
    pass
