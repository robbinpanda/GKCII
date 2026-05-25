from typing import Tuple

import torch
import torch.nn as nn
from torch.autograd import Function, Variable

#from . import pointnet2_batch_cuda as pointnet2


def farthest_point_sampling_cpu(xyz: torch.Tensor, npoint: int):
    """
    CPU版本最远点采样
    :param xyz: (B, N, 3)
    :param npoint: 采样点数
    :return: (B, npoint) 采样点索引
    """
    device = xyz.device
    B, N, C = xyz.shape
    centroids = torch.zeros(B, npoint, dtype=torch.long, device=device)
    distance = torch.ones(B, N, device=device) * 1e10
    
    # 随机初始化第一个点
    farthest = torch.randint(0, N, (B,), dtype=torch.long, device=device)
    batch_indices = torch.arange(B, dtype=torch.long, device=device)
    
    for i in range(npoint):
        centroids[:, i] = farthest
        centroid = xyz[batch_indices, farthest, :].view(B, 1, 3)
        dist = torch.sum((xyz - centroid) ** 2, -1)
        mask = dist < distance
        distance[mask] = dist[mask]
        farthest = torch.max(distance, -1)[1]
    return centroids

def gather_points_cpu(features: torch.Tensor, idx: torch.Tensor):
    """
    CPU版本特征聚集
    :param features: (B, C, N)
    :param idx: (B, npoint) 索引
    :return: (B, C, npoint) 聚集后的特征
    """
    return torch.gather(features, 2, idx.unsqueeze(1).expand(-1, features.shape[1], -1))

def gather_points_grad_cpu(grad_out: torch.Tensor, idx: torch.Tensor, C: int, N: int):
    """
    Gather反向传播
    :param grad_out: (B, C, npoint) 输出梯度
    :param idx: (B, npoint) 索引
    :return: (B, C, N) 输入梯度
    """
    grad_features = torch.zeros((grad_out.shape[0], C, N), 
                              dtype=grad_out.dtype, device=grad_out.device)
    idx_exp = idx.unsqueeze(1).expand(-1, C, -1) # (B, C, npoint)
    grad_features.scatter_add_(2, idx_exp, grad_out)
    return grad_features

def three_nn_cpu(unknown: torch.Tensor, known: torch.Tensor):
    """
    CPU版本3最近邻查找
    :param unknown: (B, N, 3) 待查询点
    :param known: (B, M, 3) 已知点
    :return: (dist, idx) 
    """
    dist = torch.cdist(unknown, known) # (B, N, M)
    dist, idx = torch.topk(dist, k=3, dim=-1, largest=False, sorted=True)
    return torch.sqrt(dist), idx

def three_interpolate_cpu(features, idx, weight):
    """
    CPU版三次插值前向计算
    :param features: (B, C, M) 输入特征
    :param idx: (B, N, 3) 邻居索引
    :param weight: (B, N, 3) 插值权重
    :return: (B, C, N) 插值结果
    """
    B, C, M = features.size()
    N = idx.size(1)
    
    # 扩展维度以便广播
    idx_exp = idx.unsqueeze(1).expand(-1, C, -1, -1)  # (B, C, N, 3)
    weight_exp = weight.unsqueeze(1)  # (B, 1, N, 3)
    
    # 收集特征并加权求和
    gathered = torch.gather(features.unsqueeze(2).expand(-1, -1, N, -1), 
                          3, idx_exp)  # (B, C, N, 3)
    return (gathered * weight_exp).sum(dim=-1)
def three_interpolate_grad_cpu(grad_out, idx, weight, C, M):
    """
    CPU版三次插值反向传播
    :param grad_out: (B, C, N) 输出梯度
    :param idx: (B, N, 3) 邻居索引 
    :param weight: (B, N, 3) 插值权重
    :return: (B, C, M) 输入特征梯度
    """
    B, _, N = grad_out.size()
    
    # 构造梯度张量
    grad_features = torch.zeros((B, C, M), 
                              dtype=grad_out.dtype,
                              device=grad_out.device)
    
    # 扩展维度进行散射累加
    idx_exp = idx.unsqueeze(1).expand(-1, C, -1, -1)  # (B, C, N, 3)
    weight_exp = weight.unsqueeze(1)  # (B, 1, N, 3)
    grad_exp = grad_out.unsqueeze(-1) * weight_exp  # (B, C, N, 3)
    
    # 散射梯度到原始位置
    return grad_features.scatter_add_(2, 
                                     idx_exp.reshape(B, C, -1),
                                     grad_exp.reshape(B, C, -1))
def ball_query_cpu(radius: float, nsample: int, xyz: torch.Tensor, new_xyz: torch.Tensor):
    """
    CPU版本球形区域查询
    :param radius: 搜索半径
    :param nsample: 最大采样数
    :param xyz: (B, N, 3) 全部点
    :param new_xyz: (B, M, 3) 查询中心
    :return: (B, M, nsample) 邻居索引
    """
    B, M, _ = new_xyz.shape
    _, N, _ = xyz.shape
    idx = torch.full((B, M, nsample), -1, dtype=torch.long, device=xyz.device)
    
    for b in range(B):
        # 计算距离矩阵
        dist = torch.norm(xyz[b].unsqueeze(1) - new_xyz[b], dim=-1) # (N, M)
        
        for m in range(M):
            # 找到半径内的点
            valid = torch.where(dist[:, m] < radius)[0]
            if len(valid) > 0:
                # 随机选择nsample个点
                selected = valid[torch.randperm(len(valid))[:nsample]]
                idx[b, m, :len(selected)] = selected
    return idx

def cpu_group_points(features: torch.Tensor, idx: torch.Tensor) -> torch.Tensor:
    """
    特征分组前向传播的 CPU 实现
    :param features: (B, C, N) 输入特征张量
    :param idx: (B, npoint, nsample) 分组索引
    :return: (B, C, npoint, nsample) 输出分组特征
    """
    B, C, N = features.shape
    B, npoint, nsample = idx.shape
    
    # 维度扩展与索引广播
    expanded_idx = idx.view(B, 1, npoint, nsample).expand(-1, C, -1, -1)  # (B, C, npoint, nsample)
    expanded_idx = expanded_idx % 16384
    # 使用 gather 收集特征
    output = torch.gather(
        features.unsqueeze(2).expand(-1, -1, npoint, -1),  # (B, C, npoint, N)
        dim=3,  # 沿原始点云维度 N 收集
        index=expanded_idx
    )
    return output.contiguous()
    
def cpu_group_points_grad(grad_out: torch.Tensor, idx: torch.Tensor, N: int) -> torch.Tensor:
    """
    特征分组反向传播的 CPU 实现
    :param grad_out: (B, C, npoint, nsample) 输出梯度
    :param idx: (B, npoint, nsample) 前向传播使用的索引
    :param N: 原始特征点数
    :return: (B, C, N) 输入特征梯度
    """
    B, C, npoint, nsample = grad_out.shape
    grad_features = torch.zeros((B, C, N), dtype=grad_out.dtype, device=grad_out.device)
    
    # 梯度散射累加（反向传播核心逻辑）
    expanded_idx = idx.view(B, 1, npoint, nsample).expand(-1, C, -1, -1)  # (B, C, npoint, nsample)
    grad_features.scatter_add_(
        dim=2,  # 沿原始点云维度 N 散射
        index=expanded_idx.reshape(B, C, -1),  # 展平后维度 (B, C, npoint*nsample)
        src=grad_out.reshape(B, C, -1)
    )
    return grad_features

class GroupingOperation(Function):
    @staticmethod
    def forward(ctx, features: torch.Tensor, idx: torch.Tensor) -> torch.Tensor:
        # 强制转换到 CPU（若输入在 GPU 需先转移到 CPU）
        ctx.cpu_mode = not features.is_cuda
        if features.is_cuda:
            features = features.cpu()
            idx = idx.cpu()
        
        # 执行 CPU 前向计算
        output = cpu_group_points(features.contiguous(), idx.contiguous())
        
        # 保存反向传播所需参数
        ctx.save_for_backward(idx)
        ctx.N = features.size(-1)
        return output.to(features.device)  # 返回与原设备一致的张量

    @staticmethod
    def backward(ctx, grad_out: torch.Tensor):
        idx, = ctx.saved_tensors
        N = ctx.N
        
        # 强制转换到 CPU
        if grad_out.is_cuda:
            grad_out = grad_out.cpu()
            idx = idx.cpu()
        
        # 执行 CPU 反向计算
        grad_features = cpu_group_points_grad(grad_out.contiguous(), idx.contiguous(), N)
        return grad_features.to(grad_out.device), None

grouping_operation = GroupingOperation.apply


class FarthestPointSampling(Function):
    @staticmethod
    def forward(ctx, xyz: torch.Tensor, npoint: int) -> torch.Tensor:
        """
        Uses iterative farthest point sampling to select a set of npoint features that have the largest
        minimum distance
        :param ctx:
        :param xyz: (B, N, 3) where N > npoint
        :param npoint: int, number of features in the sampled set
        :return:
             output: (B, npoint) tensor containing the set
        """
        assert xyz.is_contiguous()

        B, N, _ = xyz.size()
        #output = torch.cuda.IntTensor(B, npoint)
        #temp = torch.cuda.FloatTensor(B, N).fill_(1e10)
        output = torch.IntTensor(B, npoint)
        temp = torch.FloatTensor(B, N).fill_(1e10)

        #pointnet2.farthest_point_sampling_wrapper(B, N, npoint, xyz, temp, output)
        output = farthest_point_sampling_cpu(xyz, npoint)
        return output

    @staticmethod
    def backward(xyz, a=None):
        return None, None

farthest_point_sample = furthest_point_sample = FarthestPointSampling.apply


class GatherOperation(Function):

    @staticmethod
    def forward(ctx, features: torch.Tensor, idx: torch.Tensor) -> torch.Tensor:
        """
        :param ctx:
        :param features: (B, C, N)
        :param idx: (B, npoint) index tensor of the features to gather
        :return:
            output: (B, C, npoint)
        """
        assert features.is_contiguous()
        assert idx.is_contiguous()

        B, npoint = idx.size()
        _, C, N = features.size()
        #output = torch.cuda.FloatTensor(B, C, npoint)
        output = torch.FloatTensor(B, C, npoint)

        #pointnet2.gather_points_wrapper(B, C, N, npoint, features, idx, output)
        output = gather_points_cpu(features, idx)

        ctx.for_backwards = (idx, C, N)
        return output

    @staticmethod
    def backward(ctx, grad_out):
        idx, C, N = ctx.for_backwards
        B, npoint = idx.size()

        #grad_features = Variable(torch.cuda.FloatTensor(B, C, N).zero_())
        grad_features = Variable(torch.FloatTensor(B, C, N).zero_())
        grad_out_data = grad_out.data.contiguous()
        #pointnet2.gather_points_grad_wrapper(B, C, N, npoint, grad_out_data, idx, grad_features.data)
        grad_features = gather_points_grad_cpu(grad_out, idx, C, N)
        return grad_features, None


gather_operation = GatherOperation.apply


class ThreeNN(Function):

    @staticmethod
    def forward(ctx, unknown: torch.Tensor, known: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Find the three nearest neighbors of unknown in known
        :param ctx:
        :param unknown: (B, N, 3)
        :param known: (B, M, 3)
        :return:
            dist: (B, N, 3) l2 distance to the three nearest neighbors
            idx: (B, N, 3) index of 3 nearest neighbors
        """
        assert unknown.is_contiguous()
        assert known.is_contiguous()

        B, N, _ = unknown.size()
        m = known.size(1)
        #dist2 = torch.cuda.FloatTensor(B, N, 3)
        #idx = torch.cuda.IntTensor(B, N, 3)
        
        dist2 = torch.FloatTensor(B, N, 3)
        idx = torch.IntTensor(B, N, 3)
        
        #pointnet2.three_nn_wrapper(B, N, m, unknown, known, dist2, idx)
        dist2, idx = three_nn_cpu(unknown, known)
        #return torch.sqrt(dist2), idx
        return dist2, idx

    @staticmethod
    def backward(ctx, a=None, b=None):
        return None, None


three_nn = ThreeNN.apply


class ThreeInterpolate(Function):

    @staticmethod
    def forward(ctx, features: torch.Tensor, idx: torch.Tensor, weight: torch.Tensor) -> torch.Tensor:
        """
        Performs weight linear interpolation on 3 features
        :param ctx:
        :param features: (B, C, M) Features descriptors to be interpolated from
        :param idx: (B, n, 3) three nearest neighbors of the target features in features
        :param weight: (B, n, 3) weights
        :return:
            output: (B, C, N) tensor of the interpolated features
        """
        assert features.is_contiguous()
        assert idx.is_contiguous()
        assert weight.is_contiguous()

        B, c, m = features.size()
        n = idx.size(1)
        ctx.three_interpolate_for_backward = (idx, weight, m)
        #output = torch.cuda.FloatTensor(B, c, n)
        output = torch.FloatTensor(B, c, n)
        
        #pointnet2.three_interpolate_wrapper(B, c, m, n, features, idx, weight, output)
        output = three_interpolate_cpu(features, idx, weight)
        return output

    @staticmethod
    def backward(ctx, grad_out: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        :param ctx:
        :param grad_out: (B, C, N) tensor with gradients of outputs
        :return:
            grad_features: (B, C, M) tensor with gradients of features
            None:
            None:
        """
        idx, weight, m = ctx.three_interpolate_for_backward
        B, c, n = grad_out.size()

        #grad_features = Variable(torch.cuda.FloatTensor(B, c, m).zero_())
        grad_features = Variable(torch.FloatTensor(B, c, m).zero_())
        grad_out_data = grad_out.data.contiguous()

        #pointnet2.three_interpolate_grad_wrapper(B, c, n, m, grad_out_data, idx, weight, grad_features.data)
        grad_features = three_interpolate_grad_cpu(grad_out, idx, weight, c, m)
        return grad_features, None, None


three_interpolate = ThreeInterpolate.apply





class BallQuery(Function):

    @staticmethod
    def forward(ctx, radius: float, nsample: int, xyz: torch.Tensor, new_xyz: torch.Tensor) -> torch.Tensor:
        """
        :param ctx:
        :param radius: float, radius of the balls
        :param nsample: int, maximum number of features in the balls
        :param xyz: (B, N, 3) xyz coordinates of the features
        :param new_xyz: (B, npoint, 3) centers of the ball query
        :return:
            idx: (B, npoint, nsample) tensor with the indicies of the features that form the query balls
        """
        assert new_xyz.is_contiguous()
        assert xyz.is_contiguous()

        B, N, _ = xyz.size()
        npoint = new_xyz.size(1)
        #idx = torch.cuda.IntTensor(B, npoint, nsample).zero_()
        idx = torch.IntTensor(B, npoint, nsample).zero_()

        #pointnet2.ball_query_wrapper(B, N, npoint, radius, nsample, new_xyz, xyz, idx)
        idx = ball_query_cpu(radius, nsample, xyz, new_xyz)
        return idx

    @staticmethod
    def backward(ctx, a=None):
        return None, None, None, None


ball_query = BallQuery.apply


class QueryAndGroup(nn.Module):
    def __init__(self, radius: float, nsample: int, use_xyz: bool = True):
        """
        :param radius: float, radius of ball
        :param nsample: int, maximum number of features to gather in the ball
        :param use_xyz:
        """
        super().__init__()
        self.radius, self.nsample, self.use_xyz = radius, nsample, use_xyz

    def forward(self, xyz: torch.Tensor, new_xyz: torch.Tensor, features: torch.Tensor = None) -> Tuple[torch.Tensor]:
        """
        :param xyz: (B, N, 3) xyz coordinates of the features
        :param new_xyz: (B, npoint, 3) centroids
        :param features: (B, C, N) descriptors of the features
        :return:
            new_features: (B, 3 + C, npoint, nsample)
        """
        idx = ball_query(self.radius, self.nsample, xyz, new_xyz)
        xyz_trans = xyz.transpose(1, 2).contiguous()
        grouped_xyz = grouping_operation(xyz_trans, idx)  # (B, 3, npoint, nsample)
        grouped_xyz -= new_xyz.transpose(1, 2).unsqueeze(-1)

        if features is not None:
            grouped_features = grouping_operation(features, idx)
            if self.use_xyz:
                new_features = torch.cat([grouped_xyz, grouped_features], dim=1)  # (B, C + 3, npoint, nsample)
            else:
                new_features = grouped_features
        else:
            assert self.use_xyz, "Cannot have not features and not use xyz as a feature!"
            new_features = grouped_xyz

        return new_features


class GroupAll(nn.Module):
    def __init__(self, use_xyz: bool = True):
        super().__init__()
        self.use_xyz = use_xyz

    def forward(self, xyz: torch.Tensor, new_xyz: torch.Tensor, features: torch.Tensor = None):
        """
        :param xyz: (B, N, 3) xyz coordinates of the features
        :param new_xyz: ignored
        :param features: (B, C, N) descriptors of the features
        :return:
            new_features: (B, C + 3, 1, N)
        """
        grouped_xyz = xyz.transpose(1, 2).unsqueeze(2)
        if features is not None:
            grouped_features = features.unsqueeze(2)
            if self.use_xyz:
                new_features = torch.cat([grouped_xyz, grouped_features], dim=1)  # (B, 3 + C, 1, N)
            else:
                new_features = grouped_features
        else:
            new_features = grouped_xyz

        return new_features
