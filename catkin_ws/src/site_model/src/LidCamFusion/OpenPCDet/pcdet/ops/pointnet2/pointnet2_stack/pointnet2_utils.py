import torch
import torch.nn as nn
from torch.autograd import Function, Variable

class BallQuery(Function):
    @staticmethod
    def forward(ctx, radius, nsample, xyz, xyz_batch_cnt, new_xyz, new_xyz_batch_cnt):
        """
        CPU版本球查询实现
        原理：通过计算欧氏距离筛选半径内的点
        """
        B = xyz_batch_cnt.shape[0]
        M = new_xyz.shape[0]
        idx = torch.full((M, nsample), -1, dtype=torch.int32)
        
        # 分批处理逻辑
        batch_start_idx = 0
        for b in range(B):
            # 当前批次点云范围
            cur_xyz = xyz[batch_start_idx:batch_start_idx+xyz_batch_cnt[b]]
            cur_new_xyz = new_xyz[batch_start_idx:batch_start_idx+new_xyz_batch_cnt[b]]
            
            # 计算距离矩阵
            dist = torch.cdist(cur_new_xyz, cur_xyz)  # [M_cur, N_cur]
            
            # 寻找半径内的点
            for i in range(dist.shape[0]):
                mask = dist[i] < radius
                valid_idx = torch.where(mask)[0]
                
                if len(valid_idx) >= nsample:
                    selected = torch.randperm(len(valid_idx))[:nsample]
                    idx[batch_start_idx+i] = valid_idx[selected]
                else:
                    # 填充策略：重复最后一个有效点
                    if len(valid_idx) > 0:
                        idx[batch_start_idx+i, :len(valid_idx)] = valid_idx
                        idx[batch_start_idx+i, len(valid_idx):] = valid_idx[-1]
            
            batch_start_idx += new_xyz_batch_cnt[b]
        
        empty_ball_mask = (idx[:, 0] == -1)
        idx[empty_ball_mask] = 0
        
        ctx.mark_non_differentiable(idx)
        ctx.mark_non_differentiable(empty_ball_mask)
        return idx, empty_ball_mask

    @staticmethod
    def backward(ctx, *grad_outputs):
        return None, None, None, None, None, None

ball_query = BallQuery.apply

class FarthestPointSampling(Function):
    @staticmethod
    def forward(ctx, xyz, npoint):
        """
        CPU最远点采样实现
        原理：迭代选择距离已选点集最远的点
        """
        B, N, _ = xyz.shape
        device = xyz.device
        centroids = torch.zeros((B, npoint), dtype=torch.long).to(device)
        distance = torch.ones((B, N)).to(device) * 1e10
        
        # 随机初始化第一个点
        farthest = torch.randint(0, N, (B,), dtype=torch.long).to(device)
        
        for i in range(npoint):
            centroids[:, i] = farthest
            centroid = xyz[torch.arange(B), farthest, :].view(B, 1, 3)
            
            # 计算欧氏距离
            dist = torch.sum((xyz - centroid) ** 2, -1)
            mask = dist < distance
            distance[mask] = dist[mask]
            
            farthest = torch.max(distance, dim=1)[1]
            
        return centroids

    @staticmethod
    def backward(ctx, *grad_outputs):
        return None, None

farthest_point_sample = FarthestPointSampling.apply

class GroupingOperation(Function):
    @staticmethod
    def forward(ctx, features, features_batch_cnt, idx, idx_batch_cnt):
        """
        CPU特征聚合实现
        原理：基于索引张量收集特征
        """
        M, nsample = idx.shape
        C = features.shape[1]
        
        grouped_features = torch.zeros((M, C, nsample), dtype=features.dtype)
        
        # 分批处理
        batch_start_idx = 0
        for b in range(features_batch_cnt.shape[0]):
            cur_features = features[batch_start_idx:batch_start_idx+features_batch_cnt[b]]
            cur_idx = idx[batch_start_idx:batch_start_idx+idx_batch_cnt[b]]
            
            # 收集特征
            grouped_features[batch_start_idx:batch_start_idx+idx_batch_cnt[b]] = \
                cur_features[cur_idx].permute(0,2,1)
            
            batch_start_idx += idx_batch_cnt[b]
        
        ctx.save_for_backward(idx, features_batch_cnt, idx_batch_cnt)
        return grouped_features

    @staticmethod
    def backward(ctx, grad_out):
        idx, features_batch_cnt, idx_batch_cnt = ctx.saved_tensors
        B = features_batch_cnt.shape[0]
        N = features_batch_cnt.sum().item()
        C = grad_out.shape[1]
        
        grad_features = torch.zeros((N, C), dtype=grad_out.dtype)
        
        # 梯度分散回原始位置
        batch_start_idx = 0
        for b in range(B):
            cur_grad = grad_out[batch_start_idx:batch_start_idx+idx_batch_cnt[b]]
            cur_idx = idx[batch_start_idx:batch_start_idx+idx_batch_cnt[b]]
            
            grad_features[cur_idx] += cur_grad.permute(0,2,1).sum(dim=2)
            
            batch_start_idx += idx_batch_cnt[b]
            
        return grad_features, None, None, None

grouping_operation = GroupingOperation.apply

class QueryAndGroup(nn.Module):
    def __init__(self, radius, nsample, use_xyz=True):
        super().__init__()
        self.radius, self.nsample, self.use_xyz = radius, nsample, use_xyz

    def forward(self, xyz, xyz_batch_cnt, new_xyz, new_xyz_batch_cnt, features=None):
        idx, empty_ball_mask = ball_query(
            self.radius, self.nsample, xyz, xyz_batch_cnt, 
            new_xyz, new_xyz_batch_cnt
        )
        
        grouped_xyz = grouping_operation(xyz, xyz_batch_cnt, idx, new_xyz_batch_cnt)
        grouped_xyz -= new_xyz.unsqueeze(-1)
        grouped_xyz[empty_ball_mask] = 0
        
        if features is not None:
            grouped_features = grouping_operation(
                features, xyz_batch_cnt, idx, new_xyz_batch_cnt
            )
            grouped_features[empty_ball_mask] = 0
            
            if self.use_xyz:
                new_features = torch.cat([grouped_xyz, grouped_features], dim=1)
            else:
                new_features = grouped_features
        else:
            new_features = grouped_xyz
            
        return new_features, idx

class StackFarthestPointSampling(Function):
    @staticmethod
    def forward(ctx, xyz, xyz_batch_cnt, npoint):
        """
        CPU版本堆叠式最远点采样实现
        原理：逐批次独立执行FPS算法，保持各批次采样独立性
        """
        batch_size = len(xyz_batch_cnt)
        device = xyz.device
        
        # 转换npoint为张量格式
        if not isinstance(npoint, torch.Tensor):
            if isinstance(npoint, list):
                npoint = torch.tensor(npoint, device=device)
            else:
                npoint = torch.tensor([npoint]*batch_size, device=device)
        
        # 初始化输出容器
        all_indices = []
        start_idx = 0
        
        # 逐批次处理[1,3](@ref)
        for b in range(batch_size):
            batch_points = xyz[start_idx : start_idx + xyz_batch_cnt[b]]
            batch_size_N = xyz_batch_cnt[b].item()
            n_sample = npoint[b].item()
            
            # FPS核心算法[3](@ref)
            centroids = torch.zeros(n_sample, dtype=torch.long)
            distance = torch.ones(batch_size_N) * 1e10
            farthest = torch.randint(0, batch_size_N, (1,)).item()
            
            for i in range(n_sample):
                centroids[i] = farthest
                centroid = batch_points[farthest].view(1, 3)
                dist = torch.sum((batch_points - centroid) ** 2, dim=1)
                mask = dist < distance
                distance[mask] = dist[mask]
                farthest = torch.argmax(distance).item()
            
            # 调整索引为全局位置
            centroids += start_idx
            all_indices.append(centroids)
            start_idx += xyz_batch_cnt[b]
        
        # 合并所有批次结果
        output = torch.cat(all_indices)
        return output

    @staticmethod
    def backward(ctx, *grad_outputs):
        return None, None, None

stack_farthest_point_sample = StackFarthestPointSampling.apply

class ThreeNN(Function):
    @staticmethod
    def forward(ctx, unknown, unknown_batch_cnt, known, known_batch_cnt):
        """
        CPU最近邻搜索实现
        原理：基于批处理的距离矩阵计算和topk索引获取
        """
        device = unknown.device
        B = unknown_batch_cnt.shape[0]
        total_unknown = unknown.shape[0]
        
        dist = torch.zeros((total_unknown, 3), device=device)
        idx = torch.zeros((total_unknown, 3), dtype=torch.int32, device=device)
        
        # 分批次处理
        u_start, k_start = 0, 0
        for b in range(B):
            u_end = u_start + unknown_batch_cnt[b]
            k_end = k_start + known_batch_cnt[b]
            
            cur_unknown = unknown[u_start:u_end]
            cur_known = known[k_start:k_end]
            
            # 计算距离矩阵 [N_unknown, N_known]
            pairwise_dist = torch.cdist(cur_unknown, cur_known)
            
            # 获取最近3个邻居
            topk_values, topk_indices = torch.topk(pairwise_dist, k=3, dim=1, largest=False)
            
            # 转换局部索引为全局索引
            global_indices = topk_indices + k_start
            
            # 保存结果
            dist[u_start:u_end] = torch.sqrt(topk_values)
            idx[u_start:u_end] = global_indices
            
            u_start = u_end
            k_start = k_end
        
        ctx.mark_non_differentiable(dist)
        ctx.mark_non_differentiable(idx)
        return dist, idx

    @staticmethod
    def backward(ctx, *grad_outputs):
        return None, None, None, None

three_nn = ThreeNN.apply

class ThreeInterpolate(Function):
    @staticmethod
    def forward(ctx, features, idx, weight):
        """
        CPU三线性插值实现
        原理：基于索引和权重的加权求和
        """
        M, C = features.shape
        N = idx.shape[0]
        
        output = torch.zeros((N, C), device=features.device)
        
        # 权重归一化处理
        weight_sum = weight.sum(dim=1, keepdim=True)
        normalized_weights = weight / weight_sum
        
        # 三维插值计算
        for i in range(3):
            indices = idx[:, i].long().clamp(0, M-1)
            output += normalized_weights[:, i:i+1] * features[indices]
        
        ctx.save_for_backward(idx, normalized_weights)
        return output

    @staticmethod
    def backward(ctx, grad_out):
        idx, weights = ctx.saved_tensors
        M, C = grad_out.shape[1], grad_out.shape[0]
        grad_features = torch.zeros((M, C), device=grad_out.device)
        
        # 梯度反向传播
        for i in range(3):
            indices = idx[:, i].long().clamp(0, M-1)
            grad_features.index_add_(0, indices, weights[:, i:i+1] * grad_out)
        
        return grad_features, None, None

three_interpolate = ThreeInterpolate.apply

class ThreeNNForVectorPoolByTwoStep(Function):
    @staticmethod
    def forward(ctx, support_xyz, xyz_batch_cnt, new_xyz, new_xyz_grid_centers, new_xyz_batch_cnt,
                max_neighbour_distance, nsample, neighbor_type, avg_length_of_neighbor_idxs, num_total_grids,
                neighbor_distance_multiplier):
        """
        CPU两阶段最近邻搜索实现
        原理：先收集候选邻域点，再从中选取最近三点[6](@ref)
        """
        device = support_xyz.device
        num_new_xyz = new_xyz.shape[0]
        query_distance = max_neighbour_distance * neighbor_distance_multiplier

        # 第一阶段：收集候选邻域索引
        stack_neighbor_idxs, start_len = ThreeNNForVectorPoolByTwoStep._collect_neighbors(
            support_xyz, xyz_batch_cnt, new_xyz, new_xyz_batch_cnt, 
            query_distance, nsample, neighbor_type, avg_length_of_neighbor_idxs
        )

        # 第二阶段：三最近邻搜索
        new_xyz_grid_dist2, new_xyz_grid_idxs = ThreeNNForVectorPoolByTwoStep._three_nn_search(
            support_xyz, new_xyz, new_xyz_grid_centers, stack_neighbor_idxs, 
            start_len, num_total_grids
        )

        avg_length = stack_neighbor_idxs.shape[0] // num_new_xyz + 1
        return torch.sqrt(new_xyz_grid_dist2), new_xyz_grid_idxs, torch.tensor(avg_length)

    @staticmethod
    def _collect_neighbors(support_xyz, xyz_batch_cnt, new_xyz, new_xyz_batch_cnt, 
                          radius, nsample, neighbor_type, avg_length):
        """
        邻域点收集实现[3](@ref)
        """
        neighbors = []
        start_lens = []
        current_idx = 0

        # 分批次处理
        s_start, q_start = 0, 0
        for b in range(xyz_batch_cnt.shape[0]):
            s_end = s_start + xyz_batch_cnt[b]
            q_end = q_start + new_xyz_batch_cnt[b]
            
            cur_support = support_xyz[s_start:s_end]
            cur_query = new_xyz[q_start:q_end]
            
            # 计算距离矩阵
            dist = torch.cdist(cur_query, cur_support)
            
            # 邻域筛选
            for i in range(cur_query.shape[0]):
                if neighbor_type == 1:  # ball query
                    mask = dist[i] < radius
                else:  # cube query
                    offset = cur_query[i] - cur_support
                    mask = (torch.abs(offset) < radius).all(dim=1)
                
                candidate = torch.where(mask)[0] + s_start  # 转换为全局索引
                
                # 采样控制
                if nsample > 0 and candidate.shape[0] > nsample:
                    candidate = candidate[torch.randperm(candidate.shape[0])[:nsample]]
                elif candidate.shape[0] == 0:
                    candidate = torch.tensor([s_start], device=device)  # 保底采样
                
                neighbors.append(candidate)
                start_lens.append((current_idx, candidate.shape[0]))
                current_idx += candidate.shape[0]
            
            s_start = s_end
            q_start = q_end

        # 构建输出张量
        stack_neighbor_idxs = torch.cat(neighbors)
        start_len = torch.tensor(start_lens, dtype=torch.int)
        return stack_neighbor_idxs, start_len

    @staticmethod
    def _three_nn_search(support_xyz, new_xyz, grid_centers, neighbor_idxs, start_len, num_grids):
        """
        三最近邻搜索实现[6](@ref)
        """
        M, num_grids, _ = grid_centers.shape
        dist2 = torch.zeros((M, num_grids, 3), device=device)
        idx = torch.full((M, num_grids, 3), -1, dtype=torch.int32, device=device)

        for i in range(M):
            # 获取当前点的候选邻域
            s, l = start_len[i]
            candidates = neighbor_idxs[s:s+l]
            
            for g in range(num_grids):
                # 计算网格中心到候选点的距离
                center = grid_centers[i, g]
                dist = torch.norm(support_xyz[candidates] - center, dim=1)
                
                # 选取最近三点
                if len(dist) >= 3:
                    top3_idx = torch.topk(dist, 3, largest=False).indices
                    idx[i, g] = candidates[top3_idx]
                    dist2[i, g] = dist[top3_idx] ** 2
                elif len(dist) > 0:
                    idx[i, g, :len(dist)] = candidates
                    dist2[i, g, :len(dist)] = dist ** 2
                    # 重复填充最后一个有效点
                    idx[i, g, len(dist):] = candidates[-1]
                    dist2[i, g, len(dist):] = dist[-1] ** 2
        
        return dist2, idx

    @staticmethod
    def backward(ctx, *grad_outputs):
        return None, None, None, None, None, None, None, None, None, None, None

three_nn_for_vector_pool_by_two_step = ThreeNNForVectorPoolByTwoStep.apply


class VectorPoolWithVoxelQuery(Function):
    @staticmethod
    def forward(ctx, support_xyz, xyz_batch_cnt, support_features, new_xyz, new_xyz_batch_cnt, 
                num_grid_x, num_grid_y, num_grid_z, max_neighbour_distance, num_c_out_each_grid, use_xyz,
                num_mean_points_per_grid=100, nsample=-1, neighbor_type=0, pooling_type=0):
        """
        CPU体素池化实现
        原理：分批次构建体素网格，聚合邻域特征
        """
        # 基础参数校验
        assert support_xyz.dim() == 2 and support_xyz.size(1) == 3
        assert new_xyz.dim() == 2 and new_xyz.size(1) == 3
        num_total_grids = num_grid_x * num_grid_y * num_grid_z
        B = xyz_batch_cnt.shape[0]
        M = new_xyz.shape[0]

        # 初始化输出张量
        new_features = support_features.new_zeros((M, num_c_out_each_grid * num_total_grids))
        new_local_xyz = support_features.new_zeros((M, 3 * num_total_grids))
        point_cnt_of_grid = xyz_batch_cnt.new_zeros((M, num_total_grids))
        grouped_idxs = []

        # 分批次处理
        s_start, q_start = 0, 0
        for b in range(B):
            # 当前批次参数
            s_end = s_start + xyz_batch_cnt[b]
            q_end = q_start + new_xyz_batch_cnt[b]
            cur_support = support_xyz[s_start:s_end]
            cur_features = support_features[s_start:s_end]
            cur_query = new_xyz[q_start:q_end]

            # 网格划分
            grid_size = max_neighbour_distance / torch.tensor([num_grid_x, num_grid_y, num_grid_z])
            grid_centers = torch.meshgrid(
                torch.linspace(-max_neighbour_distance, max_neighbour_distance, num_grid_x),
                torch.linspace(-max_neighbour_distance, max_neighbour_distance, num_grid_y),
                torch.linspace(-max_neighbour_distance, max_neighbour_distance, num_grid_z)
            ).view(-1, 3)  # [G,3]

            # 遍历查询点
            for i in range(cur_query.shape[0]):
                center = cur_query[i]
                grid_offsets = grid_centers * grid_size + center
                
                # 邻域搜索
                dist = torch.norm(cur_support - center.unsqueeze(0), dim=1)
                if neighbor_type == 1:  # 球查询
                    mask = dist < max_neighbour_distance
                else:  # 立方体查询
                    offset = cur_support - center
                    mask = (torch.abs(offset) < max_neighbour_distance).all(dim=1)
                
                candidate_idx = torch.where(mask)[0]
                
                # 特征聚合
                if len(candidate_idx) > 0:
                    grid_assign = ((cur_support[candidate_idx] - grid_offsets[:, None]) / grid_size).long()
                    valid_grids = (grid_assign >= 0).all(dim=2) & (grid_assign < torch.tensor([num_grid_x, num_grid_y, num_grid_z]))
                    
                    for g in range(num_total_grids):
                        in_grid = valid_grids[g]
                        if in_grid.any():
                            grid_features = cur_features[candidate_idx[in_grid]]
                            
                            # 池化策略
                            if pooling_type == 0:  # 平均池化
                                new_features[q_start+i, g*num_c_out_each_grid:(g+1)*num_c_out_each_grid] = grid_features.mean(dim=0)
                            else:  # 随机采样
                                selected = torch.randint(0, grid_features.shape[0], (1,))
                                new_features[q_start+i, g*num_c_out_each_grid:(g+1)*num_c_out_each_grid] = grid_features[selected]
                            
                            # 记录索引和坐标
                            grouped_idxs.extend([(s_start + int(idx), q_start + i, g) for idx in candidate_idx[in_grid]])
                            point_cnt_of_grid[q_start+i, g] = in_grid.sum()

                # 局部坐标计算
                if use_xyz:
                    local_coords = (cur_support[candidate_idx] - center) / max_neighbour_distance
                    new_local_xyz[q_start+i] = local_coords.view(-1)

            s_start = s_end
            q_start = q_end

        # 保存反向传播参数
        grouped_idxs = torch.tensor(grouped_idxs, dtype=torch.long)
        ctx.vector_pool_for_backward = (point_cnt_of_grid, grouped_idxs, support_features.shape[0], support_features.shape[1])
        
        return new_features, new_local_xyz, torch.tensor(len(grouped_idxs)//M), point_cnt_of_grid

    @staticmethod
    def backward(ctx, grad_output, *args):
        """
        反向传播实现
        原理：根据前向索引分散梯度
        """
        point_cnt_of_grid, grouped_idxs, N, C = ctx.vector_pool_for_backward
        grad_support = torch.zeros((N, C), dtype=grad_output.dtype)
        
        # 遍历所有索引三元组(原始点索引，查询点索引，网格索引)
        for src_idx, tgt_idx, grid_idx in grouped_idxs:
            grid_size = C // (point_cnt_of_grid.shape[1] * point_cnt_of_grid.shape[2])
            grad_support[src_idx] += grad_output[tgt_idx, grid_idx*grid_size:(grid_idx+1)*grid_size] 
        
        return None, None, grad_support, None, None, None, None, None, None, None, None, None, None, None, None

vector_pool_with_voxel_query_op = VectorPoolWithVoxelQuery.apply















if __name__ == '__main__':
    pass
