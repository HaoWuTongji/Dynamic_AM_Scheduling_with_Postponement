"""
Author:吴昊 (Hao Wu)
Last Modified: 2025-11-17
"""
import math
import random
import copy
import time

import numpy as np
import pandas as pd

from two_dimensional_packing_feasibility import heuristic_check_packing_feasibility, find_all_feasible_combinations_heuristic
import scipy.stats as st
import bisect

from itertools import product

import csv
import sys


"""================================= 模块:系统外生信息的仿真与采样 ==================================="""
# region


class PartArrivalStream:
    """
    管理整个规划周期内的零件到达事件流。
    该类在初始化时生成所有零件，并提供按时间窗口高效查询的功能。
    """

    def __init__(self, T_horizon, num_initial_parts, seed=None):
        """
        Args:
            T_horizon (float): 规划周期的总时长（单位：小时）。
            num_initial_parts (int): 在时间 t=0 时预置的零件数量。
            seed (int, optional): 用于复现结果的随机种子。
        """
        if seed is not None:
            np.random.seed(seed)
        self.rng = np.random.default_rng(seed)

        self.height_map = {0: 'Low', 1: 'High'}
        self.footprint_map = {0: 'Small', 1: 'Long', 2: 'Large'}

        self.all_parts = self._generate_all_parts(T_horizon, num_initial_parts)
        # 存储到达时间列表以便快速查找
        self._arrival_times = [p['r_p'] for p in self.all_parts]

    def _generate_dimensions_for_type(self, part_type):
        """根据零件类型，在其定义的相对范围内生成具体尺寸。"""
        h_label, f_label = part_type
        L, W, H = machine_length, machine_width, machine_height

        if h_label == 'Low':
            height = np.random.uniform(H / 3, H / 2)
        else:  # High
            height = np.random.uniform(H / 2, 2 * H / 3)

        if f_label == 'Small':
            length = np.random.uniform(L / 6, L / 3)
            width = np.random.uniform(W / 6, W / 3)
        elif f_label == 'Large':
            length = np.random.uniform(L / 3, L / 2)
            width = np.random.uniform(W / 3, W / 2)
        else:  # Long
            if np.random.rand() > 0.5:
                length = np.random.uniform(L / 3, L / 2)
                width = np.random.uniform(W / 6, W / 3)
            else:
                length = np.random.uniform(L / 6, L / 3)
                width = np.random.uniform(W / 3, W / 2)

        return length, width, height

    def _calculate_price(self, part):
        """根据最新的手稿公式计算零件价格。"""
        l, w, v, s, h = part['length'], part['width'], part['volume'], part['support_volume'], part['height']
        L, W = machine_length, machine_width
        V1, V2, t1 = V_1, V_2, t_1
        C_opr = C_operator
        K_eng, K_pow = K_energy, K_powder

        amortized_costs = max(l, w) / min(L, W) * (C_opr + (K_eng + K_gas) * h * t1 / 3600)
        direct_costs = K_pow * (v + s) + (K_eng + K_gas) * (v / V1 + s / V2) / 3600
        c_est = amortized_costs + direct_costs
        price = c_est * (1 + 1 / loose_factor) * (1 + K_fine/K_fine_max)
        return price

    def _create_part(self, part_id, arrival_time, part_type):
        """创建一个具有完整属性的零件字典。"""
        l, w, h = self._generate_dimensions_for_type(part_type)
        v = l * w * h * np.random.uniform(0.05, 0.15)
        s = v * np.random.uniform(0.0, 0.3)

        part = {
            'ID': f'P{part_id}', 'type': part_type, 'r_p': arrival_time,
            'length': l, 'width': w, 'height': h, 'volume': v, 'support_volume': s
        }

        t_alo = (T + (v / V_1 + s / V_2 + h * t_1)) / 3600
        part['delivery_time'] = arrival_time + loose_factor * t_alo
        part['price'] = self._calculate_price(part)

        return part

    def _generate_all_parts(self, T_horizon, num_initial_parts):
        """为整个规划周期生成初始零件和动态到达的零件, 保证ID按时间顺序排列。"""
        arrival_events = []  # Store tuples of (arrival_time, part_type)

        part_types = [(h, f) for h in self.height_map.values() for f in self.footprint_map.values()]

        # --- 1. 生成初始零件事件 (t=0) ---
        if num_initial_parts > 0:
            type_probabilities = arrival_rate_matrix.flatten() / np.sum(arrival_rate_matrix)
            initial_part_type_indices = self.rng.choice(len(part_types), size=num_initial_parts, p=type_probabilities)

            for type_idx in initial_part_type_indices:
                part_type = part_types[type_idx]
                arrival_events.append((0.0, part_type))

        # --- 2. 生成动态到达的零件事件 (t>0) ---
        for h_idx in range(arrival_rate_matrix.shape[0]):
            for f_idx in range(arrival_rate_matrix.shape[1]):
                rate = arrival_rate_matrix[h_idx, f_idx]
                if rate <= 0: continue

                part_type = (self.height_map[h_idx], self.footprint_map[f_idx])
                current_time = 0.0
                while current_time < T_horizon:
                    inter_arrival_time = self.rng.exponential(1.0 / rate)
                    arrival_time = current_time + inter_arrival_time
                    if arrival_time >= T_horizon: break

                    arrival_events.append((arrival_time, part_type))
                    current_time = arrival_time

        # --- 3. 排序所有事件并生成完整零件信息 ---
        arrival_events.sort(key=lambda x: x[0])

        all_parts = []
        for i, (arrival_time, part_type) in enumerate(arrival_events):
            part_id = i + 1  # 赋予连续的ID
            part = self._create_part(part_id, arrival_time, part_type)
            all_parts.append(part)

        return all_parts

    def get_arrivals_in_window(self, start_time, end_time):
        """
        高效地获取在指定时间窗口 [start_time, end_time) 内到达的所有零件。

        Args:
            start_time (float): 时间窗口的起始点 (包含)。
            end_time (float): 时间窗口的结束点 (不包含)。

        Returns:
            list: 在该时间窗口内到达的零件列表。
        """
        # 使用二分查找找到窗口的起始和结束索引
        start_idx = bisect.bisect_left(self._arrival_times, start_time)
        end_idx = bisect.bisect_left(self._arrival_times, end_time)
        return self.all_parts[start_idx:end_idx]


def sample_arrivals_for_lookahead(start_time, end_time):
    """
    为MCTS的lookahead过程，独立地随机采样一个给定时间窗口内的零件到达样本。

    Args:
        start_time (float): 时间窗口的起始点。
        end_time (float): 时间窗口的结束点。

    Returns:
        list: 在该时间窗口内新随机生成的零件列表。
    """

    random.seed()

    def generate_dimensions_for_type(part_type):
        """根据零件类型，在其定义的相对范围内生成具体尺寸。"""
        h_label, f_label = part_type
        L, W, H = machine_length, machine_width, machine_height

        if h_label == 'Low':
            height = np.random.uniform(H / 3, H / 2)
        else:  # High
            height = np.random.uniform(H / 2, 2 * H / 3)

        if f_label == 'Small':
            length = np.random.uniform(L / 6, L / 3)
            width = np.random.uniform(W / 6, W / 3)
        elif f_label == 'Large':
            length = np.random.uniform(L / 3, L / 2)
            width = np.random.uniform(W / 3, W / 2)
        else:  # Long
            if np.random.rand() > 0.5:
                length = np.random.uniform(L / 3, L / 2)
                width = np.random.uniform(W / 6, W / 3)
            else:
                length = np.random.uniform(L / 6, L / 3)
                width = np.random.uniform(W / 3, W / 2)

        return length, width, height

    def calculate_price(part):
        """根据最新的手稿公式计算零件价格。"""
        l, w, v, s, h = part['length'], part['width'], part['volume'], part['support_volume'], part['height']
        L, W = machine_length, machine_width
        V1, V2, t1 = V_1, V_2, t_1
        C_opr = C_operator
        K_eng, K_pow = K_energy, K_powder

        amortized_costs = max(l, w) / min(L, W) * (C_opr + (K_eng + K_gas) * h * t1 / 3600)
        direct_costs = K_pow * (v + s) + (K_eng + K_gas) * (v / V1 + s / V2) / 3600
        c_est = amortized_costs + direct_costs
        price = c_est * (1 + 1 / loose_factor) * (1 + K_fine/K_fine_max)
        return price

    def create_part(part_id, arrival_time, part_type):
        """创建一个具有完整属性的零件字典。"""
        l, w, h = generate_dimensions_for_type(part_type)
        v = l * w * h * np.random.uniform(0.05, 0.15)
        s = v * np.random.uniform(0.0, 0.3)

        part = {
            'ID': f'P{part_id}', 'type': part_type, 'r_p': arrival_time,
            'length': l, 'width': w, 'height': h, 'volume': v, 'support_volume': s
        }

        t_alo = (T + (v / V_1 + s / V_2 + h * t_1)) / 3600
        part['delivery_time'] = arrival_time + loose_factor * t_alo
        part['price'] = calculate_price(part)

        return part

    height_map = {0: 'Low', 1: 'High'}
    footprint_map = {0: 'Small', 1: 'Long', 2: 'Large'}

    parts_in_window = []
    # part_id_counter = 0
    window_duration = end_time - start_time
    if window_duration <= 0:
        return []

    for h_idx in range(arrival_rate_matrix.shape[0]):
        for f_idx in range(arrival_rate_matrix.shape[1]):
            rate = arrival_rate_matrix[h_idx, f_idx]
            if rate <= 0: continue

            # 基于泊松分布采样该时间段内的到达数量
            expected_arrivals = rate * window_duration
            num_arrivals = np.random.poisson(expected_arrivals)

            if num_arrivals == 0: continue

            part_type = (height_map[h_idx], footprint_map[f_idx])

            # 为每个到达事件生成一个随机的到达时间
            arrival_times = np.random.uniform(start_time, end_time, size=num_arrivals)

            for t in arrival_times:
                # part_id_counter += 1
                part_id = f"L_{start_time}_{t:.2f}"
                part = create_part(part_id, t, part_type)
                parts_in_window.append(part)

    parts_in_window.sort(key=lambda p: p['r_p'])
    return parts_in_window

# endregion


"""================================= 模块:状态结构与MCTS节点结构类==================================="""
# region


# --- 状态与节点定义 ---


class State:
    def __init__(self, point, parts, machine_free_time, total_profit):
        self.point = point
        self.parts = parts
        self.machine_free_time = machine_free_time
        self.total_profit = total_profit  # 注意：在 VFA MCTS 结构中，这个 total_profit 更多是用于 rollout
        self.parts_batch = []
        self.profit_batch = 0
        self.last_arrivals = []


class Node:
    """ 决策前状态节点 (Pre-decision State Node, S̃) - 文献中的"方块" """

    def __init__(self, k, state, parent, action, rep):
        self.k = k  # 当前时刻
        self.state = state  # State 对象
        self.parent = parent  # 父节点 (也是一个 Node)
        self.action = action  # 导致这个节点的 *上一步* 动作

        self.children = {}  # type: dict[any, ActionStats]
        self.avail_actions = []

        self.N = 0  # 访问次数 N(S̃)
        self.V = 0  # 累计价值 Ṽ(S̃)
        self.q_list = []  # 用于日志记录

        # 记录以此节点为 *起点* 进行了第几次仿真
        self.simulation_start_indices = []
        # 新增: 采样权重，表示有多少 W̃ 采样合并到了这个节点
        self.sampling_weight = 0


class ActionStats:
    """ 状态-决策对的统计 (State-Action Pair, (S̃, x̃)) - 文献中的"边" """

    def __init__(self, action, C):
        self.action = action  # 决策 x̃
        self.C = C  # 直接收益 C(S̃, x̃) (the contribution)

        self.children = []  # type: list[Node]

        self.N_pair = 0  # 访问次数 N(S̃, x̃)
        self.V_children_sum = 0.0  # 子节点 S̃' 的价值总和 Σ Ṽ(S̃')

        # 新增: W̃ 的总采样次数
        self.W_sample_count = 0

    def get_Vx(self):
        """
        计算决策后状态 S̃x 的期望价值 Ṽx(S̃x)
        可以使用 Q value 概念表述
        """
        if self.N_pair == 0:
            return 0.0
        return self.V_children_sum / self.N_pair

# endregion


"""====================================== 模块:MCTS算法类 ========================================"""
# region


class MCTS_sample:
    def __init__(self, initial_state):
        self.root = Node(k=initial_state.point, state=initial_state, parent=None, action=None, rep=-1)

    def run(self, simulations):
        # 初始扩展根节点, 找到所有可行

        self.initialization(self.root)

        for i in range(simulations):
            simulation_index = i + 1

            rollout_node = self.tree_policy()

            # 记录这个节点是第 (i+1) 次仿真的起点
            if rollout_node:
                rollout_node.simulation_start_indices.append([simulation_index])

            q_rollout = self.rollout_policy(rollout_node)
            self.backup(rollout_node, q_rollout)

    def ucb_exploration(self, N_parent, N_pair):
        """ UCT 探索项 """
        if N_pair == 0:
            return float('inf')
        if N_parent == 0:  # 理论上不应发生，除非是根节点
            N_parent = 1

        return exploration_factor * math.sqrt(2 * math.log(N_parent) / N_pair)

    def _are_states_equal(self, state1, state2):
        """ 比较两个状态对象是否相同 """
        if state1.point != state2.point:
            return False

        # 使用 1e-9 (纳秒) 级别比较浮点数
        if abs(state1.machine_free_time - state2.machine_free_time) > 1e-9:
            return False

        if len(state1.parts) != len(state2.parts):
            return False

        # 比较零件列表 (假设 'ID' 是关键)
        # 排序以确保顺序无关
        s1_part_ids = sorted(p['ID'] for p in state1.parts)
        s2_part_ids = sorted(p['ID'] for p in state2.parts)

        return s1_part_ids == s2_part_ids

    def _find_matching_child(self, new_state, children_list):
        """ 在子节点列表中查找具有相同状态的节点 """
        for existing_child in children_list:
            if self._are_states_equal(new_state, existing_child.state):
                return existing_child
        return None

    def initialization(self, node):
        """ 扩展一个 "决策前" 节点: 找到所有可行决策 x̃,
            计算 C(S̃, x̃), 并创建 ActionStats. """

        # 1. 获取可行决策 x̃ 的集合
        avail_parts = node.state.parts
        current_machine_free_time = node.state.machine_free_time
        time_point = node.state.point

        node.avail_actions.append(0)  # Postpone
        feasible_combines = find_all_feasible_combinations_heuristic(avail_parts, machine_length, machine_width,
                                                                     machine_height)
        print("可行的’Produce‘决策数量:", len(feasible_combines))
        print("包括:", feasible_combines)
        feasible_combines_sorted = select_decisions_by_pareto_front(feasible_combines, avail_parts,
                                                                    current_machine_free_time, time_point)
        node.avail_actions.extend(feasible_combines_sorted)
        print("被剪枝后的可行决策集合:", node.avail_actions)

        # 2. 为每个决策创建 ActionStats
        for action in set(node.avail_actions):  # 使用 set 避免重复
            if action not in node.children:
                # 计算直接收益 C(S̃, x̃)
                C, _ = self.get_contribution_and_post_decision_state(node.state, action)
                node.children[action] = ActionStats(action=action, C=C)

    def tree_policy(self):
        """
        实现 Fig 19.9 (TreePolicy) 逻辑
        (!!!) 包含合并相同 W̃ 采样的修改
        """
        current_node = self.root
        while current_node.k < K:  # while non-terminal

            # Step 2.1: 扩展
            if not current_node.children:
                self.expansion(current_node)

            # Step 2.1 (续): 选择未尝试的动作
            untried_actions = [a for a in current_node.children.values() if a.N_pair == 0]

            if untried_actions:
                action_stat = random.choice(untried_actions)
                # 采样这个新动作, (可能)创建 S' 节点, 并返回
                new_node = self.sample_next_state(current_node, action_stat)
                return new_node

            # --- (!!!) 核心修改点: MuZero Min-Max Normalization (!!!) ---
            # Step 2.5: UCT 选择

            # 1. 找到所有 *已尝试* 动作的 (V^x) 值
            tried_stats = [s for s in current_node.children.values() if s.N_pair > 0]

            min_vx = 0.0
            max_vx = 0.0

            if len(tried_stats) > 1:
                values = [s.get_Vx() for s in tried_stats]
                min_vx = min(values)
                max_vx = max(values)
            elif len(tried_stats) == 1:
                min_vx = max_vx = tried_stats[0].get_Vx()

            # 2. 定义归一化函数
            def normalize_value(vx):
                if (max_vx - min_vx) == 0:
                    return 0.0  # 所有值都相同, 或只有一个值, 归一化为 0
                return (vx - min_vx) / (max_vx - min_vx)

            # --- 归一化结束 ---

            # 3. 基于 *归一化* 的 V^x (即 V_bar) 和 UCB 探索项来选择
            action_stat = max(
                current_node.children.values(),  # 仍然在所有子节点中选择 (未尝试的 UCB 为 inf)
                key=lambda s: (normalize_value(s.get_Vx()) +
                               self.ucb_exploration(current_node.N, s.N_pair))
            )
            # --- (!!!) 核心修改结束 (!!!) ---

            # Step 3: (Fig 19.9) - 从 S^x 采样 S'

            # 使用 W_sample_count 代替 len(action_stat.children)
            if action_stat.W_sample_count < W_SAMPLING_THRESHOLD:
                # Step 3.1-3.5: 随机结果采样未达阈值
                action_stat.W_sample_count += 1  # 递增 *总采样* 计数器

                # 采样 W̃
                # 此函数现在会处理重复状态：
                # 1. 如果是新状态, 返回新 node
                # 2. 如果是旧状态, 返回旧 node 并增加其 sampling_weight
                new_or_existing_node = self.sample_next_state(current_node, action_stat)

                return new_or_existing_node  # 返回此节点用于 rollout
            else:
                # Step 3.6-3.8: 随机结果已充分采样 (e^thr met)

                # (!!!) 修改: 按 sampling_weight 进行有权重的随机选择
                children_nodes = action_stat.children
                children_weights = [node.sampling_weight for node in children_nodes]

                selected_child = random.choices(children_nodes, weights=children_weights, k=1)[0]

                current_node = selected_child
                # 循环继续, 从 child_node (新的 S̃) 开始

        return current_node  # 返回终局节点

    def expansion(self, node):
        """ 扩展一个 "决策前" 节点: 找到所有可行决策 x̃,
            计算 C(S̃, x̃), 并创建 ActionStats. """

        # 1. 获取所有可行决策 x̃
        avail_parts = node.state.parts
        current_machine_free_time = node.state.machine_free_time
        time_point = node.state.point

        node.avail_actions.append(0)  # Postpone
        feasible_combines = find_all_feasible_combinations_heuristic(avail_parts, machine_length, machine_width,
                                                                     machine_height)
        feasible_combines_sorted = select_decisions_by_pareto_front(feasible_combines, avail_parts,
                                                                    current_machine_free_time, time_point)
        node.avail_actions.extend(feasible_combines_sorted)

        # 2. 为每个决策创建 ActionStats
        for action in set(node.avail_actions):  # 使用 set 避免重复
            if action not in node.children:
                # 计算直接收益 C(S̃, x̃)
                C, _ = self.get_contribution_and_post_decision_state(node.state, action)
                node.children[action] = ActionStats(action=action, C=C)

    def sample_next_state(self, parent_node, action_stat):
        """
        (!!!) 已修改 (!!!)
        从 (S̃, x̃) 对进行采样, 并检查状态是否重复
        """
        # 1. (S̃, x̃) -> S̃x : 获取决策后状态
        _, post_decision_state = self.get_contribution_and_post_decision_state(parent_node.state, action_stat.action)

        # 2. S̃x -> S̃' : 应用随机信息 W̃
        next_pre_decision_state = self.apply_exogenous_info(post_decision_state)

        # 3. (!!!) 检查新状态是否与已有子节点重复
        existing_child = self._find_matching_child(next_pre_decision_state, action_stat.children)

        if existing_child:
            # 3a. 状态重复: 增加权重并返回已存在的节点
            existing_child.sampling_weight += 1
            return existing_child
        else:
            # 3b. 状态不重复: 创建新节点
            rep = action_stat.N_pair
            new_node = Node(
                k=parent_node.k + 1,
                state=next_pre_decision_state,
                parent=parent_node,
                action=action_stat.action,
                rep=rep
            )
            # (new_node.sampling_weight 默认为 1)
            new_node.sampling_weight = 1
            action_stat.children.append(new_node)
            return new_node

    def get_contribution_and_post_decision_state(self, input_state, action):
        """
        计算 C(S̃, x̃) 和 S̃x
        (即 state_trans(不含 sample_arrivals_for_lookahead))
        """
        state = copy.deepcopy(input_state)
        C = 0.0  # 初始化直接收益

        if state.machine_free_time >= (state.point + 1) * u:  # 如果机器在当前时间段上不可用，即 "Still processing."
            cost_still_processing = cost_tardiness_wait(state.parts, state.point) + cost_tardiness_process(state.parts_batch, state.machine_free_time, state.point)
            state.total_profit -= cost_still_processing  # 将零件池中等待的零件产生的延迟成本计入累计收益
            C -= cost_still_processing

        else:  # 机器在当前时间段上可用，即 "available." / "not in processing."

            if state.machine_free_time >= state.point * u and state.profit_batch != 0:  # 如果有批次在这个时间段上完工了
                state.total_profit -= cost_tardiness_process(state.parts_batch, state.machine_free_time, state.point)  # 计入生产批次中零件的延迟成本, 更新累计收益
                state.profit_batch = 0
                state.parts_batch = []  # 清空生产批次中的零件

            if len(state.parts) == 0:  # 虽然机器在当前时间段上可用，当前无零件可用，即 "There is no part available."
                # 机器空闲且无零件可用,显然无延迟成本
                state.machine_free_time = (state.point + 1) * u  # 更新机器可用时间

            else:  # 机器在当前时间段上可用，并且当前有零件可用，则需要决策 “Postpone or Produce”。

                if action == 0:  # postpone
                    # --- 决策：推迟 ---
                    cost_postpone = cost_tardiness_wait(state.parts, state.point)
                    state.total_profit -= cost_postpone  # 将延迟等待的零件产生的延迟成本计入累计收益
                    C -= cost_postpone
                    state.machine_free_time = (state.point + 1) * u

                else:  # "Produce." with batch "state.parts_batch"
                    # --- 决策：生产 ---

                    for p in state.parts:  # 获取生产批次
                        if str(p['ID']) in action:
                            state.parts_batch.append(p)

                    profit, new_tau = foresee_profit_completion(state.parts_batch, state.machine_free_time)  # 获取纯利润和完工时间
                    state.profit_batch = profit - C_operator  # 获取批次真实利润
                    state.total_profit += state.profit_batch
                    C += state.profit_batch

                    cost_tardy_produce = cost_tardiness_wait([part for part in state.parts if part not in state.parts_batch], state.point) + cost_tardiness_process(state.parts_batch, new_tau, state.point)
                    state.total_profit -= cost_tardy_produce
                    C -= cost_tardy_produce

                    state.machine_free_time = new_tau
                    state.parts = [part for part in state.parts if part not in state.parts_batch]

        # 返回 (直接收益 C, 决策后状态 S̃x)
        return C, state

    def apply_exogenous_info(self, post_decision_state):
        """
        应用 W̃ (随机信息), 将 S̃x 变为 S̃'
        """
        state = post_decision_state  # 无需复制, 它已经是 S̃x 的副本

        new_arrivals = sample_arrivals_for_lookahead(state.point * u + 0.001, (state.point + 1) * u)
        state.last_arrivals = new_arrivals
        state.parts += new_arrivals
        state.point += 1

        return state  # 返回 S̃'

    def rollout_policy(self, node):
        """ 保持不变: 从一个"决策前"节点 S̃ 开始模拟 """
        current_state = copy.deepcopy(node.state)

        while current_state.point < K:
            current_state = self.state_trans_rollout(current_state)
        final_profit = current_state.total_profit
        return final_profit

    def state_trans_rollout(self, state):
        """
        rollout 策略 (注意: state_trans_rollout 内部需要调用 完整 的状态转移,即包含 决策+随机信息 的逻辑)
        """
        if state.machine_free_time >= (state.point + 1) * u:  # 如果机器在当前时间段上不可用，即 "Still processing."
            state.total_profit -= cost_tardiness_wait(state.parts, state.point)  # 将零件池中等待的零件产生的延迟成本计入累计收益
            state.total_profit -= cost_tardiness_process(state.parts_batch, state.machine_free_time, state.point)   # 计入生产批次中零件的延迟成本, 更新累计收益

        else:  # 机器在当前时间段上可用，即 "available." / "not in processing."

            if state.machine_free_time >= state.point * u and state.profit_batch != 0:  # 如果有批次在这个时间段上完工了
                state.total_profit -= cost_tardiness_process(state.parts_batch, state.machine_free_time, state.point)   # 计入生产批次中零件的延迟成本, 更新累计收益
                state.profit_batch = 0
                state.parts_batch = []  # 清空生产批次中的零件

            if len(state.parts) == 0:  # 虽然机器在当前时间段上可用，当前无零件可用，即 "There is no part available."
                # 机器空闲且无零件可用,显然无延迟成本
                state.machine_free_time = (state.point + 1) * u  # 更新机器可用时间

            else:  # 机器在当前时间段上可用，并且当前有零件可用，则需要决策 “Postpone or Produce”。

                # # --- 单步前瞻决策过程 ---

                produce_batch = batch_selection_MBP_fast(state.parts)  # MBP_fast 启发式获取生产批次
                batch_profit, complete_time = foresee_profit_completion(produce_batch, state.machine_free_time)   # 获取生产批次的纯预期收益(不包括延迟和操作员成本)和完工时间

                r_produce = batch_profit - C_operator  # 将操作员费用计入 r_produce

                v_postpone_immediate = -cost_tardiness_wait(state.parts, state.point)   # 延迟等待的零件产生的延迟成本
                hypothetical_next_state = State(  # 获取假设的下一个状态（当前时点选择延迟）
                    point=state.point + 1,
                    parts=state.parts + sample_arrivals_for_lookahead(state.point * u, (state.point + 1) * u),
                    machine_free_time=(state.point + 1) * u,
                    total_profit=state.total_profit + v_postpone_immediate    # 将延迟等待的零件产生的延迟成本计入累计收益
                )
                future_batch = batch_selection_MBP_fast(hypothetical_next_state.parts)  # MBP_fast 启发式获取生产批次
                future_batch_profit, future_complete_time = foresee_profit_completion(future_batch,hypothetical_next_state.machine_free_time)  # 获取生产批次的纯预期收益(不包括延迟和操作员成本)和完工时间

                r_postpone = future_batch_profit - C_operator + v_postpone_immediate   # 将下一个时点选择生产的批次收益计入累计收益 <----- 此处修改了

                if r_postpone >= r_produce:
                    # --- 决策：推迟 ---
                    state.total_profit += v_postpone_immediate  # 将延迟等待的零件产生的延迟成本计入累计收益
                    state.machine_free_time = (state.point + 1) * u  # 更新机器可用时间

                else:  # "Produce." with batch "state.parts_batch"
                    # --- 决策：生产 ---
                    state.parts_batch = produce_batch
                    state.profit_batch = r_produce
                    state.total_profit += r_produce  # 净贡献
                    state.total_profit -= cost_tardiness_process(produce_batch, complete_time, state.point)
                    state.total_profit += v_postpone_immediate  # 将延迟等待的零件产生的延迟成本计入累计收益
                    state.machine_free_time = complete_time
                    state.parts = [p for p in state.parts if p not in produce_batch]

        # --- 状态转移的最后一步：新零件到达 ---
        state.parts += sample_arrivals_for_lookahead(state.point * u + 0.001, (state.point + 1) * u)
        state.point += 1

        return state

    def backup(self, node, q_roll):
        """
        根据新结构重写的 Backup (Fig 19.11)
        回传 V_roll (文献中的 Ṽ(S̃_L))
        """

        current_node = node
        while current_node is not None:
            current_node.N += 1
            current_node.V += q_roll
            current_node.q_list.append(q_roll)

            if current_node.parent is not None:
                parent = current_node.parent
                action = current_node.action

                # 更新 (S̃_parent, x̃) 对的统计数据
                action_stat = parent.children[action]
                action_stat.N_pair += 1

                action_stat.V_children_sum += q_roll

            current_node = current_node.parent

    def Q_action(self):
        """ 适配新结构, 打印根节点决策统计 """
        action_stats_dict = self.root.children
        if not action_stats_dict:
            return {}

        results = {}
        print("--- Decision-wise Statistics ---")
        for action, stat in action_stats_dict.items():
            total_n = stat.N_pair
            if total_n > 0:

                # 收集所有子节点的 q_list
                all_q_values = []
                for child_node in stat.children:
                    all_q_values.extend(child_node.q_list)

                if len(all_q_values) > 1:
                    ci = st.norm.interval(0.95, loc=np.mean(all_q_values), scale=st.sem(all_q_values))
                else:
                    ci = (float('nan'), float('nan'))

                results[action] = (stat.get_Vx(), total_n, ci)
                print(f"Decision: {action}, Contribution: {stat.C:.2f}, State-Decision Value (Q Value): {stat.get_Vx():.2f}, N: {total_n}, 95% CI (of Q Value): ({ci[0]:.2f}, {ci[1]:.2f})")

        print("-----------------------------------------------------")
        return results

    def get_best_action(self):
        """ 根据文献 (Fig 19.8, Step 2) 选择最终决策 """
        action_stats_dict = self.root.children
        if not action_stats_dict:
            print("No decisions available at root.")
            return None

        results = self.Q_action()

        # 最终决策 = arg max [ Ṽx(S̃x_t) ]
        best_action = max(
            results,
            key=lambda action: results[action][0]  # results[action][0] is (V^x)
        )

        print(f"\nBest Decision: {best_action} with State-Decision Value: {results[best_action][0]:.2f}")
        return best_action

    def _format_parts_list(self, parts_list):
        if not parts_list: return ""
        return ','.join([str(p.get('ID', 'N/A')) for p in parts_list])

    def log_tree_info(self, filename=f"mcts_log.csv"):
        """ 适配新结构 (Node -> ActionStats -> Node) """

        tree_data = []
        node_queue = [(self.root, None)]  # (Node, parent_id)
        node_id_counter = 0

        while node_queue:
            current_node, parent_id = node_queue.pop(0)
            current_id = node_id_counter
            node_id_counter += 1

            avg_v = current_node.V / current_node.N if current_node.N > 0 else 0
            q_values_str = ','.join(map(lambda q: f"{q:.2f}", current_node.q_list))

            # <--- 新增 --->
            sim_starts_str = ','.join(map(str, current_node.simulation_start_indices))

            if current_node.parent:
                parts_before_str = self._format_parts_list(current_node.parent.state.parts)
            else:
                parts_before_str = self._format_parts_list(current_node.state.parts)

            parts_arrival_str = self._format_parts_list(current_node.state.last_arrivals)

            node_info = {
                'id': current_id,
                'parent_id': parent_id,
                'k': current_node.k,
                'Parts_in_Queue_before_action': parts_before_str,
                'action_taken_to_get_here': str(current_node.action),
                'Parts_Arrival_at_this_node': parts_arrival_str,
                'V': round(current_node.V, 4),
                'N': current_node.N,
                'avg_V': round(avg_v, 4),
                'Q_values': q_values_str,
                'simulation_start_indices': sim_starts_str,  # <--- 新增 --->
                'children_action_count': len(current_node.children),
                'sampling_count': current_node.sampling_weight
            }
            tree_data.append(node_info)

            # 遍历 ActionStats, 将其子 Node 加入队列
            for action_stat in current_node.children.values():
                for child_node in action_stat.children:
                    # 检查是否已在队列中 (防止重复添加? MCTS 树不应有环)
                    # 我们假设树结构正确
                    node_queue.append((child_node, current_id))

        if not tree_data:
            print("No data to log.")
            return

        try:
            fieldnames = [
                'id', 'parent_id', 'k',
                'Parts_in_Queue_before_action',
                'action_taken_to_get_here', 'Parts_Arrival_at_this_node', 'V', 'N',
                'avg_V', 'Q_values', 'simulation_start_indices',  # <--- 新增 --->
                'children_action_count','sampling_count'
            ]
            with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(tree_data)
            print(f"\nTree information with {len(tree_data)} nodes successfully saved to '{filename}'")
        except IOError as e:
            print(f"Error writing to file '{filename}': {e}")


class MCTSEntireExpansion_sample:
    def __init__(self, initial_state):
        self.root = Node(k=initial_state.point, state=initial_state, parent=None, action=None, rep=-1)

    def run(self, simulations):
        # 初始扩展根节点, 找到所有可行

        self.initialization(self.root)

        for i in range(simulations):
            simulation_index = i + 1

            rollout_node = self.tree_policy()

            # 记录这个节点是第 (i+1) 次仿真的起点
            if rollout_node:
                rollout_node.simulation_start_indices.append([simulation_index])

            q_rollout = self.rollout_policy(rollout_node)
            self.backup(rollout_node, q_rollout)

    def ucb_exploration(self, N_parent, N_pair):
        """ UCT 探索项 """
        if N_pair == 0:
            return float('inf')
        if N_parent == 0:  # 理论上不应发生，除非是根节点
            N_parent = 1

        return exploration_factor * math.sqrt(2 * math.log(N_parent) / N_pair)

    def _are_states_equal(self, state1, state2):
        """ 比较两个状态对象是否相同 """
        if state1.point != state2.point:
            return False

        # 使用 1e-9 (纳秒) 级别比较浮点数
        if abs(state1.machine_free_time - state2.machine_free_time) > 1e-9:
            return False

        if len(state1.parts) != len(state2.parts):
            return False

        # 比较零件列表 (假设 'ID' 是关键)
        # 排序以确保顺序无关
        s1_part_ids = sorted(p['ID'] for p in state1.parts)
        s2_part_ids = sorted(p['ID'] for p in state2.parts)

        return s1_part_ids == s2_part_ids

    def _find_matching_child(self, new_state, children_list):
        """ 在子节点列表中查找具有相同状态的节点 """
        for existing_child in children_list:
            if self._are_states_equal(new_state, existing_child.state):
                return existing_child
        return None

    def initialization(self, node):
        """ 扩展一个 "决策前" 节点: 找到所有可行决策 x̃,
            计算 C(S̃, x̃), 并创建 ActionStats. """

        # 1. 获取所有可行决策 x̃
        avail_parts = node.state.parts

        node.avail_actions.append(0)  # Postpone
        feasible_combines = find_all_feasible_combinations_heuristic(avail_parts, machine_length, machine_width,
                                                                     machine_height)
        print("可行的’Produce‘决策数量:", len(feasible_combines))

        node.avail_actions.extend(feasible_combines)
        print("可行决策集合:", node.avail_actions)

        # 2. 为每个决策创建 ActionStats
        for action in set(node.avail_actions):  # 使用 set 避免重复
            if action not in node.children:
                # 计算直接收益 C(S̃, x̃)
                C, _ = self.get_contribution_and_post_decision_state(node.state, action)
                node.children[action] = ActionStats(action=action, C=C)

    def tree_policy(self):
        """
        实现 Fig 19.9 (TreePolicy) 逻辑
        (!!!) 包含合并相同 W̃ 采样的修改
        """
        current_node = self.root
        while current_node.k < K:  # while non-terminal

            # Step 2.1: 扩展
            if not current_node.children:
                self.expansion(current_node)

            # Step 2.1 (续): 选择未尝试的动作
            untried_actions = [a for a in current_node.children.values() if a.N_pair == 0]

            if untried_actions:
                action_stat = random.choice(untried_actions)
                # 采样这个新动作, (可能)创建 S' 节点, 并返回
                new_node = self.sample_next_state(current_node, action_stat)
                return new_node

            # --- (!!!) 核心修改点: MuZero Min-Max Normalization (!!!) ---
            # Step 2.5: UCT 选择

            # 1. 找到所有 *已尝试* 动作的 (V^x) 值
            tried_stats = [s for s in current_node.children.values() if s.N_pair > 0]

            min_vx = 0.0
            max_vx = 0.0

            if len(tried_stats) > 1:
                values = [s.get_Vx() for s in tried_stats]
                min_vx = min(values)
                max_vx = max(values)
            elif len(tried_stats) == 1:
                min_vx = max_vx = tried_stats[0].get_Vx()

            # 2. 定义归一化函数
            def normalize_value(vx):
                if (max_vx - min_vx) == 0:
                    return 0.0  # 所有值都相同, 或只有一个值, 归一化为 0
                return (vx - min_vx) / (max_vx - min_vx)

            # --- 归一化结束 ---

            # 3. 基于 *归一化* 的 V^x (即 V_bar) 和 UCB 探索项来选择
            action_stat = max(
                current_node.children.values(),  # 仍然在所有子节点中选择 (未尝试的 UCB 为 inf)
                key=lambda s: (normalize_value(s.get_Vx()) +
                               self.ucb_exploration(current_node.N, s.N_pair))
            )
            # --- (!!!) 核心修改结束 (!!!) ---

            # Step 3: (Fig 19.9) - 从 S^x 采样 S'

            # 使用 W_sample_count 代替 len(action_stat.children)
            if action_stat.W_sample_count < W_SAMPLING_THRESHOLD:
                # Step 3.1-3.5: 随机结果采样未达阈值
                action_stat.W_sample_count += 1  # 递增 *总采样* 计数器

                # 采样 W̃
                # 此函数现在会处理重复状态：
                # 1. 如果是新状态, 返回新 node
                # 2. 如果是旧状态, 返回旧 node 并增加其 sampling_weight
                new_or_existing_node = self.sample_next_state(current_node, action_stat)

                return new_or_existing_node  # 返回此节点用于 rollout
            else:
                # Step 3.6-3.8: 随机结果已充分采样 (e^thr met)

                # (!!!) 修改: 按 sampling_weight 进行有权重的随机选择
                children_nodes = action_stat.children
                children_weights = [node.sampling_weight for node in children_nodes]

                selected_child = random.choices(children_nodes, weights=children_weights, k=1)[0]

                current_node = selected_child
                # 循环继续, 从 child_node (新的 S̃) 开始

        return current_node  # 返回终局节点

    def expansion(self, node):
        """ 扩展一个 "决策前" 节点: 找到所有可行决策 x̃,
            计算 C(S̃, x̃), 并创建 ActionStats. """

        # 1. 获取所有可行决策 x̃
        avail_parts = node.state.parts

        node.avail_actions.append(0)  # Postpone
        feasible_combines = find_all_feasible_combinations_heuristic(avail_parts, machine_length, machine_width,
                                                                     machine_height)
        node.avail_actions.extend(feasible_combines)

        # 2. 为每个决策创建 ActionStats
        for action in set(node.avail_actions):  # 使用 set 避免重复
            if action not in node.children:
                # 计算直接收益 C(S̃, x̃)
                C, _ = self.get_contribution_and_post_decision_state(node.state, action)
                node.children[action] = ActionStats(action=action, C=C)

    def sample_next_state(self, parent_node, action_stat):
        """
        (!!!) 已修改 (!!!)
        从 (S̃, x̃) 对进行采样, 并检查状态是否重复
        """
        # 1. (S̃, x̃) -> S̃x : 获取决策后状态
        _, post_decision_state = self.get_contribution_and_post_decision_state(parent_node.state, action_stat.action)

        # 2. S̃x -> S̃' : 应用随机信息 W̃
        next_pre_decision_state = self.apply_exogenous_info(post_decision_state)

        # 3. (!!!) 检查新状态是否与已有子节点重复
        existing_child = self._find_matching_child(next_pre_decision_state, action_stat.children)

        if existing_child:
            # 3a. 状态重复: 增加权重并返回已存在的节点
            existing_child.sampling_weight += 1
            return existing_child
        else:
            # 3b. 状态不重复: 创建新节点
            rep = action_stat.N_pair
            new_node = Node(
                k=parent_node.k + 1,
                state=next_pre_decision_state,
                parent=parent_node,
                action=action_stat.action,
                rep=rep
            )
            # (new_node.sampling_weight 默认为 1)
            new_node.sampling_weight = 1
            action_stat.children.append(new_node)
            return new_node

    def get_contribution_and_post_decision_state(self, input_state, action):
        """
        计算 C(S̃, x̃) 和 S̃x
        (即 state_trans(不含 sample_arrivals_for_lookahead))
        """
        state = copy.deepcopy(input_state)
        C = 0.0  # 初始化直接收益

        if state.machine_free_time >= (state.point + 1) * u:  # 如果机器在当前时间段上不可用，即 "Still processing."
            cost_still_processing = cost_tardiness_wait(state.parts, state.point) + cost_tardiness_process(state.parts_batch, state.machine_free_time, state.point)
            state.total_profit -= cost_still_processing  # 将零件池中等待的零件产生的延迟成本计入累计收益
            C -= cost_still_processing

        else:  # 机器在当前时间段上可用，即 "available." / "not in processing."

            if state.machine_free_time >= state.point * u and state.profit_batch != 0:  # 如果有批次在这个时间段上完工了
                state.total_profit -= cost_tardiness_process(state.parts_batch, state.machine_free_time, state.point)  # 计入生产批次中零件的延迟成本, 更新累计收益
                state.profit_batch = 0
                state.parts_batch = []  # 清空生产批次中的零件

            if len(state.parts) == 0:  # 虽然机器在当前时间段上可用，当前无零件可用，即 "There is no part available."
                # 机器空闲且无零件可用,显然无延迟成本
                state.machine_free_time = (state.point + 1) * u  # 更新机器可用时间

            else:  # 机器在当前时间段上可用，并且当前有零件可用，则需要决策 “Postpone or Produce”。

                if action == 0:  # postpone
                    # --- 决策：推迟 ---
                    cost_postpone = cost_tardiness_wait(state.parts, state.point)
                    state.total_profit -= cost_postpone  # 将延迟等待的零件产生的延迟成本计入累计收益
                    C -= cost_postpone
                    state.machine_free_time = (state.point + 1) * u

                else:  # "Produce." with batch "state.parts_batch"
                    # --- 决策：生产 ---

                    for p in state.parts:  # 获取生产批次
                        if str(p['ID']) in action:
                            state.parts_batch.append(p)

                    profit, new_tau = foresee_profit_completion(state.parts_batch, state.machine_free_time)  # 获取纯利润和完工时间
                    state.profit_batch = profit - C_operator  # 获取批次真实利润
                    state.total_profit += state.profit_batch
                    C += state.profit_batch

                    cost_tardy_produce = cost_tardiness_wait([part for part in state.parts if part not in state.parts_batch], state.point) + cost_tardiness_process(state.parts_batch, new_tau, state.point)
                    state.total_profit -= cost_tardy_produce
                    C -= cost_tardy_produce

                    state.machine_free_time = new_tau
                    state.parts = [part for part in state.parts if part not in state.parts_batch]

        # 返回 (直接收益 C, 决策后状态 S̃x)
        return C, state

    def apply_exogenous_info(self, post_decision_state):
        """
        应用 W̃ (随机信息), 将 S̃x 变为 S̃'
        """
        state = post_decision_state  # 无需复制, 它已经是 S̃x 的副本

        new_arrivals = sample_arrivals_for_lookahead(state.point * u + 0.001, (state.point + 1) * u)
        state.last_arrivals = new_arrivals
        state.parts += new_arrivals
        state.point += 1

        return state  # 返回 S̃'

    def rollout_policy(self, node):
        """ 保持不变: 从一个"决策前"节点 S̃ 开始模拟 """
        current_state = copy.deepcopy(node.state)

        while current_state.point < K:
            current_state = self.state_trans_rollout(current_state)
        final_profit = current_state.total_profit
        return final_profit

    def state_trans_rollout(self, state):
        """
        rollout 策略 (注意: state_trans_rollout 内部需要调用 完整 的状态转移,即包含 决策+随机信息 的逻辑)
        """
        if state.machine_free_time >= (state.point + 1) * u:  # 如果机器在当前时间段上不可用，即 "Still processing."
            state.total_profit -= cost_tardiness_wait(state.parts, state.point)  # 将零件池中等待的零件产生的延迟成本计入累计收益
            state.total_profit -= cost_tardiness_process(state.parts_batch, state.machine_free_time, state.point)   # 计入生产批次中零件的延迟成本, 更新累计收益

        else:  # 机器在当前时间段上可用，即 "available." / "not in processing."

            if state.machine_free_time >= state.point * u and state.profit_batch != 0:  # 如果有批次在这个时间段上完工了
                state.total_profit -= cost_tardiness_process(state.parts_batch, state.machine_free_time, state.point)   # 计入生产批次中零件的延迟成本, 更新累计收益
                state.profit_batch = 0
                state.parts_batch = []  # 清空生产批次中的零件

            if len(state.parts) == 0:  # 虽然机器在当前时间段上可用，当前无零件可用，即 "There is no part available."
                # 机器空闲且无零件可用,显然无延迟成本
                state.machine_free_time = (state.point + 1) * u  # 更新机器可用时间

            else:  # 机器在当前时间段上可用，并且当前有零件可用，则需要决策 “Postpone or Produce”。

                # # --- 单步前瞻决策过程 ---

                produce_batch = batch_selection_MBP_fast(state.parts)  # MBP_fast 启发式获取生产批次
                batch_profit, complete_time = foresee_profit_completion(produce_batch, state.machine_free_time)   # 获取生产批次的纯预期收益(不包括延迟和操作员成本)和完工时间

                r_produce = batch_profit - C_operator  # 将操作员费用计入 r_produce

                v_postpone_immediate = -cost_tardiness_wait(state.parts, state.point)   # 延迟等待的零件产生的延迟成本
                hypothetical_next_state = State(  # 获取假设的下一个状态（当前时点选择延迟）
                    point=state.point + 1,
                    parts=state.parts + sample_arrivals_for_lookahead(state.point * u, (state.point + 1) * u),
                    machine_free_time=(state.point + 1) * u,
                    total_profit=state.total_profit + v_postpone_immediate    # 将延迟等待的零件产生的延迟成本计入累计收益
                )
                future_batch = batch_selection_MBP_fast(hypothetical_next_state.parts)  # MBP_fast 启发式获取生产批次
                future_batch_profit, future_complete_time = foresee_profit_completion(future_batch,hypothetical_next_state.machine_free_time)  # 获取生产批次的纯预期收益(不包括延迟和操作员成本)和完工时间

                r_postpone = future_batch_profit - C_operator + v_postpone_immediate   # 将下一个时点选择生产的批次收益计入累计收益 <----- 此处修改了

                if r_postpone >= r_produce:
                    # --- 决策：推迟 ---
                    state.total_profit += v_postpone_immediate  # 将延迟等待的零件产生的延迟成本计入累计收益
                    state.machine_free_time = (state.point + 1) * u  # 更新机器可用时间

                else:  # "Produce." with batch "state.parts_batch"
                    # --- 决策：生产 ---
                    state.parts_batch = produce_batch
                    state.profit_batch = r_produce
                    state.total_profit += r_produce  # 净贡献
                    state.total_profit -= cost_tardiness_process(produce_batch, complete_time, state.point)
                    state.total_profit += v_postpone_immediate  # 将延迟等待的零件产生的延迟成本计入累计收益
                    state.machine_free_time = complete_time
                    state.parts = [p for p in state.parts if p not in produce_batch]

        # --- 状态转移的最后一步：新零件到达 ---
        state.parts += sample_arrivals_for_lookahead(state.point * u + 0.001, (state.point + 1) * u)
        state.point += 1

        return state

    def backup(self, node, q_roll):
        """
        根据新结构重写的 Backup (Fig 19.11)
        回传 V_roll (文献中的 Ṽ(S̃_L))
        """

        current_node = node
        while current_node is not None:
            current_node.N += 1
            current_node.V += q_roll
            current_node.q_list.append(q_roll)

            if current_node.parent is not None:
                parent = current_node.parent
                action = current_node.action

                # 更新 (S̃_parent, x̃) 对的统计数据
                action_stat = parent.children[action]
                action_stat.N_pair += 1

                action_stat.V_children_sum += q_roll

            current_node = current_node.parent

    def Q_action(self):
        """ 适配新结构, 打印根节点决策统计 """
        action_stats_dict = self.root.children
        if not action_stats_dict:
            return {}

        results = {}
        print("--- Decision-wise Statistics ---")
        for action, stat in action_stats_dict.items():
            total_n = stat.N_pair
            if total_n > 0:

                # 收集所有子节点的 q_list
                all_q_values = []
                for child_node in stat.children:
                    all_q_values.extend(child_node.q_list)

                if len(all_q_values) > 1:
                    ci = st.norm.interval(0.95, loc=np.mean(all_q_values), scale=st.sem(all_q_values))
                else:
                    ci = (float('nan'), float('nan'))

                results[action] = (stat.get_Vx(), total_n, ci)
                print(f"Decision: {action}, Contribution: {stat.C:.2f}, State-Decision Value (Q Value): {stat.get_Vx():.2f}, N: {total_n}, 95% CI (of Q Value): ({ci[0]:.2f}, {ci[1]:.2f})")

        print("-----------------------------------------------------")
        return results

    def get_best_action(self):
        """ 根据文献 (Fig 19.8, Step 2) 选择最终决策 """
        action_stats_dict = self.root.children
        if not action_stats_dict:
            print("No decisions available at root.")
            return None

        results = self.Q_action()

        # 最终决策 = arg max [ Ṽx(S̃x_t) ]
        best_action = max(
            results,
            key=lambda action: results[action][0]  # results[action][0] is (V^x)
        )

        print(f"\nBest Decision: {best_action} with State-Decision Value: {results[best_action][0]:.2f}")
        return best_action

    def _format_parts_list(self, parts_list):
        if not parts_list: return ""
        return ','.join([str(p.get('ID', 'N/A')) for p in parts_list])

    def log_tree_info(self, filename=f"mcts_log.csv"):
        """ 适配新结构 (Node -> ActionStats -> Node) """

        tree_data = []
        node_queue = [(self.root, None)]  # (Node, parent_id)
        node_id_counter = 0

        while node_queue:
            current_node, parent_id = node_queue.pop(0)
            current_id = node_id_counter
            node_id_counter += 1

            avg_v = current_node.V / current_node.N if current_node.N > 0 else 0
            q_values_str = ','.join(map(lambda q: f"{q:.2f}", current_node.q_list))

            # <--- 新增 --->
            sim_starts_str = ','.join(map(str, current_node.simulation_start_indices))

            if current_node.parent:
                parts_before_str = self._format_parts_list(current_node.parent.state.parts)
            else:
                parts_before_str = self._format_parts_list(current_node.state.parts)

            parts_arrival_str = self._format_parts_list(current_node.state.last_arrivals)

            node_info = {
                'id': current_id,
                'parent_id': parent_id,
                'k': current_node.k,
                'Parts_in_Queue_before_action': parts_before_str,
                'action_taken_to_get_here': str(current_node.action),
                'Parts_Arrival_at_this_node': parts_arrival_str,
                'V': round(current_node.V, 4),
                'N': current_node.N,
                'avg_V': round(avg_v, 4),
                'Q_values': q_values_str,
                'simulation_start_indices': sim_starts_str,  # <--- 新增 --->
                'children_action_count': len(current_node.children),
                'sampling_count': current_node.sampling_weight
            }
            tree_data.append(node_info)

            # 遍历 ActionStats, 将其子 Node 加入队列
            for action_stat in current_node.children.values():
                for child_node in action_stat.children:
                    # 检查是否已在队列中 (防止重复添加? MCTS 树不应有环)
                    # 我们假设树结构正确
                    node_queue.append((child_node, current_id))

        if not tree_data:
            print("No data to log.")
            return

        try:
            fieldnames = [
                'id', 'parent_id', 'k',
                'Parts_in_Queue_before_action',
                'action_taken_to_get_here', 'Parts_Arrival_at_this_node', 'V', 'N',
                'avg_V', 'Q_values', 'simulation_start_indices',  # <--- 新增 --->
                'children_action_count','sampling_count'
            ]
            with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(tree_data)
            print(f"\nTree information with {len(tree_data)} nodes successfully saved to '{filename}'")
        except IOError as e:
            print(f"Error writing to file '{filename}': {e}")


# endregion


"""=================================== 模块:交付延迟的计算函数======================================"""
# region


# Function to calculate cost of tardiness generated by parts in wait
def cost_tardiness_wait(parts_wait, k):

    def tardiness_wait(d_p):
        if d_p <= k * u:
            return u
        elif d_p > k * u:
            return max((k + 1) * u - d_p, 0)

    tardiness_wait = sum(tardiness_wait(p['delivery_time']) for p in parts_wait)

    cost_tardiness = K_fine * tardiness_wait

    return cost_tardiness


# Function to calculate cost of tardiness generated by parts in process
def cost_tardiness_process(parts_process, batch_complete, k):
    # 计算生产中的批次产生的延迟成本

    def tardiness_process(d_p):
        # batch_complete 完成时间肯定大于 k * u
        if batch_complete <= (k + 1) * u:  # 如果在这个时间区间上完成
            if d_p <= k * u:  # 如果在这个时间前已经延迟
                return batch_complete - (k * u)
            elif d_p > k * u:
                return max(batch_complete - d_p, 0)  # 如果在完成时间晚于due,计算延迟,否则无延迟

        else:  # 如果在下个时间点后完成
            if d_p <= k * u:
                return u
            elif d_p > k * u:
                return max((k + 1) * u - d_p, 0)

    tardiness_process = sum(tardiness_process(p['delivery_time']) for p in parts_process)  # 单位：小时

    cost_tardiness = K_fine * tardiness_process  # 单位：元

    return cost_tardiness
# endregion


"""=============================== 模块:problem-specific heuristics ============================="""
# region


# --- Greedy Repair Fallback Mechanism ---
def _greedy_repair_batch(candidate_parts, sort_key_for_removal, remove_worst):
    """
    Takes an infeasible candidate batch and iteratively removes the "worst" part
    until it becomes feasible.

    Args:
        candidate_parts (list): The initial infeasible list of parts.
        sort_key_for_removal (function): Lambda function to rank parts for removal.
        remove_worst (bool): If True, removes from the end of the sorted list (worst);
                             If False, removes from the beginning (worst).

    Returns:
        list: The largest possible feasible subset found, or an empty list.
    """
    temp_parts = list(candidate_parts)
    while len(temp_parts) > 0:
        if heuristic_check_packing_feasibility(temp_parts, machine_length, machine_width, machine_height) == "feasible":
            return temp_parts

        # Sort based on the removal criterion to find the "worst" part
        temp_parts.sort(key=sort_key_for_removal, reverse=remove_worst)

        # Remove the worst part
        temp_parts.pop()

    return []  # Return empty if even a single part is not feasible


# --- Fast Heuristic Implementations ---
def batch_selection_MBP_fast(parts):
    """
    Fast MBP Heuristic with a greedy repair fallback.
    """
    if not parts:
        return []

    def get_density(part):
        area = part['length'] * part['width']
        return part['price'] / area if area > 0 else 0

    sorted_parts = sorted(parts, key=get_density, reverse=True)

    candidate_parts = []
    current_area = 0
    platform_area = machine_length * machine_width
    for part in sorted_parts:
        part_area = part['length'] * part['width']
        if current_area + part_area <= platform_area:
            candidate_parts.append(part)
            current_area += part_area
        else:
            break

    if heuristic_check_packing_feasibility(candidate_parts, machine_length, machine_width, machine_height) == "feasible":
        return candidate_parts
    else:
        # Fallback: Iteratively remove the LEAST profitable part (lowest density)
        return _greedy_repair_batch(
            candidate_parts,
            sort_key_for_removal=get_density,
            remove_worst=False,  # remove_worst=False sorts descending, pop() removes last (worst)
        )

# endregion


"""==================================== 模块:系统仿真支撑函数 ====================================="""
# region


# 获取生产批次的预期收益和完工时间 ## 没考虑C_operator 和 Tardiness
def foresee_profit_completion(parts_k, tau_k):
    # Step 1: Calculate V^price_k 计算批次总价格
    V_price_k = sum(p['price'] for p in parts_k)

    # Step 2: Calculate C^cost_k 计算批次总生产成本
    # Function to calculate build time t^build: AM_BATCH
    def calculate_build_time():
        bt = sum((p['volume'] / V_1 + p['support_volume'] / V_2) for p in parts_k) + t_1 * max(
            (p['height']) for p in parts_k)
        return bt / 3600

    t_build = calculate_build_time()  # 构建时间，单位：小时

    C_energy = K_energy * t_build  # 能耗成本 单位：元
    C_material = K_powder * sum(p['volume'] + p['support_volume'] for p in parts_k) + K_gas * t_build  # 材料成本 单位：元

    t_pro = t_build + T / 3600  # 生产时间，单位：小时

    # Step 4: Calculate 生产批次的预期收益和完工时间
    V_k = V_price_k - C_energy - C_material  # 单位：元
    new_tau = tau_k + t_pro  # 单位：小时

    return V_k, new_tau


# 按即时收益排序所有可行的零件组合
def rank_combinations_by_instant_profit(feasible_combines, avail_parts):

    def calculate_V_k(combine):

        # for combine in feasible_combines:
        parts_batch = []

        for p in avail_parts:

            if str(p['ID']) in combine:
                parts_batch.append(p)

        # Step 1: Calculate V^price_k 计算批次总价格
        V_price_k = sum(p['price'] for p in parts_batch)

        # Step 2: Calculate C^cost_k 计算批次总生产成本
        # Function to calculate build time t^build: AM_BATCH
        t_build = (sum((p['volume'] / V_1 + p['support_volume'] / V_2) for p in parts_batch) + t_1 * max(
                (p['height']) for p in parts_batch)) / 3600  # 构建时间，单位：小时

        C_energy = K_energy * t_build  # 能耗成本 单位：元
        C_material = K_powder * sum(p['volume'] + p['support_volume'] for p in parts_batch) + K_gas * t_build  # 材料成本 单位：元

        # Step 4: Calculate 生产批次的预期收益
        V_k = V_price_k - C_energy - C_material  # 单位：元

        return V_k

    # 对 feasible_combines 进行排序，根据每个 combine 对应的 V_k 从高到低
    feasible_combines_sorted = sorted(feasible_combines, key=calculate_V_k, reverse=True)

    return feasible_combines_sorted


# 零件平均成本
def _calculate_avg_cost(parts):
    # 2. 计算一个整合批次的生产成本
    # 2a. 体积相关成本
    total_volume_cost = K_powder * sum(p['volume'] + p['support_volume'] for p in parts)
    total_build_time_volume_part = sum((p['volume'] / V_1 + p['support_volume'] / V_2) / 3600 for p in parts)
    total_volume_related_energy_gas_cost = (K_energy + K_gas) * total_build_time_volume_part

    # 2b. 高度相关成本 (基于最大高度)
    max_height = max(p['height'] for p in parts)
    height_related_build_time = t_1 * max_height / 3600
    height_related_energy_gas_cost = (K_energy + K_gas) * height_related_build_time

    total_production_cost = (
                C_operator + total_volume_cost + total_volume_related_energy_gas_cost + height_related_energy_gas_cost)

    avg_production_cost = total_production_cost / len(parts)

    return avg_production_cost


# 从可选决策集合里选取帕累托前沿集合
def select_decisions_by_pareto_front(feasible_combines, avail_parts, current_machine_free_time, time_point):
    """
    根据双目标评估所有可行的“生产”决策，
    并返回帕累托最优集中的一个代表性子集。

    Args:
        feasible_combines (list of tuples): 所有可行的零件组合（每个组合是一个零件ID的元组）。
        avail_parts (list of dict): 当前所有可用零件的列表。
        current_machine_free_time (float): 当前机器的可用时间。

    Returns:
        list of tuples: 一个最优决策的列表。
    """
    if not feasible_combines:
        return []

    evaluated_decisions = []

    # 将可用零件转换为字典以便快速查找
    parts_dict = {p['ID']: p for p in avail_parts}

    # 1. 为每个可行决策计算两个目标值
    for combine in feasible_combines:
        parts_in_batch = [parts_dict[pid] for pid in combine]

        # 目标 1: 即时利润
        batch_profit, completion_time = foresee_profit_completion(parts_in_batch, current_machine_free_time)
        immediate_net_profit = (batch_profit - C_operator)
        # # 目标 2:
        avg_cost = _calculate_avg_cost(parts_in_batch)

        evaluated_decisions.append({
            "combine": combine,
            "profit": immediate_net_profit,
            "fpub": avg_cost,
            "num_parts": len(combine)
        })

    # 2. 识别帕累托最优集
    pareto_front = []
    for d1 in evaluated_decisions:
        is_dominated = False
        for d2 in evaluated_decisions:
            if d1 is d2:
                continue
            # 检查 d1 是否被 d2 支配
            if (d2["profit"] >= d1["profit"] and d2["fpub"] <= d1["fpub"]) and \
                    (d2["profit"] > d1["profit"] or d2["fpub"] < d1["fpub"]):
                is_dominated = True
                break
        if not is_dominated:
            pareto_front.append(d1)

    selected_decisions = [d["combine"] for d in pareto_front]

    return selected_decisions


# endregion


"""====================================== 主程序:系统仿真 ========================================"""
# region


# 仿真主程序
def simulation_of_dynamics_system():
    """
    Approach: MCTS with Decisions in Pareto Front Expanded.
    """
    np.random.seed(random_seed)

    print("\nApproach: MCTS with Decisions in Pareto Front Expanded.")

    profit = []  # 记录累计收益
    act = []
    batch = []
    avail_parts = []
    Mcts_Time = []

    simulation_start = time.time()

    state = State(initial_point, part_stream.get_arrivals_in_window(0.0, 0.001), initial_machine_free_time, total_profit=0)  # 初始状态

    while state.point < K:  # 在时间区间内 (0 -> K-1)

        print(state.point, state.total_profit, len(state.parts))
        profit.append(state.total_profit)
        avail_parts.append(len(state.parts))

        if state.machine_free_time >= (state.point + 1) * u:  # 如果机器在当前时间段上不可用，即 "Still processing."
            print("Still processing.")
            act.append("Still processing.")
            state.total_profit -= cost_tardiness_wait(state.parts, state.point)  # 将零件池中等待的零件产生的延迟成本计入累计收益
            state.total_profit -= cost_tardiness_process(state.parts_batch, state.machine_free_time, state.point)  # 将生产中零件在本时段上延迟成本的计入累计收益
            state.parts += part_stream.get_arrivals_in_window(state.point * u + 0.001, (state.point + 1) * u)  # 更新零件集合

        else:  # 机器在当前时间段上可用，即 "available." / "not in processing."

            if state.machine_free_time >= state.point * u and state.profit_batch != 0:  # 如果有批次在这个时间段上完工了
                state.total_profit -= cost_tardiness_process(state.parts_batch, state.machine_free_time, state.point)   # 计入生产批次中零件的延迟成本, 更新累计收益
                print(f"批次生产在此时段完成,完成时间:{state.machine_free_time:.2f}")
                state.profit_batch = 0
                state.parts_batch = []  # 清空生产批次中的零件

            if len(state.parts) == 0:  # 虽然机器在当前时间段上可用，当前无零件可用，即 "There is no part available."
                print("There is no part available.")
                act.append("No part available.")
                # 机器空闲且无零件可用,显然无延迟成本
                state.machine_free_time = (state.point + 1) * u  # 更新机器可用时间
                state.parts += part_stream.get_arrivals_in_window(state.point * u + 0.001, (state.point + 1) * u)  # 更新零件集合

            else:  # 机器在当前时间段上可用，并且当前有零件可用，则需要决策 “Postpone or Produce”。

                # 利用蒙特卡洛树搜索(MCTS)来构造前瞻策略

                # 1. 当前状态作为树的根节点
                simulate_state = copy.deepcopy(state)

                # 2. 初始化 MCTS 当前状态作为树的根节点
                mcts = MCTS_sample(simulate_state)

                # 3. 运行仿真
                print(f"Running MCTS simulations (budget={sim_budget}) ...")
                start_time = time.time()
                mcts.run(simulations=sim_budget)
                end_time = time.time()
                mcts_time = end_time - start_time
                print("time consumed:", mcts_time)
                Mcts_Time.append(mcts_time)
                print("Simulations finished.")

                # 4. 获取最优动作
                action = mcts.get_best_action()

                # 5. 将树的完整信息保存到文档
                # mcts.log_tree_info(f"mcts_log_Pareto_H{K}-R{arrival_type}-L{loose_factor}-C{C_operator}-K{K_fine}-S{random_seed}_rep{rep_id}(t={state.point}).csv")

                if action == 0:  # "Postpone."
                    print("Postpone.")
                    act.append("Postpone.")
                    state.total_profit -= cost_tardiness_wait(state.parts, state.point)  # 将延迟等待的零件产生的延迟成本计入累计收益
                    state.machine_free_time = (state.point + 1) * u  # 更新机器可用时间
                    state.parts += part_stream.get_arrivals_in_window(state.point * u + 0.001, (state.point + 1) * u)  # 更新零件集合

                else:  # "Produce." with batch "state.parts_batch"
                    print("Produce.")
                    act.append("Produce.")
                    batch.append(action)
                    state.parts_batch = []
                    print(f"  - 生产批次包含零件:")
                    for p in state.parts:   # 根据 action 选择要 Produce 的零件 "state.parts_batch"
                        if p['ID'] in action:
                            print(p['ID'])
                            state.parts_batch.append(p)
                    print(f"  - 机器在 {state.machine_free_time:.2f} 时开始工作")
                    state.profit_batch, new_tau = foresee_profit_completion(state.parts_batch, state.machine_free_time)  # 获取生产批次的纯预期收益和完工时间
                    print(f"  - 机器将在 {new_tau:.2f} 时空闲")
                    state.profit_batch -= C_operator  # 将操作员费用计入批次的收益 state.profit_batch
                    state.total_profit -= cost_tardiness_process(state.parts_batch, new_tau, state.point)  # 将批次内零件在本时段上延迟成本的计入累计收益
                    state.total_profit -= cost_tardiness_wait([part for part in state.parts if part not in state.parts_batch], state.point)  # 将未生产的零件的延迟成本计入累计收益
                    state.total_profit += state.profit_batch  # 获取这个批次的收益, 更新累计收益
                    state.machine_free_time = new_tau  # 更新机器可用时间
                    state.parts = [part for part in state.parts if part not in state.parts_batch] + part_stream.get_arrivals_in_window(state.point * u + 0.001, (state.point + 1) * u)  # 更新零件集合

        state.point += 1  # 仿真推进
        print('\n')

    simulation_end = time.time()

    simulation_time = simulation_end - simulation_start

    profit.append(state.total_profit)
    print(profit)

    act.append("End of horizon.")
    print(act)

    print(avail_parts)

    print(state.point, state.total_profit, len(state.parts))  # 最终时点状态
    print("total profit:", state.total_profit)
    print("profit of batch in process:", state.profit_batch)
    print("machine available time:", state.machine_free_time)

    # 打开文件进行写入
    with open(f"Result_Pareto_Optimal_H{K}-R{arrival_type}-L{loose_factor}-C{C_operator}-K{K_fine}-S{random_seed}-rep_id{rep_id}.txt", "w") as file:  #_'theta'{exploration_factor}_'samples'{W_SAMPLING_THRESHOLD}

        file.write("profit accumulation process:")
        file.write(str(profit) + ",\n")

        file.write("decision process:")
        file.write(str(act) + ",\n")
        
        file.write("batch sequence:")
        file.write(str(batch) + ",\n")

        file.write("number of available parts process:")
        file.write(str(avail_parts) + ",\n")

        file.write("total profit:")
        file.write(str(state.total_profit))
        file.write(",\n")

        file.write("number of unserved parts:")
        file.write(str(len(state.parts)))
        file.write(",\n")

        file.write("profit of batch in process:")
        file.write(str(state.profit_batch))
        file.write(",\n")

        file.write("machine available time:")
        file.write(str(state.machine_free_time))
        file.write(",\n")

        file.write("run time per a MCTS:")
        file.write(str(np.mean(Mcts_Time)))
        file.write(",\n")

        file.write("total run time:")
        file.write(str(simulation_time))
        # file.write(",\n")

    return state.total_profit, np.mean(Mcts_Time), simulation_time
# endregion


"""=============================== Baseline:基于MCTS变体的系统仿真 ==============================="""
# region


# Baseline5: MCTS-based ADP with Entire Decision Expansion
def simulation_of_dynamics_system_expand_entire_decisions():
    """
    MCTS-based ADP with Entire Decision Expansion
    """

    np.random.seed(random_seed)

    print("\nApproach:MCTS-based ADP with Entire Decision Expansion")

    profit = []  # 记录累计收益
    act = []
    batch = []
    avail_parts = []
    Mcts_Time = []

    state = State(initial_point, part_stream.get_arrivals_in_window(0.0, 0.001), initial_machine_free_time,
                  total_profit=0)  # 初始状态

    while state.point < K:  # 在时间区间内 (0 -> K-1)

        print(state.point, state.total_profit, len(state.parts))
        profit.append(state.total_profit)
        avail_parts.append(len(state.parts))

        if state.machine_free_time >= (state.point + 1) * u:  # 如果机器在当前时间段上不可用，即 "Still processing."
            print("Still processing.")
            act.append("Still processing.")
            state.total_profit -= cost_tardiness_wait(state.parts, state.point)  # 将零件池中等待的零件产生的延迟成本计入累计收益
            state.total_profit -= cost_tardiness_process(state.parts_batch, state.machine_free_time,
                                                         state.point)  # 将生产中零件在本时段上延迟成本的计入累计收益
            state.parts += part_stream.get_arrivals_in_window(state.point * u + 0.001, (state.point + 1) * u)  # 更新零件集合

        else:  # 机器在当前时间段上可用，即 "available." / "not in processing."

            if state.machine_free_time >= state.point * u and state.profit_batch != 0:  # 如果有批次在这个时间段上完工了
                state.total_profit -= cost_tardiness_process(state.parts_batch, state.machine_free_time,
                                                             state.point)  # 计入生产批次中零件的延迟成本, 更新累计收益
                print(f"批次生产在此时段完成,完成时间:{state.machine_free_time:.2f}")
                state.profit_batch = 0
                state.parts_batch = []  # 清空生产批次中的零件

            if len(state.parts) == 0:  # 虽然机器在当前时间段上可用，当前无零件可用，即 "There is no part available."
                print("There is no part available.")
                act.append("No part available.")
                # 机器空闲且无零件可用,显然无延迟成本
                state.machine_free_time = (state.point + 1) * u  # 更新机器可用时间
                state.parts += part_stream.get_arrivals_in_window(state.point * u + 0.001,
                                                                  (state.point + 1) * u)  # 更新零件集合

            else:  # 机器在当前时间段上可用，并且当前有零件可用，则需要决策 “Postpone or Produce”。

                # 利用蒙特卡洛树搜索(MCTS)来构造前瞻策略

                # 1. 当前状态作为树的根节点
                simulate_state = copy.deepcopy(state)

                # 2. 初始化 MCTS 当前状态作为树的根节点
                mcts = MCTSEntireExpansion_sample(simulate_state)

                # 3. 运行仿真
                print(f"Running MCTS simulations (budget={sim_budget}) ...")
                start_time = time.time()
                mcts.run(simulations=sim_budget)
                end_time = time.time()
                mcts_time = end_time - start_time
                print("time consumed:", mcts_time)
                Mcts_Time.append(mcts_time)
                print("Simulations finished.")

                # 4. 获取最优动作
                action = mcts.get_best_action()

                # 5. 将树的完整信息保存到文档
                # mcts.log_tree_info(f"H{K}-R{arrival_type}-L{loose_factor}-C{C_operator}-K{K_fine}-S{random_seed}_mcts_log(t={state.point}).csv")

                if action == 0:  # "Postpone."
                    print("Postpone.")
                    act.append("Postpone.")
                    state.total_profit -= cost_tardiness_wait(state.parts, state.point)  # 将延迟等待的零件产生的延迟成本计入累计收益
                    state.machine_free_time = (state.point + 1) * u  # 更新机器可用时间
                    state.parts += part_stream.get_arrivals_in_window(state.point * u + 0.001,
                                                                      (state.point + 1) * u)  # 更新零件集合

                else:  # "Produce." with batch "state.parts_batch"
                    print("Produce.")
                    act.append("Produce.")
                    batch.append(action)
                    state.parts_batch = []
                    print(f"  - 生产批次包含零件:")
                    for p in state.parts:   # 根据 action 选择要 Produce 的零件 "state.parts_batch"
                        if p['ID'] in action:
                            print(p['ID'])
                            state.parts_batch.append(p)
                    print(f"  - 机器在 {state.machine_free_time:.2f} 时开始工作")
                    state.profit_batch, new_tau = foresee_profit_completion(state.parts_batch,
                                                                            state.machine_free_time)  # 获取生产批次的纯预期收益和完工时间
                    print(f"  - 机器将在 {new_tau:.2f} 时空闲")
                    state.profit_batch -= C_operator  # 将操作员费用计入批次的收益 state.profit_batch
                    state.total_profit -= cost_tardiness_process(state.parts_batch, new_tau,
                                                                 state.point)  # 将批次内零件在本时段上延迟成本的计入累计收益
                    state.total_profit -= cost_tardiness_wait(
                        [part for part in state.parts if part not in state.parts_batch],
                        state.point)  # 将未生产的零件的延迟成本计入累计收益
                    state.total_profit += state.profit_batch  # 获取这个批次的收益, 更新累计收益
                    state.machine_free_time = new_tau  # 更新机器可用时间
                    state.parts = [part for part in state.parts if
                                   part not in state.parts_batch] + part_stream.get_arrivals_in_window(
                        state.point * u + 0.001, (state.point + 1) * u)  # 更新零件集合

        state.point += 1  # 仿真推进
        print('\n')

    profit.append(state.total_profit)
    print(profit)

    act.append("End of horizon.")
    print(act)

    print(avail_parts)

    print(state.point, state.total_profit, len(state.parts))  # 最终时点状态
    print("total profit:", state.total_profit)
    print("profit of batch in process:", state.profit_batch)
    print("completion time of batch in process:", state.machine_free_time)

    # 打开文件进行写入
    with open(
            f"Result_expand_entire_decisions_H{K}-R{arrival_type}-L{loose_factor}-C{C_operator}-K{K_fine}-S{random_seed}-rep_id{rep_id}.txt",
            "w") as file:  # -rep_id{rep_id} _'theta'{theta}_'epsilon'{samples}

        file.write("profit accumulation process:")
        file.write(str(profit) + ",\n")

        file.write("decision process:")
        file.write(str(act) + ",\n")

        file.write("batch sequence:")
        file.write(str(batch) + ",\n")

        file.write("number of available parts process:")
        file.write(str(avail_parts) + ",\n")

        # file.write(str(state.point))
        # file.write(",\n")

        file.write("total profit:")
        file.write(str(state.total_profit))
        file.write(",\n")

        file.write("number of unserved parts:")
        file.write(str(len(state.parts)))
        file.write(",\n")

        file.write("profit of batch in process:")
        file.write(str(state.profit_batch))
        file.write(",\n")

        file.write("completion time of batch in process:")
        file.write(str(state.machine_free_time))
        file.write(",\n")

        file.write("run time per a MCTS:")
        file.write(str(np.mean(Mcts_Time)))
        # file.write(",\n")

    return state.total_profit
# endregion


"""================================ Baseline:基于启发式规则的系统仿真 =============================="""
# region


# Baseline1: Process while Available
def simulation_of_dynamics_system_process_while_available():
    """
    Process while Available
    """
    np.random.seed(random_seed)

    print("\nBaseline: 'Process while Available' Heuristic")

    profit = []  # 记录累计收益
    act = []
    batch = []
    avail_parts = []

    state = State(initial_point, part_stream.get_arrivals_in_window(0.0, 0.001), initial_machine_free_time, total_profit=0)  # 初始状态

    while state.point < K:  # 在时间区间内 (0 -> K-1)

        print(state.point, state.total_profit, len(state.parts))
        profit.append(state.total_profit)
        avail_parts.append(len(state.parts))

        if state.machine_free_time >= (state.point + 1) * u:  # 如果机器在当前时间段上不可用，即 "Still processing."
            print("Still processing.")
            act.append("Still processing.")
            state.total_profit -= cost_tardiness_wait(state.parts, state.point)  # 将零件池中等待的零件产生的延迟成本计入累计收益
            state.total_profit -= cost_tardiness_process(state.parts_batch, state.machine_free_time,
                                                         state.point)  # 将生产中零件在本时段上延迟成本的计入累计收益
            state.parts += part_stream.get_arrivals_in_window(state.point * u + 0.001, (state.point + 1) * u)  # 更新零件集合

        else:  # 机器在当前时间段上可用，即 "available." / "not in processing."

            if state.machine_free_time >= state.point * u and state.profit_batch != 0:  # 如果有批次在这个时间段上完工了
                state.total_profit -= cost_tardiness_process(state.parts_batch, state.machine_free_time,
                                                             state.point)  # 计入生产批次中零件的延迟成本, 更新累计收益
                print(f"批次生产在此时段完成,完成时间:{state.machine_free_time:.2f}")
                state.profit_batch = 0
                state.parts_batch = []  # 清空生产批次中的零件

            if len(state.parts) == 0:  # 虽然机器在当前时间段上可用，当前无零件可用，即 "There is no part available."
                print("There is no part available.")
                act.append("No part available.")
                # 机器空闲且无零件可用,显然无延迟成本
                state.machine_free_time = (state.point + 1) * u  # 更新机器可用时间
                state.parts += part_stream.get_arrivals_in_window(state.point * u + 0.001,
                                                                  (state.point + 1) * u)  # 更新零件集合

            else:  # 机器在当前时间段上可用，并且当前有零件可用，则 Produce
                # produce
                print("Produce.")

                # 方案1
                # 获取并存储所有可行的零件组合（启发式求解）
                feasible_combines = find_all_feasible_combinations_heuristic(state.parts, machine_length, machine_width,
                                                                   machine_height)
                # 按即时收益排序所有可行的零件组合，并取第一个作为决策
                feasible_combines_sorted = rank_combinations_by_instant_profit(feasible_combines, state.parts)
                action = feasible_combines_sorted[0]
                # 根据 action 选择要 Produce 的零件
                state.parts_batch = []
                for p in state.parts:
                    if p['ID'] in action:
                        state.parts_batch.append(p)

                act.append("Produce.")
                batch.append(action)

                print(f"  - 生产批次包含零件: {state.parts_batch}")
                print(f"  - 机器在 {state.machine_free_time:.2f} 时开始工作")
                state.profit_batch, new_tau = foresee_profit_completion(state.parts_batch,
                                                                        state.machine_free_time)  # 获取生产批次的纯预期收益和完工时间
                print(f"  - 机器将在 {new_tau:.2f} 时空闲")
                state.profit_batch -= C_operator  # 将操作员费用计入批次的收益 state.profit_batch
                state.total_profit -= cost_tardiness_process(state.parts_batch, new_tau,
                                                             state.point)  # 将批次内零件在本时段上延迟成本的计入累计收益
                state.total_profit -= cost_tardiness_wait(
                    [part for part in state.parts if part not in state.parts_batch],
                    state.point)  # 将未生产的零件的延迟成本计入累计收益
                state.total_profit += state.profit_batch  # 获取这个批次的收益, 更新累计收益
                state.machine_free_time = new_tau  # 更新机器可用时间
                state.parts = [part for part in state.parts if
                               part not in state.parts_batch] + part_stream.get_arrivals_in_window(
                    state.point * u + 0.001, (state.point + 1) * u)  # 更新零件集合

        state.point += 1  # 仿真推进
        print('\n')

    profit.append(state.total_profit)
    print(profit)

    act.append("End of horizon.")
    print(act)

    print(avail_parts)

    print(state.point, state.total_profit, len(state.parts))

    print("total profit:", state.total_profit)
    print("profit of batch in process:", state.profit_batch)
    print("machine available time:", state.machine_free_time)

    # 打开文件进行写入
    with open(f"Result_Process_while_Available_H{K}-R{arrival_type}-L{loose_factor}-C{C_operator}-K{K_fine}-S{random_seed}.txt", "w") as file:

        file.write("profit accumulation process:")
        file.write(str(profit) + ",\n")

        file.write("decision process:")
        file.write(str(act) + ",\n")
        
        file.write("batch sequence:")
        file.write(str(batch) + ",\n")

        file.write("number of available parts process:")
        file.write(str(avail_parts) + ",\n")

        file.write("total profit:")
        file.write(str(state.total_profit))
        file.write(",\n")

        file.write("number of unserved parts:")
        file.write(str(len(state.parts)))
        file.write(",\n")

        file.write("profit of batch in process:")
        file.write(str(state.profit_batch))
        file.write(",\n")

        file.write("machine available time:")
        file.write(str(state.machine_free_time))

    return state.total_profit


# Baseline2: Process with Waiting Buffer
def simulation_of_dynamics_system_process_with_waiting_buffer(buffer_length):
    """
    Process with Waiting Buffer
    """
    np.random.seed(random_seed)

    print(f"\nBenchmark:Process with Waiting Buffer = {buffer_length}")

    profit = []  # 记录累计收益
    act = []
    batch = []
    avail_parts = []

    # 初始化等待时间记录
    waiting_time = buffer_length

    state = State(initial_point, part_stream.get_arrivals_in_window(0.0, 0.001), initial_machine_free_time, total_profit=0)  # 初始状态

    while state.point < K:  # 在时间区间内

        avail_parts.append(len(state.parts))
        print(state.point, state.total_profit, len(state.parts))
        profit.append(state.total_profit)

        if state.machine_free_time >= (state.point + 1) * u:  # 如果机器在当前时间段上不可用，即 "Still processing."
            print("Still processing.")
            act.append("Still processing.")
            state.total_profit -= cost_tardiness_wait(state.parts, state.point)  # 将零件池中等待的零件产生的延迟成本计入累计收益
            state.total_profit -= cost_tardiness_process(state.parts_batch, state.machine_free_time,
                                                         state.point)  # 将生产中零件在本时段上延迟成本的计入累计收益
            state.parts += part_stream.get_arrivals_in_window(state.point * u + 0.001, (state.point + 1) * u)  # 更新零件集合

        else:  # 机器在当前时间段上可用，即 "available." / "not in processing."

            if state.machine_free_time >= state.point * u and state.profit_batch != 0:  # 如果有批次在这个时间段上完工了
                state.total_profit -= cost_tardiness_process(state.parts_batch, state.machine_free_time,
                                                             state.point)  # 计入生产批次中零件的延迟成本, 更新累计收益
                print(f"批次生产在此时段完成,完成时间:{state.machine_free_time:.2f}")
                state.profit_batch = 0
                state.parts_batch = []  # 清空生产批次中的零件

                waiting_time = 0    # 等待时间清零

            # 当等待时间不足buffer长度
            if waiting_time < buffer_length:  # <---- 由 "<=" 修改为 "<"

                if len(state.parts) == 0:  # 虽然机器在当前时间段上可用，当前无零件可用，即 "There is no part available."
                    print("There is no part available.")
                    act.append("No part available.")
                    # 机器空闲且无零件可用,显然无延迟成本
                    state.machine_free_time = (state.point + 1) * u  # 更新机器可用时间
                    state.parts += part_stream.get_arrivals_in_window(state.point * u + 0.001,
                                                                      (state.point + 1) * u)  # 更新零件集合
                    # 哪怕等待时间到达了buffer长度，也无法执行生产，因此等待时间加1
                    waiting_time += 1

                else:   # 当前有零件可用
                    # postpone
                    print("Postpone.")
                    act.append("Postpone.")
                    state.total_profit -= cost_tardiness_wait(state.parts, state.point)  # 将延迟等待的零件产生的延迟成本计入累计收益
                    state.machine_free_time = (state.point + 1) * u  # 更新机器可用时间
                    state.parts += part_stream.get_arrivals_in_window(state.point * u + 0.001,
                                                                      (state.point + 1) * u)  # 更新零件集合
                    # 等待时间加1
                    waiting_time += 1

            else:   # 当等待时间超过buffer长度

                if len(state.parts) == 0:  # 虽然机器在当前时间段上可用，当前无零件可用，即 "There is no part available."
                    print("There is no part available.")
                    act.append("No part available.")
                    # 机器空闲且无零件可用,显然无延迟成本
                    state.machine_free_time = (state.point + 1) * u  # 更新机器可用时间
                    state.parts += part_stream.get_arrivals_in_window(state.point * u + 0.001,
                                                                      (state.point + 1) * u)  # 更新零件集合
                    # 哪怕等待时间到达了buffer长度，也无法执行生产，因此等待时间加1
                    waiting_time += 1

                else:   # 当前有零件可用
                    # produce
                    print("Produce.")
                    act.append("Produce.")

                    # 方案1
                    # 获取并存储所有可行的零件组合
                    feasible_combines = find_all_feasible_combinations_heuristic(state.parts, machine_length, machine_width,
                                                                       machine_height)
                    # 按即时收益排序所有可行的零件组合，并取第一个作为决策
                    feasible_combines_sorted = rank_combinations_by_instant_profit(feasible_combines, state.parts)
                    action = feasible_combines_sorted[0]

                    batch.append(action)

                    state.parts_batch = []
                    for p in state.parts:  # 根据 action 选择要 Produce 的零件 "state.parts_batch"
                        if p['ID'] in action:
                            state.parts_batch.append(p)
                    print(f"  - 生产批次包含零件: {state.parts_batch}")
                    print(f"  - 机器在 {state.machine_free_time:.2f} 时开始工作")
                    state.profit_batch, new_tau = foresee_profit_completion(state.parts_batch,
                                                                            state.machine_free_time)  # 获取生产批次的纯预期收益和完工时间
                    print(f"  - 机器将在 {new_tau:.2f} 时空闲")

                    state.profit_batch -= C_operator  # 将操作员费用计入批次的收益 state.profit_batch
                    state.total_profit -= cost_tardiness_process(state.parts_batch, new_tau,
                                                                 state.point)  # 将批次内零件在本时段上延迟成本的计入累计收益
                    state.total_profit -= cost_tardiness_wait(
                        [part for part in state.parts if part not in state.parts_batch],
                        state.point)  # 将未生产的零件的延迟成本计入累计收益
                    state.total_profit += state.profit_batch  # 获取这个批次的收益, 更新累计收益
                    state.machine_free_time = new_tau  # 更新机器可用时间
                    state.parts = [part for part in state.parts if
                                   part not in state.parts_batch] + part_stream.get_arrivals_in_window(
                        state.point * u + 0.001, (state.point + 1) * u)  # 更新零件集合

        state.point += 1  # 仿真推进
        print('\n')

    profit.append(state.total_profit)
    print(profit)

    act.append("End of horizon.")
    print(act)

    print(avail_parts)

    print(state.point, state.total_profit, len(state.parts))

    print("total profit:", state.total_profit)
    print("profit of batch in process:", state.profit_batch)
    print("machine available time:", state.machine_free_time)

    # 打开文件进行写入
    with open(f"Result_Process_with_Waiting_Buffer={buffer_length}_H{K}-R{arrival_type}-L{loose_factor}-C{C_operator}-K{K_fine}-S{random_seed}.txt", "w") as file:

        file.write("profit accumulation process:")
        file.write(str(profit) + ",\n")

        file.write("decision process:")
        file.write(str(act) + ",\n")
        
        file.write("batch sequence:")
        file.write(str(batch) + ",\n")

        file.write("number of available parts process:")
        file.write(str(avail_parts) + ",\n")

        file.write("total profit:")
        file.write(str(state.total_profit))
        file.write(",\n")

        file.write("number of unserved parts:")
        file.write(str(len(state.parts)))
        file.write(",\n")

        file.write("profit of batch in process:")
        file.write(str(state.profit_batch))
        file.write(",\n")

        file.write("machine available time:")
        file.write(str(state.machine_free_time))

    return state.total_profit


# Baseline3: Process while Full Capacity
def simulation_of_dynamics_system_process_while_full_capacity(eta):
    """
    Process while Full Capacity
    """
    np.random.seed(random_seed)

    print(f"\nBenchmark:Process while Full Capacity (Area threshold = {eta})")

    profit = []  # 记录累计收益
    act = []
    batch = []
    avail_parts = []

    state = State(initial_point, part_stream.get_arrivals_in_window(0.0, 0.001), initial_machine_free_time, total_profit=0)  # 初始状态

    while state.point < K:  # 在时间区间内

        avail_parts.append(len(state.parts))
        print(state.point, state.total_profit, len(state.parts))
        profit.append(state.total_profit)

        if state.machine_free_time >= (state.point + 1) * u:  # 如果机器在当前时间段上不可用，即 "Still processing."
            print("Still processing.")
            act.append("Still processing.")
            state.total_profit -= cost_tardiness_wait(state.parts, state.point)  # 将零件池中等待的零件产生的延迟成本计入累计收益
            state.total_profit -= cost_tardiness_process(state.parts_batch, state.machine_free_time,
                                                         state.point)  # 将生产中零件在本时段上延迟成本的计入累计收益
            state.parts += part_stream.get_arrivals_in_window(state.point * u + 0.001, (state.point + 1) * u)  # 更新零件集合

        else:   # 如果机器在当前时间段上可用
            if state.machine_free_time >= state.point * u and state.profit_batch != 0:  # 如果有批次在这个时间段上完工了
                state.total_profit -= cost_tardiness_process(state.parts_batch, state.machine_free_time,
                                                             state.point)  # 计入生产批次中零件的延迟成本, 更新累计收益
                print(f"批次生产在此时段完成,完成时间:{state.machine_free_time:.2f}")
                state.profit_batch = 0
                state.parts_batch = []  # 清空生产批次中的零件

            if len(state.parts) == 0:  # 虽然机器在当前时间段上可用，当前无零件可用，即 "There is no part available."
                print("There is no part available.")
                act.append("No part available.")
                # 机器空闲且无零件可用,显然无延迟成本
                state.machine_free_time = (state.point + 1) * u  # 更新机器可用时间
                state.parts += part_stream.get_arrivals_in_window(state.point * u + 0.001,
                                                                  (state.point + 1) * u)  # 更新零件集合

            else:
                # 如果当前有零件可用
                if heuristic_check_packing_feasibility(state.parts, machine_length, machine_width, machine_height) == "feasible" and sum(p['length'] * p['width'] for p in state.parts) <= eta * machine_length * machine_width:
                    action = 0
                else:
                    action = 1

                if action == 1:
                    # produce

                    print("Produce.")
                    act.append("Produce.")

                    # 方案1
                    # 获取并存储所有可行的零件组合
                    feasible_combines = find_all_feasible_combinations_heuristic(state.parts, machine_length, machine_width,
                                                                       machine_height)
                    # 按即时收益排序所有可行的零件组合，并取第一个作为决策
                    feasible_combines_sorted = rank_combinations_by_instant_profit(feasible_combines, state.parts)
                    action = feasible_combines_sorted[0]

                    batch.append(action)

                    state.parts_batch = []
                    for p in state.parts:  # 根据 action 选择要 Produce 的零件 "state.parts_batch"
                        if p['ID'] in action:
                            state.parts_batch.append(p)
                    print(f"  - 生产批次包含零件: {state.parts_batch}")
                    print(f"  - 机器在 {state.machine_free_time:.2f} 时开始工作")
                    state.profit_batch, new_tau = foresee_profit_completion(state.parts_batch,
                                                                            state.machine_free_time)  # 获取生产批次的纯预期收益和完工时间
                    print(f"  - 机器将在 {new_tau:.2f} 时空闲")

                    state.profit_batch -= C_operator  # 将操作员费用计入批次的收益 state.profit_batch
                    state.total_profit -= cost_tardiness_process(state.parts_batch, new_tau,
                                                                 state.point)  # 将批次内零件在本时段上延迟成本的计入累计收益
                    state.total_profit -= cost_tardiness_wait(
                        [part for part in state.parts if part not in state.parts_batch],
                        state.point)  # 将未生产的零件的延迟成本计入累计收益
                    state.total_profit += state.profit_batch  # 获取这个批次的收益, 更新累计收益
                    state.machine_free_time = new_tau  # 更新机器可用时间
                    state.parts = [part for part in state.parts if
                                   part not in state.parts_batch] + part_stream.get_arrivals_in_window(
                        state.point * u + 0.001, (state.point + 1) * u)  # 更新零件集合

                else:
                    # postpone
                    print("Postpone.")
                    act.append("Postpone.")
                    state.total_profit -= cost_tardiness_wait(state.parts, state.point)  # 将延迟等待的零件产生的延迟成本计入累计收益
                    state.machine_free_time = (state.point + 1) * u  # 更新机器可用时间
                    state.parts += part_stream.get_arrivals_in_window(state.point * u + 0.001,
                                                                      (state.point + 1) * u)  # 更新零件集合

        state.point += 1  # 仿真推进
        print('\n')

    profit.append(state.total_profit)
    print(profit)

    act.append("End of horizon.")
    print(act)

    print(avail_parts)

    print(state.point, state.total_profit, len(state.parts))

    print("total profit:", state.total_profit)
    print("profit of batch in process:", state.profit_batch)
    print("machine available time:", state.machine_free_time)

    # 打开文件进行写入
    with open(f"Result_Process_while_Full_Capacity(Area_threshold={eta})_H{K}-R{arrival_type}-L{loose_factor}-C{C_operator}-K{K_fine}-S{random_seed}.txt", "w") as file:

        file.write("profit accumulation process:")
        file.write(str(profit) + ",\n")

        file.write("decision process:")
        file.write(str(act) + ",\n")
        
        file.write("batch sequence:")
        file.write(str(batch) + ",\n")

        file.write("number of available parts process:")
        file.write(str(avail_parts) + ",\n")

        file.write("total profit:")
        file.write(str(state.total_profit))
        file.write(",\n")

        file.write("number of unserved parts:")
        file.write(str(len(state.parts)))
        file.write(",\n")

        file.write("profit of batch in process:")
        file.write(str(state.profit_batch))
        file.write(",\n")

        file.write("machine available time:")
        file.write(str(state.machine_free_time))

    return state.total_profit


# endregion


# region
"""==============================用于算法测试的主执行块====================================="""


if __name__ == '__main__':

    rep_id = 'test'

    """========================Parameters for Tunning========================"""
    # # Tunable parameter
    W_SAMPLING_THRESHOLD = 5
    exploration_factor = 10

    # Trade-off parameters
    sim_budget = 3000  # 仿真预算
    u = 1  # 一个 time slot 的时长 单位：小时

    """========================Parameters of Problem Setting========================"""

    # Common machine and cost parameters
    # 定义机器参数
    V_1 = 15  # 体积相关的机器参数 单位：mm3/秒 激光粉末床熔融（SLM/L-PBF）的典型构建速率通常在 5–20 mm³/s 范围内
    V_2 = 30  # 支撑结构体积相关的机器参数 单位：mm3/秒 一般在 10–25 mm³/s 范围内
    t_1 = 200  # 高度相关的机器参数 单位: 秒/毫米 通常在 5–15 秒/层，取决于设备类型和层厚 ≈ 100–300 秒/mm
    T = 3600  # 批处理的固定时间 单位：秒 预处理（模型切片、设备预热）和后处理（取件、清理粉末、热处理等）通常需要 1–3 小时
    machine_length = 200  # 平台长 单位: 毫米 标准工业级SLM设备的构建尺寸通常在 200×200×300 mm 至 500×500×500 mm 之间
    machine_width = 200  # 平台宽 单位: 毫米
    machine_height = 200
    # 定义成本系数
    K_energy = 8  # SLM机器能耗（电费计），单位：元/小时 SLM设备功率通常在 2–10 kW，按工业电价 0.8–1.2 元/kWh 计算，每小时电费约 1.6–12 元
    K_powder = 0.001  # 金属粉末成本，单位：元/mm3， 1mm³ AlSi10Mg粉末的价格大约为 0.00099元（即约 0.001元）。
    K_gas = 3.6  # 惰性气体成本，单位：元/小时 氩气或氮气消耗量约在几十升每分钟（L/min）左右，氮气的市场价格大约在 0.2-0.6 元/立方米之间，而氩气的价格可能稍高一些，约 2-5 元/立方米
    K_fine_max = 100  # normalizing constant of the tardiness penalty coefficient (a experiential maximum penalty rate)
    # 初始信息
    initial_point = 0  # 获取初始时点，仿真时点初始化：对应时间区间 (k, k+1]
    initial_machine_free_time = 0  # 初始机器可用时间 单位：小时

    # Changeable parameters to construct test environments
    # K # entire time horizon
    # loose_factor
    # K_fine  # 零件延迟罚款系数 单位：元/（小时*件）延迟成本
    # C_operator  # 单位：元 每批次固定成本（设备折旧、人工、软件等）
    # random_seed

    arrival_scenarios = {
        'Uniform': np.array([[0.04, 0.04, 0.04], [0.04, 0.04, 0.04]]),
        'Large-dominant': np.array([[0.02, 0.04, 0.06], [0.02, 0.04, 0.06]]),
        'Small-dominant': np.array([[0.06, 0.04, 0.02], [0.06, 0.04, 0.02]])
    }

    """========================Instances========================"""

    # # 参数取值
    T_horizon_values = [144]#[36, 72, 144]
    arrival_rate_matrix_values = ['Uniform']#'Uniform', 'Large-dominant','Small-dominant'
    xi_values = [3]
    C_opr_values = [300]
    K_fine_values = [30]
    seed_values = [63]#21, 42, 84

    # 使用itertools.product生成全因素设计的组合
    instances = [
        {
            'T_horizon': T_horizon,
            'arrival_rate_matrix': arrival_rate_matrix,
            'xi': xi,
            'C_opr': C_opr,
            'K_fine': K_fine,
            'seed': seed
        }
        for T_horizon, arrival_rate_matrix, xi, C_opr, K_fine, seed in product(
            T_horizon_values,
            arrival_rate_matrix_values,
            xi_values,
            C_opr_values,
            K_fine_values,
            seed_values
        )
    ]

    """========================Tests========================"""
    # 存储所有实验结果的列表
    results = []

    for instance_def in instances:

        K = instance_def['T_horizon']
        arrival_type = instance_def['arrival_rate_matrix']
        arrival_rate_matrix = arrival_scenarios[arrival_type]
        loose_factor = instance_def['xi']
        C_operator = instance_def['C_opr']
        K_fine = instance_def['K_fine']
        random_seed = instance_def['seed']

        np.random.seed(random_seed)

        NUM_INITIAL_PARTS = np.random.choice([1, 2])  # 初始零件数量
        part_stream = PartArrivalStream(  # 创建零件到达流实例
            T_horizon=K,
            num_initial_parts=NUM_INITIAL_PARTS,
            seed=random_seed
        )
        print(f"在 {K} 小时内，总共生成了 {len(part_stream.all_parts)} 个零件 (包括 {NUM_INITIAL_PARTS} 个初始零件)。\n")
        print("\n" + "=" * 50 + "\n")

        with open(f"Instance_H{K}-R{arrival_type}-S{random_seed}-L{loose_factor}-C{C_operator}-K{K_fine}.txt", "w") as file1:

            file1.write(f"Within {K} hours, totally generated {len(part_stream.all_parts)} parts (including {NUM_INITIAL_PARTS} initial parts)." + ",\n")

            # 查询到达的零件
            window_start, window_end = 0, K
            parts_in_window = part_stream.get_arrivals_in_window(window_start, window_end)

            for part in parts_in_window:
                print(
                    f"  ID: {part['ID']}, 类型: {part['type']}, 到达时间: {part['r_p']:.2f}h, 长: {part['length']:.2f}mm, 宽: {part['width']:.2f}mm, 高: {part['height']:.2f}mm, 体积: {part['volume']:.2f}mm3, 支撑: {part['support_volume']:.2f}mm3, 交付期限: {part['delivery_time']:.2f}h, 价格: {part['price']:.2f}CNY, ")

                file1.write(
                    f"  ID: {part['ID']}, Type: {part['type']}, Arrival time: {part['r_p']:.2f}h, Length: {part['length']:.2f}mm, Width: {part['width']:.2f}mm, Height: {part['height']:.2f}mm, Volume: {part['volume']:.2f}mm3, Support: {part['support_volume']:.2f}mm3, Due: {part['delivery_time']:.2f}h, Price: {part['price']:.2f}CNY, " + "\n")

            total_price = 0
            for part in parts_in_window:
                total_price += part['price']
            file1.write(f"Total price:{total_price:.2f}CNY")

        # # 2. 根据输入文件名，自动生成日志文件名
        log_filename = f'Log_H{K}-R{arrival_type}-L{loose_factor}-C{C_operator}-K{K_fine}-S{random_seed}_rep{rep_id}.txt'

        # 3. 保存原始的标准输出
        original_stdout = sys.stdout

        # 4. 打开 (或创建) 日志文件，'w' 模式会覆盖已有内容，使用 encoding='utf-8' 来支持中文
        with open(log_filename, 'w', encoding='utf-8') as log_file:

            sys.stdout = log_file  # 将标准输出重定向到日志文件

            # 1. 运行while_available策略的结果
            profit_while_available = simulation_of_dynamics_system_process_while_available()
            results.append({
                'T_horizon': K,
                'arrival_type': arrival_type,
                'loose_factor': loose_factor,
                'C_operator': C_operator,
                'K_fine': K_fine,
                'random_seed': random_seed,
                'rep_id': rep_id,
                'strategy': 'while_available',
                'buffer_length': None,  # 该策略无缓冲区
                'area_threshold': None,
                'profit': profit_while_available
            })

            # 2. 带等待缓冲区的实验（多个buffer_length）
            for buffer_length in {3, 6, 9, 12}:
                profit_waiting_buffer = simulation_of_dynamics_system_process_with_waiting_buffer(buffer_length)
                # 将当前buffer_length的结果添加到列表
                results.append({
                    'T_horizon': K,
                    'arrival_type': arrival_type,
                    'loose_factor': loose_factor,
                    'C_operator': C_operator,
                    'K_fine': K_fine,
                    'random_seed': random_seed,
                    'rep_id': rep_id,
                    'strategy': 'waiting_buffer',
                    'buffer_length': buffer_length,
                    'area_threshold': None,
                    'profit': profit_waiting_buffer
                })

            # 3. 满容量策略的实验
            for eta in {0.2, 0.4, 0.6, 0.8}:  #
                profit_full_capacity = simulation_of_dynamics_system_process_while_full_capacity(eta)
                results.append({
                    'T_horizon': K,
                    'arrival_type': arrival_type,
                    'loose_factor': loose_factor,
                    'C_operator': C_operator,
                    'K_fine': K_fine,
                    'random_seed': random_seed,
                    'rep_id': rep_id,
                    'strategy': 'full_capacity',
                    'buffer_length': None,  # 该策略无缓冲区
                    'area_threshold': eta,
                    'profit': profit_full_capacity
                })

            profit_pareto_optimal = simulation_of_dynamics_system()

            # profit_expand_entire_decisions = simulation_of_dynamics_system_expand_entire_decisions()

            # 恢复标准输出
            sys.stdout = original_stdout

    # 所有实验结束后，将结果写入CSV
    output_csv = 'results.csv'
    # 转换为DataFrame并写入CSV（index=False避免添加行索引）
    pd.DataFrame(results).to_csv(output_csv, index=False, encoding='utf-8-sig')
    print(f"所有实验结果已保存到 {output_csv}")

# endregion