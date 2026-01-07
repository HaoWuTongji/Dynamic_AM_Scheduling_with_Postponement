"""
25-10-07 吴昊 (Hao Wu) Last Modified
"""

import gurobipy as gp
from gurobipy import GRB
import itertools
from BinPackingHeu import BestFitRotate


def heuristic_check_packing_feasibility(parts, container_L, container_W, container_H):

    # selected_parts = []
    bin_sizes = []
    # total_length_width = 0
    # 当前已选择零件的长度宽度乘积
    # remaining_parts = parts[:]  # 复制零件列表，以便在尝试过程中逐个移除零件
    #
    # while remaining_parts:
    #     part = remaining_parts.pop(0)  # 从未选择的零件中取出第一个零件

    for part in parts:

        # 判断添加当前零件后是否超过机器的面积约束
        # if total_length_width + part['length'] * part['width'] <= container_L * container_W:
        # selected_parts.append(part)
        # total_length_width += part['length'] * part['width']
        bin_sizes.append(part['length'])
        bin_sizes.append(part['width'])

    # 调用BestFitRotate装箱
    item_coor_list = BestFitRotate(bin_sizes, container_L)

    for i in range(int(len(item_coor_list) / 4)):
        # 获取第i个零件的坐标
        x1, y1, x2, y2 = item_coor_list[i * 4: (i + 1) * 4]

        # 判断装箱是否可行
        if y2 > container_W:
            # print(y2)
            return "infeasible"

    return "feasible"


# def heuristic_find_all_feasible_combinations(parts_data, container_L, container_W, container_H, max_combinations):
#     """
#     遍历所有零件组合，找到所有可行的装箱方案。
#     25-07-08 修改：输出至多 max_combinations 个可行组合。
#
#     Args:
#         parts_data (list): 包含所有待选零件的列表。
#         container_L, container_W, container_H (float): 容器尺寸。
#         25-07-08 修改：max_combinations: 输出的可行组合上限
#
#     Returns:
#         list: 一个包含所有可行组合的列表。
#     """
#     num_total_parts = len(parts_data)
#     # 使用零件的ID作为唯一标识符
#     part_ids = [p['ID'] for p in parts_data]
#     # print(part_ids)
#     parts_map = {p['ID']: p for p in parts_data}
#
#     avail_feasible_combinations = []
#
#     proven_feasible_sets = []
#
#     # print("开始从最大组合到最小组合遍历，寻找所有可行的零件组合...")
#
#     n = 0
#     # 添加一个标志位，用于彻底跳出嵌套循环
#     found_enough = False
#
#     # 从最大组合 (n个零件) 迭代到最小组合 (1个零件)
#     for k in range(num_total_parts, 0, -1):
#         # 生成所有大小为 k 的组合
#         for combo_ids_tuple in itertools.combinations(part_ids, k):
#             # 检查此组合是否已经是某个更大可行组合的子集
#             if combo_ids_tuple in proven_feasible_sets:
#                 avail_feasible_combinations.append(combo_ids_tuple)
#                 n += 1
#                 # 检查是否已达到上限
#                 if n >= max_combinations:
#                     found_enough = True
#                     break  # 跳出内层循环 (for combo_ids_tuple)
#                 continue
#
#             # 从ID映射回完整的零件数据
#             current_combo_parts = [parts_map[pid] for pid in combo_ids_tuple]
#
#             # 调用BestFitRotate检查这个特定组合的可行性
#             is_feasible = heuristic_check_packing_feasibility(
#                 current_combo_parts, container_L, container_W, container_H
#             )
#
#             if is_feasible:
#                 avail_feasible_combinations.append(combo_ids_tuple)
#                 n += 1
#
#                 # 将此可行组合的所有子集都加入“已证明可行”集合
#                 for i in range(1, k + 1):
#                     for subset_tuple in itertools.combinations(combo_ids_tuple, i):
#                         proven_feasible_sets.append(subset_tuple)
#
#                 # 再次检查是否已达到上限
#                 if n >= max_combinations:
#                     found_enough = True
#                     break  # 跳出内层循环 (for combo_ids_tuple)
#
#         # 在内层循环结束后（无论是正常结束还是被break），检查标志位
#         if found_enough:
#             break  # 如果标志位为True，跳出外层循环 (for k)
#
#     # n = 0
#     #
#     # # 从最大组合 (n个零件) 迭代到最小组合 (1个零件)
#     # for k in range(num_total_parts, 0, -1):
#     #     # print(f"\n--- 正在检查包含 {k} 个零件的组合 ---")
#     #
#     #     if n >= max_combinations:
#     #         break
#     #
#     #     # 生成所有大小为 k 的组合
#     #     for combo_ids_tuple in itertools.combinations(part_ids, k):
#     #         # 检查此组合是否已经是某个更大可行组合的子集
#     #         if combo_ids_tuple in proven_feasible_sets:
#     #             # print(f"  [ ✓ ] 组合 {list(combo_ids_tuple)} 可行。该组合已被证明是某个更大可行组合的子集。")
#     #             avail_feasible_combinations.append(combo_ids_tuple)
#     #             n += 1
#     #             continue
#     #
#     #         # 从ID映射回完整的零件数据
#     #         current_combo_parts = [parts_map[pid] for pid in combo_ids_tuple]
#     #
#     #         # 调用BestFitRotate检查这个特定组合的可行性
#     #         is_feasible = heuristic_check_packing_feasibility(
#     #             current_combo_parts, container_L, container_W, container_H
#     #         )
#     #
#     #         if is_feasible:
#     #             # print(f"  [ ✓ ] 组合 {list(combo_ids_tuple)} 可行。")
#     #             avail_feasible_combinations.append(combo_ids_tuple)
#     #             n += 1
#     #
#     #             # 将此可行组合的所有子集都加入“已证明可行”集合
#     #             for i in range(1, k + 1):
#     #                 for subset_tuple in itertools.combinations(combo_ids_tuple, i):
#     #                     proven_feasible_sets.append(subset_tuple)
#
#     # print(avail_feasible_combinations)
#
#     return avail_feasible_combinations


def check_packing_feasibility(parts_to_check, container_L, container_W, container_H):
    """
    一个辅助函数，使用 Gurobi 检查一个【特定的】零件组合是否可以被装箱。

    Args:
        parts_to_check (list): 一个包含特定零件组合的字典列表。
        container_L (float): 容器的长度 (X 轴)。
        container_W (float): 容器的宽度 (Y 轴)。
        container_H (float): 容器的高度 (Z 轴)。

    Returns:
        tuple: 一个元组 (is_feasible, placement_details)。
               - is_feasible (bool): 如果组合可行，则为 True。
               - placement_details (list): 如果可行，包含零件的布局信息；否则为 None。
    """
    num_parts = len(parts_to_check)
    if num_parts == 0:
        return True, []

    # 1. 数据预处理
    p_indices = range(num_parts)
    p_details = {i: data for i, data in enumerate(parts_to_check)}
    p_l = {i: data['length'] for i, data in enumerate(parts_to_check)}
    p_w = {i: data['width'] for i, data in enumerate(parts_to_check)}
    p_h = {i: data['height'] for i, data in enumerate(parts_to_check)}

    try:
        m = gp.Model("FeasibilityCheck")
        m.setParam(GRB.Param.LogToConsole, 0)  # 关闭冗余日志

        # “大M”常数
        M = container_L + container_W + 1

        # 2. 定义决策变量 (注意：这里不再需要 X_p 变量)
        x = m.addVars(p_indices, vtype=GRB.CONTINUOUS, lb=0, name="x")
        y = m.addVars(p_indices, vtype=GRB.CONTINUOUS, lb=0, name="y")
        o = m.addVars(p_indices, vtype=GRB.BINARY, name="o")
        PL = m.addVars(p_indices, p_indices, vtype=GRB.BINARY, name="PL")
        PB = m.addVars(p_indices, p_indices, vtype=GRB.BINARY, name="PB")

        # 3. 添加约束 (这些约束现在是无条件的，因为我们假设所有零件都必须放入)
        for p in p_indices:
            # 高度约束
            m.addConstr(p_h[p] <= container_H, name=f"height_{p}")
            # 边界约束
            m.addConstr(x[p] + p_l[p] <= container_L + M * o[p], name=f"bound_L1_{p}")
            m.addConstr(x[p] + p_w[p] <= container_L + M * (1 - o[p]), name=f"bound_L2_{p}")
            m.addConstr(y[p] + p_w[p] <= container_W + M * o[p], name=f"bound_W1_{p}")
            m.addConstr(y[p] + p_l[p] <= container_W + M * (1 - o[p]), name=f"bound_W2_{p}")

        # 防重叠约束
        for p in p_indices:
            for p_prime in p_indices:
                if p >= p_prime:
                    continue
                # 标准的防重叠公式
                m.addConstr(x[p] + p_l[p] <= x[p_prime] + M * (1 - PL[p, p_prime]) + M * o[p],
                            name=f"left_of_1_{p}_{p_prime}")
                m.addConstr(x[p] + p_w[p] <= x[p_prime] + M * (1 - PL[p, p_prime]) + M * (1 - o[p]),
                            name=f"left_of_2_{p}_{p_prime}")
                m.addConstr(x[p_prime] + p_l[p_prime] <= x[p] + M * (1 - PL[p_prime, p]) + M * o[p_prime],
                            name=f"left_of_1_{p_prime}_{p}")
                m.addConstr(x[p_prime] + p_w[p_prime] <= x[p] + M * (1 - PL[p_prime, p]) + M * (1 - o[p_prime]),
                            name=f"left_of_2_{p_prime}_{p}")
                m.addConstr(y[p] + p_w[p] <= y[p_prime] + M * (1 - PB[p, p_prime]) + M * o[p],
                            name=f"below_1_{p}_{p_prime}")
                m.addConstr(y[p] + p_l[p] <= y[p_prime] + M * (1 - PB[p, p_prime]) + M * (1 - o[p]),
                            name=f"below_2_{p}_{p_prime}")
                m.addConstr(y[p_prime] + p_w[p_prime] <= y[p] + M * (1 - PB[p_prime, p]) + M * o[p_prime],
                            name=f"below_1_{p_prime}_{p}")
                m.addConstr(y[p_prime] + p_l[p_prime] <= y[p] + M * (1 - PB[p_prime, p]) + M * (1 - o[p_prime]),
                            name=f"below_2_{p_prime}_{p}")
                # 保证位置关系
                m.addConstr(PL[p, p_prime] + PL[p_prime, p] + PB[p, p_prime] + PB[p_prime, p] >= 1,
                            name=f"disjunction_{p}_{p_prime}")

        # 4. 求解
        m.setObjective(0, GRB.MINIMIZE)  # 我们只关心可行性，不设目标
        m.setParam(GRB.Param.TimeLimit, 30)
        m.optimize()

        # 5. 返回结果
        if m.Status == GRB.OPTIMAL:
            placement_details = []
            total_area = 0
            for p in p_indices:
                is_rotated = (o[p].X > 0.5)
                eff_l = p_w[p] if is_rotated else p_l[p]
                eff_w = p_l[p] if is_rotated else p_w[p]
                placement_details.append({
                    'ID': p_details[p]['ID'],
                    'x': round(x[p].X, 2),
                    'y': round(y[p].X, 2),
                    'rotated': is_rotated
                })
                total_area += eff_l * eff_w

            stats = {
                'part_count': num_parts,
                'area_used_percentage': round(100 * total_area / (container_L * container_W), 2)
            }
            return True, {'placement': placement_details, 'stats': stats}
        else:
            return False, None

    except gp.GurobiError as e:
        print(f"Gurobi 发生错误，错误码 {e.errno}: {e}")
        return False, None
    except Exception as e:
        print(f"发生未知错误: {e}")
        return False, None


def find_all_feasible_combinations(parts_data, container_L, container_W, container_H, time_limit_per_check=10):
    """
    遍历所有零件组合，找到所有可行的装箱方案。

    Args:
        parts_data (list): 包含所有待选零件的列表。
        container_L, container_W, container_H (float): 容器尺寸。
        time_limit_per_check (int): 对每个组合进行可行性检查的时间限制（秒）。

    Returns:
        list: 一个包含所有可行组合的列表。
    """
    num_total_parts = len(parts_data)
    # 使用零件的ID作为唯一标识符
    # print(parts_data)
    part_ids = [p['ID'] for p in parts_data]
    # print(part_ids)
    parts_map = {p['ID']: p for p in parts_data}

    all_feasible_combinations = []
    # 这个集合用于存储已被证明可行的组合（的ID元组），以避免重复计算
    # proven_feasible_sets = set()
    proven_feasible_sets = []

    # print("开始从最大组合到最小组合遍历，寻找所有可行的零件组合...")
    # 从最大组合 (n个零件) 迭代到最小组合 (1个零件)
    for k in range(num_total_parts, 0, -1):
        # print(f"\n--- 正在检查包含 {k} 个零件的组合 ---")
        num_checked_this_size = 0

        # 生成所有大小为 k 的组合
        for combo_ids_tuple in itertools.combinations(part_ids, k):
            # 检查此组合是否已经是某个更大可行组合的子集
            if combo_ids_tuple in proven_feasible_sets:
                # print(f"  [ ✓ ] 组合 {list(combo_ids_tuple)} 可行。该组合已被证明是某个更大可行组合的子集。")
                continue

            num_checked_this_size += 1
            # 从ID映射回完整的零件数据
            current_combo_parts = [parts_map[pid] for pid in combo_ids_tuple]

            # 调用Gurobi检查这个特定组合的可行性
            is_feasible, details = check_packing_feasibility(
            # is_feasible = check_packing_feasibility(
                current_combo_parts, container_L, container_W, container_H
            )

            if is_feasible:
                # print(f"  [ ✓ ] 组合 {list(combo_ids_tuple)} 可行。")
                # proven_feasible_sets.append(combo_ids_tuple)
                # all_feasible_combinations.append({
                #     'combination': list(combo_ids_tuple),
                #     'details': details
                # })

                # 优化：将此可行组合的所有子集都加入“已证明可行”集合
                for i in range(1, k + 1):
                    for subset_tuple in itertools.combinations(combo_ids_tuple, i):
                        proven_feasible_sets.append(subset_tuple)
                        # proven_feasible_sets.add(subset_tuple)
            # 如果不可行，则不打印信息，以保持输出整洁
            # else:
            #     print(f"  [ X ] 组合 {list(combo_ids_tuple)} 不可行。")

        # if num_checked_this_size == 0:
        #     print("该尺寸的所有组合都已被证明是某个更大可行组合的子集，已跳过。")

    # print(proven_feasible_sets)

    return proven_feasible_sets

    # return all_feasible_combinations


# def find_all_feasible_combinations_heuristic(parts_data, container_L, container_W, container_H):
#     """
#     遍历所有零件组合，找到所有可行的装箱方案。
#
#     Args:
#         parts_data (list): 包含所有待选零件的列表。
#         container_L, container_W, container_H (float): 容器尺寸。
#
#     Returns:
#         list: 一个包含所有可行组合的列表。
#     """
#     num_total_parts = len(parts_data)
#     # 使用零件的ID作为唯一标识符
#     # print(parts_data)
#     part_ids = [p['ID'] for p in parts_data]
#     # print(part_ids)
#     parts_map = {p['ID']: p for p in parts_data}
#
#     all_feasible_combinations = []
#     # 这个集合用于存储已被证明可行的组合（的ID元组），以避免重复计算
#     # proven_feasible_sets = set()
#     proven_feasible_sets = []
#
#     # print("开始从最大组合到最小组合遍历，寻找所有可行的零件组合...")
#     # 从最大组合 (n个零件) 迭代到最小组合 (1个零件)
#     for k in range(num_total_parts, 0, -1):
#         # print(f"\n--- 正在检查包含 {k} 个零件的组合 ---")
#         # num_checked_this_size = 0
#
#         # 生成所有大小为 k 的组合
#         for combo_ids_tuple in itertools.combinations(part_ids, k):
#             # 检查此组合是否已经是某个更大可行组合的子集
#             if combo_ids_tuple in proven_feasible_sets:
#                 # print(f"  [ ✓ ] 组合 {list(combo_ids_tuple)} 可行。该组合已被证明是某个更大可行组合的子集。")
#                 continue
#
#             # num_checked_this_size += 1
#             # 从ID映射回完整的零件数据
#             current_combo_parts = [parts_map[pid] for pid in combo_ids_tuple]
#
#             # # 调用Gurobi检查这个特定组合的可行性
#             # is_feasible, details = check_packing_feasibility(
#             # # is_feasible = check_packing_feasibility(
#             #     current_combo_parts, container_L, container_W, container_H
#             # )
#             # 调用BestFitRotate检查这个特定组合的可行性
#             is_feasible = heuristic_check_packing_feasibility(
#                 current_combo_parts, container_L, container_W, container_H
#             )
#
#             if is_feasible == "feasible":
#                 # print(f"  [ ✓ ] 组合 {list(combo_ids_tuple)} 可行。")
#                 # proven_feasible_sets.append(combo_ids_tuple)
#                 # all_feasible_combinations.append({
#                 #     'combination': list(combo_ids_tuple),
#                 #     'details': details
#                 # })
#
#                 # 优化：将此可行组合的所有子集都加入“已证明可行”集合
#                 for i in range(1, k + 1):
#                     for subset_tuple in itertools.combinations(combo_ids_tuple, i):
#                         proven_feasible_sets.append(subset_tuple)
#                         # proven_feasible_sets.add(subset_tuple)
#             # 如果不可行，则不打印信息，以保持输出整洁
#             # else:
#             #     print(f"  [ X ] 组合 {list(combo_ids_tuple)} 不可行。")
#
#         # if num_checked_this_size == 0:
#         #     print("该尺寸的所有组合都已被证明是某个更大可行组合的子集，已跳过。")
#
#     # print(proven_feasible_sets)
#
#     return proven_feasible_sets
#
#     # return all_feasible_combinations


def find_all_feasible_combinations_heuristic(parts_data, container_L, container_W, container_H):
    """
    遍历所有零件组合，找到所有可行的装箱方案。

    Args:
        parts_data (list): 包含所有待选零件的列表。
        container_L, container_W, container_H (float): 容器尺寸。

    Returns:
        list: 一个包含所有可行组合的列表。
    """
    num_total_parts = len(parts_data)
    # 使用零件的ID作为唯一标识符
    part_ids = [p['ID'] for p in parts_data]
    parts_map = {p['ID']: p for p in parts_data}

    # 这个集合用于存储已被证明可行的组合（的ID元组），以避免重复计算
    proven_feasible_sets = set()
    # proven_feasible_sets = []

    # 从最大组合 (n个零件) 迭代到最小组合 (1个零件)
    for k in range(num_total_parts, 0, -1):
        # print(f"\n--- 正在检查包含 {k} 个零件的组合 ---")

        # 生成所有大小为 k 的组合
        for combo_ids_tuple in itertools.combinations(part_ids, k):
            # 检查此组合是否已经是某个更大可行组合的子集
            if combo_ids_tuple in proven_feasible_sets:
                # print(f"  [ ✓ ] 组合 {list(combo_ids_tuple)} 可行。该组合已被证明是某个更大可行组合的子集。")
                continue

            # 从ID映射回完整的零件数据
            current_combo_parts = [parts_map[pid] for pid in combo_ids_tuple]

            # 调用BestFitRotate检查这个特定组合的可行性
            is_feasible = heuristic_check_packing_feasibility(
                current_combo_parts, container_L, container_W, container_H
            )

            if is_feasible == "feasible":

                # 优化：将此可行组合的所有子集都加入“已证明可行”集合
                for i in range(1, k + 1):
                    for subset_tuple in itertools.combinations(combo_ids_tuple, i):
                        proven_feasible_sets.add(subset_tuple)
                        # proven_feasible_sets.append(subset_tuple)

    return list(proven_feasible_sets)


# # --- 示例用法 ---
# if __name__ == '__main__':
#     CONTAINER_LENGTH = 8
#     CONTAINER_WIDTH = 8
#     CONTAINER_HEIGHT = 10
#
#     parts_to_pack = [
#         {'ID': 'P1_1', 'length': 6, 'width': 6, 'height': 5},
#         {'ID': 'P1_2', 'length': 5, 'width': 4, 'height': 5},
#         {'ID': 'P1_3', 'length': 3, 'width': 3, 'height': 12},  # 太高
#         {'ID': 'P1_4', 'length': 4, 'width': 4, 'height': 8},
#         {'ID': 'P1_5', 'length': 2, 'width': 5, 'height': 9},
#     ]
#
#     # 过滤掉本身就无法放入的零件
#     initial_parts_filtered = [
#         p for p in parts_to_pack
#         if (p['height'] <= CONTAINER_HEIGHT) and
#            ((p['length'] <= CONTAINER_LENGTH and p['width'] <= CONTAINER_WIDTH) or
#            (p['width'] <= CONTAINER_LENGTH and p['length'] <= CONTAINER_WIDTH))
#     ]
#
#     print(f"容器尺寸: L={CONTAINER_LENGTH}, W={CONTAINER_WIDTH}, H={CONTAINER_HEIGHT}")
#     print("待选零件 (已过滤掉单个就无法放入的零件):")
#     for part in initial_parts_filtered:
#         print(f"  - ID: {part['ID']}, 尺寸: {part['length']}x{part['width']}x{part['height']}")
#     print("-" * 40)
#
#     feasible_solutions = find_all_feasible_combinations(
#         parts_data=initial_parts_filtered,
#         container_L=CONTAINER_LENGTH,
#         container_W=CONTAINER_WIDTH,
#         container_H=CONTAINER_HEIGHT
#     )
#
#     if feasible_solutions:
#         print(f"\n\n======= 最终结果：共找到 {len(feasible_solutions)} 个独立的可行组合 =======")
#         # # 按零件数量降序排序
#         # feasible_solutions.sort(key=lambda s: s['details']['stats']['part_count'], reverse=True)
#         # for i, solution in enumerate(feasible_solutions):
#         #     print(f"\n--- 可行组合 {i + 1} ---")
#         #     print(f"  零件列表: {solution['combination']}")
#         #     print(f"  零件数量: {solution['details']['stats']['part_count']}")
#         #     print(f"  面积利用率: {solution['details']['stats']['area_used_percentage']}%")
#         #     print("  布局详情:")
#         #     for detail in solution['details']['placement']:
#         #         print(f"    - ID: {detail['ID']}, 位置: ({detail['x']}, {detail['y']}), "
#         #               f"旋转: {'是' if detail['rotated'] else '否'}")
#     else:
#         print("\n\n======= 最终结果：没有找到任何可行的零件组合。 =======")
