import gurobipy as gp
from gurobipy import GRB
import re
import sys


def solve_deterministic_model(parts_data, params):
    """
    使用 Gurobi 求解确定性的批量调度和装箱问题的 MILP 模型。

    Args:
        parts_data (list): 一个字典列表，每个字典代表一个零件，包含其所有属性。
        params (dict): 包含所有模型参数的字典 (如机器尺寸、成本系数等)。

    Returns:
        None: 直接打印求解结果。
    """
    # 从参数字典中解包
    K = params['K']
    P_set = [p['id'] for p in parts_data]
    K_set = range(K)

    # 创建一个 part_map 以便通过 ID 快速访问零件数据
    part_map = {p['id']: p for p in parts_data}

    # 大M值，应足够大以确保逻辑约束有效
    M = 10000

    try:
        # 1. 创建模型
        m = gp.Model("Deterministic_AM_Scheduling")

        # --- (Gurobi 会自动将日志打印到当前的 sys.stdout) ---

        # 2. 定义决策变量
        # 核心决策变量
        X = m.addVars(K_set, P_set, vtype=GRB.BINARY, name="X")
        Y = m.addVars(K_set, vtype=GRB.BINARY, name="Y")

        # 状态和辅助变量
        tau = m.addVars(K + 1, vtype=GRB.CONTINUOUS, name="tau")
        c = m.addVars(P_set, vtype=GRB.CONTINUOUS, name="c")
        T = m.addVars(P_set, vtype=GRB.CONTINUOUS, name="T")
        t_bld = m.addVars(K_set, vtype=GRB.CONTINUOUS, name="t_bld")
        H = m.addVars(K_set, vtype=GRB.CONTINUOUS, name="H")
        # --- 新增: 用于 tardiness 线性化的辅助二元变量 ---
        b_tard = m.addVars(P_set, vtype=GRB.BINARY, name="b_tardiness")

        # 装箱相关变量
        x = m.addVars(K_set, P_set, vtype=GRB.CONTINUOUS, name="x")
        y = m.addVars(K_set, P_set, vtype=GRB.CONTINUOUS, name="y")
        o = m.addVars(K_set, P_set, vtype=GRB.BINARY, name="o")

        # --- FIX: 使用扁平化的索引 (k, p1, p2) 来定义 PL/PB 变量 ---
        P_all_pairs = [(p1, p2) for p1 in P_set for p2 in P_set if p1 != p2]
        pl_pb_indices = [(k, p1, p2) for k in K_set for p1, p2 in P_all_pairs]
        PL = m.addVars(pl_pb_indices, vtype=GRB.BINARY, name="PL")
        PB = m.addVars(pl_pb_indices, vtype=GRB.BINARY, name="PB")

        # 3. 设置目标函数
        total_revenue = gp.quicksum(part_map[p]['varrho_p'] * X[k, p] for k in K_set for p in P_set)

        production_cost = gp.quicksum(
            Y[k] * params['C_opr'] +
            (params['K_eng'] + params['K_gas']) * t_bld[k] +
            params['K_pow'] * gp.quicksum((part_map[p]['v_p'] + part_map[p]['s_p']) * X[k, p] for p in P_set)
            for k in K_set
        )

        tardiness_cost = gp.quicksum(params['K_fine'] * T[p] for p in P_set)

        m.setObjective(total_revenue - production_cost - tardiness_cost
                       - 0.0001 * sum(tau[k] for k in K_set), GRB.MAXIMIZE)
        # 实验发现，要得到与现实完全一致的信息，需要对sum(tau[k] for k in K_set)加目标惩罚

        # 4. 添加约束
        # Part Assignment and Availability
        for p in P_set:
            m.addConstr(gp.quicksum(X[k, p] for k in K_set) <= 1, name=f"part_once_{p}")
            for k in K_set:
                m.addConstr(k * params['u'] >= part_map[p]['r_p'] - M * (1 - X[k, p]), name=f"arrival_{k}_{p}")

        # Batch Processing Logic & Machine Time Transition
        m.addConstr(tau[0] == 0, name="tau_initial")
        for k in K_set:
            m.addConstr(Y[k] <= gp.quicksum(X[k, p] for p in P_set), name=f"process_implies_select_{k}")
            for p in P_set:
                m.addConstr(X[k, p] <= Y[k], name=f"select_implies_process_{k}_{p}")

            m.addConstr(tau[k] <= (k + 1) * params['u'] + M * (1 - Y[k]), name=f"machine_avail_{k}")

            if k < K - 1:  # 修正：原代码中 k 的循环范围有误，应为 k < K - 1
                # 约束 1: tau_{k+1} >= (k+1)u - M * Y[k]
                m.addConstr(tau[k + 1] >= (k + 1) * params['u'] - M * Y[k], name=f"tau_trans_postpone_{k}")

                # 约束 2: tau_{k+1} >= tau_k - M * Y[k]
                m.addConstr(tau[k + 1] >= tau[k] - M * Y[k], name=f"tau_trans_process_{k}")

                # 约束 3: tau_{k+1} >= tau_k + t_pre + t_bld[k] + t_post - M * (1 - Y[k])
                m.addConstr(tau[k + 1] >= tau[k] + params['t_pre'] + t_bld[k] + params['t_post'] - M * (1 - Y[k]),
                            name=f"tau_trans_process_full_{k}")

        # Batch Geometry and Build Time
        for k in K_set:
            m.addConstr(H[k] <= params['H_machine'] * Y[k], name=f"max_height_{k}")
            for p in P_set:
                m.addConstr(H[k] >= part_map[p]['h_p'] * X[k, p], name=f"batch_height_{k}_{p}")

            sum_v_term = gp.quicksum(
                (part_map[p]['v_p'] / params['V1'] + part_map[p]['s_p'] / params['V2']) * X[k, p] for p in P_set)
            m.addConstr((Y[k] == 1) >> (t_bld[k] == sum_v_term + params['t1'] * H[k]), name=f"build_time_calc_{k}")
            m.addConstr((Y[k] == 0) >> (t_bld[k] == 0), name=f"build_time_zero_{k}")

        # Part Completion and Tardiness
        is_processed = m.addVars(P_set, vtype=GRB.BINARY, name="is_processed")
        for p in P_set:
            m.addConstr(T[p] >= 0, name=f"tardiness_non_negative_{p}")
            m.addConstr(is_processed[p] == gp.quicksum(X[k, p] for k in K_set), name=f"is_processed_def_{p}")

            for k in K_set:
                m.addConstr(c[p] >= (tau[k] + params['t_pre'] + t_bld[k] + params['t_post']) - M * (1 - X[k, p]),
                            name=f"completion_time_{k}_{p}")

                # --- 替换为以下基于您正确逻辑的线性化约束 ---

                k_u_minus_d = K * params['u'] - part_map[p]['d_p']
                c_minus_d = c[p] - part_map[p]['d_p']

                # 约束 1: T[p] >= 0 (已在上面 'tardiness_non_negative' 处添加)

                # 约束 2: (is_processed[p] == 0) >> (T[p] >= K * u - d[p])
                # 使用 Big M: T[p] >= (K*u - d[p]) - M * is_processed[p]
                m.addConstr(T[p] >= k_u_minus_d - M * is_processed[p], name=f"tardiness_unprocessed_{p}")

                # 约束 3: (is_processed[p] == 1) >> (T[p] >= min(c[p] - d[p], K*u - d[p]))
                # 使用 Big M 和辅助变量 b_tard[p]
                m.addConstr(T[p] >= c_minus_d - M * (1 - b_tard[p]) - M * (1 - is_processed[p]),
                            name=f"tardiness_processed_min_c_{p}")

                m.addConstr(T[p] >= k_u_minus_d - M * b_tard[p] - M * (1 - is_processed[p]),
                            name=f"tardiness_processed_min_Ku_{p}")
                # --- 线性化结束 ---

        # Nesting Constraints
        P_pairs_ordered = [(p1, p2) for i, p1 in enumerate(P_set) for p2 in P_set[i + 1:]]

        for k in K_set:
            for p in P_set:
                m.addConstr(x[k, p] + part_map[p]['l_p'] * (1 - o[k, p]) + part_map[p]['w_p'] * o[k, p] <= params[
                    'L_machine'] + M * (1 - X[k, p]), name=f"bound_L_{k}_{p}")
                m.addConstr(y[k, p] + part_map[p]['w_p'] * (1 - o[k, p]) + part_map[p]['l_p'] * o[k, p] <= params[
                    'W_machine'] + M * (1 - X[k, p]), name=f"bound_W_{k}_{p}")

            for p1, p2 in P_pairs_ordered:
                m.addConstr(
                    x[k, p1] + part_map[p1]['l_p'] * (1 - o[k, p1]) + part_map[p1]['w_p'] * o[k, p1] <= x[k, p2] + M * (
                            3 - X[k, p1] - X[k, p2] - PL[k, p1, p2]), name=f"overlap_left_{k}_{p1}_{p2}")
                m.addConstr(
                    x[k, p2] + part_map[p2]['l_p'] * (1 - o[k, p2]) + part_map[p2]['w_p'] * o[k, p2] <= x[k, p1] + M * (
                            3 - X[k, p1] - X[k, p2] - PL[k, p2, p1]), name=f"overlap_right_{k}_{p1}_{p2}")
                m.addConstr(
                    y[k, p1] + part_map[p1]['w_p'] * (1 - o[k, p1]) + part_map[p1]['l_p'] * o[k, p1] <= y[k, p2] + M * (
                            3 - X[k, p1] - X[k, p2] - PB[k, p1, p2]), name=f"overlap_below_{k}_{p1}_{p2}")
                m.addConstr(
                    y[k, p2] + part_map[p2]['w_p'] * (1 - o[k, p2]) + part_map[p2]['l_p'] * o[k, p2] <= y[k, p1] + M * (
                            3 - X[k, p1] - X[k, p2] - PB[k, p2, p1]), name=f"overlap_above_{k}_{p1}_{p2}")

                m.addConstr(PL[k, p1, p2] + PL[k, p2, p1] + PB[k, p1, p2] + PB[k, p2, p1] >= X[k, p1] + X[k, p2] - 1,
                            name=f"disjunctive_{k}_{p1}_{p2}")

        # 5. 求解模型
        time_limit = 1800  # 1800
        m.setParam('TimeLimit', time_limit)  # 300
        m.optimize()

        # 6. 打印结果
        # (您的所有 print 语句都会被重定向到文件)
        if m.status == GRB.OPTIMAL or m.status == GRB.SUBOPTIMAL:
            print(f"\n--- 求解到最优解! 目标值 (总利润): {m.objVal + 0.0001 * sum(tau[k].X for k in K_set):.2f}CNY ---\n")

            free_time = 0
            parts_processed_so_far = set()  # <--- 新增: 跟踪已处理的零件

            for k in K_set:
                if Y[k].X > 0.5:
                    batch_parts = [p for p in P_set if X[k, p].X > 0.5]
                    print(f"** 决策点 k={k}: 处理批次 **")
                    print(f"  - 零件列表: {batch_parts}")
                    print(f"  - 批次高度 H_{k}: {H[k].X:.2f}")
                    print(f"  - 批次构建时间 t_bld_{k}: {t_bld[k].X:.2f} 小时")
                    print(f"  - 机器在 tau_{k}={tau[k].X:.2f} 时开始工作")
                    if k < K - 1:
                        print(f"  - 机器将在 tau_{k + 1}={tau[k + 1].X:.2f} 时空闲")

                    print("  - 零件布局:")
                    for p in batch_parts:
                        is_rotated = "是" if o[k, p].X > 0.5 else "否"
                        print(f"    - {p}: 坐标=({x[k, p].X:.1f}, {y[k, p].X:.1f}), 旋转={is_rotated}")
                    print("-" * 20)

                    free_time = tau[k + 1].X
                    parts_processed_so_far.update(batch_parts)  # <--- 新增: 更新已处理零件集合

                else:   # <--- Y[k].X <= 0.5, 未处理批次
                    if k > 0 and (k + 1) * params['u'] <= free_time:
                        print(f"** 决策点 k={k}: 生产中 **")
                    else:   # <--- 修改: 机器空闲，需要判断是“延迟”还是“没零件”
                        # 检查在当前时间 k*u，是否有已到达且未被处理的零件
                        has_available_parts = False
                        for p in P_set:
                            if p not in parts_processed_so_far and part_map[p]['r_p'] <= (k * params['u']):
                                has_available_parts = True
                        if has_available_parts:
                            print(f"** 决策点 k={k}: 延迟决策 **")
                        else:
                            print(f"** 决策点 k={k}: 机器空闲, 但没有零件可用 **")

            print("\n--- 零件最终状态 ---")
            for p in P_set:
                if is_processed[p].X > 0.5:
                    print(f"零件 {p}: 已生产, 完成时间={c[p].X:.2f}, 产生延迟={T[p].X:.2f}")
                else:
                    print(f"零件 {p}: 未生产, 产生延迟={T[p].X:.2f}")

        elif m.status == GRB.INFEASIBLE:
            print("模型不可行。请检查约束。")
            m.computeIIS()
            m.write("model.ilp")
        else:

            print(
                f"\n--- 时间上限({time_limit}s)到达，未求解到最优解。当前最优可行解的目标值 (总利润): {m.objVal:.2f}CNY ---\n")

            free_time = 0

            for k in K_set:
                if Y[k].X > 0.5:
                    batch_parts = [p for p in P_set if X[k, p].X > 0.5]
                    print(f"** 决策点 k={k}: 处理批次 **")
                    print(f"  - 零件列表: {batch_parts}")
                    print(f"  - 批次高度 H_{k}: {H[k].X:.2f}")
                    print(f"  - 批次构建时间 t_bld_{k}: {t_bld[k].X:.2f} 小时")
                    print(f"  - 机器在 tau_{k}={tau[k].X:.2f} 时开始工作")
                    if k < K - 1:
                        print(f"  - 机器将在 tau_{k + 1}={tau[k + 1].X:.2f} 时空闲")

                    print("  - 零件布局:")
                    for p in batch_parts:
                        is_rotated = "是" if o[k, p].X > 0.5 else "否"
                        print(f"    - {p}: 坐标=({x[k, p].X:.1f}, {y[k, p].X:.1f}), 旋转={is_rotated}")
                    print("-" * 20)

                    free_time = tau[k + 1].X


                else:
                    if k > 0 and (k + 1) * params['u'] <= free_time:
                        print(f"** 决策点 k={k}: 生产中 **")

                    else:
                        print(f"** 决策点 k={k}: 延迟决策 **")


            print("\n--- 零件最终状态 ---")
            for p in P_set:
                if is_processed[p].X > 0.5:
                    print(f"零件 {p}: 已生产, 完成时间={c[p].X:.2f}, 产生延迟={T[p].X:.2f}")
                else:
                    print(f"零件 {p}: 未生产, 产生延迟={T[p].X:.2f}")

    except gp.GurobiError as e:
        print(f"Gurobi 错误, 错误码 {e.errno}: {e}")
    except Exception as e:
        print(f"发生错误: {e}")


if __name__ == '__main__':
    # ==================================================================
    # 1. 定义文件名
    # ==================================================================
    filenames = [
        # put instances here
    ]

    for filename in filenames:

        # 2. 根据输入文件名，自动生成日志文件名
        log_filename = filename.replace('.txt', '_Gurobi_log.txt')
        if log_filename == filename:  # 避免重名
            log_filename = "model_run_log.txt"

        # 3. 保存原始的标准输出
        original_stdout = sys.stdout

        # 4. 使用 try...finally... 结构确保标准输出总是被恢复
        try:
            # 打开 (或创建) 日志文件，'w' 模式会覆盖已有内容
            # 使用 encoding='utf-8' 来支持中文
            with open(log_filename, 'w', encoding='utf-8') as log_file:

                sys.stdout = log_file  # 将标准输出重定向到日志文件

                print(f"--- 日志开始: 正在处理 {filename} ---")

                # ==================================================================
                # 5. 从文件名中提取参数
                # ==================================================================
                # 提取 K (来自 Hxx)
                k_match = re.search(r'H(\d+)', filename)
                extracted_K = int(k_match.group(1)) if k_match else 'nan'  # 默认值
                print(f"从文件名中提取 K = {extracted_K} (来自 'Hxx')")

                # 1. 定义参数
                params = {
                    # Changeable parameters to construct test environments
                    'K': extracted_K,
                    'C_opr': 300,
                    'K_fine': 30,

                    # Trade-off parameters
                    'u': 1,
                    # Common machine and cost parameters
                    'L_machine': 200,
                    'W_machine': 200,
                    'H_machine': 200,
                    'V1': 15 * 3600,
                    'V2': 30 * 3600,
                    't1': 200 / 3600,
                    't_pre': 0.5,
                    't_post': 0.5,
                    'K_eng': 8,
                    'K_gas': 3.6,
                    'K_pow': 0.001,
                }

                print("--- 模型参数 ---")
                print(params)
                print("---------------------\n")

                # 2. 定义零件数据
                # 读取文件内容
                try:
                    with open(filename, 'r', encoding='gbk') as file:
                        text = file.read()
                except FileNotFoundError:
                    print(f"错误: 输入文件 {filename} 未找到。")
                    # 抛出异常以停止执行，并允许 finally 块运行
                    raise
                except Exception as e:
                    print(f"读取文件时出错: {e}")
                    raise

                # 定义正则表达式来提取每个零件的相关信息

                pattern = re.compile(
                    r"ID: (\w+), Type: \('(\w+)', '(\w+)'\), Arrival time: (\d+\.\d+)h, Length: (\d+\.\d+)mm, Width: (\d+\.\d+)mm, Height: (\d+\.\d+)mm, Volume: (\d+\.\d+)mm3, Support: (\d+\.\d+)mm3, Due: (\d+\.\d+)h, Price: (\d+\.\d+)CNY")

                # 使用正则表达式找到所有匹配的零件数据
                matches = re.findall(pattern, text)

                # 将匹配到的数据转化为目标格式的字典
                parts = []
                for match in matches:
                    part = {
                        'id': match[0],
                        'l_p': float(match[4]),
                        'w_p': float(match[5]),
                        'h_p': float(match[6]),
                        'v_p': float(match[7]),
                        's_p': float(match[8]),
                        'r_p': float(match[3]),
                        'd_p': float(match[9]),
                        'varrho_p': float(match[10])
                    }
                    parts.append(part)

                # 打印结果 (这些 print 现在会进入日志文件)
                print("--- 解析的零件数据 ---")
                for part in parts:
                    print(part)
                print("------------------------\n")

                parts_data = parts

                # ==================================================================
                # 6. 调用模型求解
                # ==================================================================
                print("--- 开始 Gurobi 求解器 ---")
                solve_deterministic_model(parts_data, params)
                print("\n--- Gurobi 求解器结束 ---")
                print(f"--- 日志结束 ---")

        except Exception as e:
            # 如果在 try 块中（重定向前或重定向后）发生错误
            # 我们需要将其打印到原始控制台
            sys.stdout = original_stdout
            print(f"脚本执行时发生严重错误: {e}")
            # 也可以尝试追加到日志文件
            try:
                with open(log_filename, 'a', encoding='utf-8') as log_file:
                    log_file.write(f"\n\n--- 脚本执行时发生严重错误 ---\n{e}\n")
                    import traceback

                    traceback.print_exc(file=log_file)
            except:
                pass  # 如果日志文件写入也失败，则忽略

        finally:
            # 无论成功还是失败，都必须恢复标准输出
            sys.stdout = original_stdout

        # 7. 在原始控制台中打印一条最终消息，告知用户日志已生成
        print(f"脚本执行完成。所有输出已保存到: {log_filename}")