# Dynamic Single AM Machine Scheduling with Strategic Postponement

> **Strategic Postponement for Social Additive Manufacturing: A Lookahead Approximate Dynamic Programming Approach**

## 📖 Introduction

This repository contains the implementation of the simulation environment, decision-making algorithms, and experimental data for the research paper: **"Dynamic Single Additive Manufacturing Machine Scheduling Considering Postponement Decisions"**.

**Additive Manufacturing (AM)**, particularly Powder Bed Fusion (PBF), is characterized by a **subadditive cost structure**, where high fixed setup and recoating costs create a strong economic imperative for batch consolidation. In the emerging context of **Social Manufacturing**, service providers face a critical dynamic scheduling challenge:

* **Reactive Approach:** Process orders immediately to minimize tardiness?
* **Proactive Approach:** Strategically **postpone** production to accumulate more orders, improving nesting density and reducing unit costs?

This project models this problem as a finite-horizon **Markov Decision Process (MDP)** and proposes a **Direct Lookahead Approximation (DLA)** policy to solve it.

## ✨ Key Features & Methodology

The core of this repository is a decision support system that transitions AM scheduling from reactive heuristics to strategic foresight.

* **MDP Formulation:** Explicitly models "Postponement" as an active, strategic decision variable rather than passive idleness.
* **Direct Lookahead Approximation (DLA):** Utilizes a lookahead policy to estimate the value of future states, overcoming the curse of dimensionality.
* **Monte Carlo Tree Search (MCTS):** Implements a tailored MCTS framework to efficiently navigate the vast state space of dynamic order arrivals.
* **Pareto-based Pruning:** Introduces a heuristic pruning strategy to filter the combinatorially explosive decision space of 2D nesting, focusing computational resources on high-potential batches.
* **Benchmarks:** Includes a reactive policy (**PwA**), parameter-dependent heuristics (**PWB**, **PCR**), and an offline **Mixed-Integer Linear Programming (MILP)** model (solved via Gurobi/CPLEX) for theoretical upper-bound analysis.

## 📂 Repository Structure

```text
.
├── Code/
│   ├── MCTS_AM_Postpone_main.py       # Main entry for the MCTS-based DLA policy
│   ├── MILP_AM_Postpone_main.py       # Main entry for the offline MILP benchmark
│   ├── BinPackingHeu.py               # 2D Nesting heuristics (BL/Guillotine variants)
│   ├── two_dimensional_packing_feasibility.py # Feasibility checks for batching
│   └── Bestfit.dll                    # Compiled C++ extension for high-performance packing
│
├── Experiments/
│   ├── Instances/                     # Testbed instances (Arrival streams, order attributes)
│   │   ├── Testbed Instances/         # Core instances for comparative analysis
│   │   └── Parameter Calibration/     # Instances used for tuning MCTS parameters
│   └── Results/                       # Raw simulation logs and profit trajectories
│
├── Interactive Simulator/             # Visualization tool for decision process replay
│
├── Manuscript/                        # Related documentation and supplementary materials
│
└── Slides/                            # Presentation slides summarizing the work
```

## 🚀 Getting Started

### Prerequisites

* **Python 3.8+**
* **Gurobi Optimizer** (Required only for running `MILP_AM_Postpone_main.py`. Ensure you have a valid license).
* Standard scientific stack: `numpy`, `pandas`, `matplotlib`.

### Installation

1.  Clone the repository:
    ```bash
    git clone [https://github.com/haowutongji/dynamic_am_scheduling_with_postponement.git](https://github.com/haowutongji/dynamic_am_scheduling_with_postponement.git)
    cd dynamic_am_scheduling_with_postponement
    ```

2.  Install dependencies:
    ```bash
    pip install numpy pandas matplotlib gurobipy
    ```

### Running the Simulation (MCTS)

To run the main Lookahead Approximate Dynamic Programming policy:

```bash
python Code/MCTS_AM_Postpone_main.py
```

*Configuration:* You can adjust simulation parameters (e.g., `Time Horizon`, `Lookahead Depth`, `Simulation Budget`) directly within the `main` block of the script or via command-line arguments (if implemented).

### Running the Benchmark (MILP)

To calculate the theoretical offline optimum for a specific instance:

```bash
python Code/MILP_AM_Postpone_main.py
```

## 📊 Experiments & Data

The `Experiments/Instances` directory contains the stochastic scenarios used in the paper. The naming convention `Hxx-Ryy-Szz` represents:
* **H (Horizon):** Length of the decision horizon (e.g., 36, 72, 144 time slots).
* **R (Arrival Pattern):** The mix of part geometries (e.g., `RUniform`, `RLarge-dominant`, `RSmall-dominant`).
* **S (Seed):** Random seed identifier for reproducibility.

## 📝 Citation

If you find this code or research useful for your work, please cite our paper:

```bibtex
@article{Wu2025DynamicAM,
  title={Dynamic Single Additive Manufacturing Machine Scheduling Considering Postponement Decisions: A Lookahead Approximate Dynamic Programming},
  author={Wu, Hao and Yu, Chunlong},
  journal={Submitted to European Journal of Operational Research},
  year={2025}
}
```

## ⚖️ License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📧 Contact

**Hao Wu** School of Mechanical Engineering, Tongji University  
Email: [hao_wu@tongji.edu.cn](mailto:hao_wu@tongji.edu.cn)

---
*This research is supported by the School of Mechanical Engineering at Tongji University.*
