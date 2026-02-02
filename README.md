# Traveling Salesman Problem (TSP) and Vehicle Routing Problem (VRP) Implementations

This repository contains implementations of the Traveling Salesman Problem using two different formulations (DFJ and MTZ) and an extension to the Vehicle Routing Problem.

## Problem Instance

- **Cities**: V = {1, 2, 3, 4, 5, 6}
- **Cost Matrix**: Provided in the problem specification

## Formulations

### 1. DFJ Formulation (Dantzig-Fulkerson-Johnson)
- **File**: `tsp_dfj.py`
- **Approach**: Eliminates subtours by explicitly forbidding them
- **Constraint**: For every subset S of cities with |S| ≥ 2, Σ_{i∈S} Σ_{j∈S} x_ij ≤ |S| - 1

### 2. MTZ Formulation (Miller-Tucker-Zemlin)
- **File**: `tsp_mtz.py`
- **Approach**: Uses auxiliary ordering variables to prevent subtours
- **Variables**: u_i represents the position of city i in the tour
- **Constraint**: u_i - u_j + n * x_ij ≤ n - 1 for all i ≠ j, i, j ∈ V \ {1}
- **Fixed**: u_1 = 1 (city 1 is at position 1)

### 3. Vehicle Routing Problem (VRP)
- **File**: `vrp.py`
- **Extension**: Multiple vehicles starting and ending at a depot
- **Constraints**: 
  - Each customer visited exactly once
  - Flow conservation for each vehicle
  - Vehicles start and end at depot
  - DFJ subtour elimination constraints

## Installation

1. Create a virtual environment (recommended):
```bash
python3 -m venv venv
source venv/bin/activate  # On macOS/Linux
# or
venv\Scripts\activate  # On Windows
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

## Usage

Make sure your virtual environment is activated, then run:

### Run DFJ Formulation
```bash
python tsp_dfj.py
```

### Run MTZ Formulation
```bash
python tsp_mtz.py
```

### Run VRP
```bash
python vrp.py
```

## Output

Each script will:
1. Solve the optimization problem
2. Display the optimal tour/route
3. Show the total cost
4. Provide detailed route information

## Solver

The implementations use AMPL (A Mathematical Programming Language) with a mixed-integer programming solver. By default, the code uses CBC (open-source). For better performance on larger instances, you can use commercial solvers like CPLEX or Gurobi by changing the solver option in the code:

```python
ampl.setOption("solver", "cplex")  # or "gurobi", "cbc", etc.
```

Note: You need to have AMPL installed and the desired solver licensed/installed on your system.

## Notes

- The cost matrix uses a large number (M = 10000) to represent ∞ (no direct path or self-loops)
- The DFJ formulation adds constraints for all subsets, which can be computationally expensive for large instances
- The MTZ formulation is more compact but may be weaker for some problem instances
- The VRP implementation assumes 2 vehicles by default; you can modify the `m` variable to change this

