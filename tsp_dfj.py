"""
Traveling Salesman Problem (TSP) - DFJ Formulation
Dantzig-Fulkerson-Johnson (DFJ) formulation eliminates subtours by explicitly forbidding them.
"""

from ortools.linear_solver import pywraplp
import itertools

def solve_tsp_dfj():
    """
    Solve TSP using DFJ formulation.
    Cities: V = {1, 2, 3, 4, 5, 6}
    """
    
    # Problem data
    n = 6  # Number of cities
    cities = list(range(1, n + 1))
    
    # Cost matrix c_ij (from image)
    # ∞ is represented as a very large number (M)
    M = 10000
    costs = {
        (1, 1): M, (1, 2): 10, (1, 3): 15, (1, 4): 20, (1, 5): 10, (1, 6): 25,
        (2, 1): 10, (2, 2): M, (2, 3): 35, (2, 4): 25, (2, 5): 17, (2, 6): 30,
        (3, 1): 15, (3, 2): 35, (3, 3): M, (3, 4): 30, (3, 5): 28, (3, 6): 18,
        (4, 1): 20, (4, 2): 25, (4, 3): 30, (4, 4): M, (4, 5): 22, (4, 6): 14,
        (5, 1): 10, (5, 2): 17, (5, 3): 28, (5, 4): 22, (5, 5): M, (5, 6): 16,
        (6, 1): 25, (6, 2): 30, (6, 3): 18, (6, 4): 14, (6, 5): 16, (6, 6): M,
    }
    
    # Create solver
    solver = pywraplp.Solver.CreateSolver('CBC')
    if not solver:
        print('Could not create solver')
        return
    
    # Decision variables: x_ij = 1 if tour goes from city i to city j, 0 otherwise
    x = {}
    for i in cities:
        for j in cities:
            if i != j:
                x[i, j] = solver.IntVar(0, 1, f'x_{i}_{j}')
    
    # Objective: Minimize total travel cost
    objective = solver.Objective()
    for i in cities:
        for j in cities:
            if i != j:
                objective.SetCoefficient(x[i, j], costs[i, j])
    objective.SetMinimization()
    
    # Constraints: Each city is entered exactly once
    for j in cities:
        constraint = solver.Constraint(1, 1, f'enter_city_{j}')
        for i in cities:
            if i != j:
                constraint.SetCoefficient(x[i, j], 1)
    
    # Constraints: Each city is left exactly once
    for i in cities:
        constraint = solver.Constraint(1, 1, f'leave_city_{i}')
        for j in cities:
            if j != i:
                constraint.SetCoefficient(x[i, j], 1)
    
    # DFJ Subtour Elimination Constraints
    # For every subset S of cities with |S| >= 2, sum_{i in S} sum_{j in S} x_ij <= |S| - 1
    # We'll add these constraints iteratively as needed (lazy constraints)
    # For now, we add constraints for all subsets of size 2 to n-1
    subtour_constraints = []
    for subset_size in range(2, n):
        for subset in itertools.combinations(cities, subset_size):
            constraint = solver.Constraint(0, len(subset) - 1, f'subtour_{subset}')
            for i in subset:
                for j in subset:
                    if i != j:
                        constraint.SetCoefficient(x[i, j], 1)
            subtour_constraints.append(constraint)
    
    # Solve
    print('Solving TSP with DFJ formulation...')
    status = solver.Solve()
    
    if status == pywraplp.Solver.OPTIMAL:
        print(f'\nOptimal solution found!')
        print(f'Total cost: {solver.Objective().Value()}\n')
        
        # Extract tour
        tour = []
        current_city = 1
        visited = set()
        
        while len(tour) < n:
            tour.append(current_city)
            visited.add(current_city)
            for j in cities:
                if j not in visited and x[current_city, j].solution_value() > 0.5:
                    current_city = j
                    break
            else:
                # Return to start
                if x[current_city, 1].solution_value() > 0.5:
                    tour.append(1)
                break
        
        print('Tour:', ' -> '.join(map(str, tour)))
        print('\nDetailed route:')
        total_cost = 0
        for i in range(len(tour) - 1):
            from_city = tour[i]
            to_city = tour[i + 1]
            cost = costs[from_city, to_city]
            total_cost += cost
            print(f'  City {from_city} -> City {to_city}: cost = {cost}')
        print(f'\nTotal cost: {total_cost}')
        
    else:
        print('No optimal solution found.')
        print(f'Solver status: {status}')

if __name__ == '__main__':
    solve_tsp_dfj()

