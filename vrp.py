"""
Vehicle Routing Problem (VRP)
Extension of TSP where multiple vehicles start and end at a depot.
Each customer must be visited exactly once by exactly one vehicle.
No capacity restrictions on vehicles.
"""

from ortools.linear_solver import pywraplp
import itertools

def solve_vrp():
    """
    Solve VRP with DFJ subtour elimination constraints.
    V = {0, 1, 2, ..., n} where 0 is the depot
    K = {1, 2, ..., m} is the set of vehicles
    """
    
    # Problem data
    n = 6  # Number of customers (excluding depot)
    m = 2  # Number of vehicles
    depot = 0
    customers = list(range(1, n + 1))
    nodes = [depot] + customers  # All nodes including depot
    vehicles = list(range(1, m + 1))
    
    # Cost matrix c_ij (extended from TSP instance)
    # We'll use the same costs, with depot at index 0
    # For simplicity, assume depot has similar costs to city 1
    M = 10000
    costs = {}
    
    # Costs from TSP instance (cities 1-6)
    tsp_costs = {
        (1, 1): M, (1, 2): 10, (1, 3): 15, (1, 4): 20, (1, 5): 10, (1, 6): 25,
        (2, 1): 10, (2, 2): M, (2, 3): 35, (2, 4): 25, (2, 5): 17, (2, 6): 30,
        (3, 1): 15, (3, 2): 35, (3, 3): M, (3, 4): 30, (3, 5): 28, (3, 6): 18,
        (4, 1): 20, (4, 2): 25, (4, 3): 30, (4, 4): M, (4, 5): 22, (4, 6): 14,
        (5, 1): 10, (5, 2): 17, (5, 3): 28, (5, 4): 22, (5, 5): M, (5, 6): 16,
        (6, 1): 25, (6, 2): 30, (6, 3): 18, (6, 4): 14, (6, 5): 16, (6, 6): M,
    }
    
    # Build cost matrix for VRP (including depot)
    for i in nodes:
        for j in nodes:
            if i == j:
                costs[i, j] = M
            elif i == depot:
                # Cost from depot to customer j (using cost from city 1 to j as reference)
                if j in tsp_costs:
                    costs[i, j] = tsp_costs[1, j] if (1, j) in tsp_costs else 10
                else:
                    costs[i, j] = 10
            elif j == depot:
                # Cost from customer i to depot (using cost from i to city 1 as reference)
                if i in tsp_costs:
                    costs[i, j] = tsp_costs[i, 1] if (i, 1) in tsp_costs else 10
                else:
                    costs[i, j] = 10
            else:
                # Cost between customers
                costs[i, j] = tsp_costs[i, j] if (i, j) in tsp_costs else M
    
    # Create solver
    solver = pywraplp.Solver.CreateSolver('CBC')
    if not solver:
        print('Could not create solver')
        return
    
    # Decision variables: x_ij^k = 1 if vehicle k travels from node i to node j, 0 otherwise
    x = {}
    for k in vehicles:
        for i in nodes:
            for j in nodes:
                if i != j:
                    x[i, j, k] = solver.IntVar(0, 1, f'x_{i}_{j}_{k}')
    
    # Objective: Minimize total travel cost
    objective = solver.Objective()
    for k in vehicles:
        for i in nodes:
            for j in nodes:
                if i != j:
                    objective.SetCoefficient(x[i, j, k], costs[i, j])
    objective.SetMinimization()
    
    # Constraints: Each customer is visited exactly once
    for i in customers:
        constraint = solver.Constraint(1, 1, f'visit_customer_{i}')
        for k in vehicles:
            for j in nodes:
                if j != i:
                    constraint.SetCoefficient(x[j, i, k], 1)
    
    # Constraints: Flow conservation for each vehicle
    # For each node i and vehicle k: sum of incoming = sum of outgoing
    for k in vehicles:
        for i in nodes:
            constraint = solver.Constraint(0, 0, f'flow_{i}_{k}')
            for j in nodes:
                if j != i:
                    constraint.SetCoefficient(x[i, j, k], 1)
                    constraint.SetCoefficient(x[j, i, k], -1)
    
    # Constraints: Each vehicle can leave the depot at most once
    for k in vehicles:
        constraint = solver.Constraint(0, 1, f'leave_depot_{k}')
        for j in customers:
            constraint.SetCoefficient(x[depot, j, k], 1)
    
    # Constraints: Each vehicle can return to the depot at most once
    for k in vehicles:
        constraint = solver.Constraint(0, 1, f'return_depot_{k}')
        for i in customers:
            constraint.SetCoefficient(x[i, depot, k], 1)
    
    # DFJ Subtour Elimination Constraints
    # For every subset S of customers (S ⊆ V \ {0}, 2 <= |S| <= n):
    # sum_{k in K} sum_{i in S} sum_{j in S} x_ij^k <= |S| - 1
    print('Adding DFJ subtour elimination constraints...')
    for subset_size in range(2, n + 1):
        for subset in itertools.combinations(customers, subset_size):
            constraint = solver.Constraint(0, len(subset) - 1, f'subtour_{subset}')
            for k in vehicles:
                for i in subset:
                    for j in subset:
                        if i != j:
                            constraint.SetCoefficient(x[i, j, k], 1)
    
    # Solve
    print('Solving VRP with DFJ formulation...')
    status = solver.Solve()
    
    if status == pywraplp.Solver.OPTIMAL:
        print(f'\nOptimal solution found!')
        print(f'Total cost: {solver.Objective().Value()}\n')
        
        # Extract routes for each vehicle
        all_visited = set()
        for k in vehicles:
            print(f'Vehicle {k} route:')
            route = []
            current_node = depot
            visited = set()
            
            # Find starting point (depot -> customer)
            for j in customers:
                if x[depot, j, k].solution_value() > 0.5:
                    current_node = j
                    route.append(depot)
                    route.append(j)
                    visited.add(j)
                    all_visited.add(j)
                    break
            
            # Follow the route
            while current_node != depot:
                found_next = False
                for j in nodes:
                    if j != current_node and x[current_node, j, k].solution_value() > 0.5:
                        if j == depot:
                            route.append(depot)
                            break
                        elif j not in visited:
                            route.append(j)
                            visited.add(j)
                            all_visited.add(j)
                            current_node = j
                            found_next = True
                            break
                if not found_next or current_node == depot:
                    break
            
            if route:
                route_str = ' -> '.join(map(str, route))
                print(f'  {route_str}')
                
                # Calculate route cost
                route_cost = 0
                for i in range(len(route) - 1):
                    from_node = route[i]
                    to_node = route[i + 1]
                    route_cost += costs[from_node, to_node]
                print(f'  Route cost: {route_cost}')
            else:
                print('  (Vehicle not used)')
            print()
        
        # Summary
        print('All customers visited:', len(all_visited) == n)
        if len(all_visited) == n:
            print(f'Visited customers: {sorted(all_visited)}')
        
    else:
        print('No optimal solution found.')
        print(f'Solver status: {status}')

if __name__ == '__main__':
    solve_vrp()

