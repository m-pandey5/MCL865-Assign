"""
Vehicle Routing Problem (VRP) using AMPL
Extension of TSP where multiple vehicles start and end at a depot.
Each customer must be visited exactly once by exactly one vehicle.
No capacity restrictions on vehicles.
"""

from amplpy import AMPL
import itertools

def solve_vrp():
    """
    Solve VRP with DFJ subtour elimination constraints using AMPL.
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
    
    # Create AMPL instance
    ampl = AMPL()
    
    # Define model
    ampl.eval("""
    # Sets
    set V;  # Set of nodes (including depot 0)
    set Customers;  # Set of customers (V \ {0})
    set K;  # Set of vehicles
    
    # Parameters
    param c{V, V} >= 0;  # Cost matrix
    
    # Decision variables
    var x{V, V, K} binary;  # x[i,j,k] = 1 if vehicle k travels from node i to node j
    
    # Objective: Minimize total travel cost
    minimize TotalCost: sum{k in K, i in V, j in V: i != j} c[i,j] * x[i,j,k];
    
    # Constraints: Each customer is visited exactly once
    subject to VisitCustomer{i in Customers}:
        sum{k in K, j in V: j != i} x[j,i,k] = 1;
    
    # Constraints: Flow conservation for each vehicle
    # For each node i and vehicle k: sum of incoming = sum of outgoing
    subject to FlowConservation{i in V, k in K}:
        sum{j in V: j != i} x[i,j,k] = sum{j in V: j != i} x[j,i,k];
    
    # Constraints: Each vehicle can leave the depot at most once
    subject to LeaveDepot{k in K}:
        sum{j in Customers} x[0,j,k] <= 1;
    
    # Constraints: Each vehicle can return to the depot at most once
    subject to ReturnDepot{k in K}:
        sum{i in Customers} x[i,0,k] <= 1;
    """)
    
    # Set data
    ampl.set["V"] = nodes
    ampl.set["Customers"] = customers
    ampl.set["K"] = vehicles
    
    # Set cost matrix
    for i in nodes:
        for j in nodes:
            ampl.param["c"][i, j] = costs[i, j]
    
    # Add DFJ Subtour Elimination Constraints
    # For every subset S of customers (S ⊆ V \ {0}, 2 <= |S| <= n):
    # sum_{k in K} sum_{i in S} sum_{j in S} x_ij^k <= |S| - 1
    print('Adding DFJ subtour elimination constraints...')
    constraint_num = 1
    for subset_size in range(2, n + 1):
        for subset in itertools.combinations(customers, subset_size):
            subset_str = "{" + ", ".join(map(str, subset)) + "}"
            ampl.eval(f"""
            subject to SubtourElim_{constraint_num}:
                sum{{k in K, i in {subset_str}, j in {subset_str}: i != j}} x[i,j,k] <= {len(subset) - 1};
            """)
            constraint_num += 1
    
    # Set solver (using CBC as default, can be changed to CPLEX, Gurobi, etc.)
    ampl.setOption("solver", "cbc")
    
    # Solve
    print('Solving VRP with DFJ formulation using AMPL...')
    ampl.solve()
    
    # Check solution status
    solve_result = ampl.getValue("solve_result")
    if solve_result == "solved" or solve_result == "solved?":
        print(f'\nOptimal solution found!')
        objective_value = ampl.getValue("TotalCost")
        print(f'Total cost: {objective_value}\n')
        
        # Extract routes for each vehicle
        all_visited = set()
        for k in vehicles:
            print(f'Vehicle {k} route:')
            route = []
            current_node = depot
            visited = set()
            
            # Find starting point (depot -> customer)
            for j in customers:
                x_val = ampl.getVariable("x")[depot, j, k].value()
                if x_val > 0.5:
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
                    if j != current_node:
                        x_val = ampl.getVariable("x")[current_node, j, k].value()
                        if x_val > 0.5:
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
        print(f'Solution status: {solve_result}')
        print('No optimal solution found.')

if __name__ == '__main__':
    solve_vrp()
