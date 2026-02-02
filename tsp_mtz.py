"""
Traveling Salesman Problem (TSP) - MTZ Formulation using AMPL
Miller-Tucker-Zemlin (MTZ) formulation uses auxiliary ordering variables to prevent subtours.
"""

from amplpy import AMPL

def solve_tsp_mtz():
    """
    Solve TSP using MTZ formulation with AMPL.
    Cities: V = {1, 2, 3, 4, 5, 6}
    City 1 is fixed at position 1 (u_1 = 1)
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
    
    # Create AMPL instance
    ampl = AMPL()
    
    # Define model
    ampl.eval("""
    # Sets
    set V;  # Set of cities
    
    # Parameters
    param c{V, V} >= 0;  # Cost matrix
    param n;  # Number of cities
    
    # Decision variables
    var x{V, V} binary;  # x[i,j] = 1 if tour goes from city i to city j
    var u{V} >= 1, <= n;  # Auxiliary variable: position of city i in the tour
    
    # Objective: Minimize total travel cost
    minimize TotalCost: sum{i in V, j in V: i != j} c[i,j] * x[i,j];
    
    # Constraints: Each city is entered exactly once
    subject to EnterCity{j in V}:
        sum{i in V: i != j} x[i,j] = 1;
    
    # Constraints: Each city is left exactly once
    subject to LeaveCity{i in V}:
        sum{j in V: j != i} x[i,j] = 1;
    
    # MTZ: Fix position of city 1 (u_1 = 1)
    subject to FixU1:
        u[1] = 1;
    
    # MTZ Subtour Elimination Constraints
    # u_i - u_j + n * x_ij <= n - 1 for all i != j, i, j in V \ {1}
    subject to MTZ{i in V, j in V: i != j and i != 1 and j != 1}:
        u[i] - u[j] + n * x[i,j] <= n - 1;
    """)
    
    # Set data
    ampl.set["V"] = cities
    ampl.param["n"] = n
    
    # Set cost matrix
    for i in cities:
        for j in cities:
            ampl.param["c"][i, j] = costs[i, j]
    
    # Set solver (using CBC as default, can be changed to CPLEX, Gurobi, etc.)
    ampl.setOption("solver", "cbc")
    
    # Solve
    print('Solving TSP with MTZ formulation using AMPL...')
    ampl.solve()
    
    # Check solution status
    solve_result = ampl.getValue("solve_result")
    if solve_result == "solved" or solve_result == "solved?":
        print(f'\nOptimal solution found!')
        objective_value = ampl.getValue("TotalCost")
        print(f'Total cost: {objective_value}\n')
        
        # Extract tour
        tour = []
        current_city = 1
        visited = set()
        
        while len(tour) < n:
            tour.append(current_city)
            visited.add(current_city)
            for j in cities:
                if j not in visited:
                    x_val = ampl.getVariable("x")[current_city, j].value()
                    if x_val > 0.5:
                        current_city = j
                        break
            else:
                # Return to start
                x_val = ampl.getVariable("x")[current_city, 1].value()
                if x_val > 0.5:
                    tour.append(1)
                break
        
        print('Tour:', ' -> '.join(map(str, tour)))
        print('\nCity positions in tour (u_i values):')
        for city in cities:
            u_val = ampl.getVariable("u")[city].value()
            print(f'  City {city}: position {u_val:.2f}')
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
        print(f'Solution status: {solve_result}')
        print('No optimal solution found.')

if __name__ == '__main__':
    solve_tsp_mtz()
