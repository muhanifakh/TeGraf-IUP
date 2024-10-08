# Group 10 Graph Theory Week 7 Project

| Name           | NRP        |
| ---            | ---        |
| Reino Yuris Kusumanegara | 5025231075 | 
| Muhammad Rizqy Hidayat| 5025231161 |
| Muhammad Hanif Fakriansyah| 5025231082 |
| Fazle Robby Pratama| 5025231011 |

## A. Travelling Salesman Problem

**TSP Function:**
```
def tsp(start, v, graph):
    # Store all vertices except the starting vertex.
    vertices = [i for i in range(v) if i != start]
    
    min_cost = float('inf')
    best_route = []

    # Generate all permutations of vertices.
    for permutation in itertools.permutations(vertices):
        current_cost = 0
        current_route = []

        k = start

        # Calculate the cost of the current permutation.
        for next_vertex in permutation:
            current_cost += graph[k][next_vertex][0]  # Add the edge weight
            current_route.append(graph[k][next_vertex][1])  # Add the edge name
            k = next_vertex

        # Add the cost to return to the starting node.
        current_cost += graph[k][start][0]
        current_route.append(graph[k][start][1])

        # Update the minimum cost and best route if a lower cost is found.
        if current_cost < min_cost:
            min_cost = current_cost
            best_route = current_route

    print(f"Cost: {min_cost}")
    print("Route: " + ", ".join(map(str, best_route)))
```
- Explanation

  <br>1. Function definition <br>
  The function tsp takes three parameters: <br>
  - start: The starting vertex for the TSP.
  - v: The total number of vertices in the graph.
  - graph: A 2D list representing the graph where graph[i][j] holds a tuple (weight, edge_name):
  - weight: The distance or cost between vertices i and j.
  - edge_name: A label or identifier for the edge between vertices i and j. 

  <br>2. Creates a list of all vertices excluding the starting vertex. This is used to generate all possible routes that the salesperson can take.
  ```
  vertices = [i for i in range(v) if i != start]
  ```
 
  <br>3. Initiate Variables :
  
  - min_cost is initialized to infinity (float('inf')) because we are looking for the minimum cost route. It will hold the lowest travel cost found. 
  - best_route will store the sequence of edges corresponding to the optimal path.
  ```
  min_cost = float('inf')
  best_route = []
  ```
  <br>4. The itertools.permutations function generates all possible orders in which the salesperson can visit the vertices (excluding the starting    vertex). Each permutation represents a potential route.
  ```
  for permutation in itertools.permutations(vertices):
  ```
  <br>5. Iterate Through the Current Permutation
  ```
  for next_vertex in permutation:
    current_cost += graph[k][next_vertex][0]
    current_route.append(graph[k][next_vertex][1])
    k = next_vertex
  ```
  - For each vertex in the current permutation, add the weight (distance/cost) from the current vertex (k) to the next vertex (next_vertex) to      current_cost.
  - Append the name of the edge used to current_route.
  - Update k to be the next_vertex, which moves the current position to the next vertex in the route.
  
  <br>6. Add the Cost to Return to the Starting Node
  ```
  current_cost += graph[k][start][0]
  current_route.append(graph[k][start][1])
  ```
  - After visiting all vertices, add the cost of returning from the last vertex (k) back to the start vertex to current_cost.
  - Add the edge name of this return trip to current_route.
  
  <br>7. If the total cost of the current permutation (current_cost) is less than min_cost, update min_cost and best_route to reflect the better route.
  ```
  if current_cost < min_cost:
    min_cost = current_cost
    best_route = current_route
  ```
  <br>8. After all permutations have been checked, print out the minimum cost (min_cost) and the corresponding route (best_route).
  
**Main Function:**
```
    v = int(input())
    e = int(input())

    # Initialize the graph with infinite weights.
    graph = [[(float('inf'), -1) for _ in range(v)] for _ in range(v)]

    # Read edges and store in the graph.
    for _ in range(e):
        name, ver1, ver2, weight = map(int, input().split())
        ver1 -= 1  # Convert to zero-indexed
        ver2 -= 1  # Convert to zero-indexed

        # Update the graph with the minimum weight for each edge.
        if weight < graph[ver1][ver2][0]:
            graph[ver1][ver2] = (weight, name)
            graph[ver2][ver1] = (weight, name)

    # Solve the TSP starting from vertex 0 (which is node 1 in one-indexed input).
    tsp(0, v, graph)
```
- Explanation
  <br>1. Input the Number of Vertices and Edges <br>
  <br>2. Initialize the Graph
  ```
  graph = [[(float('inf'), -1) for _ in range(v)] for _ in range(v)]
  ```
  - Initializes a 2D list graph with size v x v.
  - Each entry graph[i][j] holds a tuple (weight, edge_name), initially set to (float('inf'), -1), which means there is no direct connection between vertices i and j.
  - float('inf') is used to represent an infinite weight, indicating that the edge does not exist initially.
  
  <br>3. Read Edges and Store in the Graph
  ```
  for _ in range(e):
    name, ver1, ver2, weight = map(int, input().split())
    ver1 -= 1  # Convert to zero-indexed
    ver2 -= 1  # Convert to zero-indexed
  ```
  - Iterates e times to read the input edges.
  - Each edge has:
    - name: The identifier for the edge.
    - ver1 and ver2: The vertices (converted to zero-indexed by subtracting 1).
    - weight: The cost or distance of the edge between ver1 and ver2.
  <br>4. Update the Graph with Edge Weights
  ```
  if weight < graph[ver1][ver2][0]:
    graph[ver1][ver2] = (weight, name)
    graph[ver2][ver1] = (weight, name)
  ```
  - Checks if the current edge weight is smaller than the existing weight in graph[ver1][ver2].
  - If it is, the edge is updated with the new weight and name.
  - This ensures that the graph always stores the edge with the minimum weight between any two vertices.
  
  <br>5. Solve the TSP Starting from Vertex 0
  ```
  tsp(0, v, graph)
  ```
  - Calls the tsp function, starting from vertex 0.
  - The input is assumed to be in a one-indexed format, so the starting vertex 1 corresponds to index 0 after conversion.
  - This function call finds the optimal route for the Traveling Salesperson Problem using the previously constructed graph.
- Output : <br>
![image](https://github.com/user-attachments/assets/0e73e9ad-266e-45bc-9209-af7943857d73)

B. Knight's Tour Problem
------------------------

**Knight's Tour Function:**

```

# Get inputs for the board size and starting position
board_size = tuple(map(int, input("Enter the board size (e.g., '5 5'): ").split()))
start_position = tuple(map(int, input("Enter the starting position of the knight (e.g., '2 2'): ").split()))

# Possible moves of a knight in a fixed order
knight_moves = [
    (2, 1), (1, 2), (-1, 2), (-2, 1),
    (-2, -1), (-1, -2), (1, -2), (2, -1)
]

# Function to check if a move is within the board and unvisited
def is_valid_move(pos, visited, board_size):
    x, y = pos
    return 0 <= x < board_size[0] and 0 <= y < board_size[1] and pos not in visited

# Function to generate a knight's tour using backtracking without randomization
def knights_tour(current_position, visited, board_size):
    if len(visited) == board_size[0] * board_size[1]:
        # All squares have been visited exactly once; this is a valid tour.
        return visited

    x, y = current_position
    for move in knight_moves:
        next_pos = (x + move[0], y + move[1])
        if is_valid_move(next_pos, visited, board_size):
            visited.append(next_pos)
            result = knights_tour(next_pos, visited, board_size)
            if result:
                return result
            visited.pop()  # Backtrack if the path didn't work out

    return None

# Start the tour counting from the given starting position
initial_visited = [start_position]
tour_path = knights_tour(start_position, initial_visited, board_size)

# Check if a valid tour was found and print the result
if tour_path:
    print("The knight successfully visits all squares with a systematic tour:")
else:
    print("The knight's tour is incomplete or could not be found.")

# Output the path
for idx, pos in enumerate(tour_path):
    print(f"Move {idx + 1}: {pos}")

```
- Explanation

<br>1. Input the Board Size and Starting Position <br> The user is prompted to input:

- `board_size`: A tuple representing the dimensions of the board (e.g., 5 5 for a 5x5 board).
- `start_position`: A tuple representing the starting coordinates of the knight on the board.

<br>2. Define Possible Knight Moves <br> The `knight_moves` list contains all possible moves that a knight can make on a chessboard from any position, such as `(2, 1)` or `(-2, -1)`.

<br>3. Validation Function <br> The `is_valid_move` function checks if a move is within the boundaries of the board and if the position has not been visited yet:
```
def is_valid_move(pos, visited, board_size):
    x, y = pos
    return 0 <= x < board_size[0] and 0 <= y < board_size[1] and pos not in visited

```

<br>4. Generate Knight's Tour <br> The `knights_tour` function attempts to find a complete knight's tour using backtracking:

- The knight moves through all possible positions based on `knight_moves`.
- If a move is valid, it is added to the list of visited positions.
- If a dead end is reached, the function backtracks by removing the last move.
```
def knights_tour(current_position, visited, board_size):
    if len(visited) == board_size[0] * board_size[1]:
        # All squares have been visited exactly once; this is a valid tour.
        return visited    ...
```

<br>5. Start the Tour <br> Begins the knight's tour from the user-specified starting position and finds a valid path, if possible.

<br>6. Output the Path <br> If a complete tour is found, the sequence of moves is printed. Otherwise, a message indicates that the tour could not be completed.

- Example:
```
Enter the board size (e.g., '5 5'): 5 5
Enter the starting position of the knight (e.g., '2 2'): 2 2
Move 1: (2, 2)
Move 2: (4, 3)
Move 3: (2, 4)
Move 4: (0, 3)
...

```

- Conclusion

This implementation of the Knight's Tour problem uses a backtracking approach to find a solution. It takes the board size and starting position as input, allowing for flexibility in the problem setup. The solution attempts to find a path where the knight visits each square exactly once, but the nature of the solution means it may not always succeed in finding a complete tour, especially on smaller boards or with less favorable starting conditions. Nonetheless, it provides an interesting approach to the classic problem, using elements of algorithmic backtracking.


C. Chinese Postman Problem
------------------------

```
import math
from collections import defaultdict

def min_cost_matching(odd_vertices, dist):
    n = len(odd_vertices)
    dp = [math.inf] * (1 << n)
    dp[0] = 0

    for mask in range(1 << n):
        x = bin(mask).count('1')
        if x % 2 != 0:
            continue

        for i in range(n):
            if mask & (1 << i):
                continue

            for j in range(i + 1, n):
                if mask & (1 << j):
                    continue

                new_mask = mask | (1 << i) | (1 << j)
                dp[new_mask] = min(dp[new_mask], dp[mask] + dist[odd_vertices[i]][odd_vertices[j]])

    return dp[(1 << n) - 1]

def find_eulerian_path(adj_list, edge_count, start, edge_to_id):
    stack = [start]
    path = []

    while stack:
        u = stack[-1]

        if any(edge_count[u, v] > 0 for v in adj_list[u]):
            for v in adj_list[u]:
                if edge_count[u, v] > 0:
                    edge_count[u, v] -= 1
                    edge_count[v, u] -= 1
                    stack.append(v)  
                    path.append(edge_to_id[(u, v)]) 
                    break
        else:
            stack.pop()

    return path

def chinese_postman_problem(n, e, edges, start):
    adj_list = defaultdict(list)
    edge_count = defaultdict(int)
    edge_to_id = {}
    dist = [[math.inf] * n for _ in range(n)]

    for edge_id, u, v, cost in edges:
        u, v = u-1, v-1 
        adj_list[u].append(v)
        adj_list[v].append(u)
        edge_count[u, v] += 1
        edge_count[v, u] += 1
        edge_to_id[u, v] = edge_id
        edge_to_id[v, u] = edge_id
        dist[u][v] = dist[v][u] = min(dist[u][v], cost)

    for k in range(n):
        for i in range(n):
            for j in range(n):
                dist[i][j] = min(dist[i][j], dist[i][k] + dist[k][j])

    odd_vertices = [i for i in range(n) if sum(edge_count[i, j] for j in adj_list[i]) % 2 != 0]

    min_matching_cost = min_cost_matching(odd_vertices, dist)

    total_cost = sum(cost for _, _, _, cost in edges) + min_matching_cost

    route = find_eulerian_path(adj_list, edge_count, start - 1, edge_to_id)
    
    print(f"Cost: {total_cost}")
    print(f"Route: {', '.join(map(str, route))}")


def main():
    n = int(input())
    e = int(input())
    edges = []

    for _ in range(e):
        edge_id, u, v, cost = map(int, input().split())
        edges.append((edge_id, u, v, cost))

    start = int(input())

    chinese_postman_problem(n, e, edges, start)

if __name__ == "__main__":
    main()
```

**Explanation**

- ```min_cost_matching``` Function:
  
This function pairs up vertices with odd degrees with the minimum possible cost to make the graph Eulerian.
It uses dynamic programming to compute the minimum cost of pairing up all odd-degree vertices.

Input:

```odd_vertices```: List of vertices with odd degrees.

```dist```: A 2D list where ```dist[u][v]``` holds the shortest distance between vertices u and v.

Process:
Iterates over all subsets of odd-degree vertices using a bitmask.
For each subset, tries to pair up vertices and computes the total pairing cost.
```dp[mask]``` stores the minimum cost for the subset represented by mask.
Output: Returns the minimum cost required to pair up all odd-degree vertices.
- ```find_eulerian_path``` Function:

This function constructs an Eulerian path after the graph is made Eulerian.
Uses a stack-based DFS-like approach to traverse the graph.
Input:

```adj_list```: The adjacency list of the graph.

```edge_count```: A dictionary tracking the remaining number of times an edge between u and v is available.

```start```: The starting vertex for the Eulerian path.

```edge_to_id```: Maps the edge between two vertices (u, v) to a unique edge ID.

Process:
Traverses edges as long as they are available.
Removes edges as they are traversed to ensure each is used only once.
Backtracks when no more edges are available from the current vertex.
Output: Returns a list of edge IDs that form the Eulerian path.
- ```chinese_postman_problem``` Function:
  
Orchestrates the overall process of solving the CPP.
Input:
n: Number of vertices.
e: Number of edges.
edges: List of tuples (edge_id, u, v, cost) representing each edge's ID, vertices, and weight.
start: Starting vertex for the Eulerian path.
Process:
Builds the graph with an adjacency list, edge counts, and distance matrix (dist).
Uses the Floyd-Warshall algorithm to compute the shortest path between all pairs of vertices, populating dist.
Identifies odd-degree vertices.
Calls min_cost_matching to find the minimum additional cost required to make the graph Eulerian.
Adds this cost to the original graph's edge costs to get the total cost.
Calls find_eulerian_path to generate the route of the Eulerian circuit.
Output:
Prints the total cost of the CPP solution.
Prints the Eulerian path as a sequence of edge IDs.
- ```main``` Function:
  
Handles input and invokes the ```chinese_postman_problem``` function.
Reads the number of vertices n, number of edges e, and the list of edges.
Input Format:
Each edge is given as ```edge_id u v``` cost, where ```edge_id``` uniquely identifies the edge between vertices u and v with a weight cost.
Reads the start vertex for the path and calls the ```chinese_postman_problem```.
