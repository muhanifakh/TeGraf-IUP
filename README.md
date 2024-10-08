# Group 10 Graph Theory Week 7 Project

| Name           | NRP        |
| ---            | ---        |
| Reino Yuris Kusumanegara | 5025231075 | 
| Muhammad Rizqy Hidayat| 5025231161 |
| Muhammad Hanif Fakriansyah| 5025231082 |
| Fazle Robby Pratama| 5025 |

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


# Group 10 Graph Theory Week 7 Project

| Name           | NRP        |
| ---            | ---        |
| Reino Yuris Kusumanegara | 5025231075 | 
| Muhammad Rizqy Hidayat| 5025231161 |
| Muhammad Hanif Fakriansyah| 5025231082 |
| Fazle Robby Pratama| 5025 |

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

`import random

# Get inputs for the board size and starting position
board_size = tuple(map(int, input("Enter the board size (e.g., '5 5'): ").split()))
start_position = tuple(map(int, input("Enter the starting position of the knight (e.g., '2 2'): ").split()))

# Possible moves of a knight
knight_moves = [
    (2, 1), (1, 2), (-1, 2), (-2, 1),
    (-2, -1), (-1, -2), (1, -2), (2, -1)
]

# Function to check if a move is within the board and unvisited
def is_valid_move(pos, visited, board_size):
    x, y = pos
    return 0 <= x < board_size[0] and 0 <= y < board_size[1] and pos not in visited

# Function to generate a random knight's tour using backtracking
def random_knights_tour(current_position, visited, board_size):
    if len(visited) == board_size[0] * board_size[1]:
        # All squares have been visited exactly once; this is a valid tour.
        return visited

    # Shuffle the possible knight moves to add randomness
    random.shuffle(knight_moves)

    x, y = current_position
    for move in knight_moves:
        next_pos = (x + move[0], y + move[1])
        if is_valid_move(next_pos, visited, board_size):
            visited.append(next_pos)
            result = random_knights_tour(next_pos, visited, board_size)
            if result:
                return result
            visited.pop()  # Backtrack if the path didn't work out

    return None
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

<br>4. Generate a Random Knight's Tour <br> The `random_knights_tour` function attempts to find a complete knight's tour using backtracking:

- The knight moves through all possible positions in random order.
- If a move is valid, it is added to the list of visited positions.
- If a dead end is reached, the function backtracks by removing the last move.
```
def random_knights_tour(current_position, visited, board_size):
    if len(visited) == board_size[0] * board_size[1]:
        return visited
    random.shuffle(knight_moves)
    ...
```

<br>5. Start the Tour <br> Begins the knight's tour from the user-specified starting position and finds a valid path, if possible.

<br>6. Output the Path <br> If a complete tour is found, the sequence of moves is printed. Otherwise, a message indicates that the tour could not be completed.

- Example:
```
Enter the board size (e.g., '5 5'): 5 5
Enter the starting position of the knight (e.g., '2 2'): 2 2
The knight successfully visits all squares with a randomized tour:
Move 1: (2, 2)
Move 2: (4, 3)
Move 3: (2, 4)
Move 4: (0, 3)
...

```

- Conclusion

This implementation of the Knight's Tour problem uses a randomized backtracking approach to find a solution. It takes the board size and starting position as input, allowing for flexibility in the problem setup. The solution attempts to find a path where the knight visits each square exactly once, but the random nature of the solution means it may not always succeed in finding a complete tour, especially on smaller boards or with less favorable starting conditions. Nonetheless, it provides an interesting approach to the classic problem, combining elements of randomness and algorithmic backtracking.
