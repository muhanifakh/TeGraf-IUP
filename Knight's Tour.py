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
