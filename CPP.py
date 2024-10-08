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
