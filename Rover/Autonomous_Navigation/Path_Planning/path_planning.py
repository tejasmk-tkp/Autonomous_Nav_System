import matplotlib.pyplot as plt
import heapq

def grassfire(grid, start, destination):
    
    #Compute grid dimensions
    rows = len(grid)
    cols = len(grid[0])

    queue = [(start, 0)]    #Stores cells to be processed in form (cell, distance to target)
    visited = set()     #Stores data of cells already visited
    visited.add(start)

    while queue:
        current, distance = queue.pop(0)    #pop the cell and its distance and store in respective variables

        if current == destination:
            break   #stop when destination reached
        
        #Define adjacent cells (up, down, left, right)
        neighbours = [
                (current[0] - 1, current[1]),   #Up
                (current[0] + 1, current[1]),   #Down
                (current[0], current[1] - 1),   #Left
                (current[0], current[1] + 1)    #Right
                ]

        for neighbour in neighbours:
            row, col = neighbour
            if 0 <= row < rows and 0 <= col < cols and grid[row][col] == 0 and neighbour not in visited:    #Check if neighbour is within grid boundaries, is traversable and not visited
                visited.add(neighbour)
                grid[row][col] = distance + 1   #Assign distance from starting point
                queue.append((neighbour, distance + 1))     #Add neighbour to the queue with updated distance

    def extract_path(grid, start, destination):
    
        destination_value = grid[destination[0]][destination[1]]
        if destination_value <= 0:
            path=[]
            #print('path not found')
            return path

        else:
            path = [destination]    #Initialize list path with destination as its first member
            current = destination   #Set current position as destination
            while current != start:
                row, col = current
                current_val = grid[row][col]    #retrive distance of current cell
                neighbours = [
                    (row - 1, col),     #Up
                    (row + 1, col),     #Down
                    (row, col - 1),     #Left
                    (row, col + 1)      #Right
                    ]

                for neighbour in neighbours:
                    n_row, n_col = neighbour
                    if 0 <= n_row < len(grid) and 0 <= n_col < len(grid[0]):    #check if neighbour falls within grid
                        if grid[n_row][n_col] == current_val - 1:
                            path.append(neighbour)
                            current = neighbour
                            break
    
            return path[::-1]

    path = extract_path(grid, start, destination)
    return path

def visualize_path(grid, path, start, destination):
    rows = len(grid)
    cols = len(grid[0])

    fig, ax = plt.subplots()

    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == -1: #Plot obstacles in red
                ax.add_patch(plt.Rectangle((c, r), 1, 1, color = 'black'))
            elif (r, c) in path and (r, c) != start and (r, c) != destination:    #Plot path in blue
                ax.add_patch(plt.Rectangle((c, r), 1, 1, color = 'blue'))
            elif (r, c) == start:   #Mark start in black
                ax.add_patch(plt.Rectangle((c, r), 1, 1, color = 'green'))
            elif (r, c) == destination: #Mark destination in green
                ax.add_patch(plt.Rectangle((c, r), 1, 1, color = 'red'))
            else:
                ax.add_patch(plt.Rectangle((c, r), 1, 1, color = 'white'))

    ax.set_aspect('equal')
    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)

    plt.grid(True)
    plt.show()

def dijkstra(grid, start, destination):
    rows = len(grid)
    cols = len(grid[0])
    
    def edge_weight(node1, node2):
        return 1    #Assuming uniform edge weight

    distances = {node: float('inf') for row in grid for node in row}
    distances[start] = 0
    previous = {}
    queue = [(0, start)]

    directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]

    while queue:
        current_distance, current_node = heapq.heappop(queue)

        if current_node == destination:
            break

        if current_distance > distances[current_node]:
            continue

        for dx, dy in directions:
            new_x = current_node[0] + dx
            new_y = current_node[1] + dy

            if 0 <= new_x < rows and 0 <= new_y < cols and grid[new_x][new_y] == 0:
                neighbour = (new_x, new_y)
                tentative_distance = distances[current_node] + edge_weight(current_node, neighbour)

                if neighbour not in distances:
                    distances[neighbour] = float('inf')

                if tentative_distance < distances[neighbour]:
                    distances[neighbour] = tentative_distance
                    previous[neighbour] = current_node
                    heapq.heappush(queue, (tentative_distance, neighbour))

    path = []
    node = destination
    while node in previous:
        path.append(node)
        node = previous[node]
    path.append(start)
    path.reverse()

    return path



'''grid = [[0, 0, 0, 0], [-1, -1, -1, 0], [0, -1, 0, 0]]    # 0 represents traversable path and -1 represents and obstacle
start = (0, 0)
destination = (2, 2)

result = grassfire_algorithm(grid, start, destination)

for row in result:
    print(row)

path = extract_path(result, start, destination)

for row in path:
    print(row)

visualize_path(grid, path, start, destination)'''
