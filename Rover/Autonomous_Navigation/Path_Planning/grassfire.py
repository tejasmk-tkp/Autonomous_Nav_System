import matplotlib.pyplot as plt

def grassfire_algorithm(grid, start, destination):
    
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

    return grid

def extract_path(resultant_grid, start, destination):
    
    destination_value = resultant_grid[destination[0]][destination[1]]
    if destination_value <= 0:
        path=[]
        #print('path not found')
        return path

    else:
        path = [destination]    #Initialize list path with destination as its first member
        current = destination   #Set current position as destination
        while current != start:
            row, col = current
            current_val = resultant_grid[row][col]    #retrive distance of current cell
            neighbours = [
                (row - 1, col),     #Up
                (row + 1, col),     #Down
                (row, col - 1),     #Left
                (row, col + 1)      #Right
                ]

            for neighbour in neighbours:
                n_row, n_col = neighbour
                if 0 <= n_row < len(resultant_grid) and 0 <= n_col < len(resultant_grid[0]):    #check if neighbour falls within grid
                    if resultant_grid[n_row][n_col] == current_val - 1:
                        path.append(neighbour)
                        current = neighbour
                        break
    
    return path[::-1]

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
