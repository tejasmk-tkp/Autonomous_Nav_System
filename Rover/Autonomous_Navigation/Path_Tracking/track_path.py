import numpy as np

def generate_cylindrical_grid(linear_end, linear_increment, circular_increment):
    linear_range = np.arange(0, linear_end, linear_increment)
    circular_range = np.arange(0, 2*np.pi, circular_increment)

    linear_grid, circular_grid = np.meshgrid(linear_range, circular_range)

    #circular_grid[-1, :] = circular_grid[0, :]

    cylindrical_grid = np.column_stack((linear_grid.flatten(), circular_grid.flatten()))

    return (cylindrical_grid, linear_increment, circular_increment)

class Agent:
    def __init__(self, grid):
        self.x = 0
        self.y = 0
        self.path = [(0, 0)]
        self.grid = grid[0]
        self.x_increment = grid[1]
        self.y_increment = grid[2]

    def set_start_point(self, coordinates):
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.path[0] = coordinates
    
    def get_start_point(self):
        return self.path[0]
    
    def get_agent_grid(self):
        return self.grid

    def move(self, direction, steps=1):
        for i in range(steps):
            try:
                if direction == 'forward':
                    new_x = self.x + self.x_increment
                    new_y = self.y + 0
                elif direction == 'backward':
                    new_x = self.x - self.x_increment
                    new_y = self.y + 0
                elif direction == 'left':
                    new_x = self.x + 0
                    new_y = self.y - self.y_increment
                elif direction == 'right':
                    new_x = self.x + 0
                    new_y = self.y + self.y_increment
            except ValueError:
                print("Use 'forward', 'backward', 'left', or 'right'")

            if (new_x, new_y) in self.grid.tolist():
                self.x = new_x
                self.y = new_y
                self.path.append((self.x, self.y))

            else:
                print("Agent reached the edge of the grid")
                return None
    
    def get_current_position(self):
        return self.path[-1]
    
    def get_followed_path(self):
        return self.path

    def backtrack_path(self, target):
        if target in self.grid.tolist():
            target_x, target_y = target

            while self.x != target_x:
                if self.x < target_x:
                    self.move('forward')
                else:
                    self.move('backward')

            while self.y != target_y:
                if self.y < target_y:
                    self.move('right')
                else:
                    self.move('left')
        else:
            print("Target not in path followed")
            return None
