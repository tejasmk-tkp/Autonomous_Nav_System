class Agent:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.path = [(0, 0)]

    def set_start_point(self, coordinates):
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.path[0] = coordinates
    
    def get_start_point(self):
        return self.path[0]

    def move(self, direction, steps=1):
        for i in range(steps):
            if direction == 'forward':
                self.y += 1
            elif direction == 'backward':
                self.y -= 1
            elif direction == 'left':
                self.x -= 1
            elif direction == 'right':
                self.x += 1
            else:
                return None

            self.path.append((self.x, self.y))
    
    def get_current_position(self):
        return self.path[-1]
    
    def get_followed_path(self):
        return self.path

    def backtrack_path(self, target):
        try:
            index = self.path.index(target)
            reversed_path = self.path[index:-1]
            reversed_path.append(self.get_current_position())
            reversed_path = reversed_path[::-1]
            return reversed_path
        except ValueError:
            return None
