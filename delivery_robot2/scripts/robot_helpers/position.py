class PositionManager:
    def __init__(self):
        self.position = [0.0, 0.0]

    def get_position(self):
        return self.position[:]

    def update_position(self, new_position):
        self.position[:] = new_position[:]