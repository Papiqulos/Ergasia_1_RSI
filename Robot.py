import numpy as np



class Robot:
    def __init__(self, width, height, x, y):
        self.width = width
        self.height = height
        self.x = x
        self.y = y
        self.anchor = np.array([x, y])

        point1 = self.anchor
        point2 = self.anchor + np.array([0., self.height])
        point3 = self.anchor + np.array([self.width, self.height])
        point4 = self.anchor + np.array([self.width, 0])
        self.points = [point1, point2, point3, point4]