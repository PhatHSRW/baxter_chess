import cv2
import numpy as np


class LineClass:

    def __init__(self, x1, y1, x2, y2):
        "End Points and gradient of line"
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

        self.dx = self.x2 - self.x1
        self.dy = self.y2 - self.y1

#         if abs(self.dx) > abs(self.dy):
#             self.orientation = "horizontal"
#         else: self.orientation = "vertical"

        if abs(self.dy) < 20:
            self.orientation = "horizontal"
        elif abs(self.dx) < 20:
            self.orientation = "vertical"
        else: self.orientation = "none"

    def draw(self, image, color=(0,0,255), thickness=3):
        cv2.line(image, (self.x1, self.y1), (self.x2, self.y2), color, thickness)

    def find_intersection(self, other):
        "Finds intersection of this line and other. One line must be horizontal and the other must be vertical"
        x = ((self.x1*self.y2 - self.y1*self.x2)*(other.x1-other.x2) - (self.x1-self.x2)*(other.x1*other.y2 - other.y1*other.x2))/ ((self.x1-self.x2)*(other.y1-other.y2) - (self.y1-self.y2)*(other.x1-other.x2))
        y = ((self.x1*self.y2 - self.y1*self.x2)*(other.y1-other.y2) - (self.y1-self.y2)*(other.x1*other.y2 - other.y1*other.x2))/ ((self.x1-self.x2)*(other.y1-other.y2) - (self.y1-self.y2)*(other.x1-other.x2))
        x = int(x)
        y = int(y)

        return x,y
