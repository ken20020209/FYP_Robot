import Controller
from DOGZILLALib import DOGZILLA

class Movement(Controller.MovementController):
    def __init__(self):
        self.g_dog = DOGZILLA()
        print("Movement Controller Initialized")
    def forward(self, speed):
        self.g_dog.forward(speed)
    def back(self, speed):
        self.g_dog.back(speed)
    def left(self, speed):
        self.g_dog.left(speed)
    def right(self, speed):
        self.g_dog.right(speed)
    def turnLeft(self, speed):
        self.g_dog.turnleft(speed)
    def turnRight(self, speed):
        self.g_dog.turnright(speed)


if __name__ == "__main__":
    movement = Movement()
    movement.moveForward(50)
    # movement.moveBack(50)
    # movement.moveSideLeft(50)
    # movement.moveSideRight(50)
    # movement.rotateLeft(50)
    # movement.rotateRight(50)
    movement.stop()