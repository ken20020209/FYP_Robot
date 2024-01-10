from abc import ABC, abstractmethod

class MovementController(ABC):
    @abstractmethod
    def forward(self, speed):
        return NotImplemented
    @abstractmethod
    def back(self, speed):
        return NotImplemented
    @abstractmethod
    def left(self, speed):
        return NotImplemented
    @abstractmethod
    def right(self, speed):
        return NotImplemented
    @abstractmethod
    def turnLeft(self, speed):
        return NotImplemented
    @abstractmethod
    def turnRight(self, speed):
        return NotImplemented
    def stop(self):
        return NotImplemented
    
class NavigationController(ABC):
    @abstractmethod
    def moveToPoint(self, x, y):
        return NotImplemented
    @abstractmethod
    def patrolPoints(self, x, y):
        return NotImplemented
    @abstractmethod
    def stopNavigation(self, x, y):
        return NotImplemented
    @abstractmethod
    def followObject(self, obj):
        return NotImplemented
    @abstractmethod
    def getMap(self):
        return NotImplemented

class VoiceController(ABC):
    @abstractmethod
    def say(self, text):
        return NotImplemented
    @abstractmethod
    def listen(self):
        return NotImplemented