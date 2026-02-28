from abc import ABC, abstractmethod
#This is so we can use the same code for sims and real motors
class MotorInterface(ABC):
    @abstractmethod
    def setVelocity(self,velocity:float):
        pass
