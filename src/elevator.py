from vex import *
class Elevator:
    def __init__(self, port_number):
        self.solenoid = Pneumatics(port=port_number)

    def extend(self):
        self.solenoid.open()

    def retract(self):
        self.solenoid.close()

brain = Brain()
controller = Controller()
elevator = Elevator(brain.three_wire_port.g)
controller.buttonA.pressed(lambda: elevator.extend())
controller.buttonB.pressed(lambda: elevator.retract())
        
