#!/bin/python3

import re
import math
import sys
import time
import numpy as np
import o80
import franka_o80

def strtod(s, pos):
    m = re.match(r'[+-]?\d*[.]?\d*(?:[eE][+-]?\d+)?', s[pos:])
    if m.group(0) == '': return 0, pos
    return float(m.group(0)), pos + m.end()

def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    r = np.zeros(4, dtype = np.float64)
    r[0] = -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0
    r[1] = x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0
    r[2] = -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0
    r[3] = x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0
    return r

def quaternion_inverse(quaternion):
    w, x, y, z = quaternion
    r = np.zeros(4, dtype = np.float64)
    r[0] = w
    r[1] = -x
    r[2] = -y
    r[3] = -z
    return r

class Control:
    @staticmethod
    def help():
        print("Welcome to franka_o80 control!")
        print("The program is created to control franka_o80 backend")
        print("\n")

        print("Usage:")
        print("./control ID")
        print("Where ID is backend identifier")
        print("\n")

        print("Possible commands:" )
        print("1..7  - Joints 1 to 7.            Syntax: 1..7  +/-/= degree")
        print("x/y/z - X, Y and Z coordinates.   Syntax: x/y/z +/-/= meter")
        print("q     - Rotation in quaterions.   Syntax: q     +/-/= w      x      y      z")
        print("r     - Rotation in Euler angles. Syntax: r     +/-/= degree degree degree")
        print("g     - Gripper width.            Syntax: g     +/-/= meter")
        print("k     - Gripper force.            Syntax: k     +/-/= newton")
        print("t     - Execution time.           Syntax: t     +/-/= second")
        print("i     - Impedances.               Syntax: i     +/-/= joint  trans  rot")
        print("d     - Default position.         Syntax: d")
        print("p     - Pass.                     Syntax: p")
        print("e     - Echo.                     Syntax: e")
        print("f     - Finish.                   Syntax: f")
        print("#     - Comment.")

    def __init__(self, segment_id):
        self.finish_ = False
        self.execution_time_ = 5.0
        self.front_ = franka_o80.FrontEnd(segment_id)
        self.front_.add_command(franka_o80.control_mode, franka_o80.State(franka_o80.Mode.intelligent_position), o80.Mode.QUEUE)
        self.front_.reset_next_index()
        self.oldtarget_ = self.front_.wait_for_next().get_observed_states()
        self.commands_ = set()
        self.impedances_ = dict()
        self.impedances_[0] = self.oldtarget_.get(franka_o80.joint_stiffness(0)).get_real() / franka_o80.default_states().get(franka_o80.joint_stiffness(0)).get_real()
        self.impedances_[1] = self.oldtarget_.get(franka_o80.cartesian_stiffness(0)).get_real() / franka_o80.default_states().get(franka_o80.cartesian_stiffness(0)).get_real()
        self.impedances_[2] = self.oldtarget_.get(franka_o80.cartesian_stiffness(3)).get_real() / franka_o80.default_states().get(franka_o80.cartesian_stiffness(3)).get_real()

    def commands_count(self, commands):
        for c in commands:
            if c != ' ' and c in self.commands_: return True
        return False

    def commands_insert(self, commands):
        for c in commands:
            if c != ' ': self.commands_.add(c)

    def echo(self):
        self.front_.reset_next_index()
        states = self.front_.wait_for_next().get_observed_states()

        #General
        print("control_mode         : ", states.get(franka_o80.control_mode).to_string())
        print("control_error        : ", states.get(franka_o80.control_error).to_string())
        print("control_reset        : ", states.get(franka_o80.control_reset).get_real())
        print("execution time       : ", self.execution_time_)

        #Gripper
        print("gripper_width        : ", states.get(franka_o80.gripper_width).get_real())
        print("gripper_temperature  : ", states.get(franka_o80.gripper_temperature).get_real())
        print("gripper_force        : ", states.get(franka_o80.gripper_force).get_real())

        #Robot joints
        print("joint_position       :")
        for i in range(7): print(" ", 180.0 * states.get(franka_o80.joint_position(i)).get_real() / math.pi)
        print("\n")
        print("joint_torque         :")
        for i in range(7): print(" ", states.get(franka_o80.joint_torque(i)).get_real())
        print("\n")
        print("joint_impedance      :")
        print(self.impedances_[0])

        #Robot cartesian
        print("cartesian_position   :")
        for i in range(3): print(" ", states.get(franka_o80.cartesian_position(i)).get_real())
        print("\n")
        print("cartesian_orientation:")
        for i in range(4): print(" ", states.get(franka_o80.cartesian_orientation).get_wxyz()[i])
        print(" (")
        for i in range(3): print(" ", 180.0 * states.get(franka_o80.cartesian_orientation).get_euler()[i] / math.pi)
        print(" )" )
        print("cartesian_impedance  :")
        print(self.impedances_[1], " ", self.impedances_[2])

    def command_pass(self):
        #Check contradictions
        if self.commands_count("1234567 xyzq p"):
            print("Сontradictory command")
            return
        #Add commands
        self.commands_insert("p")

    def command_default(self):
        #Check contradictions
        if self.commands_count("1234567 xyzq p"):
            print("Сontradictory command")
            return
        #Add commands
        self.commands_insert("xyzq")
        for i in range(3): self.newtarget_.set(franka_o80.cartesian_position(i), franka_o80.default_states().get(franka_o80.cartesian_position(i)))
        self.newtarget_.set(franka_o80.cartesian_orientation, franka_o80.default_states().get(franka_o80.cartesian_orientation))

    def command_joint_position(self, command, sign, value):
        #Check contradictions
        if (self.commands_count("xyzq p") or self.commands_count(command)):
            print("Сontradictory command")
            return
        #Create actuator state
        actuator = franka_o80.joint_position(ord(command) - ord('1'))
        state = franka_o80.State()
        if sign == '+': state.set_real(self.newtarget_.get(actuator).get_real() + math.pi * value / 180.0)
        elif sign == '-': state.set_real(self.newtarget_.get(actuator).get_real() - math.pi * value / 180.0)
        else: state.set_real(math.pi * value / 180.0)
        #Add command
        self.commands_insert(command)
        self.newtarget_.set(actuator, state)

    def command_cartesian_position(self, command, sign, value):
        #Check contradictions
        if self.commands_count("1234567 p") or self.commands_count(command):
            print("Сontradictory command")
            return
        #Create state
        actuator = franka_o80.cartesian_position(ord(command) - ord('x'))
        state = franka_o80.State()
        if sign == '+': state.set_real(self.newtarget_.get(actuator).get_real() + value)
        elif sign == '-': state.set_real(self.newtarget_.get(actuator).get_real() - value)
        else: state.set_real(value)
        #Add command
        self.commands_insert(command)
        self.newtarget_.set(actuator, state)

    def command_cartesian_orientation(self, command, sign, values):
        #Check contradictions
        if self.commands_count("1234567 p") or self.commands_count("q"):
            print("Сontradictory command")
            return
        #Create state
        state = franka_o80.State()
        if command == 'q':
            wxyz = np.zeros(4, dtype = np.float64)
            for i in range(4): wxyz[i] = values[i]
            state.set_wxyz(wxyz)
        else:
            euler = np.zeros(3, dtype = np.flaot64)
            for i in range(3): euler[i] = math.pi * values[i] / 180
            state.set_euler(euler)
        
        if sign == '+': state.set_wxyz(quaternion_multiply(state.get_wxyz(), self.newtarget_.get(franka_o80.cartesian_orientation).get_wxyz()))
        elif sign == '-': state.set_wxyz(quaternion_multiply(quaternion_inverse(state.get_wxyz()), self.newtarget_.get(franka_o80.cartesian_orientation).get_wxyz()))
        #Add command
        self.commands_insert("q")
        self.newtarget_.set(franka_o80.cartesian_orientation, state)

    def command_gripper_width(self, command, sign, value):
        #Check contradictions
        if (self.commands_count("g p")):
            print("Сontradictory command")
            return
        #Create state
        state = franka_o80.State()
        if sign == '+': state.set_real(self.newtarget_.get(franka_o80.gripper_width).get_real() + value)
        elif sign == '-': state.set_real(self.newtarget_.get(franka_o80.gripper_width).get_real() - value)
        else: state.set_real(value)
        #Add command
        self.commands_insert("g")
        self.newtarget_.set(franka_o80.gripper_width, state)

    def command_gripper_force(self, command, sign, value):
        #Create state
        state = franka_o80.State()
        if sign == '+': state.set_real(self.newtarget_.get(franka_o80.gripper_force).get_real() + value)
        elif sign == '-': state.set_real(self.newtarget_.get(franka_o80.gripper_force).get_real() - value)
        else: state.set_real(value)
        #Add command
        self.newtarget_.set(franka_o80.gripper_force, state)
        self.front_.add_command(franka_o80.gripper_force, state, o80.Mode.QUEUE)

    def command_impedance(self, command, sign, values):
        if sign == '+':
            for i in range(3): self.impedances_[i] += values[i]
        elif sign == '-':
            for i in range(3): self.impedances_[i] -= values[i]
        else:
            for i in range(3): self.impedances_[i] = values[i]
        
        for i in range(3):
            if self.impedances_[i] < 0.1: self.impedances_[i] = 0.1
        
        for i in range(7):
            self.front_.add_command(franka_o80.joint_stiffness(i), franka_o80.State(franka_o80.default_states().get(franka_o80.joint_stiffness(i)).get_real() * self.impedances_[0]), o80.Mode.QUEUE)
            self.front_.add_command(franka_o80.joint_damping(i), franka_o80.State(franka_o80.default_states().get(franka_o80.joint_damping(i)).get_real() * math.sqrt(self.impedances_[0])), o80.Mode.QUEUE)
        
        for i in range(3):
            self.front_.add_command(franka_o80.cartesian_stiffness(i), franka_o80.State(franka_o80.default_states().get(franka_o80.cartesian_stiffness(i)).get_real() * self.impedances_[1]), o80.Mode.QUEUE)
            self.front_.add_command(franka_o80.cartesian_damping(i), franka_o80.State(franka_o80.default_states().get(franka_o80.cartesian_damping(i)).get_real() * math.sqrt(self.impedances_[1])), o80.Mode.QUEUE)
            self.front_.add_command(franka_o80.cartesian_stiffness(i + 3), franka_o80.State(franka_o80.default_states().get(franka_o80.cartesian_stiffness(i + 3)).get_real() * self.impedances_[2]), o80.Mode.QUEUE)
            self.front_.add_command(franka_o80.cartesian_damping(i + 3), franka_o80.State(franka_o80.default_states().get(franka_o80.cartesian_damping(i + 3)).get_real() * math.sqrt(self.impedances_[2])), o80.Mode.QUEUE)
    
    def execute(self, expression):
        if len(expression) == 0: return
        p = 0
        command = '\0'
        sign = '\0'
        values = dict()
        parser = "wait_command"

        while (True):
            if parser == "wait_command":
                if p == len(expression) or expression[p] == '#':
                    return
                elif expression[p] == '\t' or expression[p] == ' ':
                    p += 1
                elif expression[p] == 'd':
                    p += 1
                    self.command_default() 
                elif expression[p] == 'e':
                    p += 1
                    self.command_echo() 
                elif expression[p] == 'p':
                    p += 1
                    self.command_pass() 
                elif expression[p] == 'f':
                    self.finish_ = True
                    return
                elif not expression[p] in "1234567 xyzqr g kti":
                    help()
                    return
                else:
                    command = expression[p]
                    p += 1
                    parser = "wait_sign"

            elif parser == "wait_sign":
                if expression[p] == ' ' or expression[p] == '\t':
                    p += 1
                elif not expression[p] in "+-=":
                    help()
                    return
                else:
                    sign = expression[p]
                    p += 1
                    parser = "wait_value1"

            elif parser == "wait_value1":
                if expression[p] == ' ' or expression[p] == '\t':
                    p += 1
                    continue
                values[0], np = strtod(expression, p)
                if (np == p):
                     help()
                     return
                p = np
                if command in "qri":
                    parser = "wait_value2"
                    continue
                elif command in "1234567": self.command_joint_position(command, sign, values[0])
                elif command in "xyz"    : self.command_cartesian_position(command, sign, values[0])
                elif command == 'g'      : self.command_gripper_width(command, sign, values[0])
                elif command == 'k'      : self.command_gripper_force(command, sign, values[0])
                else:
                    if sign == '+': self.execution_time_ += values[0]
                    elif sign == '-': self.execution_time_ -= values[0]
                    else: self.execution_time_ = values[0]
                    if (self.execution_time_ < 1.0): self.execution_time_ = 1.0
                parser = "wait_command"

            elif parser == "wait_value2":
                if expression[p] == ' ' or expression[p] == '\t':
                    p += 1
                    continue
                values[1], np = strtod(expression, p)
                if (np == p):
                    help()
                    return
                p = np
                parser = "wait_value3"
            
            elif parser == "wait_value3":
                if expression[p] == ' ' or expression[p] == '\t':
                    p += 1
                    continue
                values[2], np = strtod(expression, p)
                if (np == p):
                    help()
                    return
                p = np
                if command == 'q':
                    parser = "wait_value4"
                    continue 
                elif command == 'r': self.command_cartesian_orientation(command, sign, values)
                else: self.command_impedance(command, sign, values)
                parser = "wait_command"

            else:
                if expression[p] == ' ' or expression[p] == '\t':
                    p += 1
                    continue
                values[3], np = strtod(expression, p)
                if (np == p):
                    help()
                    return
                p = np
                self.command_cartesian_orientation(command, sign, values)
                parser = "wait_command"
    
    def loop(self):
        while not self.finish_:
            #Process text commands
            self.newtarget_ = self.oldtarget_
            expression = input()
            self.execute(expression)

            #Transition to state
            if self.commands_count("1234567"):
                for i in range(7):
                    if self.newtarget_.get(franka_o80.joint_position(i)).get_real() > franka_o80.joint_position_max(i) or self.newtarget_.get(franka_o80.joint_position(i)).get_real() < franka_o80.joint_position_min(i):
                        print("Invalid joint position")
                        break
                else:
                    for i in range(7):
                        self.front_.add_command(franka_o80.joint_position(i), self.newtarget_.get(franka_o80.joint_position(i)), o80.Duration_us.milliseconds(int(1000 * self.execution_time_)), o80.Mode.QUEUE)
                        self.oldtarget_.set(franka_o80.joint_position(i), self.newtarget_.get(franka_o80.joint_position(i)))
                    franka_o80.joint_to_cartesian(self.oldtarget_)
            
            if self.commands_count("xyzq"):
                try:                
                    franka_o80.cartesian_to_joint(self.newtarget_)
                except:                
                    print("Invalid cartesian position")
                else:
                    for i in range(7): self.front_.add_command(franka_o80.joint_position(i), self.newtarget_.get(franka_o80.joint_position(i)), o80.Duration_us.milliseconds(int(1000 * self.execution_time_)), o80.Mode.QUEUE)
                    for i in range(3): self.oldtarget_.set(franka_o80.cartesian_position(i), self.newtarget_.get(franka_o80.cartesian_position(i)))
                    self.oldtarget_.set(franka_o80.cartesian_orientation, self.newtarget_.get(franka_o80.cartesian_orientation))
            
            if self.commands_count("g"):
                if self.newtarget_.get(franka_o80.gripper_width).get_real() > 0.1 or self.newtarget_.get(franka_o80.gripper_width).get_real() < 0:
                    print("Invalid gripper position")
                else:
                    self.front_.add_command(franka_o80.gripper_width, self.newtarget_.get(franka_o80.gripper_width), o80.Duration_us.milliseconds(int(1000 * self.execution_time_)), o80.Mode.QUEUE)
                    self.oldtarget_.set(franka_o80.gripper_width, self.newtarget_.get(franka_o80.gripper_width))
            
            if self.commands_count("p"):
                time.sleep(self.execution_time_)
            
            if self.commands_count("1234567 xyzq g"):
                self.front_.pulse_and_wait()
            
            self.commands_ = set()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        Control.help()
        sys.exit(1)
    try:
        control = Control(sys.argv[1])
        control.loop()
    except:
        print("Exception occured: ", sys.exc_info()[0])
        sys.exit(1)