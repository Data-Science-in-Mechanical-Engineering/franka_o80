#!/bin/python

import re
import math
import sys
import o80
import franka_o80

def strtod(s, pos):
	m = re.match(r'[+-]?\d*[.]?\d*(?:[eE][+-]?\d+)?', s[pos:])
	if m.group(0) == '': return 0, pos
	return float(m.group(0)), pos + m.end()

class Control:
	@staticmethod
	def help():
		print("Welcome to franka_o80 control!\n")
		print("The program is created to control backends")
		print("\n")
		print("Usage:")
		print("./control ID\n")
		print("Where ID is backend identifier\n")
		print("\n")
		print("Control loop syntax: [command [+/-/= value]]+ [# comment]\n" )
		print("Possible commands:\n")
		print("   1..7     - joints 1..7\n")
		print("   x, y, z  - X, Y and Z coordinates\n"  )
		print("   u, v, w  - Roll, pitch, yaw Euler angles\n")
		print("   g        - gripper width\n")
		print("   t        - execution time\n")
		print("   e        - echo\n")
		print("   q        - quit\n")
	
	def __init__(self, segment_id):
		self.front = franka_o80.FrontEnd(segment_id)
		self.front.add_command(franka_o80.control_mode, franka_o80.Mode.intelligent_position, o80.Duration_us.seconds(1.0), o80.Mode.QUEUE)
		self.front.pulse_and_wait()
		self.execution_time = 5.0
		self.quit_request = False

	def echo(self):
		self.front.reset_next_index()
		states = front.wait_for_next().get_observed_states()
		#General
		print("control_mode : ", states.get(franka_o80.control_mode).to_string())
		print("control_error: ", states.get(franka_o80.control_error).to_string())
		print("control_reset: ", states.get(franka_o80.control_reset).get())

		#Gripper
		print("gripper_width: ", states.get(franka_o80.gripper_width).get())
		print("gripper_temperature  : ", states.get(franka_o80.gripper_temperature).get())

		#Robot joints
		print("joint_position   :")
		for i in range(7):
			print(" ", states.get(franka_o80.joint_position(i)).get())
		print("\n")

		print("joint_torque :")
		for i in range(7):
			print(" ", states.get(franka_o80.joint_torque(i)).get())
		print("\n")

		#Robot cartesian
		print("cartesian_position   :")
		for i in range(3):
			print(" ", states.get(franka_o80.cartesian_position(i)).get())
		print("\n")

		print("cartesian_orientation:")
		for i in range(3):
			print(" ", 180.0 * states.get(franka_o80.cartesian_orientation(i)).get() / math.pi)
		print("\n")

		#Time
		print("time : ", self.execution_time)

	def execute(self, expression):
		if len(expression) == 0:
			return
		i = 0
		command = '\0'
		sign = '\0'
		state = 'c'

		while True:
			#Wait for [C]ommand state
			if state == 'c':
				if i == len(expression):
					return
				elif expression[i] == '#':
					return
				elif expression[i] == ' ' || expression[i] == '\t':
					i = i + 1
				elif expression[i] == 'e':
					echo()
					i = i + 1
				elif expression[i] == 'q':
					self.quit_request = True
					return
				elif "1234567 xyz uvw g t".find(expression[i]) != -1:
					state = 's'
					command = expression[i]
					i = i + 1
				else:
					help()
					return

			#Wait for [S]ign state
			elif state == 's':
				if expression[i] == ' ' || expression[i] == '\t':
					i = i + 1
				elif "+-=".find(expression[i]) != -1:
					state = 'v'
					sign = expression[i]
					i = i + 1
				else:
					help()
					return
			
			#Wait for [V]alue
			else:
				if expression[i] == ' ' || expression[i] == '\t':
					i = i + 1
					continue
				value, ni = strtod(expression, i)
				if ni == i:
					help()
					return
				state = 'c'
				i = ni
				self.front.reset_next_index()
				states = self.front.wait_for_next().get_observed_states()
				if "1234567".find(expression[i]) != -1:
					value = math.pi * value / 180.0
					actuator = franka_o80.joint_position(ord(command) - ord('1'))
					if sign == '+': states.set(actuator, states.get(actuator).get() + value)
					elif sign == '-': states.set(actuator, states.get(actuator).get() - value)
					else: states.set(actuator, value)
					self.front.add_command(actuator, states.get(actuator).get(), o80.Duration_us.seconds(self.execution_time), o80.Mode.QUEUE)
				elif "xyz".find(expression[i] != -1:
					actuator = franka_o80.cartesian_position(ord(command) - ord('x'))
					if sign == '+': states.set(actuator, states.get(actuator).get() + value)
					elif sign == '-': states.set(actuator, states.get(actuator).get() - value)
					else: states.set(actuator, value)
					franka_o80.cartesian_to_joint(states)
					for j in range(7):
						self.front.add_command(franka_o80.joint_position(j), states.get(franka_o80.joint_position(i)).get(), o80.Duration_us.seconds(self.execution_time), o80.Mode.QUEUE)
				elif "uvw".find(expression[i]) != -1:
					value = math.pi * value / 180.0
					actuator = franka_o80.cartesian_position(ord(command) - ord('u'))
					if sign == '+': states.set(actuator, states.get(actuator).get() + value)
					elif sign == '-': states.set(actuator, states.get(actuator).get() - value)
					else: states.set(actuator, value)
					franka_o80.cartesian_to_joint(states)
					for j in range(7):
						self.front.add_command(franka_o80.joint_position(j), states.get(franka_o80.joint_position(i)).get(), o80.Duration_us.seconds(self.execution_time), o80.Mode.QUEUE)
				elif expression[i] == 'g' != -1:
					actuator = franka_o80.gripper_width
					if sign == '+': states.set(actuator, states.get(actuator).get() + value)
					elif sign == '-': states.set(actuator, states.get(actuator).get() - value)
					else: states.set(actuator, value)
					self.front.add_command(actuator, states.get(actuator).get(), o80.Duration_us.seconds(self.execution_time), o80.Mode.QUEUE)
				else:
					if sign == '+': self.execution_time = self.execution_time + value
					elif sign == '-': self.execution_time = self.execution_time - value
					else: self.execution_time = value
					if (self.execution_time < 1.0) self.execution_time = 1.0

	def loop(self):
		while (!self.quit_request):
			execute(input())
			self.front.pulse_and_wait()

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
