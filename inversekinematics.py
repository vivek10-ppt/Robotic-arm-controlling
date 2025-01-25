import serial
import time
import numpy as np
from scipy.optimize import fsolve

# Configure the serial port
serial_port = serial.Serial('COM4', 9600, timeout=1)  # Adjust COM port as per your setup
time.sleep(2)  # Allow time for serial communication to establish

def move_motors(steps1, steps2, steps3):
    command = f"<{steps1},{steps2},{steps3}>\n"
    serial_port.write(command.encode())
    print(f"Sent command: {command.strip()}")

    response = serial_port.readline().decode().strip()
    print(f"Arduino response: {response}")

def ik(x, y, z):
    # Define the equations based on the inverse kinematics equations
    def equations(vars, P, Q, X, Z):
        A, B = vars
        eq1 = P * np.cos(A) + Q * np.sin(B) - X
        eq2 = P * np.sin(A) - Q * np.cos(B) - Z
        return [eq1, eq2]

    # Initialize the known values
    P = 40  # Replace with actual value
    Q = 30  # Replace with actual value
    d = np.sqrt(x**2 + y**2)  # Replace with actual value

    # Initial guesses for A and B
    initial_guesses = [0.1, 0.1]  # Non-zero initial guesses to avoid trivial solutions

    # Solve the equations
    solution = fsolve(equations, initial_guesses, args=(P, Q, d, z))
    A, B = solution
    A = ((A * 180) / np.pi)//(360/1600)
    B = ((B * 180) / np.pi)//(360/1600)
    C = ((np.arctan(y / x) * 180) / np.pi)//(360/1600)
    return A, B, C

x = 20
y = 30
z = 30

# Example usage:
steps_to_rotate1, steps_to_rotate2, steps_to_rotate3 = ik(x, y, z)
move_motors(-400,400,400)
time.sleep(2)  # Allow time for motors to move

serial_port.close()  # Close the serial port when done