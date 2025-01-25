import numpy as np
from scipy.optimize import fsolve
def ik(x, y, z):
    # Define the equations based on the inverse kinematics equations
    def equations(vars, P, Q, X, Z):
        A, B = vars
        eq1 = P * np.cos(A) + Q * np.sin(B) - X
        eq2 = P * np.sin(A) - Q * np.cos(B) - Z
        return [eq1, eq2]

    # Initialize the known values
    P = 40  # Replace with actual value
    Q = 30 # Replace with actual value
    d = np.sqrt(x**2 + y**2)  # Replace with actual value

    # Initial guesses for A and B
    initial_guesses = [0.1, 0.1]  # Non-zero initial guesses to avoid trivial solutions

    # Solve the equations
    solution = fsolve(equations, initial_guesses, args=(P, Q, d, z))
    A, B = solution
    A = ((A * 180) / np.pi)
    B = ((B * 180) / np.pi)
    B= 90+A-B
    C = ((np.arctan(y / x) * 180) / np.pi)
    return A, B, C
array=np.zeros((100,3))
angles=np.zeros((100,3))

for i in range(100):
    array[i,0]=np.random.randint(1,20)
    array[i,1]=np.random.randint(1,40)
for i in range(100):
    x,y,z=ik(array[i,0],array[i,1],array[i,2])
    angles[i,0]=int(x)
    angles[i,1]=int(y)
    angles[i,2]=int(z)
print(angles)    