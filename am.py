import numpy as np

class AerialManipulator(object):

    def __init__(self) -> None:
        self.m = 1
        self.l = 0.2



    def f(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """Function representing the dynamics via a state-space system:
           x_dot = f(x,u)
           
           @param x Current System State
           @param u Current Control Input
           
           @returns x_dot the state derivative
        """

        # Extract current system state
        q = x[:3]
        # The system is first order, i.e. the state consist of pose and
        # its derivative -> the pose derivative is simply the other state variables
        q_dot = x[3:]

        # The state accelerations are computed via the equations of motion -> Lagrange-
        # Euler formalism to find "Manipulator"-Equation
        q_ddot = np.matmul(np.linalg.inv(self.M(q)), 
                           np.matmul(self.A(q), u)
                           - self.G(q))  

        # Return the state vector derivative
        return np.concatenate((q_dot, q_ddot))

    def M(self, q: np.ndarray) -> np.ndarray:
        """Function that returns the system's state dependent mass-inertia matrix.
           
           @param q Current System State
           
           @returns M: The system's mass-inertia matrix   
        """
        # Compute Inertia around the the CoM
        # Assume thin rod around its center, i.e I = 1/12 m L^2
        I = 0.83333 * self.m * self.l**2

        # Populate the mass-inertia matrix
        M = np.array([[self.m,   0, 0],
                        [0, self.m, 0],
                        [0,      0, I]])

        return M

    def G(self, q: np.ndarray) -> np.ndarray:
        """Function that returns the state dependent gravity contribution 
           to the generalized forces
         
           @param q Current System State
           
           @returns G: Contribution by gravity to the generalized forces
        """

        # Gravity is always acting in negative y direcion only on position not on orientation
        return np.array([0, 9.81 * self.m, 0])

    def A(self, q: np.ndarray) -> np.ndarray:
        """Function that returns the mapping matrix of the actuation to 
           the generalized forces.
           
           @param q Current System State
           
           @returns A: State dependend matrix that maps the control input
                       to the generalized forces
        """

        # Auxillery variables to not recompute the sin/cos of the orientation all the time
        sT = np.sin(q[2])
        cT = np.cos(q[2])

        # The positional force is simply added for all rotors, weighted according to 
        # the orientation. The orientational torque is the difference in torque created
        # by both lever arms
        return np.array([[sT, sT], [cT, cT], [0.5 * self.l, -0.5 * self.l]])