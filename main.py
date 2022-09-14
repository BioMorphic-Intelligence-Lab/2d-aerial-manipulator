import sys
import numpy as np
import matplotlib.pyplot as plt

from scipy import integrate
from am import AerialManipulator

def do_main(argv):

    am = AerialManipulator()
 
    u = lambda x: np.array([4.900, 4.900])
    y = lambda x, t: am.f(x, u(x))

    y0 = np.array([0, 0, 0, 0, 0, 0])
    t = np.arange(0, 10,0.1)
    output = integrate.odeint(y, y0, t)

    plt.plot(t, output[:,:3])    
    plt.legend(["x", "y", "theta"])
    plt.show()


if __name__ == "__main__":
    do_main(sys.argv)