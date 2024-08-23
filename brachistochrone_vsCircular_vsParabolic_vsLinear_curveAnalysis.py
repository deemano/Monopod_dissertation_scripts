import numpy as np
from scipy.optimize import newton
from scipy.integrate import quad
import matplotlib.pyplot as plt

# Acceleration due to gravity (m/s^2); final position of bead (m).
g = 9.81
x2, y2 = 1, 0.65
extra_force = 0.1  # Additional horizontal force (arbitrary units)

def brachistochrone(x2, y2, extra_force, N=100):
    """Return the path of the Brachistochrone curve with additional force."""
    def f(theta):
        return y2/x2 - (1-np.cos(theta))/(theta-np.sin(theta))
    theta2 = newton(f, np.pi/2)
    R = y2 / (1 - np.cos(theta2))
    theta = np.linspace(0, theta2, N)
    x = R * (theta - np.sin(theta))
    y = R * (1 - np.cos(theta))
    
    # Adjust for additional force
    effective_g = g + extra_force  # Total effective acceleration
    T = theta2 * np.sqrt(R / effective_g)
    
    return x, y, T

def linear(x2, y2, extra_force, N=100):
    """Return the path of a straight line with additional force."""
    m = y2 / x2
    x = np.linspace(0, x2, N)
    y = m * x
    
    # Adjust for additional force
    effective_g = g + extra_force
    T = np.sqrt(2*(1+m**2)/effective_g/m * x2)
    
    return x, y, T

def circle(x2, y2, extra_force, N=100):
    """Return the path of a circular arc with additional force."""
    r = (x2**2 + y2**2) / (2 * x2)
    
    def f(x):
        return np.sqrt(2 * r * x - x**2)
    
    def fp(x):
        return (r - x) / f(x)
    
    x = np.linspace(0, x2, N)
    y = f(x)
    
    # Adjust for additional force
    effective_g = g + extra_force
    T = quad(lambda x: np.sqrt((1 + fp(x)**2) / (2 * effective_g * f(x))), 0, x2)[0]
    
    return x, y, T

def parabola(x2, y2, extra_force, N=100):
    """Return the path of a parabolic arc with additional force."""
    c = (y2 / x2) ** 2
    
    def f(x):
        return np.sqrt(c * x)
    
    def fp(x):
        return c / 2 / f(x)
    
    x = np.linspace(0, x2, N)
    y = f(x)
    
    # Adjust for additional force
    effective_g = g + extra_force
    T = quad(lambda x: np.sqrt((1 + fp(x)**2) / (2 * effective_g * f(x))), 0, x2)[0]
    
    return x, y, T

# Calculate times for all paths with the additional force
times = {}
for curve in ('brachistochrone', 'circle', 'parabola', 'linear'):
    _, _, times[curve] = globals()[curve](x2, y2, extra_force)

# Calculate the percentage differences compared to the Brachistochrone time
t_brachistochrone = times['brachistochrone']
percent_diffs = {curve: 100 * (T - t_brachistochrone) / t_brachistochrone for curve, T in times.items() if curve != 'brachistochrone'}

# Plot the paths
fig, ax = plt.subplots()
for curve in ('brachistochrone', 'circle', 'parabola', 'linear'):
    x, y, T = globals()[curve](x2, y2, extra_force)
    if curve == 'brachistochrone':
        label = '{}: {:.3f} s'.format(curve, T)
    else:
        percent_diff = percent_diffs[curve]
        label = '{}: {:.3f} s ({:.2f}% slower)'.format(curve, T, percent_diff)
    ax.plot(x, y, lw=2, alpha=0.5, label=label)

ax.legend()
ax.set_xlabel('$x$')
ax.set_ylabel('$y$')
ax.set_xlim(0, x2)  # Use the updated x2 value
ax.set_ylim(0.8, 0)
plt.title('Fastest Leg Curvature under Gravitational Torgue, No Friction')
plt.savefig('brachistochrone_with_force.png')
plt.show()