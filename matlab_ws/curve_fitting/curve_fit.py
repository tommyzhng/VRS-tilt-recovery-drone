import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Combined data from all RPM settings
Vz_Vh = np.array([0, -0.2, -0.4, -0.6, -0.8, -0.9, -0.95, -1, -1.2, -1.4, -1.6, -1.8, -2,
                  0, -0.2, -0.4, -0.6, -0.8, -0.9, -0.95, -1, -1.2, -1.4, -1.6, -1.8, -2,
                  0, -0.2, -0.4, -0.6, -0.8, -0.9, -0.95, -1, -1.2, -1.4, -1.6, -1.8, -2])
Ct_Ct0 = np.array([1, 1.018, 1.091, 1.140, 1.237, 1.294, 1.339, 1.404, 1.619, 1.697, 1.772, 1.862, 2.011,
                   1, 1.035, 1.087, 1.164, 1.255, 1.312, 1.338, 1.370, 1.531, 1.689, 1.835, 1.969, 2.134,
                   1, 1.036, 1.085, 1.155, 1.244, 1.288, 1.308, 1.335, 1.484, 1.651, 1.802, 1.917, 2.033])

# Define a quartic polynomial function for curve fitting
def quartic_func(x, a, b, c, d, e):
    return a * x**4 + b * x**3 + c * x**2 + d * x + e

# Perform curve fitting
params, _ = curve_fit(quartic_func, Vz_Vh, Ct_Ct0)

# Plot the data and the fitted curve
plt.scatter(Vz_Vh, Ct_Ct0, label='Data')
Vz_Vh_range = np.linspace(-2, 0, 100)
plt.plot(Vz_Vh_range, quartic_func(Vz_Vh_range, *params), label='Fitted curve', color='red')
plt.xlabel('Vz/Vh')
plt.ylabel('Ct/Ct0')
plt.legend()
plt.show()

print(f"Fitted parameters: a={params[0]}, b={params[1]}, c={params[2]}, d={params[3]}, e={params[4]}")
