import numpy as np
import matplotlib.pylab as plt

x = np.linspace(-2*np.pi, 2*np.pi, 401)
x_diff = x[1]-x[0]
sin = np.sin(x)

cos = np.zeros(len(sin))
cos[0] = -1
for i in range(1, len(sin)):
    cos[i] = cos[i-1]+sin[i]*x_diff
plt.plot(x,sin, x,cos)

print(np.trapz(sin,dx=x_diff))
plt.show()
