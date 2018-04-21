import matplotlib.pyplot as plt
import numpy as np
import matplotlib.mlab as mlab

mu, mu2 = 62.5, 89.63
sigma, sigma2 = 14.36, 23.48
x = np.linspace(mu - 3*sigma, mu + 3*sigma, 100)
x2 = np.linspace(mu2 - 3*sigma2, mu2 + 3*sigma2, 100)
plt.plot(x,mlab.normpdf(x, mu, sigma))
plt.plot(x2,mlab.normpdf(x2, mu2, sigma2))
plt.show()