import numpy as np
import matplotlib.pyplot as plt

# Define the coefficients of the denominator polynomial in descending order of powers
coefficients = [1, 1, 1, 2]

# Find the roots of the polynomial
roots = np.roots(coefficients)

# Count the number of roots with positive real parts
right_half_plane_poles = sum(root.real > 0 for root in roots)

# Display the roots and the count of right-half-plane poles
print("\n\nRoots of the polynomial:", roots)
print(f"Number of right-half-plane poles: {right_half_plane_poles}\n\n")

# Frequency range for Nyquist plot (from low to high frequencies)
omega = np.logspace(-2, 2, 500)  # from 0.01 to 100 rad/s

# Compute the transfer function response at each frequency
G = 1 / (1j * omega)**3 + 1 / (1j * omega)**2 + 1 / (1j * omega) + 2

# Plot the Nyquist plot
plt.figure(figsize=(8, 8))
plt.plot(G.real, G.imag, label='Nyquist Path')       # Positive frequency path
plt.plot(G.real, -G.imag, label='Nyquist Path (Mirror)')  # Mirror for negative frequency path
plt.plot(-1, 0, 'ro', label='-1 + j0 (Critical Point)')   # The critical point -1 + j0

# Labels and grid
plt.xlabel('Real Part')
plt.ylabel('Imaginary Part')
plt.title('Nyquist Plot')
plt.axhline(0, color='black', lw=0.5)
plt.axvline(0, color='black', lw=0.5)
plt.legend()
plt.grid()

# Show the plot
plt.show()
