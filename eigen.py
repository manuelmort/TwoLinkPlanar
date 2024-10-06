import numpy as np

# Define the matrix B
B = 1/8 * np.array([
    [9, 0, -3],
    [10, -8, 2],
    [3, 0, -1]
])

# Calculate the characteristic polynomial coefficients
char_poly = np.poly(B)

# Compute the eigenvalues
eigenvalues = np.linalg.eigvals(B)

# Compute the eigenvectors
eigenvalues, eigenvectors = np.linalg.eig(B)

print("Characteristic Polynomial:", char_poly)
print("Eigenvalues:", eigenvalues)
print("Eigenvectors:", eigenvectors)
