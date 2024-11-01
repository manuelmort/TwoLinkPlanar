import numpy as np

def generate_matrix():
    """
    Generates a 2x2 matrix that's not diagonalizable
    Returns both the matrix and its eigenvalues
    """
    # Creating a matrix with repeated eigenvalue λ=2
    # and only one linearly independent eigenvector
    matrix = np.array([[2, 1],
                      [0, 2]])
    
    return matrix

def compute_eigenvalues(matrix):
    """Compute eigenvalues of the matrix"""
    return np.linalg.eigvals(matrix)

def compute_eigenvectors(matrix, eigenvalue):
    """Compute eigenvectors for a given eigenvalue"""
    # A-λI
    I = np.eye(2)
    A_minus_lambda = matrix - eigenvalue * I
    # Solve (A-λI)v = 0
    # Using null space calculation
    return np.linalg.null_space(A_minus_lambda)

# Generate and display the matrix
matrix = generate_matrix()
print("Original Matrix:")
print(matrix)

# Compute and display eigenvalues
eigenvalues = compute_eigenvalues(matrix)
print("\nEigenvalues:")
print(eigenvalues)

# For each eigenvalue, compute eigenvectors
for eigenvalue in np.unique(eigenvalues):
    eigenvectors = compute_eigenvectors(matrix, eigenvalue)
    print(f"\nEigenvectors for eigenvalue {eigenvalue}:")
    print(eigenvectors)