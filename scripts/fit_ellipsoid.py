
import numpy as np
from numpy.linalg import inv, eig, svd

def fit_ellipsoid(data):
    x, y, z = data[:, 0], data[:, 1], data[:, 2]

    # Design matrix for general quadratic form
    D = np.column_stack([
        x * x,
        y * y,
        z * z,
        x * y,
        x * z,
        y * z,
        x,
        y,
        z,
        np.ones_like(x)
    ])

    # Solve D·v = 0 using SVD
    _, _, V = svd(D)
    v = V[-1, :]  # Last row of V gives the solution

    # Unpack coefficients
    A, B, C, D_, E, F, G, H, I, J = v

    # Form the symmetric matrix Q for the quadratic terms
    Q = np.array([
        [A, D_ / 2, E / 2],
        [D_ / 2, B, F / 2],
        [E / 2, F / 2, C]
    ])

    # Form the linear term vector
    u = np.array([G, H, I])

    # Compute the ellipsoid center (hard iron offset)
    center = -0.5 * inv(Q) @ u

    # Evaluate the constant term for canonical form
    val = center @ Q @ center
    scale = val - J  # Scale factor to normalize the ellipsoid equation

    # Normalize Q to unit ellipsoid form
    Q_normalized = Q / scale

    return {
        "coefficients": v,
        "center": center,
        "Q": Q_normalized
    }

# === Usage ===
import os

# Load data from file
path = os.path.expanduser("~/gambos-project/scripts/mag_data.txt")
data = np.loadtxt(path, delimiter=',')

result = fit_ellipsoid(data)

print("Ellipsoid center (hard iron offset):")
print(result["center"])

print("\nSoft iron correction matrix (Q):")
print(result["Q"])


Q = result['Q']

eigvals, eigvecs = np.linalg.eigh(Q)
print("Eigenvalues:", eigvals)

# Inverse square root of eigenvalues
inv_sqrt_eigvals = np.diag(1.0 / np.sqrt(eigvals))

# Soft iron correction matrix
soft_iron_correction = eigvecs @ inv_sqrt_eigvals @ eigvecs.T

print("Soft iron correction matrix to apply:")
print(soft_iron_correction)



"""
Ellipsoid center (hard iron offset):
[-0.37870076  0.07493372 -0.07979247]

Soft iron correction matrix (Q):
[[ 5.29631365  0.06475673 -0.00571778]
 [ 0.06475673  5.60442441 -0.1123009 ]
 [-0.00571778 -0.1123009   5.65498391]]
 """
