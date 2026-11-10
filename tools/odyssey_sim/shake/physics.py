"""Torque/column + relaxing bicycle + sprung roll. SI internally; no road feedback.

Motor raw-to-Nm is a gauge assumption, not a measured calibration. Outputs are
wheel radians, v*yaw, roll radians, roll rate, CG acceleration, seat specific force.
Substeps enforce a nonsmooth Coulomb stop rather than smoothing away stiction.
"""
import numpy as np
try:
  from numba import njit
except ImportError:
  def njit(*args, **kwargs):
    return lambda f: f

# J, B, Fc, trail, Cf, Cr, Iz, motor_tau, relaxation_length, roll_hz,
# roll_zeta, sprung_h_over_I, Dahl presliding displacement (0 = Coulomb).
DEFAULT = np.array([.04, .6, .35, .025, 120000., 140000., 4000., .04, .5, 1.3, .3, .7, 0.])
MASS, SPRUNG, RATIO, A, B = 2163.35, 1900., 14.35, 1.35, 1.65


@njit(cache=True)
def motor_map(u):
  return np.interp(abs(u), np.array([0., .028, .1, .2, .3, .4, .5, .6, .7, .8, 1.]),
                   np.array([0., 0., 6., 14., 25., 39., 50., 57., 63., 67., 75.])) * np.sign(u)


@njit(cache=True)
def advance(u, speed, p, initial):
  """100 Hz command in curvature-positive convention; deterministic 1 ms stepping."""
  theta, omega, vy, r, phi, q, ff, fr = initial[:8]
  motor = initial[8] if len(initial) > 8 else 0.
  applied = initial[9] if len(initial) > 9 else u[0]
  z = initial[10] if len(initial) > 10 else 0.
  result = np.zeros((len(u), 6))
  for i in range(len(u)):
    v = max(speed[i], 5.)
    for _ in range(10):
      dt = .001
      applied += min(3.*dt, max(-3.*dt, u[i]-applied))
      motor += (motor_map(applied)*.05-motor)*(-np.expm1(-dt/p[7]))
      drive = motor - p[1]*omega - p[3]*ff/RATIO
      if p[12] > 0:
        # Dahl: friction bristle deflection has finite presliding displacement.
        z += dt*(omega - abs(omega)*z/p[12])
        z = min(p[12], max(-p[12], z))
        new_omega = omega + dt*(drive-p[2]*z/p[12])/p[0]
      elif abs(omega) < 1e-8 and abs(drive) <= p[2]:
        new_omega = 0.
      else:
        direction = np.sign(omega) if abs(omega) > 1e-8 else np.sign(drive)
        new_omega = omega + dt*(drive-p[2]*direction)/p[0]
        if omega*new_omega < 0 and abs(drive) <= p[2]:
          new_omega = 0.
      omega = new_omega
      theta += dt*omega
      ff += (p[4]*(theta/RATIO-(vy+A*r)/v)-ff)*(-np.expm1(-dt*v/p[8]))
      fr += (p[5]*(-(vy-B*r)/v)-fr)*(-np.expm1(-dt*v/p[8]))
      ay = (ff+fr)/MASS
      vy += dt*(ay-v*r)
      r += dt*(A*ff-B*fr)/p[6]
      phidd = p[11]*ay - 2*p[10]*2*np.pi*p[9]*q - (2*np.pi*p[9])**2*phi
      q += dt*phidd
      phi += dt*q
    result[i] = np.array([theta, v*r, phi, q, ay, ay-.5*phidd-9.81*phi])
  return result, np.array([theta, omega, vy, r, phi, q, ff, fr, motor, applied, z])


@njit(cache=True)
def simulate(u, speed, p, initial):
  return advance(u, speed, p, initial)[0]
