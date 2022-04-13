# TO-DO: Change repository name to 'space-rocket-controller-clc-equations-of-motion' (?) 
# TO-DO: Containerize to microservice
import numpy as np

# Solves for the motion of the flight body
# Requires: All external forces and moments (engine, air, and gravity)
# Required by: Nonlinear motion solver
# Outputs: Sum of external forces and moments
class MotionSolver():
  def __init__(self, metadata_distributor):
    self.metadata_distributor = metadata_distributor
    self.xyz_a = self.metadata_distributor.get_var('xyz_a')
    self.xyz_T = self.metadata_distributor.get_var('xyz_T')
    self.t_hb = self.metadata_distributor.get_var('t_hb')
    self.g_prime_xyz = self.metadata_distributor.get_var('g_prime_xyz')
    self.lmn_a = self.metadata_distributor.get_var('lmn_a')
    self.lmn_T = self.metadata_distributor.get_var('lmn_T')
    self.flightbody_mass = self.metadata_distributor.get_var('flightbody_mass')

    # Solving for equation of motion
    def get_external_force(self):
      xyz = self.xyz_a + self.xyz_T + self.t_hb*self.flightbody_mass*self.g_prime_xyz
      self.metadata_distributor.set({'xyz': xyz})
      return xyz

    def get_external_moment(self):
      lmn = self.lmn_a + self.lmn_T
      self.metadata_distributor.set({'lmn': lmn})
      return lmn