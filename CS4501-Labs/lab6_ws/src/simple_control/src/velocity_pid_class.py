#!/usr/bin/env python

from ftplib import error_reply


class PID:
  # TODO FOR CHECKPOINT 0
  # On node initialization
  def __init__(self, p=0.0, i=0.0, d=0.0):
    self.p = p
    self.i = i
    self.d = d
    
    self.integral = 0.0
    self.lasterr = 0.0
    pass

   # TODO FOR CHECKPOINT 0
  def pid_loop(self, error, dt):
    self.integral += error * dt
    self.deriv = (error - self.lasterr) / dt
    output = self.p * error + self.i * self.integral + self.d * self.deriv
    self.lasterr = error
    return output