#! /usr/bin/env python
import math
import numpy

def nearly_equal(a,b,sig_fig=3):
  return (a==b or int(a*10**sig_fig) == int(b*10**sig_fig))
