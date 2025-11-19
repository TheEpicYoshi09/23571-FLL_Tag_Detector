# Write code to find the difference between an inputted number and pi
import math as m

def pi_diff(n):
    if n > m.pi:
        return n - m.pi
    
    else:
        return m.pi - n
