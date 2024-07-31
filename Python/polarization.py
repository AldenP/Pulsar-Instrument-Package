# polarization.py - program for finding polarization from input data, and optionally displaying it
# David Patenaude - 5.31.2024

import numpy as np
import matplotlib.pyplot as plt

# assume a method for importing the data from a .FITS format already exists.

def findParam_I(d_0, d_90):
    """ Find the Stokes' Parameter I (intensity) by summing the squares of 2 orthogonal polarizations.
        d_0 - the data for the 0deg detector
        d_90 - the data for the 90deg detector
        Future note: make sure dimensions of arrays are good. 
    """
    # raw intensity = (d_0)^2 + (d_90)^2
    #np.square will square element-by-element and remain an array
    #np.add adds two arrays
    I_raw = np.add( np.square(d_0), np.square(d_90) )
    
    # To normalize the data, find the max and divide all elements by it
    I_max = np.amax(I_raw, axis=0)  # axis may need to change, depending on future implemtation
    I_norm = np.divide(I_raw, I_max)    # divide the raw by the max value.
    return I_norm   #return the normalized Intensity matrix

def findParam_Q(d_0, d_90):
    """ Find the Stokes' Parameter Q (polarization term 1) by taking the difference of the squares of 2 orthogonal polarizations.
        d_0 - the data for the 0deg detector
        d_90 - the data for the 90deg detector
        Future note: make sure dimensions of arrays are good. 
    """
    # Q = (d_0)^2 - (d_90)^2
    Q_raw = np.subtract( np.square(d_0), np.square(d_90) ) 
    # now Q needs to be normalized ... 
    # does this require using the normalized Intensity, or should d_0, d_90 be normalized first? 

def findParam_U(d_45, d_135):
    """ Find the Stokes' Parameter U (polarization term 2) by taking the difference of the squares of 2 different orthogonal polarizations.
        d_45 - the data for the 45deg detector
        d_135 - the data for the 135deg detector
        Future note: make sure dimensions of arrays are good. 
    """
    # U = (d_45)^2 - (d_135)^2
    U_raw = np.subtract( np.square(d_45), np.square(d_135) )
    # again, U must be normalized.

def findPolarizationAngle(Q_norm, U_norm):
    """ Finds the polarization angle, knowing the Q, and U parameter (normalized)
        Q_norm: Q-parameter normalized
        U_norm: U-parameter normalized
    """
    return NotImplementedError('PolarizationAngle not yet implemented')


def main() -> None:
    pass


if __name__ == '__main__':
    main()