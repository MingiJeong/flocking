#!/usr/bin/env python

# import of python modules
import math
import numpy as np

def normalize(v):
    """
    normalize vector based on L1 norm
    """
    norm=np.linalg.norm(v, ord=1)
    if norm==0:
        norm=np.finfo(v.dtype).eps
    return v/norm

def check_vector_angle(vec1, vec2):
    """
    check angle between two vectors by using vec 1 as base.
    - arg: vec1, vec2 format np.array([x,y]) 1x2 
    - return: angle between the vectors abs 0 to pi (+: if coincide with check vector, -: if not)
    """

    # inner dot calculation
    inner_dot = np.dot(vec1, vec2)

    # vector length calculation
    vec1_len = np.linalg.norm(vec1)
    vec2_len = np.linalg.norm(vec2)

    # between angle calculation
    between_angle = math.acos(round(np.asscalar(inner_dot / (vec1_len * vec2_len)),6))

    # check vector (assuming "between angle" is correct, we get rotation vector of vec1 based on that angle)
    # length normalization and magnitude reflection
    check_vector = [vec1[0] * math.cos(between_angle) - vec1[1] * math.sin(between_angle),
                        vec1[0] * math.sin(between_angle) + vec1[1] * math.cos(between_angle)]
    check_vector_normalized = [check_vector[i] / vec1_len for i in range(len(check_vector))]
    vec2_normalized = [vec2[i] / vec2_len for i in range(len(check_vector))]

    # check if the vector is the same as what we want
    # note that 0.1 is just a threshhold for comparison
    if abs(check_vector_normalized[0] - vec2_normalized[0]) < 0.1 and abs(check_vector_normalized[1] - vec2_normalized[1]) < 0.1:
        return between_angle
    return -between_angle