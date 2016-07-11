#!/usr/bin/python

import sys
from math import pi
import numpy as np

resolution_ = 0.05
number_of_angles_ = 16
number_of_intermediate_poses_ = 2

forward_cost_mult_ = 1
arc_cost_mult_ = 1
sideways_cost_mult_ = 1
turn_in_place_cost_mult_ = 1

# base prims contains the information for all primitives
# for 0deg, 22.5deg, and 45 deg. From this, all other
# primitives are generated. The format for each prim is
# fwd displacement, sideways displacement, and angle displacement
# (CCW, from 0 to number_of_angles_)
# and lastly the cost for that primitive.
base_prims = [
    {'angle': 0, 'prims': {
        # fwd
        (1, 0, 0, forward_cost_mult_),
        # rotate in plate
        (0, 0, 4, turn_in_place_cost_mult_),
        (0, 0, -4, turn_in_place_cost_mult_),
        # side step
        (0, 1, 0, sideways_cost_mult_),
        (0, -1, 0, forward_cost_mult_),
        # diagonal
        (1, 1, 0, sideways_cost_mult_),
        (1, -1, 0, sideways_cost_mult_)
        }
    },
    # 22.5 is the same as 45 for convenience
    {'angle': 22.5, 'prims': {
        # fwd
        (1, 0, 0, forward_cost_mult_),
        # rotate in plate
        (0, 0, 4, turn_in_place_cost_mult_),
        (0, 0, -4, turn_in_place_cost_mult_),
        # side step
        (0, 1, 0, sideways_cost_mult_),
        (0, -1, 0, sideways_cost_mult_),
        # diagonal
        (1, 1, 0, sideways_cost_mult_),
        (1, -1, 0, sideways_cost_mult_)
        }
    },
    {'angle': 45, 'prims': {
        # fwd
        (1, 1, 0, forward_cost_mult_),
        # rotate in place
        (0, 0, 4, turn_in_place_cost_mult_),
        (0, 0, -4, turn_in_place_cost_mult_),
        # side step
        (-1, 1, 0, sideways_cost_mult_),
        (1, -1, 0, sideways_cost_mult_),
        # diagonal
        (0, 1, 0, sideways_cost_mult_),
        (1, 0, 0, sideways_cost_mult_)
        }
    },
]

number_of_prims_per_angle = len(base_prims[0]['prims'])
total_prims = number_of_angles_ * number_of_prims_per_angle

def generate_prims(f):
    primid = 0
    angle = 0

    deg_0 = base_prims[0]
    for i in range(4):
        angle = 4 * i
        for prim in deg_0['prims']:
            write_prim(f, angle, prim, primid)
            primid += 1

    deg_45 = base_prims[2]
    for i in range(4):
        angle = 4 * i + 2
        for prim in deg_45['prims']:
            write_prim(f, angle, prim, primid)
            primid += 1

    deg_22_5 = base_prims[1]
    for i in range(4):
        angle = 1 + (i * 4)
        for prim in deg_22_5['prims']:
            write_prim(f, angle, prim, primid)
            primid += 1
    for i in range(4):
        # here we rotate around y=x
        angle = 1 + (i * 4) + 2
        for prim in deg_22_5['prims']:
            new_prim = (prim[1], prim[0], -prim[2], prim[3])
            write_prim(f, angle, new_prim, primid)
            primid += 1


def write_prim(f, angle, prim, primid):

    #rotate x/y by angle
    rad = 2 * pi * angle / number_of_angles_
    c = np.cos(rad)
    s = np.sin(rad)
    rot_m = np.array([[c, -s], [s, c]])

    xy = np.dot(rot_m, np.array([prim[0], prim[1]]))

    end_t = angle + prim[2]
    if end_t > 15:
        end_t -= 16

    f.write('primID: %d\n' % primid)
    f.write('startangle_c: %d\n' % angle)
    f.write('endpose_c: %d %d %d\n' % (round(xy[0]), round(xy[1]), end_t))
    f.write('additionalactioncostmult: %d\n' % prim[3])
    f.write('intermediateposes: %d\n' % number_of_intermediate_poses_)

    dx = xy[0] * resolution_/ (number_of_intermediate_poses_ - 1)
    dy = xy[1] * resolution_/ (number_of_intermediate_poses_ - 1)
    dt = prim[2] * resolution_/ (number_of_intermediate_poses_ - 1)
    x = 0
    y = 0
    t = 2 * pi * (angle / number_of_angles_)
    for i in range(number_of_intermediate_poses_):
        f.write('%f %f %f\n' % (x, y, t))
        x += dx
        y += dy
        t += dt

if len(sys.argv) == 2:
    output_file = sys.argv[1]
    f = open(output_file, 'w')
    print "writing to file ", f.name
    f.write('resolution_m: %f\n' % resolution_)
    f.write('numberofangles: %d\n' % number_of_angles_)
    f.write('totalnumberofprimitives: %d\n' % total_prims)
    generate_prims(f)

