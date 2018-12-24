from ConfigParser import SafeConfigParser

from pymobility.models.mobility import gauss_markov, reference_point_group, \
    tvc, truncated_levy_walk, random_direction, random_waypoint, random_walk
import numpy as np
import logging
import sys #for argv
from scipy.spatial.distance import cdist

#configPath = "/path/to/fog-vmalloc/python"

class myGenerator:
    def __init__(self, configFile):
    
        #configPath = "/path/to/fog-vmalloc/python"

        parser = SafeConfigParser()
        #parser.read(configPath+"/config.ini")
        parser.read(configFile)

        nr_nodes = parser.getint('WayPoint','nr_nodes')
        MAX_X = parser.getint('WayPoint', 'MAX_X')
        MAX_Y = parser.getint('WayPoint', 'MAX_Y')
        MIN_V = parser.getint('WayPoint', 'MIN_V')
        MAX_V = parser.getint('WayPoint', 'MAX_V')
        MAX_WT = parser.getint('WayPoint', 'MAX_WT')

        self.step = 0

        #np.random.seed(0xffff)
        #np.random.seed(parser.get('WayPoint', 'seed'))
        if parser.has_option('WayPoint', 'seed'):
            np.random.seed(parser.getint('WayPoint', 'seed'))
        else:
            np.random.seed(0xffff)

        self.rwp = random_waypoint(nr_nodes, dimensions=(MAX_X, MAX_Y), velocity=(MIN_V, MAX_V), wt_max=MAX_WT)

        FN_x_len = 0.1 * MAX_X
        FN_y_len = 0.1 * MAX_Y
        X_CENTER = MAX_X / 2.0
        Y_CENTER = MAX_Y / 2.0
        self.FN_X_MIN = X_CENTER - FN_x_len / 2.0
        self.FN_X_MAX = X_CENTER + FN_x_len / 2.0
        self.FN_Y_MIN = Y_CENTER - FN_y_len / 2.0
        self.FN_Y_MAX = Y_CENTER + FN_y_len / 2.0

    @property
    def next(self):

        xy = next(self.rwp)
        self.step += 1

        # compute distributions in the Fog Node area
        users_in_fog_area = 0
        for point in xy:
            if point[0] <= self.FN_X_MAX and point[0] >= self.FN_X_MIN and point[1] <= self.FN_Y_MAX and point[
                1] >= self.FN_Y_MIN:
                users_in_fog_area += 1
        return users_in_fog_area
        # print "There are",users_in_fog_area,"users in the FN area"


gen = myGenerator(sys.argv[3])

def nextPlease(path):
    #print('==> CALL method')
    x = gen.next
    return x
