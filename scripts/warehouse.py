#!/usr/bin/env python

from collections import OrderedDict
from math import isnan
from utils import utils
import copy
import numpy as np
        
class Warehouse():
    def __init__(self):
        self.init_props()
    
    def init_props(self):
        self.grid_size = (float('nan'), float('nan'))
        self.depot_location = (float('nan'), float('nan'), float('nan'))
        self.depot_location_xyz = (float('nan'), float('nan'), float('nan'))
        self.reference_location_pickup = (float('nan'), float('nan'), float('nan'))
        self.reference_location_drop = (float('nan'), float('nan'), float('nan'))
        self.deliveries = OrderedDict()
        self.deliveries_xyz = OrderedDict()
    
    def set_grid_size(self, row, col):
        self.grid_size = (row, col)

    def set_depot_location(self, latitude, longitude, altitude):
        self.depot_location = (latitude, longitude, altitude)
        self.depot_location_xyz = (utils.lat_to_x(latitude), utils.lon_to_y(longitude), altitude)
    
    def set_reference_location_pickup(self, latitude, longitude, altitude):
        self.reference_location_pickup = (latitude, longitude, altitude)

    def set_reference_location_drop(self, latitude, longitude, altitude):
        self.reference_location_drop = (latitude, longitude, altitude)
    
    def get_cell_location(self, cell, package_type):
        col_alpha = cell[0]
        if(package_type == 'DELIVERY'):
            col = (ord(col_alpha)-ord('A')) + 1 #indexing from 1
            row = int(cell[1])
            cell_location = (self.reference_location_pickup[0] + (col-1)*0.000013552, self.reference_location_pickup[1] + (row-1)*0.000014245, self.reference_location_pickup[2])
        elif(package_type == 'RETURN'):
            col = (ord(col_alpha)-ord('X')) + 1
            row = int(cell[1])
            cell_location = (self.reference_location_drop[0] + (col-1)*0.000013552, self.reference_location_drop[1] + (row-1)*0.000014245, self.reference_location_drop[2])
        return cell_location 

    def load(self, csv_file):
        delivery_number = 1
        with open(csv_file, 'r') as delivery_file:
            while True:
                line = delivery_file.readline().replace(' ', '').replace('\n', '')
                
                if(len(line)):
                    package_details = line.rstrip().split(',')
                    package_type = package_details[0]
                    pickup_location_raw = package_details[1]
                    drop_location_raw = package_details[2]

                    if(package_type == 'DELIVERY'):
                        pickup_location = self.get_cell_location(pickup_location_raw, package_type)
                        drop_location_raw = drop_location_raw.split(';')
                        drop_location = [float(drop_location_raw[0]), float(drop_location_raw[1]), float(drop_location_raw[2])]
                    elif(package_type == 'RETURN'):
                        pickup_location_raw = pickup_location_raw.split(';')
                        pickup_location = [float(pickup_location_raw[0]), float(pickup_location_raw[1]), float(pickup_location_raw[2])]
                        drop_location = self.get_cell_location(drop_location_raw, package_type)

                    pickup_location_xyz = [utils.lat_to_x(pickup_location[0]), utils.lon_to_y(pickup_location[1]), pickup_location[2]]
                    drop_location_xyz = [utils.lat_to_x(drop_location[0]), utils.lon_to_y(drop_location[1]), drop_location[2]]

                    package_info = {'package_type':package_type, 'package_retrieval':pickup_location, 'package_delivery':drop_location}
                    self.deliveries[delivery_number] = package_info

                    package_info_xy = package_info = {'package_type':package_type, 'package_retrieval':pickup_location_xyz, 'package_delivery':drop_location_xyz}
                    self.deliveries_xyz[delivery_number] = package_info_xy

                    delivery_number += 1
                else:
                    break

    def to_cost_matrix(self):
        size = len(self.deliveries_xyz) + 1

        vertices = copy.deepcopy(self.deliveries_xyz)
        vertices[0] = {'package_type':None, 'package_retrieval':self.depot_location_xyz, 'package_delivery':self.depot_location_xyz}

        profit_matrix = np.zeros([size, size])

        for key_i, value_i in vertices.items():
            for key_j, value_j in vertices.items():
                if(key_i != key_j):
                    profit_matrix[key_i][key_j] = int(
                        round(5 - 0.2*utils.distance_between_two_locations(value_i['package_delivery'], value_j['package_retrieval'])
                        + 0.1*utils.distance_between_two_locations(value_j['package_retrieval'], value_j['package_delivery'])
                        - 0.00025*utils.distance_between_two_locations(value_j['package_retrieval'], value_j['package_delivery'])**2
                        , 0))

        cost_matrix = np.max(profit_matrix) - profit_matrix
        
        for i in range(len(cost_matrix)):
            cost_matrix[i][i] = 0
        
        cost_matrix_integer = []

        for i in range(len(cost_matrix)):
            row = []
            for j in range(len(cost_matrix)):
                row.append(int(cost_matrix[i][j]))
            cost_matrix_integer.append(row)

        return cost_matrix_integer
