from warehouse import *
from optimizer2 import *
from optimizer import *

warehouse = Warehouse()
warehouse.set_grid_size(3, 3)
warehouse.set_reference_location_pickup(18.9998102845, 72.000142461, 16.757981)
warehouse.set_reference_location_drop(18.9999367615, 72.000142461, 16.757981)
warehouse.set_depot_location(18.9998887906, 72.0002184402, 16.75)
warehouse.load('/home/omkar/catkin_ws/src/vitarana_drone/scripts/manifest.csv')
x = warehouse.to_cost_matrix()
DPO = DronePathOptimizer()
DPO.set_cost_matrix(x)
DPO.run()
route = DPO.get_optimum_route()
print(route)

# RETURN ,19.0010346854;72.0009217464;22.1052524305  ,X3