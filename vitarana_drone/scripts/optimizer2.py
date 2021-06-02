from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

class DronePathOptimizer():
    def __init__(self):
        self.cost_matrix = None
        self.data = {}
        self.routing = None
        self.manager = None
        self.assignment = None

    def set_cost_matrix(self, cost_matrix):
        self.cost_matrix = cost_matrix

    def get_optimum_route(self):
        route = []
        dropped_nodes = 'Dropped nodes:'
        for node in range(self.routing.Size()):
            if self.routing.IsStart(node) or self.routing.IsEnd(node):
                continue
            if self.assignment.Value(self.routing.NextVar(node)) == node:
                dropped_nodes += ' {}'.format(self.manager.IndexToNode(node))
        print(dropped_nodes)
        # Display routes
        total_distance = 0
        total_load = 0
        for vehicle_id in range(self.data['num_vehicles']):
            index = self.routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
            route_distance = 0
            route_load = 0
            while not self.routing.IsEnd(index):
                node_index = self.manager.IndexToNode(index)
                route_load += self.data['demands'][node_index]
                plan_output += ' {0} -> '.format(node_index)
                previous_index = index
                index = self.assignment.Value(self.routing.NextVar(index))
                route_distance += self.routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
                route.append(node_index)
            plan_output += ' {0}\n'.format(self.manager.IndexToNode(index))
            route.append(route[0])
            plan_output += 'Distance of the route: {} units\n'.format(route_distance)
            plan_output += 'Load of the route: {}\n'.format(route_load)
            print(plan_output)
            total_distance += route_distance
            total_load += route_load
        print('Total Distance of all routes: {} units'.format(total_distance))
        print('Total Load of all routes: {}'.format(total_load))

        return route
    
    def create_data_model(self):
        self.data['distance_matrix'] = self.cost_matrix
        demands = [0]
        demands.extend((len(self.cost_matrix)-1)*[1])
        self.data['demands'] = demands
        self.data['vehicle_capacities'] = [len(self.cost_matrix)-1]
        self.data['num_vehicles'] = 1
        self.data['depot'] = 0
    
    def run(self):
        self.create_data_model()

        # Create the routing index manager.
        self.manager = pywrapcp.RoutingIndexManager(len(self.data['distance_matrix']), self.data['num_vehicles'], self.data['depot'])

        # Create Routing Model.
        self.routing = pywrapcp.RoutingModel(self.manager)


        # Create and register a transit callback.
        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = self.manager.IndexToNode(from_index)
            to_node = self.manager.IndexToNode(to_index)
            return self.data['distance_matrix'][from_node][to_node]

        transit_callback_index = self.routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc.
        self.routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)


        # Add Capacity constraint.
        def demand_callback(from_index):
            """Returns the demand of the node."""
            # Convert from routing variable Index to demands NodeIndex.
            from_node = self.manager.IndexToNode(from_index)
            return self.data['demands'][from_node]

        demand_callback_index = self.routing.RegisterUnaryTransitCallback(
            demand_callback)
        self.routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,  # null capacity slack
            self.data['vehicle_capacities'],  # vehicle maximum capacities
            True,  # start cumul to zero
            'Capacity')
        # Allow to drop nodes.
        penalty = 1000
        for node in range(1, len(self.data['distance_matrix'])):
            self.routing.AddDisjunction([self.manager.NodeToIndex(node)], penalty)

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        search_parameters.time_limit.FromSeconds(1)

        # Solve the problem.
        self.assignment = self.routing.SolveWithParameters(search_parameters)