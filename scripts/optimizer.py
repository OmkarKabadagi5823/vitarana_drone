import networkx as nx
import numpy as np
import copy

class OptimizerBase():
    def __init__(self, diGraph, start_node):
        assert type(diGraph)==type(nx.DiGraph()), "Required a networkx DiGraph"
        self.G = diGraph
        self.start_node = start_node
        self.path = -1*np.ones(self.G.order())
        self.path_cost = float('nan')

class BranchAndBound(OptimizerBase):
    def __init__(self, diGraph, start_node):
        OptimizerBase.__init__(self, diGraph, start_node)
        self._current_best_cost = float('inf')
        self._current_best_path = [float('nan')]*self.G.order()
        self._init_working_props()

    def _init_working_props(self):
        self._current_path = [float('nan')]*self.G.order()
        self._current_lower_bound = 0
        self._current_weight = 0
        self._current_visited = dict()#np.full(self.G.order(), False, dtype=bool)
        for node in self.G.nodes:
            self._current_visited[node] = False 

    def _min_edge_of_node(self, node, order=1):
        weights = []
        
        for end_node, attr in self.G.adj[node].items():
            weights.append(attr['weight'])

        return sorted(weights)[order-1]

    def _set_initial_working_props(self):
        self._current_path[0] = self.start_node
        self._current_weight = 0
        self._current_visited[self.start_node] = True

        for node in self.G.nodes:
            self._current_lower_bound += (self._min_edge_of_node(node) + self._min_edge_of_node(node, order=2)) / 2.0 

    def _calculate_level_lower_bound(self, level, exclude_node):
        if(level == 0):
            self._current_lower_bound -= (self._min_edge_of_node(self._current_path[level]) + self._min_edge_of_node(self._current_path[level], 2)) / 2.0
        else:
            self._current_lower_bound -= (self._min_edge_of_node(self._current_path[level]) + self._min_edge_of_node(exclude_node, 2)) / 2.0

    def _set_working_props_for_next_node(self, next_node, level):
        self._current_weight += self.G.adj[self._current_path[level]][next_node]['weight']
        self._calculate_level_lower_bound(level, next_node)
        self._current_path[level+1] = next_node
        self._current_visited[next_node] = True

    def _log_working_props(self):
        temp = dict()
        temp['path'] = copy.deepcopy(self._current_path)
        temp['lower_bound'] = copy.deepcopy(self._current_lower_bound)
        temp['weight'] = copy.deepcopy(self._current_weight)
        temp['visited'] = self._current_visited.copy()

        return copy.deepcopy(temp)
    
    def _revert_working_props(self, log):
        self._current_path = log['path']
        self._current_lower_bound = log['lower_bound']
        self._current_weight = log['weight']
        self._current_visited = log['visited']
    
    def _is_current_bound_better_than_best_cost(self):
        return self._current_lower_bound + self._current_weight < self._current_best_cost

    def _search(self, level):
        if(level == self.G.order() - 1):
            if(self.G.has_edge(self._current_path[level], self._current_path[0])):
                cost = self._current_weight + self.G.adj[self._current_path[level]][self._current_path[0]]['weight']
                if(cost < self._current_best_cost):
                    self._current_best_cost = cost
                    self._current_best_path = self._current_path
            return

        for next_node in self.G.adj[self._current_path[level]].keys():
            if(not self._current_visited[next_node]):
                log = self._log_working_props()
                self._set_working_props_for_next_node(next_node, level)

                if(self._is_current_bound_better_than_best_cost()):
                    self._search(level+1)

                self._revert_working_props(log)

    def optimize(self):
        self._set_initial_working_props()
        self._search(0)
        self.path = self._current_best_path
        self.path_cost = self._current_best_cost

class DronePathOptimizerFactory():
    def __init__(self, diGraph, start_node='default'):
        self.G =  diGraph
        self.start_node = list(diGraph.nodes)[0]
        if(start_node != 'default'):
            self.start_node = start_node
    
    def create_optimizer(self, optimizer):
        if(optimizer=='branch_and_bound'):
            return BranchAndBound(self.G, self.start_node)


# G = nx.DiGraph()
# G.add_nodes_from(['W', '1', '2', '3'])
# G.add_weighted_edges_from([('W','1',10), ('W','2',20), ('W','3',30),
# ('1','W',10), ('1','2',30), ('1','3',30),
# ('2','W',0), ('2','1',10), ('2','3',20),
# ('3','W',20), ('3','1',30), ('3','2',20)])

# DPO = DronePathOptimizerFactory(G, start_node='W')
# BNB = DPO.create_optimizer(optimizer='branch_and_bound')
# BNB.optimize()
# print(BNB.path, BNB.path_cost)

    