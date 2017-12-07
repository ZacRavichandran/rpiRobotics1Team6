#!/usr/bin/env python
import heapq
import numpy as np

"""
Package to create car insructions 
"""

class DuckietownEdge():
	def __init__(self, from_node, to_node, cost):
		self.from_node = from_node
		self.to_node = to_node 
		self.cost = cost

class DuckietownNode():
	"""
	All directions oriented facing door to the lab
	"""
	def __init__(self, node_id, n, s, e, w):
		self.node_id = node_id
		self.edges = {"n": DuckietownEdge(node_id, n[0], n[1]),
		 "s":DuckietownEdge(node_id, s[0], s[1]), 
		 "e":DuckietownEdge(node_id, e[0], e[1]),
		 "w":DuckietownEdge(node_id, w[0], w[1])}

class DuckietownGraph():
	def __init__(self, nodes = {}):
		self.nodes = nodes 

	def add_node(self, node):
		self.nodes[node.node_id] = node

	def trim_graph(self):
		for node in self.nodes.values():
			for edge_key in node.edges.keys():
				itr_vals = node.edges.keys()
				itr_vals.remove(edge_key)
				for comp_edge_key in itr_vals:
					if node.edges[edge_key].to_node == node.edges[comp_edge_key].to_node:
						if node.edges[edge_key].cost < node.edges[comp_edge_key]:
							node.edges[comp_edge_key].to_node = -1


	def find_path(self, start_node_id, start_edge, end_node):
		self.trim_graph()
		start_node = self.nodes[start_node_id]
		costs = {0:np.Inf, 1:np.Inf, 2:np.Inf, 3:np.Inf}
		back_node = {0:None, 1:None, 2:None, 3:None}
		queue = []

		costs[start_node_id] = 0
		back_node[start_node_id] = -1

		for key in start_node.edges.keys():
			if key is not start_edge and start_node.edges[key].to_node is not -1:
				costs[start_node.edges[key].to_node] = start_node.edges[key].cost
				back_node[start_node.edges[key].to_node] = 0
				#print("To nodes: %d" % start_node.edges[key].to_node)
				heapq.heappush(queue, (start_node.edges[key].cost, start_node.edges[key].to_node))

		
		for t in queue:
			cost = t[0]
			node = self.nodes[t[1]]
		
			if node.node_id == end_node:
				break

			for key in node.edges.keys():
				if node.edges[key].to_node is not -1:
					if cost + node.edges[key].cost < costs[node.edges[key].to_node]:
						back_node[node.edges[key].to_node] = node.edges[key].from_node
						heapq.heappush(queue, (cost + node.edges[key].cost, node.edges[key].to_node))

		
		path = [end_node]
		while True:
			if back_node[path[-1]] == -1:
				break
			else:
				path.append(back_node[path[-1]])

		path.reverse()
		print(self.get_turns(path, start_edge))

	def egde_to_turn(self, se, ee):
		edge_to_num = {'s':0, 'e':1, 'n':2, 'w':3}


	def get_turns(self, path, start_edge):
		last_edge = start_edge 
		last_node = path[0]
		turns = []
		for next_node in path[1:]:
			for key in self.nodes[last_node].edges.keys():
				if self.nodes[last_node].edges[key].to_node == next_node:
					turn_edge = key
					print(turn_edge)
					last_edge = key 
					last_node = next_node

def main():
	graph = DuckietownGraph()
	graph.add_node( DuckietownNode(0, (2,1), (-1,-1), (-1, -1), (1, 1)) )
	graph.add_node( DuckietownNode(1, (2, 1), (3, 1), (0, 1), (3,1)) )
	graph.add_node( DuckietownNode(2, (-1,-1), (1, 1), (0, 1), (3, 1)) )
	graph.add_node( DuckietownNode(3, (2,1), (1,1), (1,1), (-1,-1)) )

	graph.find_path(0, 's', 2)


if __name__ == "__main__":
	main()

