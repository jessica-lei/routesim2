import copy
import json
import time

from simulator.node import Node


class Distance_Vector_Node(Node):
    def __init__(self, id):
        super().__init__(id)
        self.dv_dict = {id: 0}  # current distance vector
        self.path_dict = {id: []}  # shortest known paths to each node
        self.latest_update_times = {}

        self.neighbor_costs = {}
        self.neighbor_dv_dicts = {}
        self.neighbor_path_dicts = {}

    # Return a string
    def __str__(self):
        return "Rewrite this function to define your node dump printout"

    # Fill in this function
    def link_has_been_updated(self, neighbor, latency):
        # Copy DV and path dict for later comparison
        prev_dv_dict = copy.deepcopy(self.dv_dict)
        prev_path_dict = copy.deepcopy(self.path_dict)

        # If cost (latency) to neighbor is -1 (i.e., it has been deleted):
        if latency == -1:
            # If path to any node has `neighbor` as first hop (including neighbor itself)
            # delete from path_dict and dv_dict
            del self.latest_update_times[neighbor]
            del self.neighbor_costs[neighbor]
            del self.neighbor_dv_dicts[neighbor]
            del self.neighbor_path_dicts[neighbor]

        else:  # Otherwise, update neighbor data
            self.neighbor_costs[neighbor] = latency

        # Recompute DV and paths using the updated cost to neighbor (or deleted neighbor)
        self.recompute_dv_and_paths()

        # If dv_dict or path_dict has changed as a result, broadcast to all neighbors
        if self.dv_dict != prev_dv_dict or self.path_dict != prev_path_dict:
            self.broadcast_msg()

    def process_incoming_routing_message(self, m):
        prev_dv_dict = copy.deepcopy(self.dv_dict)
        prev_path_dict = copy.deepcopy(self.path_dict)

        # Unpack DV / path message coming from neighbor
        neighbor_dv_dict, neighbor_path_dict, sent_at, neighbor = self.unpack_msg(m)

        # If it is an out-of-order message from an ex-neighbor (after link has been deleted)
        if neighbor not in self.neighbor_costs:
            return  # don't do anything

        # If this is not the most recent message from this neighbor, disregard
        if neighbor in self.latest_update_times:
            if sent_at < self.latest_update_times[neighbor]:
                return  # don't do anything

        # Otherwise, update the time/DV/ path info for this neighbor
        self.latest_update_times[neighbor] = sent_at
        self.neighbor_dv_dicts[neighbor] = neighbor_dv_dict
        self.neighbor_path_dicts[neighbor] = neighbor_path_dict

        # Recalculate the distance vector and path dictionary
        self.recompute_dv_and_paths()

        # If dv_dict or path_dict has changed as a result, broadcast to all neighbors
        if self.dv_dict != prev_dv_dict or self.path_dict != prev_path_dict:
            self.broadcast_msg()

    # Return a neighbor, -1 if no path to destination
    def get_next_hop(self, destination):
        return self.path_dict[destination][0] if destination in self.path_dict else -1

    # Send message to neighbors
    def broadcast_msg(self):
        msg = {
            "dv_dict": self.dv_dict, "path_dict": self.path_dict,
            "sent_at": time.time(), "sent_by": self.id
        }
        return self.send_to_neighbors(json.dumps(msg))

    # Unpack msg on receive
    def unpack_msg(self, msg_str):
        msg = json.loads(msg_str)
        dv_dict, path_dict, sent_at, sent_by = (
            msg["dv_dict"], msg["path_dict"], msg["sent_at"], msg["sent_by"]
        )
        modified_dv_dict, modified_path_dict = {}, {}
        # Convert string keys to int keys
        for str_key in dv_dict:
            modified_dv_dict[int(str_key)] = dv_dict[str_key]
            modified_path_dict[int(str_key)] = path_dict[str_key]
        return modified_dv_dict, modified_path_dict, sent_at, sent_by

    # Recompute dv_dict and path_dict
    def recompute_dv_and_paths(self):
        # Get all nodes we know exist so far
        all_nodes = set()
        for neighbor_dv_dict in self.neighbor_dv_dicts.values():
            all_nodes = all_nodes.union(neighbor_dv_dict.keys())

        # For each node, consider all neighboring DVs, choose the shortest path, update DV
        self.dv_dict, self.path_dict = {self.id: 0}, {self.id: []}  # Reset and repopulate
        for node in all_nodes:
            if node == self.id:
                continue  # no need to update cost/path for self

            shortest_path_cost = float("inf")
            shortest_path = None
            for neighbor, neighbor_dv_dict in self.neighbor_dv_dicts.items():
                if node not in neighbor_dv_dict:
                    continue  # cannot reach node via given neighbor

                neighbor_path_dict = self.neighbor_path_dicts[neighbor]
                path_from_neighbor = neighbor_path_dict[node]
                # Prevent routing loops by ignoring paths with self.id
                if self.id in path_from_neighbor:
                    continue

                cost_to_neighbor = self.neighbor_costs[neighbor]
                cost_from_neighbor = neighbor_dv_dict[node]
                # If the cost to the node via this neighbor is the shortest we've seen
                if cost_to_neighbor + cost_from_neighbor < shortest_path_cost:
                    # Replace the shortest path cost & shortest path
                    shortest_path_cost = cost_to_neighbor + cost_from_neighbor
                    shortest_path = [neighbor] + path_from_neighbor

            # Set shortest path cost and shortest path in DV and path dict
            if shortest_path is not None:
                self.dv_dict[node] = shortest_path_cost
                self.path_dict[node] = shortest_path

        # For neighbor nodes only: if node has not received a DV from that neighbor in
        # the past, check to see if we can set cost and path manually in dv_dict/path_dict
        for neighbor, cost in self.neighbor_costs.items():
            if neighbor not in self.dv_dict or cost < self.dv_dict[neighbor]:
                self.dv_dict[neighbor] = cost
                self.path_dict[neighbor] = [neighbor]
