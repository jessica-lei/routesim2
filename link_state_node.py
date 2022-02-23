import json
from collections import defaultdict

from simulator.node import Node


class Link_State_Node(Node):

    def __init__(self, id):
        super().__init__(id)
        self.link_cost_dict = defaultdict(dict)
        self.next_hop_dict = {}
        self.link_state_msg_dict = {}

    # Return a string
    def __str__(self):
        return "Rewrite this function to define your node dump printout"

    # Fill in this function
    def link_has_been_updated(self, neighbor, latency):
        # If link has been deleted
        if latency == -1:
            del self.link_cost_dict[self.id][neighbor]
            del self.link_cost_dict[neighbor][self.id]
        else:  # Replace cost of immediate neighbor with new latency
            self.link_cost_dict[self.id][neighbor] = latency
            self.link_cost_dict[neighbor][self.id] = latency

        # Run Dijkstra's and calculate new next hops
        self.update_next_hops()

        # Broadcast a new link-state message for this update to EVERY adjacent node
        ls_msg = {"link_src": self.id, "link_dst": neighbor, "cost": latency}
        link_key = frozenset((self.id, neighbor))

        if link_key in self.link_state_msg_dict:
            # Increment sequence number if existence of link already known
            ls_msg["seq_num"] = self.link_state_msg_dict[link_key]["seq_num"] + 1
        else:
            # Else, initialize sequence number to 0
            ls_msg["seq_num"] = 0
            # Since it is a new link (not a cost update), send all known link-state msgs
            for m in self.link_state_msg_dict.values():
                self.send_to_neighbor(neighbor, self.pack_ls_msg(m))

        # Store updated link-state message in link_state_msg_dict
        self.link_state_msg_dict[link_key] = ls_msg
        # Send link-state message to neighbors
        self.send_to_neighbors(self.pack_ls_msg(ls_msg))

    # Fill in this function
    def process_incoming_routing_message(self, m):
        # Unpack the incoming message
        ls_msg, from_node_id = self.unpack_ls_msg(m)
        link_src, link_dst, cost, seq_num = (
            ls_msg["link_src"], ls_msg["link_dst"], ls_msg["cost"], ls_msg["seq_num"]
        )

        # If sequence number is <= the most recent seq num for that link, send newest msg back
        link_key = frozenset((link_src, link_dst))
        if link_key in self.link_state_msg_dict:
            most_recent_ls_msg = self.link_state_msg_dict[link_key]
            # If the sequence number is lower than that of the most recent message we received,
            # send the new message back to the neighbor that sent us the message
            if most_recent_ls_msg["seq_num"] > seq_num:
                self.send_to_neighbor(from_node_id, self.pack_ls_msg(most_recent_ls_msg))
                return  # do not update
            # If sequence number is the same, all adjacent nodes should have an updated copy
            if most_recent_ls_msg["seq_num"] == seq_num:
                return

        # Else if link has not been observed OR current seq_num > most recent messgae
        self.link_state_msg_dict[link_key] = ls_msg
        if cost == -1:  # the link has been deleted
            # If the node thinks that this link currently exists, delete
            if link_dst in self.link_cost_dict[link_src]:
                del self.link_cost_dict[link_src][link_dst]
                del self.link_cost_dict[link_dst][link_src]
        else:
            self.link_cost_dict[link_src][link_dst] = cost
            self.link_cost_dict[link_dst][link_src] = cost

        # Run Dijkstra's and calculate new next hops
        self.update_next_hops()

        # Broadcast message to all adjacent nodes
        self.send_to_neighbors(self.pack_ls_msg(ls_msg))

    # Return a neighbor, -1 if no path to destination
    def get_next_hop(self, destination):
        return self.next_hop_dict[destination] if destination in self.next_hop_dict else -1

    # Uses Dijkstra's algorithm to update the next hop for each node
    def update_next_hops(self):
        unvisited_node_costs = dict.fromkeys(self.link_cost_dict.keys(), float("inf"))
        prev_nodes = {}
        unvisited_node_costs[self.id] = 0

        # While there are nodes that still need to be visited
        while len(unvisited_node_costs) > 0:
            # Get unvisited node with shortest distance
            min_cost_node_id = min(unvisited_node_costs, key=unvisited_node_costs.get)
            neighbors = self.link_cost_dict[min_cost_node_id]
            # For each neighbor of the unvisited node w/ shortest distance
            for neighbor_id, neighbor_cost in neighbors.items():
                # Disregard neighbor if it has already been visited
                if neighbor_id not in unvisited_node_costs:
                    continue
                new_cost = unvisited_node_costs[min_cost_node_id] + neighbor_cost
                # Replace with new minimum cost to that node
                if new_cost < unvisited_node_costs[neighbor_id]:
                    unvisited_node_costs[neighbor_id] = new_cost
                    prev_nodes[neighbor_id] = min_cost_node_id
            # Remove node w/ shortest distance from unvisited nodes
            del unvisited_node_costs[min_cost_node_id]

        # For each node, store the next hop
        self.next_hop_dict = {}  # reset
        for node_id in prev_nodes:
            if node_id == self.id:
                continue  # No need to store next hop for oneself
            # Backtrack in prev_nodes until we get direct neighbor of self
            curr_node_id = node_id
            while prev_nodes[curr_node_id] != self.id:
                curr_node_id = prev_nodes[curr_node_id]
            # Store next hop for that node in self.next_hop_dict
            self.next_hop_dict[node_id] = curr_node_id

    # Return stringified format of link-state message to send
    def pack_ls_msg(self, ls_msg):
        ls_msg["from_node_id"] = self.id
        return json.dumps(ls_msg)

    # Return link-state message dict + neighbor node id from message received
    def unpack_ls_msg(self, m):
        ls_msg = json.loads(m)
        from_node_id = ls_msg["from_node_id"]
        del ls_msg["from_node_id"]
        return ls_msg, from_node_id
