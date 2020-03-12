from project_utils import theta, D
import uuid

class GVGExplore:
    def run_other_dfs(self, edge):
        parent_id = self.get_id()
        node_id = self.get_id()
        parent_nodes = {parent_id: edge[0]}
        stack_nodes = {node_id: edge[1]}
        parent = {parent_id: None}
        visited = []
        S = [node_id]
        while len(S) > 0:
            u = S.pop()
            self.fetch_graph()
            self.localize_parent_nodes(parent_nodes)
            leave_node = stack_nodes[u]
            ancestor_id = parent[u]
            if leave_node not in self.adj_list and parent[u]:
                leave_node = self.localize_leaf_node(parent_nodes[ancestor_id], parent_nodes[u], stack_nodes[u])
                stack_nodes[u] = leave_node
            angle = theta(parent_nodes[ancestor_id], leave_node)
            self.move_to_frontier(leave_node, theta=angle)
            neighbors = self.adj_list[leave_node]
            for v in neighbors:
                if not self.already_explored(v):
                    v_id = self.get_id()
                    stack_nodes[v_id] = v
                    S.append(v_id)
                    parent[v_id] = u
            visited.append(u)
            parent_nodes[u] = leave_node

    def already_explored(self, v):
        is_visited = False
        dists = {}
        visited = []
        S = [v]
        while len(S) > 0:
            u = S.pop()
            dists[D(v, u)] = u
            neighbors = self.adj_list[u]
            for v in neighbors:
                if v not in visited:
                    S.append(u)
            visited.append(u)
        closest_distance = min(dists.keys())
        if closest_distance < self.lidar_scan_radius:
            is_visited = True
        return is_visited

    def localize_parent_nodes(self, parent_nodes):
        all_nodes = list(self.adj_list)
        node_keys = list(parent_nodes)
        node_dist = {}
        for n in all_nodes:
            for node_id, val in parent_nodes.items():
                n_dist = D(val, n)
                if node_id not in node_dist:
                    node_dist[node_id] = {n_dist: n}
                else:
                    node_dist[node_id][n_dist] = n
        for k in node_keys:
            n_dists = node_dist[k].keys()
            new_n = node_dist[k][min(n_dists)]
            parent_nodes[k] = new_n

    def localize_leaf_node(self, ancestor_node, parent_node, leaf_node):
        node_dists = {}
        parent = {parent_node: None}
        visited = [ancestor_node]
        S = [leaf_node]
        while len(S) > 0:
            u = S.pop()
            node_dists[D(leaf_node, u)] = u
            neighbors = self.adj_list[u]
            for v in neighbors:
                if v not in visited:
                    S.append(v)
                    parent[v] = u
            visited.append(u)

        node = node_dists[min(node_dists.keys())]
        return node

    def get_id(self):
        id = uuid.uuid4()
        return id
