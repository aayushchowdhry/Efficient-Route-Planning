from typing import List, Type, Tuple
from RoadNodeClasses import *

################################ Graph Classes ################################

class Node:
    """
    Class representing a graph node
    """

    def __init__(self):
        self.edges = []     # Contains edges containing this node
        self.neighbors = [] # Contains neighboring nodes

    def _add_edge(self, edge):
        """ Add an edge to this Node """
        self.edges.append(edge)
        for node in edge.nodes:
            if node != self and node not in self.neighbors:
                self.neighbors.append(node)

    def _add_edges(self, edges):
        """ Add multiple edges to this Node """
        self.edges.extend(edges)
        for edge in edges:
            for node in edge.nodes:
                if node != self and node not in self.neighbors:
                    self.neighbors.append(node)

    def find_edges_to(self, other_node):
        """ Returns a list of edges connecting this node and [other_node] """
        return filter(lambda n: other_node in n.nodes, self.edges)


class Edge:
    """
    Class representing an undirected weighted edge
    """

    def __init__(self, weight : float, node1 : Node, node2 : Node):
        assert node1 != node2
        self.weight = weight
        self.nodes = (node1, node2)
        node1._add_edge(self)
        node2._add_edge(self)

    def getOther(self,node):
        """
        Return the other node of the edge.

        Parameter node: Node where this edge 'begins'
        Precondition: This edge is connected to this Node.
        """
        assert node in self.nodes
        if node==self.nodes[0]:
            return self.nodes[1]
        return self.nodes[0]


class RoadNode(Node):
    """
    Class representing a road feature as a Node with an Airsim coordinate
    """

    def __init__(self, x, y):
        assert type(x)==float and type(y)==float
        Node.__init__(self)
        self.x = x
        self.y = y

    def __str__(self):
        return str((self.x,self.y))


########################### Shortest Path Finder ##############################


def plan_route(graph, current_position, goal_point, heading=0): # type (List[RoadNode], Tuple[float, float], Tuple[float, float], float between 0 and 2pi
    """
    Return an ordered list of nodes representing the shortest? path starting from the node closest
    to [current_position] to the node closest to [goal_point].
    """
    # Find Nodes closest to start and end points
    graphKDTree = RoadNodeKDTree(graph)
    start = graphKDTree.getClosestForwardNode(current_position, heading)
    end = graphKDTree.getClosestNode(goal_point)

    # The priority of a node will be the length of discovered
    # shortest path from v to the node.
    F= RoadNodeHeap(False);

    # SandF contains the required node information for all nodes in the settled
    # and frontier sets.
    # Keys and RoadNode objects and Values are RoadNodeInfo objects.
    SandF = {}

    # Initialize Settled={}, F={start}, d[start]=0, bckptr[start]=None
    F.add(start, 0.0);
    SandF[start]= RoadNodeInfo(0.0, None);

    # Invariant:
    # (1) For a node s in Settled set S, a shortest v --> s path exists that
    # contains only settled nodes; d[s] is its length (distance) and bk[s] is
    # s's backpointer.
    # (2) For each node f in Frontier set F, a v --> f path exists that contains
    # only settled nodes except for f; d[f] is the length (distance) of the
    # shortest such path and bk[f] is f's backpointer on that path.
    # (3) All edges leaving S go to F.
    while (len(F) != 0):
        # f= node in F with minimum d value. The path to this node is the shortest path.
        f= F.poll()

        # if f is end we have found the route to take.
        if f==end:
            path = []
            while (end != None):
                path.append(end)
                end= SandF[end].bkptr
            # UNCOMMENT TO add path from current_position to start node (closest node)
            # path.append(RoadNode(current_position[0],current_position[1]))
            path.reverse()
            # UNCOMMENT TO add path from end node (node closest to goal) to goal_point
            # path.append(RoadNode(goal_point[0],goal_point[1]))
            return path

        fInfo= SandF[f]
        edges= f.edges
        for edge in edges:
            w= edge.getOther(f) #get neighbor for each edge.
            pathLength= fInfo.dist + edge.weight
            if w not in SandF.keys(): # if w is in far off set
                F.add(w, pathLength) # add it to the frontier
                SandF[w]= RoadNodeInfo(pathLength, f)
            elif pathLength < SandF[w].dist: # if w is in F and if newPath<d[w]
                wInfo= SandF[w]              # update priority and info
                wInfo.dist= pathLength
                wInfo.bkptr= f
                F.updatePriority(w, pathLength)
