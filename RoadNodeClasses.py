"""
Data structures for RoadNode to help find shortest path.
"""
import math
import matplotlib.pyplot as plt
from sklearn.neighbors import KDTree

class RoadNodeKDTree(KDTree):
    """
    KDTree of RoadNodes to get the nearest RoadNode to a given point.
    """

    def __init__(self, data):
        """
        Creates a KDTree of RoadNode objects.

        Parameter data: The RoadNode obejcts of the graph to create a KDTree for.
        Precondition: data is a List of RoadNode objects.
        """
        self._data = data
        self._coordinates = self._projectTo2D(self._data)
        super().__init__(self._coordinates)


    def getClosestNode(self, point):
        """
        Return closest RoadNode to the given point.

        Parameter point: the point to which the closest node is to be found.
        Precondition: point is a list of length 2
        """
        assert len(point)==2
        index = self.query([point], return_distance = False)[0][0]
        return self._data[index]


    def getClosestForwardNode(self, point, heading):
        """
        Return closest RoadNode in front of the given point.

        Parameter point: the point to which the closest node is to be found.
        Precondition: point is a list of length 2

        Parameter heading: the current heading of the object in radians
        Precondition: heading is between 0 and 2pi
        """
        assert 0<=heading<2*math.pi
        heading = heading - 2*math.pi if heading>math.pi else heading
        indices = self.query([point], k=4, return_distance = False)[0]
        for i in indices:
            nodeCoordinate = self._coordinates[i]
            vector = [nodeCoordinate[0]-point[0],nodeCoordinate[1]-point[1]]
            if vector[0]==0:
                if vector[1]==0:
                    angleFromPoint= 0
                else:
                    angleFromPoint = math.pi/2 if vector[1]>0 else -math.pi/2
            else:
                angleFromPoint = math.atan2(vector[1],vector[0])
            headingFromPoint = math.pi/2 - heading - angleFromPoint
            if (-math.pi/2 <= headingFromPoint <= math.pi/2):
                return self._data[i]
        return self.getClosestNode(point)


    @classmethod
    def plot(self, data , path, label = None):
        """
        Static procedure to plot data for visualization.

        Parameter data: graph to plot
        Precondition: data is a list of RoadNodes

        Parameter path: Indicates whether the data being plotted is a path
        Precondition: path is a boolean

        Optional Parameter label: Graph label
        """
        x=[];y=[];
        for node in data:
            x.append(node.x);y.append(node.y);
        plt.figure(1)
        if path:
            plt.plot(x,y,label=label)
        else:
            plt.scatter(x, y, label = label, s=1)
        plt.xlim(min(x)-1,max(x)+1)
        plt.ylim(min(y)-1,max(y)+1)
        plt.xlabel('x - axis')
        plt.ylabel('y - axis')
        plt.title('Bird\'s Eye View')
        plt.show()


    def _projectTo2D(self, data):
        """
        Helper function to extract a list of coordinates from a list of RoadNodes.

        Parameter data: The RoadNode obejcts of the graph to create a KDTree for.
        Precondition: data is a List of RoadNode objects.
        """
        result = []
        for node in data:
            result.append([node.x,node.y])
        return result


class RoadNodeInfo():
    """
    Class to store information regarding the shortest path of various RoadNodes.
    """
    # _dist= shortest known distance from the start node to this one
    # _bckptr= backpointer on path (with shortest known distance)
    #           from start node to this one

    @property
    def dist(self):
        return self._dist

    @property
    def bckptr(self):
        return self._bckptr

    @bckptr.setter
    def bckptr(self,bp):
        assert isinstance(bp,RoadNode)
        self._bckptr = bp

    @dist.setter
    def dist(self,d):
        assert type(d)==float
        self._dist = d

    def __init__(self, dist, bp):
        """
        Initializes an RoadNodeInfor with distance from the start node and a
        backpointer.

        Parameter dist: shortest known distance from the start node to this one
        Precondition: dist is a float

        Parameter bp: backpointer on path (with shortest known distance)
                        from start node to this one
        Precondition: bp is a RoadNode
        """
        self.dist= dist
        self.bkptr= bp

    def __str__(self):
        """
        Overrides python function str(RoadNodeInfo)
        """
        return "distance " + str(self._dist) + ", backpointer " + str(self._bkptr)


class RoadNodeHeap():
    """
    Class to maintain min or max heaps for RoadNodes
    """
	# _isMaxHeap = indicator if the heap is min or max
	# _elementList = list of elements of the heap
	# _size = number of elements in the heap
	# _map = mapping of elements to ids in the heap.

    def __init__(self,isMax):
        """
        Initializes an empty max-RoadNodeHeap if isMax is true.
        Initializes an empty min-RoadNodeHeap if isMax is false.

        Parameter isMax: whether the heap is a max heap or min heap
        Precondition: isMax is a bool
        """
        assert type(isMax) == bool
        self._isMaxHeap= isMax
        self._elementList= []
        self._size = 0
        self._map = {}


    class Element():
        """
        An object of class Element houses a RoadNode and its priority in the
        RoadNodeHeap
        """
        #_node the road node
        #_priority the priority in the heap of the roadnode

        @property
        def node(self):
            return self._node

        @property
        def priority(self):
            return self._priority

        @priority.setter
        def priority(self, p):
            self._priority=p

        def __init__(self, node, priority):
            """
            Initializes an Element of RoadNodeHeap storing the node and priority
            """
            self._node= node
            self._priority= priority


        def __str__(self):
            """
            Overrides python function str(RoadNodeHeap.Element)
            """
            return "(" + str(self._node) + ", " + str(self._priority) + ")"


        def __eq__(self,ob):
            """
            Overrides "==" for elements
            """
            if ob is None or not isinstance(ob, RoadNodeHeap.Element):
                return False
            return self._node == ob.node and self._priority == ob.priority


    def add(self,node, priority):
        """
        Add a node with the respective priority the heap.
        Expected: O(log(size))
        Worst-case: O(size)

        Parameter node: the element to be added in the heap
        Precondition: node is not in the heap

        Parameter priority: the priority of node
        Precondition: priority is a float
        """
        if (node in self._map.keys()):
            raise Exception("node is already in the heap")
        self._map[node]= self._size
        self._elementList.append(self.Element(node, priority));
        self._size+= 1;
        self._bubbleUp(self._size - 1);

    def __len__(self):
        """
        Overrides python function "len(RoadNodeHeap)"
        """
        return self._size;

    def _swap(self, h, k):
        """
        Helper method to maintain class invariant
        """
        assert 0<=h<self._size and 0<=k<self._size;
        temph= self._elementList[h]
        tempk= self._elementList[k]
        self._elementList[h]= tempk
        self._elementList[k]= temph
        self._map[self._elementList[h].node]= h
        self._map[self._elementList[k].node]= k


    def _compareTo(self,p1, p2):
        """
        Helper method to maintain class invariant
        """
        if p1 == p2:
            return 0
        if self._isMaxHeap:
            return 1 if p1 > p2 else -1
        return 1 if p1 < p2 else -1;

    def _compareToIndex(self,h,k):
        """
        Helper method to maintain class invariant
        """
        return self._compareTo(self._elementList[h].priority, self._elementList[k].priority)

    def _bubbleUp(self,h):
        """
        Helper method to maintain class invariant
        """
        if h >= self._size:
            return
        # Invariant: 0 <= h < size and<br>
        #.......... The class invariant is true, except perhaps<br>
        #.......... that b[h] belongs above its parent (if h > 0) in the heap, not below it.
        while h > 0:
            p= (h - 1) // 2; # p is h's parent
            if self._compareToIndex(h, p) <= 0:
                return
            self._swap(h, p)
            h= p

    def peek(self):
        """
        Return node value with lowest priority if heap in min.
        Return node value with highest priority if heap in max.

        Time: O(1)

        If heap is empty, raises exception.
        """
        if self._size <= 0:
            raise Exception("heap is empty")
        return self._elementList[0].node

    def _bubbleDown(self, h):
        """
        Helper method to maintain class invariant
        """
        if h < 0 or self._size <= h:
            return
        k= 2*h + 1
        # Invariant: Class invariant is true except perhaps that
        # .......... b[h] belongs below one or both of its children and
        # .......... k is h's left child.
        while k < self._size: # while b[h] has a child
            uc= k if (k + 1 == self._size or self._compareToIndex(k, k + 1) >= 0) else k + 1 # uc is the bigger child
            if self._compareToIndex(h, uc) >= 0:
                return;
            self._swap(h, uc)
            h= uc
            k= 2*h + 1

    def poll(self):
        """
        Return and remove node value with lowest priority if heap in min.
        Return and remove node value with highest priority if heap in max.

        Expected time: O(log(size))
        Worst-case time: O(size)

        If heap is empty, raises exception.
        """
        if (self._size <= 0):
            raise Exception("heap is empty")
        node= self._elementList[0].node
        self._swap(0, self._size - 1)
        del self._elementList[self._size - 1]
        del self._map[node]
        self._size-= 1
        self._bubbleDown(0)
        return node

    def updatePriority(self, node, priority):
        """
        Change the priority of value of a node in the heap.
        Expected: O(log(size))
        Worst-case: O(size)

        Parameter node: the element whose priority needs updating
        Precondition: node is in the heap

        Parameter priority: the new priority of node
        Precondition: priority is a float
        """
        index= self._map[node]
        oldPriority= self._elementList[index].priority
        self._elementList[index].priority= priority
        t= self._compareTo(priority, oldPriority)
        if t == 0:
            return
        elif t < 0:
            self._bubbleDown(index)
        else:
            self._bubbleUp(index)
