from RoadNodeClasses import RoadNodeKDTree, RoadNodeHeap
from route_planner_py import *

# NOTE: Test script is not comprehensive, more of a sanity check.

def testGraph():
    a = RoadNode(0.0, 0.0)
    b = RoadNode(1.0, 0.0)
    c = RoadNode(0.0, 1.0)
    d = RoadNode(1.0, 1.0)
    e = RoadNode(0.5, 0.5)
    Edge(2, a, b)
    Edge(4, a, c)
    Edge(2, b, d)
    Edge(2, c, d)
    Edge(1, a, e)
    Edge(2, b, e)
    Edge(2, c, e)
    Edge(4, d, e)
    for edge in e.edges:
        if edge.getOther(e)==a:
            assert edge.weight==1
        elif edge.getOther(e)==b:
            assert edge.weight==2
        elif edge.getOther(e)==c:
            assert edge.weight==2
        elif edge.getOther(e)==d:
            assert edge.weight==4
    for node in e.neighbors:
        if node==a:
            assert node.x==0.0 and node.y == 0.0
        elif node==b:
            assert node.x==1.0 and node.y == 0.0
        elif node==c:
            assert node.x==0.0 and node.y == 1.0
        elif node==d:
            assert node.x==1.0 and node.y == 1.0
    return [a,b,c,d,e]

def testRoadNodeKDTree():
    nodeList = testGraph()
    graphTree = RoadNodeKDTree(nodeList)
    assert graphTree.getClosestNode((0.0,0.0))==nodeList[0]
    assert graphTree.getClosestNode((1.0,0.0))==nodeList[1]
    assert graphTree.getClosestNode((0.0,1.0))==nodeList[2]
    assert graphTree.getClosestNode((1.0,1.0))==nodeList[3]
    assert graphTree.getClosestNode((0.5,0.5))==nodeList[4]
    assert graphTree.getClosestNode((0.1,0.2))==nodeList[0]
    assert graphTree.getClosestNode((0.9,0.1))==nodeList[1]
    assert graphTree.getClosestNode((-0.2,1.3))==nodeList[2]
    assert graphTree.getClosestNode((2.0,2.0))==nodeList[3]
    assert graphTree.getClosestNode((0.6,0.4))==nodeList[4]
    assert graphTree.getClosestForwardNode((0.1,0.1),math.pi/4)==nodeList[4]
    assert graphTree.getClosestForwardNode((0.2,0.1),3*math.pi/4)==nodeList[1]
    assert graphTree.getClosestForwardNode((0.1,0.1),5*math.pi/4)==nodeList[0]
    assert graphTree.getClosestForwardNode((0.1,0.2),7*math.pi/4)==nodeList[2]

def testRoadNodeHeap():
    nodeHeap = RoadNodeHeap(False)
    nodeList = testGraph()
    for i in range(len(nodeList)):
        nodeHeap.add(nodeList[i],5-i)
    # Heap should be
    #         e
    #      d      c
    #   b     a
    assert len(nodeHeap)==5
    assert nodeHeap.peek() == nodeList[4]
    nodeHeap.updatePriority(nodeList[4],10)
    assert nodeHeap.poll() == nodeList[3]
    assert nodeHeap.peek() == nodeList[2]
    nodeHeap.updatePriority(nodeList[0],0)
    nodeHeap.peek == nodeList[0]

def testShortestPathFinder():
    g = testGraph()
    path= [g[0],g[1],g[3]]
    assert path==plan_route(g,(0.1,-0.1),(1.0,2.0))
    path= [g[0]]
    assert path==plan_route(g,(0.1,-0.1),(-0.1,0.1))
    path= [g[0],g[4],g[2]]
    assert path==plan_route(g,(0.4,-0.4),(0.4,2.0))

testRoadNodeKDTree()
testRoadNodeHeap()
testShortestPathFinder()
