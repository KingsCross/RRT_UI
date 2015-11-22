# Developed by T.S.
# RRT Algorithm Research Project

import sys, random, math, pygame,time
from pygame.locals import *
from math import *

# global variables
cmPerPx = 4 # 1 px = 4 cm
xLength = 500 
yLength = 500 
delta =  25# delta disk for final node capture (1 m radius)
max_dist = 50 #  max distance from waypoint to waypoint (1 m)
max_nodes = 10000

white = 230, 230, 240 #for edges
yellow = 230,230,0 #for node
black = 10, 10, 10 #for screen
red = 255, 0, 0 #for target node
green = 0, 255, 0 #for start node
blue = 100,100,255 #for wall
other = 200,150,255 # for other stuff

# 2-D array of wall lines. comment out if not using
# initial_loc = 320,50 # in new coord. sys.(origin in bottom left)
# target_loc = 450,450 
# wallArr = [
# 		   [(500,250),(125,250)],
# 		   [(250,0),(250,150)],
# 		   [(250,350),(250,250)],
# 		   [(250,425),(250,500)],
# 		   [(0,250),(25,250)],
# 		   [(350,150),(250,150)]
# 		   ]

initial_loc = 50,50
target_loc = 450,50
wallArr = [
		   [(100,0),(100,400)],
		   [(250,500),(250,100)],
		   [(400,0),(400,400)],
		   ]


####################################################################################
######################### GRAPH STRUCTURE MANIPULATION #############################
####################################################################################

# adds node to graph data structure/dictionary
def appendNodeToGraph (g,source,target):
	if not (target in g.keys()):
		g[target]=None
	if not (source in g.keys()):
		return
	elif g[source] == None:
		g[source] = [target]
	else:
		g[source].append(target)

# main function to add a new target node into the graph/tree
# calls other functions to constrain node, add it to graph, & display it
def addTargetNode (g,source,target):
	newTargNode = constrainTargetNode(source,target)
	# display target node
	# displayNode(newTargNode,'other')
	# print "New Target Node:", newTargNode
	# print "From Source Node: ", source
	# print "is node of wall: ", isNodeOffWall(source,newTargNode)
	# displayEdge(source,newTargNode,'test')
	if (isNodeOffWall(source,newTargNode)):
		appendNodeToGraph(g,source,newTargNode)
		displayEdge(source,newTargNode)
		return newTargNode
	return False

# check to see if the node is off all walls
def isNodeOffWall(source,target):
	for wallLine in wallArr:
		iPoint = getIntersectionPoint(source,target,wallLine[0],wallLine[1])
		if iPoint != False:
			if isPointOnLine(iPoint,wallLine[0],wallLine[1]) == True and isPointOnLine(iPoint,source,target) == True:
				return False
	return True

# find the nearest node
def getNearestNode (g,node):
	# initializing variables
	nearestNode = initial_loc
	closestDist = distBtwnNodes(node,initial_loc)
	for key in g.keys():
		actDist = distBtwnNodes(node,key)
		if actDist < closestDist:
			closestDist = actDist
			nearestNode = key

	for vals in g.values():
		if not (vals == None):
			for valNode in vals:
				actDist = distBtwnNodes(node,valNode)
				if  actDist < closestDist:
					closestDist = actDist
					nearestNode = valNode

	return nearestNode

####################################################################################
############################## TRACEBACK FUNCTIONS #################################
####################################################################################
# tracing back from the target, to the source
# this is more efficient (as i)
def getPath(graph,node=target_loc,path=[]):
	path.append(node)
	if node == initial_loc:
		return path
	for origin,targetNodes in graph.items():
		if targetNodes != None:
			for target in targetNodes:
				if target == node:
					path.append(origin)
					getPath(graph,origin,path)
	return path

def displayPath(path):
	for i in range(0,len(path)-1):
		nodeA = path[i]
		nodeB = path[i+1]
		displayEdge(nodeA,nodeB,'path')


####################################################################################
####################### MATHEMATICAL NODE MANIPULATION #############################
####################################################################################

# transform from default coordinate axis so that
# origin is in bottom left position
# output value is transformed value to this coord. sys.
def transformCoord (coord):
	newY = yLength - coord[1]
	return coord[0],newY

# find the distance between the nodes
def distBtwnNodes (coordA, coordB):
	return math.sqrt(pow((coordA[0]-coordB[0]),2)+pow((coordA[1]-coordB[1]),2))

# constrain the target node to the max_dist limit
def constrainTargetNode (source,target):
	if distBtwnNodes(source,target) <= max_dist:
		return target
	dx = target[0]-source[0]
	dy = target[1]-source[1]
	theta = atan2(dy,dx)
	x_new = int(source[0]+max_dist*cos(theta))
	y_new = int(source[1]+max_dist*sin(theta))
	return x_new,y_new

# get the intersection point between 2 lines (each defined by 2 points)
# reference: https://www.topcoder.com/community/data-science/data-science-tutorials/geometry-concepts-line-intersection-and-its-applications/
def getIntersectionPoint (p1_lineA, p2_lineA, p1_lineB, p2_lineB):
	A_1 = p2_lineA[1] - p1_lineA[1]
	B_1 = p1_lineA[0] - p2_lineA[0]
	C_1 = A_1*p1_lineA[0] + B_1*p1_lineA[1]

	A_2 = p2_lineB[1] - p1_lineB[1]
	B_2 = p1_lineB[0] - p2_lineB[0]
	C_2 = A_2*p1_lineB[0] + B_2*p1_lineB[1]

	det = A_1*B_2 - A_2*B_1

	if not (det == 0):
		x_int = (B_2*C_1 - B_1*C_2)/det
		y_int = (A_1*C_2 - A_2*C_1)/det
		return x_int,y_int
	return False

# check to see if a point is on a line specified by 2 points
def isPointOnLine (p,p1_line,p2_line):
	# check if min(p1_line_x,p2_line_x) < p_x < min(p1_line_x,p2_line_x) and same for y
	if min(p1_line[0],p2_line[0]) <= p[0] and p[0] <= max(p1_line[0],p2_line[0]) :
		if min(p1_line[1],p2_line[1]) <= p[1] and p[1] <= max(p1_line[1],p2_line[1]) :
			return True
	return False


####################################################################################
################################# PYGAME DISPLAY ###################################
####################################################################################

# display a node (vertex)
def displayNode (node,str=''):
	newNode = transformCoord(node)
	#TODO: inefficient way of choosing color & size. Just send them as an fxn arg.
	if str == 'initial':
		pygame.draw.circle(screen,green,newNode,7,0)
	elif str == 'final':
		pygame.draw.circle(screen,red,newNode,7,0)
		pygame.draw.circle(screen,red,newNode,delta,1)
	elif str == 'other':
		pygame.draw.circle(screen,other,newNode,1,0)
	else:
		pygame.draw.circle(screen,yellow,newNode,1,0)
	pygame.display.update()	

# display an edge (line)
def displayEdge (source,target,str=''):
	newSource = transformCoord(source)
	newTarget = transformCoord(target)
	#TODO: inefficient way of choosing color. Just send it as an fxn arg.
	color = blue
	width = 1
	if str=='wall':
		color = white
	elif str == 'test':
		color = 80,80,80
	elif str == 'path':
		color = green
		width = 2
	pygame.draw.line(screen,color,newSource,newTarget,width)
	pygame.display.update()	

def initView ():
	displayNode(initial_loc,'initial')
	displayNode(target_loc,'final')
	for wallLine in wallArr:
		displayEdge(wallLine[0],wallLine[1],'wall')

####################################################################################
################################## RRT FUNCTIONS ###################################
####################################################################################

# generate a node at a random location
def generateRandNode ():
	return (random.randint(0,xLength),random.randint(0,yLength))

# main function for creating RRT
def runRRT(graph):
	count = 0
	foundTarget = False
	while count < max_nodes and foundTarget==False:
		randNode = generateRandNode()
		
		# print "Random Node:", randNode 
		if graph[initial_loc] == None:
			nearNode = initial_loc
		else:
			nearNode = getNearestNode(graph,randNode)
		# print "Nearest Node:", nearNode
		newNode = addTargetNode(graph,nearNode,randNode)

		if not (newNode == False):
			if (distBtwnNodes(target_loc,newNode) <= delta):
				appendNodeToGraph(graph,newNode,target_loc)
				displayEdge(newNode,target_loc)
				foundTarget = True
				connectingPath = getPath(graph)
				displayPath(connectingPath)

		#print graph

		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				sys.exit()
		count+=1

# main function that displays initial & target nodes 
# initializes graph, and starts the RRT program
def main():
	running = True
	# displayEdge(initial_loc,target_loc)
	initView()
	graph = {initial_loc:None} #initialize graph/tree
	# print graph
	calledRRT = False
	while (running):

		if (calledRRT == False):
			start = time.clock()
			runRRT(graph)
			end = time.clock()
			print "Time to find target node:", end-start
			calledRRT = True

		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False
				sys.exit()

if __name__ == '__main__':
	pygame.init()
	screen = pygame.display.set_mode([xLength,yLength])
	pygame.display.set_caption('RRT Display Window')
	screen.fill(black)
	main()