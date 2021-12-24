import csv
import sys

def loadKRF(filename):
	"""Loads a Kaolin Routing File into a list of nodes.

	Returns nodeList in the format [{routing, connection, orientation, x, y}]
	"""

	csvFile = open(filename, newline = '')
	csvReader = csv.reader(csvFile, delimiter='	')

	nodeList = []
	for node in csvReader:
		line = int(node[0])
		routingCode, connectionCode, orientationCode = node[1]
		
		routing = {'E':'endpoint', 'M':'midpoint', 'T':'tee', 'I':'isolated'}[routingCode]
		connection = {'N':'none', 'P':'pad', 'S':'smd', 'H':'hole'}[connectionCode]
		orientation = {'V': 'vertical', 'Z': 'horizontal', 'C': 'compressed', 'X':'none'}[orientationCode]
		xPosition = float(node[2])
		yPosition = float(node[3])
		nodeList += [{'node':line, 'routing':routing, 'connection': connection, 
						'orientation':orientation, 'xPosition':xPosition,
						'yPosition': yPosition}]
	return nodeList


def move(node):
	print(str(node['node']) + " " + node['routing'].upper() + " (X"+str(node['xPosition']) + " Y"+str(node['yPosition']) + ")")

def isFirstNodeInPath():
	return currentPosition == pathStartPosition

def startOfPathNotice():
	print("")
	print("START OF PATH")

def startPath():
	global pathStartPosition
	global junctionRecord
	global currentJunctionPosition
	global passive
	pathStartPosition = currentPosition
	junctionRecord = 0
	currentJunctionPosition = 0
	passive = 0

# READ HEAD FUNCTIONS
def goForward():
	global currentPosition
	global endPosition
	global direction

	currentPosition += 1
	endPosition = currentPosition
	direction = 1

def goReverse():
	global currentPosition
	global direction

	currentPosition -= 1
	direction = 0

def skipTerminal():
	global endPosition
	global currentPosition
	global direction
	global currentJunctionPosition

	endPosition += 1
	currentPosition = endPosition
	direction = 1
	currentJunctionPosition = 0

def isPassive():
	return passive

def setPassive():
	global passive
	passive = 1

def setActive():
	global passive
	passive = 0

# JUNCTION TRACKER FUNCTIONS
def newJunction():
	global junctionRecord
	global currentJunctionMask
	global currentJunctionPosition
	junctionRecord = (junctionRecord << 2) + 1
	currentJunctionPosition = 0


def visitJunction():
	"""Returns the state of the visited junction.

	0: Junction is passive
	2: Second visit to junction
	3: Third visit to junction
	"""
	global junctionRecord
	global currentJunctionPosition

	#right shift and mask
	junctionState = (junctionRecord >> currentJunctionPosition) & 0b11

	#SECOND VISIT
	if junctionState == 1:
		#change junction state to 2
		junctionRecord += (1<<currentJunctionPosition)
		currentJunctionPosition = currentJunctionPosition + 2
		return 2

	#THIRD VISIT
	elif junctionState == 2:
		#This is our third visit. Change junction state to 0 to inactivate going forwards.
		junctionRecord &= ~(2<<currentJunctionPosition)
		currentJunctionPosition = currentJunctionPosition + 2
		return 3

	elif junctionState == 0:
		#The junction is inactive
		currentJunctionPosition = currentJunctionPosition + 2
		return 0





def traverseNodes(nodeList):
	"""A test algorithm to traverse the node list."""

	while True:
		node = nodeList[currentPosition]

		# GOING FORWARDS
		if direction == 1:
			if isFirstNodeInPath(): startOfPathNotice()
			move(node)

			#FIRST NODE IN PATH
			if isFirstNodeInPath():
				goForward()

				#ISOLATED NODE
				if node['routing'] == 'isolated':
					startPath()
					continue
				
				#START OF A PATH
				else:
					continue

			#IN PATH
			else:
				#ENDPOINT
				if node['routing'] == 'endpoint':
					goReverse()

				#MIDPOINT
				elif node['routing'] == 'midpoint':
					goForward()

				#TEE
				elif node['routing'] == 'tee':
					newJunction()
					goForward()
				continue


		# GOING IN REVERSE
		if direction == 0:
			#BACK AT THE START, DONE WORKING ON PATH
			if isFirstNodeInPath():
				move(node)
				skipTerminal()
				if currentPosition < len(nodeList):
					startPath()
					continue
				else:
					print("")
					return

			#ENDPOINT
			if node['routing'] == 'endpoint':
				setPassive() #we aren't at the start, so we're going passive.
				# Hitting an non-path-start endpoint in reverse simply means we're reading
				# a non-active path.
				goReverse()
				continue

			#MIDPOINT
			elif node['routing'] == 'midpoint':
				#PASSIVE
				if isPassive():
					goReverse()
					continue
				#ACTIVE
				else:
					move(node)
					goReverse()
					continue

			#TEE
			elif node['routing'] == 'tee':
				junctionState = visitJunction()
				#Hit a junction for the second time
				if junctionState == 2:
					move(node)
					setActive()
					skipTerminal()
					continue
				#Hit a junction for the third time, just pass thru it
				elif junctionState == 3:
					move(node)
					setActive()
					goReverse()

				elif junctionState == 0:
					goReverse()
					setPassive()
					continue


# Initialize state. Let's keep track of how much state is required.
pathStartPosition = 0 #the memory index of the path start
currentPosition = 0 #tracks the index of the current read position
terminalPosition = 0 #tracks the index of the last read position
junctionRecord = 0 #records all encountered T junctions within a path
					#this maps to the orde in which they are stored in memory
currentJunctionPosition = 0 #the current position within the junction record
direction = 1 #1: forwards, 0: reverse
passive = 0 #if 1, we are not reading nodes in reverse

nodeList = loadKRF(sys.argv[1])
traverseNodes(nodeList)

