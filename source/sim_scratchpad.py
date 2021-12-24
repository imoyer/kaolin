import sys
import utilities

def move(node):
	print(str(node['node']) + " " + node['routing'].upper() + " (X"+str(node['xPosition']) + " Y"+str(node['yPosition']) + ")")

def sim_isFirstNodeInPath(self):
	return self.sim_currentPosition == self.sim_pathStartPosition

def startOfPathNotice():
	print("")
	print("START OF PATH")

def sim_startPath(self):
	self.sim_pathStartPosition = self.sim_currentPosition
	self.sim_junctionRecord = 0
	self.sim_currentJunctionPosition = 0
	self.sim_passive = 0

# READ HEAD FUNCTIONS
def sim_goForward(self):
	self.sim_currentPosition += 1
	self.sim_terminalPosition = self.sim_currentPosition
	self.sim_direction = 1

def sim_goReverse(self):
	self.sim_currentPosition -= 1
	self.sim_direction = 0

def sim_skipTerminal():
	global self.sim_terminalPosition
	global self.sim_currentPosition
	global self.sim_direction
	global self.sim_currentJunctionPosition

	self.sim_terminalPosition += 1
	self.sim_currentPosition = self.sim_terminalPosition
	self.sim_direction = 1
	self.sim_currentJunctionPosition = 0

def sim_isPassive(self):
	return self.sim_passive

def sim_setPassive():
	self.sim_passive = 1

def setActive():
	global self.sim_passive
	self.sim_passive = 0

# JUNCTION TRACKER FUNCTIONS
def newJunction():
	global self.sim_junctionRecord
	global currentJunctionMask
	global self.sim_currentJunctionPosition
	self.sim_junctionRecord = (self.sim_junctionRecord << 2) + 1
	self.sim_currentJunctionPosition = 0


def sim_visitJunction():
	"""Returns the state of the visited junction.

	0: Junction is self.sim_passive
	2: Second visit to junction
	3: Third visit to junction
	"""
	global self.sim_junctionRecord
	global self.sim_currentJunctionPosition

	#right shift and mask
	junctionState = (self.sim_junctionRecord >> self.sim_currentJunctionPosition) & 0b11

	#SECOND VISIT
	if junctionState == 1:
		#change junction state to 2
		self.sim_junctionRecord += (1<<self.sim_currentJunctionPosition)
		self.sim_currentJunctionPosition = self.sim_currentJunctionPosition + 2
		return 2

	#THIRD VISIT
	elif junctionState == 2:
		#This is our third visit. Change junction state to 0 to inactivate going forwards.
		self.sim_junctionRecord &= ~(2<<self.sim_currentJunctionPosition)
		self.sim_currentJunctionPosition = self.sim_currentJunctionPosition + 2
		return 3

	elif junctionState == 0:
		#The junction is inactive
		self.sim_currentJunctionPosition = self.sim_currentJunctionPosition + 2
		return 0





def traverseNodes(nodeList):
	"""A test algorithm to traverse the node list."""

	while True:
		node = nodeList[self.sim_currentPosition]

		# GOING FORWARDS
		if self.sim_direction == 1:
			if sim_isFirstNodeInPath(): startOfPathNotice()
			move(node)

			#FIRST NODE IN PATH
			if sim_isFirstNodeInPath():
				sim_goForward()

				#ISOLATED NODE
				if node['routing'] == 'isolated':
					sim_startPath()
					continue
				
				#START OF A PATH
				else:
					continue

			#IN PATH
			else:
				#ENDPOINT
				if node['routing'] == 'endpoint':
					sim_goReverse()

				#MIDPOINT
				elif node['routing'] == 'midpoint':
					sim_goForward()

				#TEE
				elif node['routing'] == 'tee':
					newJunction()
					sim_goForward()
				continue


		# GOING IN REVERSE
		if self.sim_direction == 0:
			#BACK AT THE START, DONE WORKING ON PATH
			if sim_isFirstNodeInPath():
				move(node)
				sim_skipTerminal()
				if self.sim_currentPosition < len(nodeList):
					sim_startPath()
					continue
				else:
					print("")
					return

			#ENDPOINT
			if node['routing'] == 'endpoint':
				setself.sim_passive() #we aren't at the start, so we're going self.sim_passive.
				# Hitting an non-path-start endpoint in reverse simply means we're reading
				# a non-active path.
				sim_goReverse()
				continue

			#MIDPOINT
			elif node['routing'] == 'midpoint':
				#self.sim_passive
				if isself.sim_passive():
					sim_goReverse()
					continue
				#ACTIVE
				else:
					move(node)
					sim_goReverse()
					continue

			#TEE
			elif node['routing'] == 'tee':
				junctionState = sim_visitJunction()
				#Hit a junction for the second time
				if junctionState == 2:
					move(node)
					setActive()
					sim_skipTerminal()
					continue
				#Hit a junction for the third time, just pass thru it
				elif junctionState == 3:
					move(node)
					setActive()
					sim_goReverse()

				elif junctionState == 0:
					sim_goReverse()
					setself.sim_passive()
					continue


# Initialize state. Let's keep track of how much state is required.
self.sim_pathStartPosition = 0 #the memory index of the path start
self.sim_currentPosition = 0 #tracks the index of the current read position
self.sim_terminalPosition = 0 #tracks the index of the last read position
self.sim_junctionRecord = 0 #records all encountered T junctions within a path
							#this maps to the orde in which they are stored in memory
self.sim_currentJunctionPosition = 0 #the current position within the junction record
self.sim_direction = 1 #1: forwards, 0: reverse
self.sim_passive = 0 #if 1, we are not reading nodes in reverse


print("")
print("KAOLIN SIMULATOR")

if len(sys.argv == 1):
	print("####  USAGE  ####")
	print("python3 simulate.py -mode [-filename] [-address]")
	print("")
	print("MODES")
	print("  file -- reads from a KRF file. Please provide filename as next parameter.")
	print("  chip -- reads directly from a chip in a UPP. Please provide the starting memory address in hex format")
	exit()

elif sys.arvg[1] == 'file':
	nodeList = utilities.loadKRFFromFile(sys.argv[2])
elif sys.argv[1] == 'chip':
	nodeList = utilities.loadKRFFromChip(sys.argv[2])


traverseNodes(nodeList)

