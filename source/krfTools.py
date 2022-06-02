# "KAOLIN ROUTING FORMAT" TOOLS
# A PART OF THE KAOLIN PROJECT

# ILAN E. MOYER
# DEC. 23RD, 2021

# This module provides tools for interacting with the Kaolin Routing Format.
# What is Kaolin? -- well, the goal is evolving, but right now the basic idea is to create a system by which
# Doppel nodes can store their own PCB information alongside their microcode. This would enable a user to simply
# plug a Doppel personality chip into a specialized PCB milling machine (perhaps controlled by a Doppel network)
# and for that machine to route the PCB for a doppel node. This starts to get more exciting when you imagine devices
# that are constructed by tiled doppel nodes, and each node has a modular PCB design that can be tiled right in the
# PCB milling machine.

# The Kaolin Routing Format is quite limited, but a brief summary of the format is as follows:
#
# Traces are defined as a series of connected nodes. Nodes can be of four Routing types:
#     (E)ndpoint
#     (M)idpoint		Midpoints have two junctions
#     (T)ee 			Tees always have exactly three junctions
#     (I)solated
#
# Each node can have four types of Connection to electrical components:
#     (N)one
#     (P)ad
#     (S)MD
#     (H)ole
#
# These connections can have four Orientations:
#     (V)ertical
#     (H)orizontal
#     (C)ompressed
#     (X) None
#
# Each node is positioned in XY with a resolution of 0.01 inches, and 9 bits of dynamic range (5.12 inchs)
# 
# A KRF file is stored as a sequence following this general pattern:
#     [Node #] <tab> [Routing][Connection][Orientation] <tab> [X Position] <tab> [Y Position]
#
# For example:
#      1	EPC		0.3		0
#      2	TNX		0.3		0.2
#      3	EPC		0		0.2
#      4	TNX		0.3		0.4
#      5	TNX		0.3		0.6
#      6	EPC		0		0.6
#      7	TNX		0.3		0.8
#      8	EPC		0		0.8
#      9	EPC		0.3		1
#      10	EPC		0.6		0.4
#
# These files are sequenced according to the following rules:
#      (1) They always start at an endpoint
#      (2) Nodes are listed in the order they are encountered, always making a right-hand turn at a Tee.
#      (3) Each node only appears once in the sequence.
#      (3) Multiple paths are simply listed one after the other. The topological rules allow the interpreter to identify these as distinct.
#
# BYTE ENCODING
#
# Each node is encoded in 3 bytes, which appear in memory in big endian sequence.
#      [XX][XX][XX][XXXXXXXXX][XXXXXXXXX]
#       R   C   O   X          Y
#       O   O   R
#       U   N   I   P          P
#       T   N   E   O          O
#       I   E   N   S          S
#       N   C   T   I          I
#       G   T   A   T          T
#           I   T   I          I
#           O   I   O          O
#           N   O   N          N
#               N
#
#       ROUTING: [00: Endpoint, 01: Midpoint, 10: Tee, 11: Isolated]
#       CONNECTION: [00: None, 01: Pad, 10: SMD, 11: Hole]
#       ORIENTATION: [00: Vertical, 01: Horizontal, 10: Compressed, 11: None]
#       X POSITION: Encoded as 9 bytes, with a resolution of 0.01"
#       Y POSITION: Encoded as 9 bytes, with a resolution 0f 0.01"


import csv
import os, sys
from pygestalt import nodes
from pygestalt import interfaces
from doppel.UPP import UPPGestaltNode #For reading directly from a chip
import time, math

import urumbu_xyz_mill

import eagleToKRF, toolpath


class krf(list):
	"""The KRF node list object, which stores an ordered list of nodes.

		Each node is stored as a dictionary in the following format:
		
		{'index': lineNumber,
		'routing':'endpoint' or 'midpoint' or 'tee' or 'isolated',
		'connection': 'none' or 'pad' or 'smd' or 'hole',
		'orientation': 'vertical' or 'horizontal' or 'compressed' or 'none'
		'position': (xPosition, yPosition)}

		where xPosition and yPosition are stored as standard inches, i.e. with three significant digits.
	"""
	def __str__(self):
		"""Returns a formatted representation of the KRF file."""
		returnString = "\nKAOLIN ROUTING FILE (" + str(len(self)) + " NODES) \n"

		for node in self:
			returnString += "[" + str(node['index']) + "]" + chr(9) + {'endpoint':'E', 'midpoint':'M', 'tee':'T', 'isolated':'I'}[node['routing']] + " (X " + str(node['position'][0]) + ", Y " + str(node['position'][1]) + ")\n"

		return returnString


	def travel(self):
		"""Travels the node list and returns an itinerary.

		Returns itinerary -- a list of paths that were traveled [[stop1, stop2, stop3, ...], [stop1, stop2, stop3]]
		"""

		thisItinerary = itinerary()
		pathList = []
		self.travel_initializeParameters()


		while True:

			node = self[self.travel_currentPosition]

			# GOING FORWARDS
			if self.travel_direction == 1:
				pathList += [node] #add node to the pathList

				#FIRST NODE IN PATH
				if self.travel_isFirstNodeInPath():
					self.travel_goForward()

					#ISOLATED NODE
					if node['routing'] == 'isolated':
						thisItinerary += [pathList]
						pathList = []
						self.travel_startPath()
						continue
					
					#START OF A PATH
					else:
						continue

				#IN PATH
				else:
					#ENDPOINT
					if node['routing'] == 'endpoint':
						self.travel_goReverse()

					#MIDPOINT
					elif node['routing'] == 'midpoint':
						self.travel_goForward()

					#TEE
					elif node['routing'] == 'tee':
						self.travel_newJunction()
						self.travel_goForward()
					continue


			# GOING IN REVERSE
			if self.travel_direction == 0:
				#BACK AT THE START, DONE WORKING ON PATH
				if self.travel_isFirstNodeInPath():
					pathList += [node]
					self.travel_skipTerminal()
					if self.travel_currentPosition < len(self):
						thisItinerary += [pathList]
						pathList = []
						self.travel_startPath()
						continue
					else:
						thisItinerary += [pathList]
						return thisItinerary

				#ENDPOINT
				if node['routing'] == 'endpoint':
					self.travel_setPassive() #we aren't at the start, so we're going travel_passive.
					# Hitting an non-path-start endpoint in reverse simply means we're reading
					# a non-active path.
					self.travel_goReverse()
					continue

				#MIDPOINT
				elif node['routing'] == 'midpoint':
					#PASSIVE
					if self.travel_isPassive():
						self.travel_goReverse()
						continue
					#ACTIVE
					else:
						pathList += [node]
						self.travel_goReverse()
						continue

				#TEE
				elif node['routing'] == 'tee':
					junctionState = self.travel_visitJunction()
					#Hit a junction for the second time
					if junctionState == 2:
						pathList += [node]
						self.travel_setActive()
						self.travel_skipTerminal()
						continue
					#Hit a junction for the third time, just pass thru it
					elif junctionState == 3:
						pathList += [node]
						self.travel_setActive()
						self.travel_goReverse()

					elif junctionState == 0:
						self.travel_goReverse()
						self.travel_setPassive()
						continue	

	def travel_initializeParameters(self):
		# POSITIONAL INDICES INTO THE NODE LIST
		self.travel_pathStartPosition = 0 	#index of the current path start
		self.travel_currentPosition = 0 	#index of the current read position
		self.travel_terminalPosition = 0 	#index of the last position to be read


		# JUNCTION RECORD
		# Here we keep a record of all the tee junctions that have been traversed,
		# in the order of how they appear in the node list.

		self.travel_junctionRecord = 0 # a bitfield, ...[XX][XX][XX][XX]
									# pairs of bits are appended from the right, whenever a new tee is encountered
									# while traversing the nodeList in the forwards direction.
									# Therefore the least significant bits represent the bottom of the nodeList (as currently traversed)
									# 00: 	Tee has been traveled 3 times already, and will be treated as out-of-path going
									# 01: 	Tee was has only been traveled once, and in the forward direction (first time is always forwards!)
									#		Next traverse will be in reverse, and will branch forwards to the end of the nodeList
									# 10: 	Tee has been traveled 2 times. Next traverse will be in reverse, and will pass thru the T.
		self.travel_currentJunctionPosition = 0 	# the current bit position within the junction record.
												# This is always 0 when traveling forwards

		# TRAVERSE STATE
		self.travel_direction = 1 #1: traversing forwards down the node list, 0: traversing in reverse down the node list
		self.travel_passive = 0 #if 1, we are traversing thru a stretch of the nodeList that is not on the active path


	def travel_isFirstNodeInPath(self):
		"""Returns True if the current position is the first node on this path."""
		return self.travel_currentPosition == self.travel_pathStartPosition


	def travel_startPath(self):
		"""Initializes a new path."""
		self.travel_pathStartPosition = self.travel_currentPosition
		self.travel_junctionRecord = 0
		self.travel_currentJunctionPosition = 0
		self.travel_passive = 0

	def travel_goForward(self):
		"""Traverses forwards down the nodeList"""
		self.travel_currentPosition += 1
		self.travel_terminalPosition = self.travel_currentPosition
		self.travel_direction = 1	

	def travel_goReverse(self):
		"""Traverses in reverse down the nodeList"""
		self.travel_currentPosition -= 1
		self.travel_direction = 0

	def travel_skipTerminal(self):
		"""Traverses one past the last reached node."""
		self.travel_terminalPosition += 1
		self.travel_currentPosition = self.travel_terminalPosition
		self.travel_direction = 1
		self.travel_currentJunctionPosition = 0

	def travel_isPassive(self):
		"""Returns True if we are passively traversing the nodeList."""
		return self.travel_passive

	def travel_setPassive(self):
		"""Places the read head into passive mode."""
		self.travel_passive = 1

	def travel_setActive(self):
		"""Places the read head into active mode."""
		self.travel_passive = 0

	def travel_newJunction(self):
		"""Adds a new junction to the junction record."""
		self.travel_junctionRecord = (self.travel_junctionRecord << 2) + 1
		self.travel_currentJunctionPosition = 0

	def travel_visitJunction(self):
		"""Returns the state of the visited junction.

		0: Junction is passive
		2: Second visit to junction
		3: Third visit to junction
		"""

		#right shift and mask
		junctionState = (self.travel_junctionRecord >> self.travel_currentJunctionPosition) & 0b11

		#SECOND VISIT
		if junctionState == 1:
			#change junction state to 2
			self.travel_junctionRecord += (1<<self.travel_currentJunctionPosition)
			self.travel_currentJunctionPosition = self.travel_currentJunctionPosition + 2
			return 2

		#THIRD VISIT
		elif junctionState == 2:
			#This is our third visit. Change junction state to 0 to inactivate going forwards.
			self.travel_junctionRecord &= ~(2<<self.travel_currentJunctionPosition)
			self.travel_currentJunctionPosition = self.travel_currentJunctionPosition + 2
			return 3

		elif junctionState == 0:
			#The junction is inactive
			self.travel_currentJunctionPosition = self.travel_currentJunctionPosition + 2
			return 0


class itinerary(list):
	"""This itinerary object stores an ordered list of nodes as they are visited.

	Format is [[stop1, stop2, stop3, ...], [stop4, stop5, stop6,...]]

	Each node is stored in the same dictionary format as a krf node list:
		{'index': lineNumber,
		'routing':'endpoint' or 'midpoint' or 'tee' or 'isolated',
		'connection': 'none' or 'pad' or 'smd' or 'hole',
		'orientation': 'vertical' or 'horizontal' or 'compressed' or 'none'
		'position': (xPosition, yPosition)}

		where xPosition and yPosition are stored as standard inches, i.e. with three significant digits.
	"""
	def __str__(self):
		returnString = ""

		for path in self:
			returnString += "-- START OF PATH (" + str(len(path)) + " STOPS) --" + "\n"

			for node in path:
				returnString += str(node['index']) + " " + node['routing'].upper() + " (X"+str(node['position'][0]) + " Y"+str(node['position'][1]) + ")" + "\n"

			returnString += "\n"

		return returnString



##### LOAD KRF FROM FILE #####

def loadFromFile(filename):
	"""Loads a Kaolin Routing Format File (.krf) into the nodeList.

	filename -- the name of the file to load
	"""

	csvFile = open(filename, newline = '')
	csvReader = csv.reader(csvFile, delimiter='	')

	nodeList = krf()
	for node in csvReader:
		line = int(node[0])
		routingCode, connectionCode, orientationCode = node[1]
		
		routing = {'E':'endpoint', 'M':'midpoint', 'T':'tee', 'I':'isolated'}[routingCode]
		connection = {'N':'none', 'P':'pad', 'S':'smd', 'H':'hole'}[connectionCode]
		orientation = {'V': 'vertical', 'Z': 'horizontal', 'C': 'compressed', 'X':'none'}[orientationCode]
		xPosition = float(node[2])
		yPosition = float(node[3])
		nodeList += [{'index':line, 'routing':routing, 'connection': connection, 
						'orientation':orientation, 'position':(xPosition, yPosition)}]
		
	return nodeList



##### LOAD KRF FROM CHIP #####

def loadFromChip(address = 0x1A00):
	"""Loads nodeList from the memory of a chip using the Universal Personality Programmer (UPP)

	address -- the starting address of the nodelist in the target chip's program memory
	"""
	
	print("\n### LOADING KRF FROM CHIP ###\n")
	time.sleep(0.1)
	print("Connecting to UPP...")

	# Connect to the UPP
	serialInterface1 = interfaces.serialInterface(baudrate = 115200, interfaceType = 'ftdi')
	programmer = nodes.soloGestaltNode(name = 'programmer1', interface = serialInterface1, module = UPPGestaltNode)

	# Identify Chip
	print("Identifying Chip...")
	chipType = programmer.identifyChip()
	if chipType != UPPGestaltNode.attiny84 and chipType != UPPGestaltNode.attiny85:
		print("Kaolin only works with ATTiny84 or ATTiny85 chips... unable to read this " + str(chipType))
		return False

	# Read Memory
	numberOfPages = int((8192 - address)/64) #64 bytes (32 words) per page, 128 pages total in attiny84/85
	
	print("Reading "+ str(numberOfPages) + " pages, starting at 0x" + str(hex(address)).upper()[2:] + "...")
	memoryList = []
	for page in range(numberOfPages):
		pageAddress = address + page*64
		memoryList += programmer.readDataRequest(pageAddress, 64)

	# Convert into a KRF node list
	byteList = memoryListToByteList(memoryList)
	extendedNodeList = byteListToKRF(byteList)
	return krf(extendedNodeList[0:findLastNodeIndex(extendedNodeList)])



def findLastNodeIndex(extendedNodeList):
	"""Returns the index of the last node in a list read from chip memory"""
	for index, node in enumerate(extendedNodeList):
		if node['routing'] == 'isolated' and node['connection'] == 'none' and node['orientation'] == 'none':
			return index


def memoryListToByteList(memoryList):
	"""Splits a raw memoryList into a node byteList.

	memoryList -- a raw list of bytes stored in memory [B0, B1, B2...]

	returns byteList [[B3, B2, B1], [B3, B2, B1]]
	"""
	
	numberOfNodes = len(memoryList) // 3 #three bytes per node

	byteList = []
	for index in range(numberOfNodes):
		byteList += [memoryList[index*3:(index+1)*3]]

	return byteList


def byteListToKRF(byteList):
	"""Converts a byteList into a KRF node list."""
	nodeList = krf()
	for index, triplet in enumerate(byteList):
		routing_encoded = (triplet[0]>>6) & 0b11							#[XX][000000]
		connection_encoded = (triplet[0]>>4) & 0b11							#[00][XX][0000]
		orientation_encoded = (triplet[0]>>2) & 0b11						#[0000][XX][00]



		xPosition_encoded = ((triplet[0] & 0b11)<<7) + (triplet[1]>>1) 		#[00000000][XX] [XXXXXXX][0]
		yPosition_encoded = ((triplet[1]&0b1)<<8) + triplet[2]				#[0000000000]   [0000000][X] [XXXXXXX]

		routing = {0:'endpoint', 1:'midpoint', 2:'tee', 3:'isolated'}[routing_encoded]
		connection = {0:'none', 1:'pad', 2:'smd', 3:'hole'}[connection_encoded]
		orientation = {0:'vertical', 1:'horizontal', 2:'compressed', 3:'none'}[orientation_encoded]
		xPosition = round(float(xPosition_encoded)/100.0, 2)
		yPosition = round(float(yPosition_encoded)/100.0, 2)
		nodeList += [{'index': index+1, 'routing': routing, 'connection': connection, 'orientation': orientation, 
					'position': (xPosition, yPosition)}]

	return nodeList



##### WRITE OUT HEX FILE #####

def writeHexFile(nodeList, filename, startAddress = 0x1A00, maxByteCount = 16):
	"""Writes out an intel hex file based on the provided byte list.

	nodeList -- a KRF node list object
	filename -- the filename to output
	startAddress -- the starting memory address of the file
	"""

	byteList = nodeListToByteList(nodeList + generateTerminatingNode(index = len(nodeList) + 1))

	records = []

	numberOfFullRecords = len(byteList)//maxByteCount
	partialRecordLength = len(byteList)%maxByteCount

	# split full records
	for index in range(numberOfFullRecords):
		records += [byteList[index*maxByteCount:(index+1)*maxByteCount]]

	#split partial record
	if(partialRecordLength):
		startIndex = numberOfFullRecords*maxByteCount

		records += [byteList[startIndex:startIndex + partialRecordLength]]

	#open a file
	file = open(filename, "w")

	#write the data records
	file.writelines([encodeDataRecord(record, startAddress + index*maxByteCount) + "\n" for index, record in enumerate(records)])

	#write the end record
	file.write(":00000001FF")

	file.close()
	print("WROTE " + filename + ": " + str(int(len(byteList)/3 - 1)) + " NODES (" + str(len(byteList) - 3) + " BYTES) @ 0x" + str(hex(startAddress)).upper()[2:] + "\n")


def generateTerminatingNode(index):
	"""Generates and returns a special node that indicates the end of the routing data.

	index -- the index at which to put the terminating node.

	Note that for now, a single isolated node with no connection and no orientation indicates the end of data.
	Eventually, we will need to revisit to support things like board outlines.
	"""
	return [{'node': index, 'routing':'isolated', 'connection': 'none', 'orientation':'none', 'position': (0.00, 0.00)}]


def nodeListToByteList(nodeList):
	"""Converts a nodeList into data bytes.

	nodeList -- a KRF node list object

	Returns a list of byte values representing the encoding.
	"""

	encodedList = []

	for node in nodeList:
		encodedValue = 0

		#Routing
		encodedValue += {'endpoint': 	0x000000, 	#0b00000000 00000000 00000000
						'midpoint': 	0x400000, 	#0b01000000 00000000 00000000
						'tee': 			0x800000,	#0b10000000 00000000 00000000
						'isolated': 	0xC00000, 	#0b11000000 00000000 00000000
						}[node['routing']]

		#Connection
		encodedValue += {'none':		0x000000,	#0b00000000 00000000 00000000
						'pad':			0x100000,	#0b00010000 00000000 00000000
						'smd':			0x200000,	#0b00100000 00000000 00000000
						'hole':			0x300000,	#0b00110000 00000000 00000000
						}[node['connection']]

		#Orientation
		encodedValue += {'vertical':	0x000000,	#0b00000000 00000000 00000000
						'horizontal':	0x040000,	#0b00000100 00000000 00000000
						'compressed':	0x080000,	#0b00001000 00000000 00000000
						'none':			0x0C0000,	#0b00001100 00000000 00000000
						}[node['orientation']]


		#X Position
		encodedValue += (int(node["position"][0]*100))<<9 # 9 bits, 0.01" resolution

		#Y Position
		encodedValue += (int(node["position"][1]*100)) # 9 lower nine bits, 0.01" resolution

		encodedList += [(encodedValue & 0xFF0000)>>16, (encodedValue & 0x00FF00)>>8, (encodedValue & 0x0000FF)] #big endian

	return encodedList


def hexString(value, numBytes):
	"""Returns a fixed length hex string encoding a provided value.

	value -- an integer value
	numBytes -- the byte length to encode
	"""

	rawEncoding = str(hex(value))	#convert number to hex string
	encoding = rawEncoding[2:]	#strip off "0x"

	#Fill out full length
	for i in range(numBytes*2 - len(encoding)):
		encoding = "0" + encoding

	return encoding.upper()



def encodeDataRecord(data, address):
	"""Encodes a data record in the intel hex format.

	data -- a list of data bytes
	address -- the starting address for the record

	returns the data record as a string
	"""
	record = ":" 						# START CHARACTER
	record += hexString(len(data), 1) 	# RECORD LENGTH
	record += hexString(address, 2)		# ADDRESS
	record += "00"						# RECORD TYPE: DATA

	checksum = 0

	# DATA BYTES
	for dataByte in data:
		record += hexString(dataByte, 1)
		checksum += dataByte

	# CHECKSUM
	checksum = checksum & 0xFF #MASK LSB
	checksum = ((checksum ^ 0xFF) + 1) & 0xFF
	record += hexString(dataByte, 1)

	return record


##### READ EAGLE BRD FILE #####
def loadFromBRD(filename):
	"""Reads in an Eagle BRD file, and returns a KRF node list.

	filename -- the filename of the Eagle BRD file

	Returns a KRF object
	"""
	print("")
	print("KAOLIN PCB CONVERSION TOOL")
	print("--------------------------")

	brd = eagleToKRF.board(filename)

	nodeStream = brd.getStream() 	#a list of node objects from the eagleToKRF module


	output = krf()

	for index, node in enumerate(nodeStream):
		
		routingDictionary = {0: "isolated", 1: "endpoint", 2: "midpoint", 3: "tee"}
		numAttachments = len(node.attachments)
		if numAttachments in routingDictionary:
			routing = routingDictionary[numAttachments]
		else:
			print("ERROR: NODE AT " + str(node.position) + " HAS TOO MANY (" + str(len(node.attachments)) + ") CONNECTIONS.")
			print("")
			return None

		connection = node.connection["type"]
		orientation = node.connection["orientation"]
		position = node.position

		output += [{'index': index+1, 'routing': routing, 'connection': connection, 'orientation': orientation, 'position': position}]


	return output


##### WRITE KRF TO FILE #####
def writeKRF(nodeList, filename):
	"""Writes a KRF file based on an input KRF node list.

	nodeList -- a krf object representing a node list
	filename -- the name of the file to write.
	"""

	outputFile = open(filename, "w")

	routingDictionary = {"isolated": "I", "endpoint": "E", "midpoint": "M", "tee": "T"}
	connectionDictionary = {"none": "N", "pad": "P", "smd": "S", "hole": "H"}
	orientationDictionary = {"vertical": "V", "horizontal": "Z", "compressed": "C", "none": "X"}

	for node in nodeList:

		index = node["index"]
		descriptor = routingDictionary[node["routing"]] + connectionDictionary[node["connection"]] + orientationDictionary[node["orientation"]]
		position = node["position"]

		outputFile.write(str(index) + "\t" + descriptor + "\t" + str(position[0]) + "\t" + str(position[1]) + "\n")


	print("WROTE KRF FILE " + filename + ": " + str(len(nodeList)) + " NODES")





##### WHEN MODULE IS CALLED FROM TERMINAL #####

if __name__ == "__main__":

	# myKRF = loadFromChip()
	myKRF = loadFromBRD("555.brd")
	# myKRF = loadFromFile('555.krf')
	# writeKRF(myKRF, "555-test.krf")

	thisItinerary = myKRF.travel()

	print(thisItinerary)

	thisToolpath = toolpath.generateToolpath(myKRF, depth = 0.008, feedrate = 0.2, traverseHeight = 0.1, traceWidth = 0.04, toolDia = 0.03)

	print(thisToolpath)

	toolpath.simulate(thisToolpath)

	# myKRF.loadFromChip()

	# myKRF.urumbu(depth = 0.008, feedrate = 0.2, traverseHeight = 0.1, traceWidth = 0.04, toolDia = 0.03, simulate = True)

	# startPoint = (0, 0)
	# midPoint = (0, -1)
	# endPoint_90 = (-1, -1)
	# endPoint_180 = (0, -2)
	# endPoint_270 = (1, -1)
	# endPoint_360 = (0, 0)

	# print(myKRF.tp_getJunctionAngles(startPoint, midPoint, endPoint_90))
	# print(myKRF.tp_getJunctionAngles(startPoint, midPoint, endPoint_180))
	# print(myKRF.tp_getJunctionAngles(startPoint, midPoint, endPoint_270))
	# print(myKRF.tp_getJunctionAngles(startPoint, midPoint, endPoint_360))

	# print()

	# startPoint = (0,0)
	# endPoint = (0, -1)
	# print(myKRF.tp_absAngle(startPoint, endPoint))
	# print(myKRF.tp_absAngle(midPoint, endPoint_270))

	# writeHexFile(myKRF, '555_PCB-test.hex')
	# print(myKRF)
	# myKRF.printItinerary(myKRF.traverseNodeList())
	
