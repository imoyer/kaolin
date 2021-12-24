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
import time


class krf(object):

	def __init__(self, nodeList = None):
		self.nodeList = nodeList 	# stores a list of nodes, in the following format:
									# {'node': lineNumber,
									#  'routing':'endpoint' or 'midpoint' or 'tee' or 'isolated',
									#  'connection': 'none' or 'pad' or 'smd' or 'hole',
									#  'orientation': 'vertical' or 'horizontal' or 'compressed' or 'none'
									#  'xPosition': xPosition,
									#  'yPosition': yPosition}


	def loadFromFile(self, filename):
		"""Loads a Kaolin Routing Format File (.krf) into the nodeList.

		filename -- the name of the file to load
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
		
		self.nodeList = nodeList

	def __str__(self):
		"""Returns a formatted representation of the KRF file."""
		returnString = "\nKAOLIN ROUTING FILE (" + str(len(self.nodeList)) + " NODES) \n"

		for node in self.nodeList:
			returnString += "[" + str(node['node']) + "]" + chr(9) + {'endpoint':'E', 'midpoint':'M', 'tee':'T', 'isolated':'I'}[node['routing']] + " (X " + str(node['xPosition']) + ", Y " + str(node['yPosition']) + ")\n"

		return returnString


	##### LOAD KRF FROM CHIP #####

	def loadFromChip(self, address = 0x1C00):
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
		if chipType != UPPGestaltNode.attiny84:
			print("Kaolin only works with ATTiny84 chips... unable to read this " + str(chipType))
			return False

		# Read Memory
		numberOfPages = int((8192 - address)/64) #64 bytes (32 words) per page, 128 pages total in attiny84
		
		print("Reading "+ str(numberOfPages) + " pages, starting at 0x" + str(hex(address)).upper()[2:] + "...")
		memoryList = []
		for page in range(numberOfPages):
			pageAddress = address + page*64
			memoryList += programmer.readDataRequest(pageAddress, 64)

		# Convert into a nodeList
		byteList = krf.memoryListToByteList(memoryList)
		extendedNodeList = krf.byteListToNodeList(byteList)
		self.nodeList = extendedNodeList[0:krf.findLastNodeIndex(extendedNodeList)]



	@staticmethod
	def findLastNodeIndex(extendedNodeList):
		"""Returns the index of the last node in a list read from chip memory"""
		for index, node in enumerate(extendedNodeList):
			if node['routing'] == 'isolated' and node['connection'] == 'none' and node['orientation'] == 'none':
				return index


	@staticmethod
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

	@staticmethod
	def byteListToNodeList(byteList):
		"""Converts a byteList into a nodeList."""
		nodeList = []
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
			nodeList += [{'node': index+1, 'routing': routing, 'connection': connection, 'orientation': orientation, 
						'xPosition': xPosition, 'yPosition':yPosition}]

		return nodeList



	##### WRITE OUT HEX FILE #####

	def writeHexFile(self, filename, startAddress = 0x1C00, maxByteCount = 16):
		"""Writes out an intel hex file based on the provided byte list.

		filename -- the filename to output
		startAddress -- the starting memory address of the file
		"""

		byteList = krf.nodeListToByteList(self.nodeList + self.generateTerminatingNode())

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
		file.writelines([krf.encodeDataRecord(record, startAddress + index*maxByteCount) + "\n" for index, record in enumerate(records)])

		#write the end record
		file.write(":00000001FF")

		file.close()
		print("WROTE " + filename + ": " + str(int(len(byteList)/3 - 1)) + " NODES (" + str(len(byteList) - 3) + " BYTES) @ 0x" + str(hex(startAddress)).upper()[2:] + "\n")


	def generateTerminatingNode(self):
		"""Generates and returns a special node that indicates the end of the routing data.

		Note that for now, a single isolated node with no connection and no orientation indicates the end of data.
		Eventually, we will need to revisit to support things like board outlines.
		"""
		return [{'node':len(self.nodeList) + 1, 'routing':'isolated', 'connection': 'none', 'orientation':'none', 'xPosition': 0, 'yPosition': 0}]


	@staticmethod
	def nodeListToByteList(nodeList):
		"""Converts a nodeList into data bytes.

		nodeList -- a list of nodes in the Kaolin nodeList format

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
			encodedValue += (int(node["xPosition"]*100))<<9 # 9 bits, 0.01" resolution

			#Y Position
			encodedValue += (int(node["yPosition"]*100)) # 9 lower nine bits, 0.01" resolution

			encodedList += [(encodedValue & 0xFF0000)>>16, (encodedValue & 0x00FF00)>>8, (encodedValue & 0x0000FF)] #big endian

		return encodedList

	@staticmethod
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


	@staticmethod
	def encodeDataRecord(data, address):
		"""Encodes a data record in the intel hex format.

		data -- a list of data bytes
		address -- the starting address for the record

		returns the data record as a string
		"""
		record = ":" 						# START CHARACTER
		record += krf.hexString(len(data), 1) 	# RECORD LENGTH
		record += krf.hexString(address, 2)		# ADDRESS
		record += "00"						# RECORD TYPE: DATA

		checksum = 0

		# DATA BYTES
		for dataByte in data:
			record += krf.hexString(dataByte, 1)
			checksum += dataByte

		# CHECKSUM
		checksum = checksum & 0xFF #MASK LSB
		checksum = ((checksum ^ 0xFF) + 1) & 0xFF
		record += krf.hexString(dataByte, 1)

		return record

	##### SIMULATE THE ROUTING PROCESS

	def sim_initializeParameters(self):
		# POSITIONAL INDICES INTO THE NODE LIST
		self.sim_pathStartPosition = 0 	#index of the current path start
		self.sim_currentPosition = 0 	#index of the current read position
		self.sim_terminalPosition = 0 	#index of the last position to be read


		# JUNCTION RECORD
		# Here we keep a record of all the tee junctions that have been traversed,
		# in the order of how they appear in the node list.

		self.sim_junctionRecord = 0 # a bitfield, ...[XX][XX][XX][XX]
									# pairs of bits are appended from the right, whenever a new tee is encountered
									# while traversing the nodeList in the forwards direction.
									# Therefore the least significant bits represent the bottom of the nodeList (as currently traversed)
									# 00: 	Tee has been traveled 3 times already, and will be treated as out-of-path going
									# 01: 	Tee was has only been traveled once, and in the forward direction (first time is always forwards!)
									#		Next traverse will be in reverse, and will branch forwards to the end of the nodeList
									# 10: 	Tee has been traveled 2 times. Next traverse will be in reverse, and will pass thru the T.
		self.sim_currentJunctionPosition = 0 	# the current bit position within the junction record.
												# This is always 0 when traveling forwards

		# TRAVERSE STATE
		self.sim_direction = 1 #1: traversing forwards down the node list, 0: traversing in reverse down the node list
		self.sim_passive = 0 #if 1, we are traversing thru a stretch of the nodeList that is not on the active path


	def sim_isFirstNodeInPath(self):
		"""Returns True if the current position is the first node on this path."""
		return self.sim_currentPosition == self.sim_pathStartPosition


	def sim_startPath(self):
		"""Initializes a new path."""
		self.sim_pathStartPosition = self.sim_currentPosition
		self.sim_junctionRecord = 0
		self.sim_currentJunctionPosition = 0
		self.sim_passive = 0

	def sim_goForward(self):
		"""Traverses forwards down the nodeList"""
		self.sim_currentPosition += 1
		self.sim_terminalPosition = self.sim_currentPosition
		self.sim_direction = 1	

	def sim_goReverse(self):
		"""Traverses in reverse down the nodeList"""
		self.sim_currentPosition -= 1
		self.sim_direction = 0

	def sim_skipTerminal(self):
		"""Traverses one past the last reached node."""
		self.sim_terminalPosition += 1
		self.sim_currentPosition = self.sim_terminalPosition
		self.sim_direction = 1
		self.sim_currentJunctionPosition = 0

	def sim_isPassive(self):
		"""Returns True if we are passively traversing the nodeList."""
		return self.sim_passive

	def sim_setPassive(self):
		"""Places the read head into passive mode."""
		self.sim_passive = 1

	def sim_setActive(self):
		"""Places the read head into active mode."""
		self.sim_passive = 0

	def sim_newJunction(self):
		"""Adds a new junction to the junction record."""
		self.sim_junctionRecord = (self.sim_junctionRecord << 2) + 1
		self.sim_currentJunctionPosition = 0

	def sim_visitJunction(self):
		"""Returns the state of the visited junction.

		0: Junction is passive
		2: Second visit to junction
		3: Third visit to junction
		"""

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


	def traverseNodeList(self):
		"""Traverses the nodeList and returns a travelList.

		Returns travelList -- a list of paths that were traveled [[stop1, stop2, stop3, ...], [stop1, stop2, stop3]]
		"""

		travelList = []
		pathList = []
		self.sim_initializeParameters()


		while True:

			node = self.nodeList[self.sim_currentPosition]

			# GOING FORWARDS
			if self.sim_direction == 1:
				pathList += [node] #add node to the pathList

				#FIRST NODE IN PATH
				if self.sim_isFirstNodeInPath():
					self.sim_goForward()

					#ISOLATED NODE
					if node['routing'] == 'isolated':
						travelList += [pathList]
						pathList = []
						self.sim_startPath()
						continue
					
					#START OF A PATH
					else:
						continue

				#IN PATH
				else:
					#ENDPOINT
					if node['routing'] == 'endpoint':
						self.sim_goReverse()

					#MIDPOINT
					elif node['routing'] == 'midpoint':
						self.sim_goForward()

					#TEE
					elif node['routing'] == 'tee':
						self.sim_newJunction()
						self.sim_goForward()
					continue


			# GOING IN REVERSE
			if self.sim_direction == 0:
				#BACK AT THE START, DONE WORKING ON PATH
				if self.sim_isFirstNodeInPath():
					pathList += [node]
					self.sim_skipTerminal()
					if self.sim_currentPosition < len(self.nodeList):
						travelList += [pathList]
						pathList = []
						self.sim_startPath()
						continue
					else:
						travelList += [pathList]
						return travelList

				#ENDPOINT
				if node['routing'] == 'endpoint':
					self.sim_setPassive() #we aren't at the start, so we're going sim_passive.
					# Hitting an non-path-start endpoint in reverse simply means we're reading
					# a non-active path.
					self.sim_goReverse()
					continue

				#MIDPOINT
				elif node['routing'] == 'midpoint':
					#PASSIVE
					if self.sim_isPassive():
						self.sim_goReverse()
						continue
					#ACTIVE
					else:
						pathList += [node]
						self.sim_goReverse()
						continue

				#TEE
				elif node['routing'] == 'tee':
					junctionState = self.sim_visitJunction()
					#Hit a junction for the second time
					if junctionState == 2:
						pathList += [node]
						self.sim_setActive()
						self.sim_skipTerminal()
						continue
					#Hit a junction for the third time, just pass thru it
					elif junctionState == 3:
						pathList += [node]
						self.sim_setActive()
						self.sim_goReverse()

					elif junctionState == 0:
						self.sim_goReverse()
						self.sim_setPassive()
						continue

	def printItinerary(self, travelList):
		for path in travelList:
			print("-- START OF PATH (" + str(len(path)) + " STOPS) --")

			for node in path:
				print(str(node['node']) + " " + node['routing'].upper() + " (X"+str(node['xPosition']) + " Y"+str(node['yPosition']) + ")")

			print("")

##### WHEN MODULE IS CALLED FROM TERMINAL #####

if __name__ == "__main__":
	myKRF = krf()
	# myKRF.loadFromFile('testPCB.krf')
	# myKRF.writeHexFile('testPCB.hex')
	myKRF.loadFromChip()
	print(myKRF)
	myKRF.printItinerary(myKRF.traverseNodeList())
	
