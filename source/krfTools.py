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

from PIL import Image, ImageDraw


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

	def loadFromChip(self, address = 0x1A00):
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

	def writeHexFile(self, filename, startAddress = 0x1A00, maxByteCount = 16):
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

	
	def tp_absAngle(self, startPoint, endPoint):
		"""Returns the absolute angle of the line drawn from the start point to the end point, relative to the right-going horizontal.

		startPoint, endPoint -- (x, y)

		returns angle in degrees
		"""
		x_start, y_start = startPoint
		x_end, y_end = endPoint

		angle = math.atan2((y_end-y_start), (x_end-x_start))

		angle = int(angle * 360 / (2*math.pi))

		if angle >= 0: 
			return angle
		else:
			return 360 + angle

	def tp_getJunctionAngles(self, startPoint, midPoint, endPoint):
		"""Returns the key angles for a given junction.

		startPoint -- the point leading into the junction (x, y)
		midPoint -- the point at the junction (x, y)
		endPoint -- the next point after the junction (x, y)

		returns jointAngle, junctionAngle

			jointAngle -- 	the angle created between the entry and exit vectors, measured in degrees going CCW, 
							while rotating the junction such that the entry vector to be horizontal

			junctionAngle -- 	the angle of the entire junction, measured in degrees CCW of the entry vector
								relative to the right-going horizontal.
		"""

		junctionAngle = self.tp_absAngle(startPoint, midPoint)
		jointAngle = self.tp_absAngle(endPoint, midPoint) - junctionAngle

		if jointAngle >= 0:
			return jointAngle, junctionAngle
		else:
			return 360 + jointAngle, junctionAngle


	def tp_connection_none(self, jointAngle, orientation, traceWidth, toolDia, firstNode = False, lastNode = False):
		"""Generates a RELATIVE toolpath for entering a joint with connection type of 'none'.

		jointAngle -- the CCW angle formed by the joint, measured from the right-going horizontal
		orientation -- the orientation of the connection. No meaning for a connection type of 'none'

		returns a list of points that traverse around the joint, RELATIVE TO THE CENTER POINT OF THE JOINT.
		"""
		offset = traceWidth / 2.0 + toolDia / 2.0
		
		if firstNode:
			return [(-offset, 0), (-offset, -offset), (0, -offset)]

		elif lastNode:
			return [(0, -offset), (offset, -offset), (offset, 0)]

		else: #not an end node
			if jointAngle == 0:
				# An odd one, for when joints double back on themselves
				return [(offset, -offset), (offset, offset)]
			elif jointAngle == 90:
				return [(-offset, -offset)]

			elif jointAngle == 180:
				return [(0, -offset)]

			elif jointAngle == 270:
				return [(offset, -offset)]

			else:
				print("ERROR: JOINT ANGLE " + str(jointAngle) + " IS NOT SUPPORTED")
				return None

	def tp_connection_pad(self, jointAngle, orientation, traceWidth, toolDia, firstNode = False, lastNode = False):
		"""Generates a RELATIVE toolpath for entering a joint with connection type of 'pad'.

		jointAngle -- the CCW angle formed by the joint, measured from the right-going horizontal
		orientation -- the orientation of the connection. No meaning for a connection type of 'none'

		returns a list of points that traverse around the joint, RELATIVE TO THE CENTER POINT OF THE JOINT.
		"""

		# --- PAD DEFINITIONS ---

		# These assume a horizontally oriented pad
		pad_height = 0.1 * 25.4 #mm, this is the same as the width for a compressed pad


		# Pre-calculated offsets
		offset_trace = traceWidth / 2.0 + toolDia / 2.0
		offset_C = pad_height/2.0 + toolDia / 2.0 #compressed pad

		
		if firstNode:
			return [(-offset_C, 0), (-offset_C, -offset_C), (offset_C, -offset_C), (offset_C, -offset_trace)]

		elif lastNode:
			return [(-offset_C, -offset_trace), (-offset_C, -offset_C), (offset_C, -offset_C), (offset_C, 0)]

		else: #not an end node
			if jointAngle == 0:
				# An odd one, for when joints double back on themselves
				return [(-offset_C, -offset_trace), (-offset_C, -offset_C), (offset_C, -offset_C), (offset_C, offset_C), (-offset_C, offset_C), (-offset_C, offset_trace)]
			
			elif jointAngle == 90:
				return [(-offset_C, -offset_trace), (-offset_C, -offset_C), (-offset_trace, -offset_C)]

			elif jointAngle == 180:
				return [(-offset_C, -offset_trace), (-offset_C, -offset_C), (offset_C, -offset_C), (offset_C, -offset_trace)]

			elif jointAngle == 270:
				return [(-offset_C, -offset_trace), (-offset_C, -offset_C), (offset_C, -offset_C), (offset_C, offset_C), (offset_trace, offset_C)]

			else:
				print("ERROR: JOINT ANGLE " + str(jointAngle) + " IS NOT SUPPORTED")
				return None


	def tp_positionAndRotate(self, inputPointList, junctionAngle, center):
		"""Offsets and rotates a list of points about a centerpoint.

		inputPointList -- the list of points to offset, in format [(x1, y1), (x2, y2), (x3, y3),...]
		junctionAngle -- a CCW rotation angle, relative to the right-going horizontal, in degrees
		center -- the center position of the node, as (x, y)
		"""

		outputPointList = []
		x_c, y_c = center
		for point in inputPointList:
			x, y = point

			# ROTATE
			if junctionAngle == 0:
				x_r = x
				y_r = y
			elif junctionAngle == 90:
				x_r = -y
				y_r = x

			elif junctionAngle == 180:
				x_r = -x
				y_r = -y

			elif junctionAngle == 270:
				x_r = y
				y_r = -x

			else:
				print("ERROR: JOINT ANGLE " + str(jointAngle) + " IS NOT SUPPORTED")

			# TRANSLATE
			outputPointList += [(x_r + x_c, y_r + y_c)]

		return outputPointList



	def urumbu(self, depth, feedrate, traverseHeight, traceWidth, toolDia, metric = False, simulate = True):
		"""Routes a PCB using urumbu_xyz_mill.

		depth -- the depth to which to cut, AS A POSITIVE NUMBER
		traverseHeight -- the z position at which traversing motion should occur
		metric -- if True, all input parameters are metric
		simulate -- if True, this will use PIL to output a simulation of the route.

		NOTE: Unlike the rest of this module, within this function, all units are mm.
		"""

		# convert inputs to metric if necessary
		if not metric:
			depth = depth * 25.4
			feedrate = feedrate * 25.4
			traverseHeight = traverseHeight * 25.4
			toolDia = toolDia * 25.4
			traceWidth = traceWidth * 25.4
		
		#get all closed paths
		paths = self.traverseNodeList()

		#generate toolpaths
		toolpath = [(0.0, 0.0, traverseHeight)] #stored as a list of tuples (X, Y, Z)
		drills = [] #a list of drill positions, stored as (X, Y)
		priorPosition = (0.0, 0.0) #stored as (x pos, y pos, z pos)

		connectionFunctions = {"none": self.tp_connection_none, "pad": self.tp_connection_pad, "smd": self.tp_connection_none, "hole": self.tp_connection_none}

		for path in paths:
			length = len(path)
			for index, node in enumerate(path):

				#pull properties
				connection = node["connection"]
				orientation = node["orientation"]
				x = node["xPosition"] * 25.4 #positions are always stored internally as inch, so convert to metric
				y = node["yPosition"] * 25.4

				thisPosition = (x, y)

				if length > 1: #multi-point path

					# --- SETUP ---

					#Is this the first node on the path?
					if index == 0:
						pathStart = True
					else:
						pathStart = False

					#Is this the last node on the path?
					if index == length -1:
						pathEnd = True
					else:
						pathEnd = False

					if not pathEnd:
						x_next = path[index + 1]["xPosition"] * 25.4
						y_next = path[index + 1]["yPosition"] * 25.4
						nextPosition = (x_next, y_next)

					connectionFunction = connectionFunctions[connection]


					# --- TOOLPATH ALGORITHMS ---

					if pathStart:
						# Calculate points that traverse the joint
						junctionAngle = self.tp_absAngle(thisPosition, nextPosition)
						pointsAboutJoint = connectionFunction(jointAngle = 0, orientation = orientation, traceWidth = traceWidth, toolDia = toolDia, firstNode = True)
						absolutePoints = self.tp_positionAndRotate(pointsAboutJoint, junctionAngle, thisPosition)

						# Generate traverse to the first point
						x_start, y_start = absolutePoints[0]
						toolpath += [(x_start, y_start, traverseHeight)]

						# Load up toolpath
						for point in absolutePoints:
							x_point, y_point = point
							toolpath += [(x_point, y_point, -depth)]

					elif pathEnd:
						# Calculate points that traverse the joint						
						junctionAngle = self.tp_absAngle(priorPosition, thisPosition)
						pointsAboutJoint = connectionFunction(jointAngle = 0, orientation = orientation, traceWidth = traceWidth, toolDia = toolDia, lastNode = True)
						absolutePoints = self.tp_positionAndRotate(pointsAboutJoint, junctionAngle, thisPosition)

						# Load up toolpath
						for point in absolutePoints:
							x_point, y_point = point
							toolpath += [(x_point, y_point, -depth)]

						# Retract					
						x_end, y_end = absolutePoints[-1]
						toolpath += [(x_end, y_end, traverseHeight)]

					else:
						# Calculate points that traverse the joint
						jointAngle, junctionAngle = self.tp_getJunctionAngles(priorPosition, thisPosition, nextPosition)
						pointsAboutJoint = connectionFunction(jointAngle, orientation, traceWidth, toolDia)
						absolutePoints = self.tp_positionAndRotate(pointsAboutJoint, junctionAngle, thisPosition)

						#load up toolpath
						for point in absolutePoints:
							x_point, y_point = point
							toolpath += [(x_point, y_point, -depth)]

				if connection == "pad" or connection == "hole":
					drills += [thisPosition]

				priorPosition = thisPosition						

		x_point, y_point = thisPosition
		toolpath += [(x_point, y_point, traverseHeight)]
		toolpath += [(0, 0, traverseHeight)]

			# # --- TOOLPATHING ALGORITHM HERE ---

			# 	if index == 0: #start of path
			# 		position = (xPosition, yPosition, traverseHeight)
			# 		toolpath += [position]
			# 		position = (xPosition, yPosition, -depth)
			# 		toolpath += [position]
			# 	else:
			# 		position = (xPosition, yPosition, -depth)
			# 		toolpath += [position]


			# toolpath += [(xPosition, yPosition, traverseHeight)]

			# # --- END TOOLPATHING ALGORITHM ---




		#simulate
		if simulate:
			resolution = 0.05 #mm/pixel
			pixelPitch = 1.0/resolution #pixels/mm

			canvasWidth = 125 #mm
			canvasHeight = 125 #mm

			canvas = Image.new('RGB', (int(canvasWidth*pixelPitch), int(canvasHeight*pixelPitch)), (255, 255, 255))

			draw = ImageDraw.Draw(canvas)

			print("RUNNING TOOLPATH IN SIMULATION")

			lastPosition = (0,0,0)

			for position in toolpath:
				x_last, y_last, z_last = lastPosition
				x_next, y_next, z_next = position 

				if z_last > 0 and z_next > 0: #traverse
					fill = (0, 255, 0) #Green

				elif z_last < 0 and z_next < 0:
					fill = (0, 0, 255) #Blue

				else:
					fill = (255, 0, 0) #Red

				draw.line((int(x_last*pixelPitch), int(canvasHeight*pixelPitch - y_last*pixelPitch), int(x_next*pixelPitch), int(canvasHeight*pixelPitch - y_next*pixelPitch)), fill = fill, width=int(toolDia*pixelPitch))

				lastPosition = position


			x_last, y_last , z_last = lastPosition


			for drillPosition in drills:
				x_next, y_next = drillPosition
				# draw.line((int(x_last*pixelPitch), int(canvasHeight*pixelPitch - y_last*pixelPitch), int(x_next*pixelPitch), int(canvasHeight*pixelPitch - y_next*pixelPitch)), fill = (0, 255, 0), width=int(toolDia*pixelPitch))

				#Draw Drill Hole
				renderRadius = toolDia

				x1 = x_next - renderRadius
				y1 = y_next + renderRadius
				x2 = x_next + renderRadius
				y2 = y_next - renderRadius

				draw.ellipse((int(x1 * pixelPitch), int(canvasHeight*pixelPitch - y1*pixelPitch), int(x2 * pixelPitch), int(canvasHeight*pixelPitch - y2*pixelPitch)), fill=(255, 0, 255), outline=(0, 0, 0))

				x_last, y_last = x_next, y_next

			draw.line((int(x_next*pixelPitch), int(canvasHeight*pixelPitch - y_next*pixelPitch), 0, canvasHeight*pixelPitch), fill = (0,255,0), width= int(toolDia*pixelPitch))

			canvas.save('simulate.png')			

		#run urumbu
		else:

			print("RUNNING TOOLPATH ON URUMBU")

			actionQueue = urumbu_xyz_mill.init_machine("/dev/tty.usbmodem11401", "/dev/tty.usbmodem11301", "/dev/tty.usbmodem11201", "/dev/tty.usbmodem111401")

			#turn on spindle
			# actionQueue.put(urumbu_xyz_mill.SpindleAction('spindle', True))
			# actionQueue.put(urumbu_xyz_mill.WaitAction(1))

		    #load in program
			for position in toolpath:
				x, y, z = position

				print("   X: " + str(round(x, 2)) + "MM, Y: " + str(round(y,2)) + "MM, Z: " + str(round(z,2)) + "MM @ " + str(round(feedrate,0)) + "MM/SEC")
				actionQueue.put(urumbu_xyz_mill.Line([x, y, z], feedrate))


			#Drill all holes
			for drillPosition in drills:
				x, y = drillPosition
				actionQueue.put(urumbu_xyz_mill.Line([x, y, traverseHeight], feedrate))
				actionQueue.put(urumbu_xyz_mill.Line([x, y, -1.6], feedrate))
				actionQueue.put(urumbu_xyz_mill.Line([x, y, traverseHeight], feedrate))

			actionQueue.put(urumbu_xyz_mill.Line([0, 0, traverseHeight], feedrate))
			actionQueue.put(urumbu_xyz_mill.Line([0, 0, 0], feedrate))

		    #turn off spindle
			# actionQueue.put(urumbu_xyz_mill.SpindleAction('spindle', False))

		    #wait
			time.sleep(10)




##### WHEN MODULE IS CALLED FROM TERMINAL #####

if __name__ == "__main__":
	myKRF = krf()
	# myKRF.loadFromFile('555.krf')
	myKRF.loadFromChip()

	myKRF.urumbu(depth = 0.008, feedrate = 0.2, traverseHeight = 0.1, traceWidth = 0.04, toolDia = 0.03, simulate = False)

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

	# myKRF.writeHexFile('555_PCB.hex')
	# print(myKRF)
	# myKRF.printItinerary(myKRF.traverseNodeList())
	
