# Kaolin PCB Converter Utility
#
# ILAN E. MOYER
# April 27th, 2022
#
# One goal of the Kaolin project is to embed the design for an MCU's PCB within the memory of the MCU
# This script reads in an Autodesk Eagle PCB (.brd) file, and converts it into the Kaolin format, and
# saves out an Intel HEX file.

import xml.etree.ElementTree as ET
import os, sys
import math


class node(object):
	"""A position on the signal, with connections to other nodes."""
	def __init__(self, position, allTerminations):
		self.position = position
		self.connections = [] #a list of all nodes that are connected to this node
		self.termination = self.findTermination(allTerminations) #termination style

	def __repr__(self):
		return self.termination["type"] + " @ " + str(self.position)

	def nodeString(self):
		"""Returns the standard Kaolin desciptor string for the node.

		The standard format is XYZ, where:
			X (Routing) 	- E: Endpoint, M: Midpoint, T: Tee, I: Isolated
			Y (Connection) 	- N: None, P: Pad, S: SMD, H: Hole
			Z (Orientation) - V: Vertical, Z: Horizontal, C: Compressed, X: None
		"""

		routingDict = {0: "I", 1: "E", 2: "M", 3: "T"} #indexed by number of connections
		connectionDict = {"none": "N", "pad": "P", "smd": "S", "hole": "H"}
		orientationDict = {"vertical": "V", "horizontal": "Z", "compressed": "C", "none": "X"}

		if len(self.connections) in routingDict:
			return routingDict[len(self.connections)] + connectionDict[self.termination["type"]] + orientationDict[self.termination["orientation"]]
		else:
			print("ERROR: NODE AT " + str(self.position) + " HAS TOO MANY (" + str(len(self.connections)) + ") CONNECTIONS.")
			print("")
			return None




	def findTermination(self, allTerminations):
		"""Finds the termination type of the node.

		This function will search the provided avaliable position-indexed terminations dictionary, 
		and find a match based on position.
		"""
		if self.position in allTerminations:
			return allTerminations[self.position]
		else:
			return {"type" : "none", "orientation" : "none"}



	def addConnection(self, node):
		"""Adds a connection to the node."""
		self.connections += [node]

	def isEndpoint(self):
		"""Returns whether this node is an endpoint."""
		return (len(self.connections) == 1)

	def getAngle(self, entryNode, exitNode):
		"""Returns the angle in degrees between the lines [entryNode] --> [thisNode] and [thisNode] --> [exitNode]

		Angles are returned in degrees, measured in the clockwise direction.
		"""

		x_this, y_this = self.position
		
		if(entryNode):
			x_entry, y_entry = entryNode.position
		else:
			x_entry, y_entry = (x_this - 1.0, y_this)
		
		x_exit, y_exit = exitNode.position

		entryAngle = math.atan2(y_this - y_entry, x_this - x_entry)
		exitAngle = math.atan2(y_this - y_exit, x_this - x_exit)

		totalAngle = exitAngle - entryAngle

		while totalAngle < 0:
			totalAngle += 2*math.pi

		return math.degrees(totalAngle)

	def getConnections(self, entryNode = None):
		"""Returns a list of connections in the format [(node1, angle1), (node2, angle2), ...].

		entryNode -- If provided, the line between entryNode and this node creates a frame of reference for angle measurements.
					 Note that if this isn't provided, an absolute angle is returned (0 deg is right-going horizontal, 90 deg is upward horizontal)

		The returned list is sorted by angle. If an entryNode is provided, the reciprocal connection to the entryNode is listed last, as it is a 360 deg angle.
		"""
		angles = [self.getAngle(entryNode, thisNode) for thisNode in self.connections]

		if entryNode != None:
			angles = [360.0 if angle == 0 else angle for angle in angles] #changes 0 to 360 for the reciprocal node

		nodesAndAngles = list(zip(self.connections, angles))


		nodesAndAngles.sort(key=lambda y: y[1])

		return nodesAndAngles



class signal(object):
	"""Stores all data related to an connected set of traces."""
	def __init__(self, xmlElement, terminations):

		self.nodes = []	#this will be where we store all nodes

		self.name = xmlElement.get('name')

		self.allTerminations = terminations #a list of all avaliable terminations on the board.

		#  pulls in wires from the signal as [((X1, Y1), (X2, Y2)), ...]
		
		self.wires = []

		for thisWire in xmlElement.iter('wire'):
			x1 = standardInch(thisWire.get('x1'))
			y1 = standardInch(thisWire.get('y1'))
			x2 = standardInch(thisWire.get('x2'))
			y2 = standardInch(thisWire.get('y2'))
			self.wires += [((x1, y1), (x2, y2))]

		
		#add nodes from wires
		for wire in self.wires:
			self.addWire(wire)


	def addWire(self, wire):
		"""Adds nodes and/or connections to self.nodes, based on the wire endpoints"""

		#create temporary nodes
		tempStartNode, tempEndNode = (node(wire[0], self.allTerminations), node(wire[1], self.allTerminations))

		startNode, endNode = (self.doesNodeExist(tempStartNode), self.doesNodeExist(tempEndNode))

		if startNode == False:
			startNode = self.addNode(tempStartNode)

		if endNode == False:
			endNode = self.addNode(tempEndNode)

		startNode.addConnection(endNode)
		endNode.addConnection(startNode)



	def addNode(self, node):
		self.nodes += [node]
		return node



	def doesNodeExist(self, testNode):
		"""Tests whether the provided node already exists in the nodeList.

		Returns the matching node if found, or else 
		"""
		for thisNode in self.nodes:
			if(testNode.position == thisNode.position):
				return thisNode

		return False

	def getEndpoints(self):
		"""Returns all nodes that are endpoints."""
		return list(filter(lambda x: x.isEndpoint(), self.nodes))

	def getStartNode(self):
		"""Returns a starting node by finding the uppermost endpoint node.

		This definition is arbitrary.
		"""
		endpoints = self.getEndpoints()
		endpoints.sort(key=lambda y: y.position[1])
		return endpoints[-1]

	def serialize(self):
		"""Returns a serialized list of nodes, according to the Kaolin convention.

		This serialized list starts at the Start Node, and traverses the tree on the right-hand
		side of signal's wires, until it returns to the start node. Each node is only listed once.
		"""
		startNode = self.getStartNode() #the starting node. For now, arbitrarily chosen as the
										#node with the highest Y value.

		traversedNodes = {} #stores all nodes that we've already traversed, in the format {node:remainingConnections}, excluding the start node
		serialStream = [startNode] #stores the built-up serialization stream

		lastNode = startNode
		nextNode = startNode.getConnections()[0][0]

		#iterate until we reach the start again
		while nextNode != startNode:

			#We've never been to this node before
			if not nextNode in traversedNodes:
				serialStream += [nextNode] #add the node to the serialization list

				connections = nextNode.getConnections(lastNode)

				traversedNodes.update({nextNode:connections})
			
			#Switch to this node
			lastNode = nextNode

			nextNode = traversedNodes[lastNode].pop(0)[0]

		
		return serialStream


def standardInch(metricString):
	"""Converts a position from metric to inches, with three digits of precision."""
	return round(float(metricString)/25.4, 3)


def library(xmlElement_library):
	"""Ingests a library xml element, and returns a dictionary containing all packages and associated
		relevent elements."""
	return {thisPackage.get("name"): package(thisPackage) for thisPackage in xmlElement_library.iter("package")}


def package(xmlElement_package):

	elements = []

	#Add all pads
	elements += [{"type":"pad", "x": float(element.get("x")), "y": float(element.get("y"))} for element in xmlElement_package.iter("pad")]

	return elements


def getTerminations(elements, libraries):
	"""Returns all terminations in the BRD file.

	The format is {(xPos, yPos): {"type":typeCode, "orientation":orientationCode}}
		typeCode: "pad", "smd", "hole"
		orientationCode: "vertical", "horizontal", "compressed"
	"""

	terminations = {}

	for element in elements:
		xPos = float(element.get("x"))
		yPos = float(element.get("y"))
		libraryName = element.get("library")
		elementName = element.get("name")
		packageName = element.get("package")

		if libraryName in libraries:
			library = libraries[libraryName]
		else:
			print("ERROR: Element '" + elementName + "' REFERENCES MISSING LIBRARY '" + libraryName + "'")
			return {}

		if packageName in library:
			package = library[packageName]
		else:
			print("ERROR: Element '" + elementName + "' REFERENCES MISSING PACKAGE + '" + packageName + "' IN LIBRARY '" + libraryName + "'")
			return {}

		for term in package:
			#first, get the board positions of the terminations
			termX = standardInch(term["x"] + xPos) #Note: until now we keep in metric to avoid rounding errors
			termY = standardInch(term["y"] + yPos)

			terminations.update({(termX, termY): {"type": term["type"], "orientation": "compressed"}})

	return terminations


def writeKRF(stream, filename):
	outputFile = open(filename, "w")
	for index, thisNode in enumerate(stream):
		descriptor = thisNode.nodeString()
		position = thisNode.position
		outputFile.write(str(index+1) + "\t" + descriptor + "\t" + str(position[0]) + "\t" + str(position[1]) + "\n")

	return len(stream)


def convertBRDToKRF(filename):
	"""Converts an Eagle .BRD file to a Kaolin .KRF file"""

	# Parse XML contents of .BRD file
	print("... PARSING " + filename)
	tree = ET.parse(filename)

	root = tree.getroot()

	libraries = {thisLibrary.get('name'): library(thisLibrary) for thisLibrary in root.iter("library")}
	print("... IMPORTED " + str(len(libraries)) + " LIBRARIES:")
	for thisLibrary in libraries:
		print("    - " + thisLibrary)

	elements = root.iter("element")

	terminations = getTerminations(elements, libraries)

	elementsForReporting = [thisElement for thisElement in root.iter("element")]


	print("... PROCESSED " + str(len(elementsForReporting)) + " ELEMENTS")
	for thisElement in elementsForReporting:
		print("    - " + thisElement.get("name"))

	print("... FOUND " + str(len(terminations)) + " POSSIBLE TERMINATIONS")

	signals = [signal(thisSignal, terminations) for thisSignal in root.iter("signal")]

	print("... TRAVERSED " + str(len(signals)) + " SIGNALS")
	for thisSignal in signals:
		print("    - " + thisSignal.name + " WITH " + str(len(thisSignal.nodes)) + " NODES, AND " + str(len(thisSignal.wires)) + " WIRES.")

	stream = []
	for thisSignal in signals:
		stream += thisSignal.serialize()
	print("... SERIALIZED INTO " + str(len(stream)) + " WORDS")


	krfFilename = filename.split(".", 1)[0] + ".krf"

	numLines = writeKRF(stream, krfFilename)
	print("... WROTE FILE '" + krfFilename + "' (" + str(numLines) + " LINES)")


	




##### WHEN MODULE IS CALLED FROM TERMINAL #####

if __name__ == "__main__":

	print("")
	print("KAOLIN PCB CONVERSION TOOL")
	print("--------------------------")
	if len(sys.argv) == 1:
		print("Please Provide .brd Filename to Convert")
		print("")
		exit()
	else:
		filename = sys.argv[1]
		convertBRDToKRF(filename)

	#angle test
	# centerNode = node((0.0, 0.0)) 	#this node
	# topNode = node((0.0, 1.0))		#entry node above
	# leftNode = node((1.0, 0.0))		#exit node to the left (+90)
	# bottomNode = node((0.0, -1.0))	#exit node below (+180)
	# rightNode = node((-1.0, 0.0))		#exit node to the right (+270)


	# centerNode.addConnection(rightNode)
	# centerNode.addConnection(leftNode)
	# centerNode.addConnection(bottomNode)
	# centerNode.addConnection(topNode)




	# print(centerNode.getConnections(topNode))


