# Kaolin Eagle PCB Converter Utility
#
# ILAN E. MOYER
# April 27th, 2022
#
# This module reads in an Autodesk Eagle board file (.brd) and converts it into the internal nodeList format 
# used by krfTools.py. From there, a .KRF or .HEX file can be exported.

import xml.etree.ElementTree as ET
import os, sys
import math


# ---- UTILITIES ----
def standardInch(metricString):
	"""Converts a position from metric to inches, with three digits of precision."""
	return round(float(metricString)/25.4, 3)


# ---- CORE CLASSES ----

class library(object):
	"""This class stores all library packages in the board."""

	def __init__(self, xmlElementList):
		"""The library is initialized by injecting a list of xml elements - EACH REPRESENTING A SINGLE LIBRARY -
		into an internal representation format.

		The internal format is as follows:
			{libraryName : {packageName : [connection1, connection2, ...], packageName: [...]}, libraryName: {...}}

			Connections are pads, smts, holes, etc. Each connection is stored as a dictionary:
				{"name": connectionName, type": connectionType, "orientation": orientation, "position": position}
					
					connectionName: the name of the connection within the package
					connectionType: "pad", "smd", "hole"
					orientation: "vertical", "horizontal", "compressed"
					position: a tuple in (x, y) format.
		
		xmlElementList should be a list of ElementTree objects.
		"""

		self.libraries = {} #stores all imported libraries.


		print("... IMPORTING " + str(len(xmlElementList)) + " LIBRARIES:")

		for thisLibrary in xmlElementList:
			libraryName = self.addLibrary(thisLibrary)
			print("    - " + libraryName)	

	def hasLibrary(self, libraryName):
		"""Returns True if a library with the specified name exists."""
		return libraryName in self.libraries

	def getLibrary(self, libraryName):
		"""Returns a library dictionary with the specified name."""
		if libraryName in self.libraries:
			return self.libraries[libraryName]
		else:
			return False


	def addLibrary(self, library_xmlElement):
		"""Adds a library to the internal libraries dictionary.

		library_xmlElement -- a library ElementTree object.
		"""
		name = library_xmlElement.get("name")
		self.libraries.update({name: self.getPackages(library_xmlElement)})

		return name


	def getPackages(self, library_xmlElement):
		"""Returns a dictionary of packages contained within a library.

		library_xmlElement -- a library ElementTree object.

		Returns {packageName : [connection1, connection2, ...], packageName: [...]}
		"""	
		return {package.get("name"): self.getConnections(package) for package in library_xmlElement.iter("package")}

	def getConnections(self, package_xmlElement):
		"""Returns a list of connection dictionaries contained within a package.

		Connections are pads, smts, holes, etc. Each connection is stored as a dictionary:
				{"name": connectionName, type": connectionType, "orientation": orientation, "position": position}

					connectionName: the name of the connection within the package
					connectionType: "pad", "smd", "hole"
					orientation: "vertical", "horizontal", "compressed"
					position: a tuple in (x, y) format, STORED AS METRIC to maintain precision for now.

				Note that position is defined as a relative position with respect to the package datum.
		"""

		connections = [] #stores all connections found in the package

		#add pads
		for connection in package_xmlElement.iter("pad"):
			name = connection.get("name")
			orientation = self.getPadOrientation(connection)
			position = (float(connection.get("x")), float(connection.get("y")))
			connections += [{"name": name, "type": "pad", "orientation": orientation, "position": position}]

		#add smt here...

		return connections


	def getPadOrientation(self, connection_xmlElement):
		"""Returns the relative orientation of a pad connection, with respect to the package.

		For now, we only support "compressed" orientations, so that's all that we return.
		"""
		return "compressed"


class node(object):
	"""A position on the signal, with attachments to other nodes."""
	def __init__(self, position, allConnections):
		self.position = position
		self.attachments = [] #a list of all nodes that are connected to this node
		self.connection = self.findConnection(allConnections) #connections i.e. pad, smd, etc

	def __repr__(self):
		return self.connection["type"] + " @ " + str(self.position)

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

		if len(self.attachments) in routingDict:
			return routingDict[len(self.attachments)] + connectionDict[self.connection["type"]] + orientationDict[self.connection["orientation"]]
		else:
			print("ERROR: NODE AT " + str(self.position) + " HAS TOO MANY (" + str(len(self.attachments)) + ") CONNECTIONS.")
			print("")
			return None




	def findConnection(self, allConnections):
		"""Finds the connection type and orientation of the node.

		This function will search the provided avaliable position-indexed connections dictionary, 
		and find a match based on position.
		"""
		if self.position in allConnections:
			return allConnections[self.position]
		else:
			return {"type" : "none", "orientation" : "none"}



	def addAttachment(self, node):
		"""Adds an attachment to the node."""
		self.attachments += [node]

	def isEndpoint(self):
		"""Returns whether this node is an endpoint."""
		return (len(self.attachments) == 1)

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

	def getAttachments(self, entryNode = None):
		"""Returns a list of attached nodes in the format [(node1, angle1), (node2, angle2), ...].

		entryNode -- If provided, the line between entryNode and this node creates a frame of reference for angle measurements.
					 Note that if this isn't provided, an absolute angle is returned (0 deg is right-going horizontal, 90 deg is upward horizontal)

		The returned list is sorted by angle. If an entryNode is provided, the reciprocal attachment to the entryNode is listed last, as it is a 360 deg angle.
		"""
		angles = [self.getAngle(entryNode, thisNode) for thisNode in self.attachments]

		if entryNode != None:
			angles = [360.0 if angle == 0 else angle for angle in angles] #changes 0 to 360 for the reciprocal node

		nodesAndAngles = list(zip(self.attachments, angles))


		nodesAndAngles.sort(key=lambda y: y[1])

		return nodesAndAngles



class signal(object):
	"""Stores all data related to an attached set of traces."""
	def __init__(self, xmlElement, connections):

		self.nodes = []	#this will be where we store all nodes

		self.name = xmlElement.get('name')

		self.allConnections = connections #a list of all avaliable connections on the board.

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
		tempStartNode, tempEndNode = (node(wire[0], self.allConnections), node(wire[1], self.allConnections))

		startNode, endNode = (self.doesNodeExist(tempStartNode), self.doesNodeExist(tempEndNode))

		if startNode == False:
			startNode = self.addNode(tempStartNode)

		if endNode == False:
			endNode = self.addNode(tempEndNode)

		startNode.addAttachment(endNode)
		endNode.addAttachment(startNode)



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
		nextNode = startNode.getAttachments()[0][0]

		#iterate until we reach the start again
		while nextNode != startNode:

			#We've never been to this node before
			if not nextNode in traversedNodes:
				serialStream += [nextNode] #add the node to the serialization list

				attachments = nextNode.getAttachments(lastNode)

				traversedNodes.update({nextNode:attachments})
			
			#Switch to this node
			lastNode = nextNode

			nextNode = traversedNodes[lastNode].pop(0)[0]

		
		return serialStream





class board(object):
	"""This class is used to represent the salient elements of an Eagle board file."""

	def __init__(self, filename):
		"""Initializes the board object using an Eagle .brd file.

		filename -- the filename to be read into the board object.
		
		An Eagle board file has several elements that are important to us (and many that are not.) The important ones are:

			libraries: 		Collections of packages, each of which contain the relative locations of pads, smds, and holes.
			components: 	These are instances of packages that are actually used, and their absolute locations on the board.
			signals: 		Each signal represents a set of electrically connected wires.
		"""

		print("... PARSING " + filename)

		#PARSE BRD FILE AS XML
		tree = ET.parse(filename)
		root = tree.getroot()

		#READ IN ALL ELEMENTS
		xmlElements_libraries = [thisLibrary for thisLibrary in root.iter("library")]
		xmlElements_components = [thisComponent for thisComponent in root.iter("element")]
		xmlElements_signals = [thisSignal for thisSignal in root.iter("signal")]

		self.libraries = library(xmlElements_libraries)

		print("... PROCESSED " + str(len(xmlElements_components)) + " COMPONENTS")
		for thisComponent in xmlElements_components:
			print("    - " + thisComponent.get("name"))

		self.connections = self.getConnections(xmlElements_components)

		print("... FOUND " + str(len(self.connections)) + " POSSIBLE CONNECTIONS")


		self.signals = [signal(thisSignal, self.connections) for thisSignal in xmlElements_signals]
		print("... TRAVERSED " + str(len(self.signals)) + " SIGNALS")
		for thisSignal in self.signals:
			print("    - " + thisSignal.name + " WITH " + str(len(thisSignal.nodes)) + " NODES, AND " + str(len(thisSignal.wires)) + " WIRES.")

		self.stream = []
		for thisSignal in self.signals:
			self.stream += thisSignal.serialize()
		print("... SERIALIZED INTO " + str(len(self.stream)) + " WORDS")

	def getStream(self):
		"""Returns a serialized list of node objects representing the KRF sequence."""
		return self.stream


	def getConnections(self, component_xmlElements):
		"""Returns all connections (i.e. pads, smds, holes) on the board.

		component_xmlElements -- a list of components that are placed on the board.

		This method will locate each board component in the library, and return the
		connection elements (including their position on the board).

		Returns a dictionary keyed by board position:
			{(xPos, yPos): {"type":typeCode, "orientation":orientationCode}}
					typeCode: "pad", "smd", "hole"
					orientationCode: "vertical", "horizontal", "compressed"
		"""

		connections = {}

		for component in component_xmlElements:

			xPos = float(component.get("x"))
			yPos = float(component.get("y"))
			libraryName = component.get("library")
			componentName = component.get("name")
			packageName = component.get("package")

			if self.libraries.hasLibrary(libraryName):
				library = self.libraries.getLibrary(libraryName)
			else:
				print("ERROR: Element '" + componentName + "' REFERENCES MISSING LIBRARY '" + libraryName + "'")
				return {}

			if packageName in library:
				package = library[packageName]
			else:
				print("ERROR: Element '" + componentName + "' REFERENCES MISSING PACKAGE + '" + packageName + "' IN LIBRARY '" + libraryName + "'")
				return {}

			for connection in package:
				#first, get the board positions of the terminations
				x, y = connection["position"]
				connection_x = standardInch(x + xPos) #Note: until now we keep in metric to avoid rounding errors
				connection_y = standardInch(y + yPos)

				connections.update({(connection_x, connection_y): {"type": connection["type"], "orientation": connection["orientation"]}})

		return connections



def BRDtoKRF(filename):
	"""Converts a BRD file into a KRF nodeList.

	filename -- the filename of the BRD file.

	Returns nodeList
	"""
	pass 

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
		BRDtoKRF(filename)


