import csv

def loadKRFFromFile(filename):
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