# Kaolin Toolpath Module
#
# ILAN E. MOYER
# June 1st, 2022
#
# Based on work done during Haystack Labs 2022
#
# This module generates a toolpath based on a KRF file.

from PIL import Image, ImageDraw

import math

class toolpath(list):
	"""A list subtype for storing toolpath information.

	Toolpaths are stored as a tuple (x, y, z, {tags}), where tags are a dictionary containing
	any relevant additional information.
	"""

	def __str__(self):

		returnString = "--- TOOLPATH WITH " + str(len(self)) + " MOVES ---" + "\n"

		for index, point in enumerate(self):
			x, y, z, tags = point
			mode = tags["mode"]

			if mode == "cut" or mode == "home":
				returnString += "MOVE " + str(index+1) + " (" + mode.upper() + ")\t \t X" + str(round(x,2)) + "\t Y" + str(round(y,2)) + "\t Z" + str(round(z,2)) + "\n"
			
			else:
				returnString += "MOVE " + str(index+1) + " (" + mode.upper() + ")\t X" + str(round(x,2)) + "\t Y" + str(round(y,2)) + "\t Z" + str(round(z,2)) + "\n"

		return returnString


def generateToolpath(krf_nodeList, depth, feedrate, traverseHeight, traceWidth, toolDia, metric = False):
	"""Routes a PCB using urumbu_xyz_mill.

	krf_nodeList -- a krf node list object containing the nodes to toolpath
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
	paths = krf_nodeList.travel()

	#generate toolpaths
	thisToolpath = toolpath([(0.0, 0.0, traverseHeight, {"mode":"home"})]) #stored as a list of tuples (X, Y, Z)
	drills = [] #a list of drill positions, stored as (X, Y)
	priorPosition = (0.0, 0.0) #stored as (x pos, y pos, z pos)

	connectionFunctions = {"none": tp_connection_none, "pad": tp_connection_pad, "smd": tp_connection_none, "hole": tp_connection_none}

	for path in paths:
		length = len(path)
		for index, node in enumerate(path):

			#pull properties
			connection = node["connection"]
			orientation = node["orientation"]
			x = node["position"][0] * 25.4 #positions are always stored internally as inch, so convert to metric
			y = node["position"][1] * 25.4

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
					x_next = path[index + 1]["position"][0] * 25.4
					y_next = path[index + 1]["position"][1] * 25.4
					nextPosition = (x_next, y_next)

				connectionFunction = connectionFunctions[connection]


				# --- TOOLPATH ALGORITHMS ---

				if pathStart:
					# Calculate points that traverse the joint
					junctionAngle = absAngle(thisPosition, nextPosition)
					pointsAboutJoint = connectionFunction(jointAngle = 0, orientation = orientation, traceWidth = traceWidth, toolDia = toolDia, firstNode = True)
					absolutePoints = positionAndRotate(pointsAboutJoint, junctionAngle, thisPosition)

					# Generate traverse to the first point
					x_start, y_start = absolutePoints[0]
					thisToolpath += [(x_start, y_start, traverseHeight, {"mode":"traverse"})]

					# Load up toolpath
					for point in absolutePoints:
						x_point, y_point = point
						thisToolpath += [(x_point, y_point, -depth, {"mode":"cut"})]

				elif pathEnd:
					# Calculate points that traverse the joint						
					junctionAngle = absAngle(priorPosition, thisPosition)
					pointsAboutJoint = connectionFunction(jointAngle = 0, orientation = orientation, traceWidth = traceWidth, toolDia = toolDia, lastNode = True)
					absolutePoints = positionAndRotate(pointsAboutJoint, junctionAngle, thisPosition)

					# Load up toolpath
					for point in absolutePoints:
						x_point, y_point = point
						thisToolpath += [(x_point, y_point, -depth, {"mode":"cut"})]

					# Retract					
					x_end, y_end = absolutePoints[-1]
					thisToolpath += [(x_end, y_end, traverseHeight, {"mode":"retract"})]

				else:
					# Calculate points that traverse the joint
					jointAngle, junctionAngle = getJunctionAngles(priorPosition, thisPosition, nextPosition)
					pointsAboutJoint = connectionFunction(jointAngle, orientation, traceWidth, toolDia)
					absolutePoints = positionAndRotate(pointsAboutJoint, junctionAngle, thisPosition)

					#load up toolpath
					for point in absolutePoints:
						x_point, y_point = point
						thisToolpath += [(x_point, y_point, -depth, {"mode":"cut"})]

			priorPosition = thisPosition						

	x_point, y_point = thisPosition
	thisToolpath += [(x_point, y_point, traverseHeight, {"mode": "retract"})]

	# Drill Holes
	for node in krf_nodeList:
		if node["connection"] == "pad" or node["connection"] == "hole":
			x_drill = node["position"][0]*25.4
			y_drill = node["position"][1]*25.4
			thisToolpath += [(x_drill, y_drill, traverseHeight, {"mode":"traverse"})]
			thisToolpath += [(x_drill, y_drill, -depth, {"mode":"drill"})]
			thisToolpath += [(x_drill, y_drill, traverseHeight, {"mode":"retract"})]

	thisToolpath += [(0, 0, traverseHeight, {"mode": "traverse"})]

	return thisToolpath


def absAngle(startPoint, endPoint):
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

def getJunctionAngles(startPoint, midPoint, endPoint):
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

	junctionAngle = absAngle(startPoint, midPoint)
	jointAngle = absAngle(endPoint, midPoint) - junctionAngle

	if jointAngle >= 0:
		return jointAngle, junctionAngle
	else:
		return 360 + jointAngle, junctionAngle


def tp_connection_none(jointAngle, orientation, traceWidth, toolDia, firstNode = False, lastNode = False):
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

def tp_connection_pad(jointAngle, orientation, traceWidth, toolDia, firstNode = False, lastNode = False):
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


def positionAndRotate(inputPointList, junctionAngle, center):
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


def simulate(toolpath, filename = "simulation.png", toolDia = 0.8, resolution = 0.05, canvasWidth = 125, canvasHeight = 125):
	"""Simulates running a toolpath, and outputs to the provided filename.

		toolpath -- the toolpath object to simulate
		filename -- the output filename
		toolDia -- the diameter of the cutting tool, in mm
		resolution -- image resolution in mm/pixel
		canvasWidth -- the width of the canvas in mm
		canvasHeight -- the height of the canvas in mm
		"""

	pixelPitch = 1.0/resolution #pixels/mm

	canvas = Image.new('RGB', (int(canvasWidth*pixelPitch), int(canvasHeight*pixelPitch)), (255, 255, 255))

	draw = ImageDraw.Draw(canvas)

	print("OUTPUTING TOOLPATH SIMULATION TO " + filename)

	lastPoint = (0,0,0, {"mode":"home"})

	inDrillSection = False #flag for when in the drilling section of the toolpath, b/c we'll stop showing traverses


	for point in toolpath:
		x_last, y_last, z_last, tags_last = lastPoint
		x_next, y_next, z_next, tags_next = point
		mode = tags_next["mode"]


		if mode == "traverse": #traverse
			fill = (0, 255, 0) #Green

		elif mode == "cut":
			fill = (0, 0, 255) #Blue

		elif mode == "drill":
			inDrillSection = True

		else:
			fill = (255, 0, 0) #Red

		if not inDrillSection: #draw all moves
			draw.line((int(x_last*pixelPitch), int(canvasHeight*pixelPitch - y_last*pixelPitch), int(x_next*pixelPitch), int(canvasHeight*pixelPitch - y_next*pixelPitch)), fill = fill, width=int(toolDia*pixelPitch))
		
		else: #render drill only
			renderRadius = toolDia

			x1 = x_next - renderRadius
			y1 = y_next + renderRadius
			x2 = x_next + renderRadius
			y2 = y_next - renderRadius

			draw.ellipse((int(x1 * pixelPitch), int(canvasHeight*pixelPitch - y1*pixelPitch), int(x2 * pixelPitch), int(canvasHeight*pixelPitch - y2*pixelPitch)), fill=(255, 0, 255), outline=(0, 0, 0))


		lastPoint = point


	x_last, y_last , z_last, tags_last = lastPoint

	draw.line((int(x_next*pixelPitch), int(canvasHeight*pixelPitch - y_next*pixelPitch), 0, canvasHeight*pixelPitch), fill = (0,255,0), width= int(toolDia*pixelPitch))

	canvas.save(filename)			

	# #run urumbu
	# else:

	# 	print("RUNNING TOOLPATH ON URUMBU")

	# 	actionQueue = urumbu_xyz_mill.init_machine("/dev/tty.usbmodem11401", "/dev/tty.usbmodem11301", "/dev/tty.usbmodem11201", "/dev/tty.usbmodem111401")

	# 	#turn on spindle
	# 	# actionQueue.put(urumbu_xyz_mill.SpindleAction('spindle', True))
	# 	# actionQueue.put(urumbu_xyz_mill.WaitAction(1))

	#     #load in program
	# 	for position in toolpath:
	# 		x, y, z = position

	# 		print("   X: " + str(round(x, 2)) + "MM, Y: " + str(round(y,2)) + "MM, Z: " + str(round(z,2)) + "MM @ " + str(round(feedrate,0)) + "MM/SEC")
	# 		actionQueue.put(urumbu_xyz_mill.Line([x, y, z], feedrate))


	# 	#Drill all holes
	# 	for drillPosition in drills:
	# 		x, y = drillPosition
	# 		actionQueue.put(urumbu_xyz_mill.Line([x, y, traverseHeight], feedrate))
	# 		actionQueue.put(urumbu_xyz_mill.Line([x, y, -1.6], feedrate))
	# 		actionQueue.put(urumbu_xyz_mill.Line([x, y, traverseHeight], feedrate))

	# 	actionQueue.put(urumbu_xyz_mill.Line([0, 0, traverseHeight], feedrate))
	# 	actionQueue.put(urumbu_xyz_mill.Line([0, 0, 0], feedrate))

	#     #turn off spindle
	# 	# actionQueue.put(urumbu_xyz_mill.SpindleAction('spindle', False))

	#     #wait
	# 	time.sleep(10)