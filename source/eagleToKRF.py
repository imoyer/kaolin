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



class library(object):
	"""This class stores all library packages in the board."""

	def __init__(self, xmlElementList):
		"""The library is initialized by injecting a list of xml elements - each representing a single library -
		into an internal representation format.

		The internal format is as follows:
			{libraryName : {packageName : [connection1, connection2, ...], packageName: [...]}, libraryName: {...}}

			Each connection is stored as a dictionary as well:
				{"type": connectionType, ""
		"""




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

		#PARSE BRD FILE AS XML
		tree = ET.parse(filename)
		root = tree.getroot()

		#READ IN ALL ELEMENTS
		self.xmlElements_libraries = [thisLibrary for thisLibrary in root.iter("library")]
		self.xmlElements_components = [thisComponent for thisComponent in root.iter("element")]
		self.xmlElements_signals = [thisSignal for thisSignal in root.iter("signal")]






