import utilities
import os, sys

def encodeNodeList(nodeList):
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


		#Y Position
		encodedValue += (int(node["yPosition"]*100))<<9 # 9 bits, 0.01" resolution

		#X Position
		encodedValue += (int(node["xPosition"]*100)) # 9 lower nine bits, 0.01" resolution

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





def hexFile(byteList, filename, startAddress, maxByteCount = 16):
	"""Writes out an intel hex file based on the provided byte list.

	byteList -- an ordered list of bytes in big-endian format [node0_byte2, node0_byte1, node0_byte0, node1_byte2,... ]
	filename -- the filename to output
	startAddress -- the starting memory address of the file
	"""

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
	print("WROTE " + filename + ": " + str(int(len(byteList)/3)) + " NODES (" + str(len(byteList)) + " BYTES) @ 0x" + str(hex(startAddress)).upper()[2:])



nodeList = utilities.loadKRFFromFile(sys.argv[1])
hexFilename = os.path.splitext(sys.argv[1])[0] +".hex"

byteList = encodeNodeList(nodeList)
hexFile(byteList, hexFilename, 0x1A00) #last 1k of attiny84 program memory


