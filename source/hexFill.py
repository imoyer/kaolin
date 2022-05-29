startLine = 0x00B0
endLine = 0x1A00

lineSize = 16

startLine_decimal = int(startLine)
endLine_decimal = int(endLine)

numLines = int((endLine_decimal - startLine_decimal) /lineSize)
print(numLines)

print("GENERATING " + str(numLines) + " LINES")

for line in range(numLines):
	address = startLine_decimal + line*lineSize

	padding = 6
	hexString = f"{address:#0{padding}x}"

	print(":10" + hexString[2:] + "00FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF")
