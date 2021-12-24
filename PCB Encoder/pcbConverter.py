# Kaolin PCB Converter Utility
#
# ILAN E. MOYER
# March 30th, 2021
#
# One goal of the Kaolin project is to embed the design for an MCU's PCB within the memory of the MCU
# This script reads in an Autodesk Eagle PCB (.brd) file, and converts it into the Kaolin format, and
# saves out an Intel HEX file.

import xml.etree.ElementTree as ET

