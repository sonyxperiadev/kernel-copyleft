# Copyright (c) 2018, 2020 The Linux Foundation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 and
# only version 2 as published by the Free Software Foundation.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# Cachedump parser

import os
import sys
import getopt
from struct import unpack
from math import log10, ceil, log
import ast

#-------------------------------------------------------------------------------
# Generic data structure to hold a single cache-line(can be tag or data)
# Position of the cacheline is indicated by 'set' and 'way'
# Holds an array of 32bit(4Byte) elements
#-------------------------------------------------------------------------------
class CacheLine():
    def __init__(self):
        self.set = -1
        self.way = -1
        self.elements = [] # array of unsigned integers(4B)
    def setSet(self, s):
        self.set = s
    def getSet(self):
        return self.set
    def setWay(self, w):
        self.way = w
    def getWay(self):
        return self.way
    def addElement(self, e):
        self.elements.append(e)
    def getNumElements(self):
        return len(self.elements)
    def numElements(self):
        return self.getNumElements()
    def getElement(self, i, j = None):
        if j is None:
            return self.elements[i]
        else:
            return self.elements[i] | (self.elements[j] << 32)
    def getElementStr(self, i):
        return '{:>08X}'.format(self.getElement(i))
    def getElements(self):
        return self.elements
    def setElements(self, e):
        self.elements = e
    def __str__(self):
        return '{:>4}'.format(self.set) + ' ' + '{:>2}'.format(self.way) + ' ' + ' '.join([self.getElementStr(i) for i in range(self.getNumElements())])

# Generic data structure to hold an array of cacheLines
class CacheRam():
    def __init__(self, numWays, numSets, numOffsets, cacheLines, cls):
        self.sets = numSets
        self.ways = numWays
        self.offsets = numOffsets
        self.lines = cacheLines
        self.cls = cls
    def morph(self):
        arr = []
        for i in self.lines:
            arr.append(self.cls(i))
        del(self.lines[:])
        self.lines = arr
        return self
    def __str__(self):
        s = ''
        for l in self.lines:
            s = s + str(l) + '\n'
        return s

class Attribute():
    def __init__(self, name, offset, width, val=None, h=None):
        self.name = name
        self.offset = offset%32 # in bits
        self.width = width # no. of bits
        if val != None:
            self.val = self.getFromWord(val)
        else:
            self.val = None
        self.hex = h
    def getMask(self):
        mask = 0
        for i in range(0, self.width):
            mask = mask + 2**i
        return mask
    def getBitMask(self):
        return self.getMask() << self.offset
    def getFromWord(self, w):
        return (w >> self.offset) & self.getMask()
    def dispWidth(self):
        if self.hex != None:
            width = ceil(log(self.getMask(),16))
        else:
            width = ceil(log10(self.getMask()))
        return int(width)
    def __str__(self):
        s = ''
        width = self.dispWidth()
        if self.hex != None:
            s = '{:>0{width}X}'.format(self.val, width=width)
        else:
            s = '{:>{width}}'.format(self.val, width=width)

        return s

def getAttrStr(line):
    s = ''
    for attr in line.attrArr:
        s = s + str(attr.val) + ' '
    return s

def getHeaderStr(line):
    try:
        return 'Set Way ' + ' '.join(a.name + ('(0x)' if a.hex else '') for a in line.attrArr) + '\n'
    except AttributeError:
        return 'Set Way ' + ' '.join('{:>8X}'.format(i*4) for i in range(0,len(line.elements))) + '\n'

#-------------------------------------------------------------------------------
# Kryo-3 Silver cache descriptor data structures
#-------------------------------------------------------------------------------
class Kryo3SilverL1DCacheTagLine(CacheLine):
    def __init__(self, cl):
        self.setWay(cl.getWay())
        self.setSet(cl.getSet())
        self.attrArr = []
        self.attrArr.append(Attribute('OuterAllocationHint', 0, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('Age', 1, 2, cl.getElement(0)))
        self.attrArr.append(Attribute('Shareability', 3, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('Dirty', 4, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('TagAddr', 1, 28, cl.getElement(1), 1))
        self.attrArr.append(Attribute('NS', 29, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('MESI', 30, 2, cl.getElement(1)))
    def __str__(self):
        return '{:>4}'.format(self.set) + ' ' + '{:>2}'.format(self.way) + ' ' + ' '.join(str(a) for a in self.attrArr)

class Kryo3SilverL1ICacheTagLine(CacheLine):
    def __init__(self, cl):
        self.setWay(cl.getWay())
        self.setSet(cl.getSet())
        self.attrArr = []
        self.attrArr.append(Attribute('TagAddr', 0, 28, cl.getElement(0), 1))
        self.attrArr.append(Attribute('NS', 28, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('ValidAndSetMode', 29, 2, cl.getElement(0)))
    def __str__(self):
        return '{:>4}'.format(self.set) + ' ' + '{:>2}'.format(self.way) + ' ' + ' '.join(str(a) for a in self.attrArr)

class Kryo3SilverL2TLBTagLine(CacheLine):
    def __init__(self, cl):
        self.setWay(cl.getWay())
        self.setSet(cl.getSet())
        self.attrArr = []
        self.attrArr.append(Attribute('Valid', 0, 1, cl.getElement(0), 1))
        self.attrArr.append(Attribute('NS', 1, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('ASID', 2, 16, cl.getElement(0), 1))
        self.attrArr.append(Attribute('VMID', 18, 16, cl.getElement(0), 1))
        self.attrArr.append(Attribute('size', 34, 3, cl.getElement(1)))
        self.attrArr.append(Attribute('nG', 37, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('APHyp', 38, 3, cl.getElement(1)))
        self.attrArr.append(Attribute('S2AP', 41, 2, cl.getElement(1)))
        self.attrArr.append(Attribute('Dom', 43, 4, cl.getElement(1)))
        self.attrArr.append(Attribute('S1Size', 47, 3, cl.getElement(1)))
        self.attrArr.append(Attribute('AddrSignBit', 50, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('VA', 51, 28, (cl.getElement(2)<<32)|cl.getElement(1), 1))
        self.attrArr.append(Attribute('DBM', 79, 1, cl.getElement(2)))
        self.attrArr.append(Attribute('Parity', 80, 2, cl.getElement(2)))
    def __str__(self):
        return '{:>4}'.format(self.set) + ' ' + '{:>2}'.format(self.way) + ' ' + ' '.join(str(a) for a in self.attrArr)

class Kryo3SilverL2TLBDataLine(CacheLine):
    def __init__(self, cl):
        self.setWay(cl.getWay())
        self.setSet(cl.getSet())
        self.attrArr = []
        self.attrArr.append(Attribute('XS1Usr', 0, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('XS1NonUsr', 1, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('XS2Usr', 2, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('XS2NonUsr', 3, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('MemTypeAndShareability', 4, 8, cl.getElement(0), 1))
        self.attrArr.append(Attribute('S2Level', 12, 2, cl.getElement(0)))
        self.attrArr.append(Attribute('NS', 14, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('PA', 15, 28, (cl.getElement(1)<<32)|cl.getElement(0), 1))
        self.attrArr.append(Attribute('Parity', 43, 1, cl.getElement(1)))
    def __str__(self):
        return '{:>4}'.format(self.set) + ' ' + '{:>2}'.format(self.way) + ' ' + ' '.join(str(a) for a in self.attrArr)

#-------------------------------------------------------------------------------
# Kryo-3 Gold cache descriptor data structures
#-------------------------------------------------------------------------------
class Kryo3GoldL1DCacheTagLine(CacheLine):
    def __init__(self, cl):
        self.setWay(cl.getWay())
        self.setSet(cl.getSet())
        self.attrArr = []
        self.attrArr.append(Attribute('PBHA', 0, 2, cl.getElement(0)))
        self.attrArr.append(Attribute('LineState', 2, 2, cl.getElement(0)))
        self.attrArr.append(Attribute('MemAttr', 4, 3, cl.getElement(0)))
        self.attrArr.append(Attribute('RRIP', 7, 3, cl.getElement(0)))
        self.attrArr.append(Attribute('TagAddr', 10, 32, (cl.getElement(1)<<32)|cl.getElement(0), 1))
        self.attrArr.append(Attribute('NS', 10, 1, cl.getElement(1)))
    def __str__(self):
        return '{:>4}'.format(self.set) + ' ' + '{:>2}'.format(self.way) + ' ' + ' '.join(str(a) for a in self.attrArr)

class Kryo3GoldL1ICacheTagLine(CacheLine):
    def __init__(self, cl):
        self.setWay(cl.getWay())
        self.setSet(cl.getSet())
        self.attrArr = []
        self.attrArr.append(Attribute('TagAddr', 0, 28, cl.getElement(0), 1))
        self.attrArr.append(Attribute('NS', 28, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('Valid', 29, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('Reg1', 0, 32, cl.getElement(1), 1))
    def __str__(self):
        return '{:>4}'.format(self.set) + ' ' + '{:>2}'.format(self.way) + ' ' + ' '.join(str(a) for a in self.attrArr)

class Kryo3GoldL2TLBTagLine(CacheLine):
    def __init__(self, cl):
        self.setWay(cl.getWay())
        self.setSet(cl.getSet())
        self.attrArr = []
        self.attrArr.append(Attribute('S1Level', 0, 2, cl.getElement(0)))
        self.attrArr.append(Attribute('S1TrMode', 2, 2, cl.getElement(0)))
        self.attrArr.append(Attribute('TrRegime', 4, 2, cl.getElement(0)))
        self.attrArr.append(Attribute('PageSize', 6, 2, cl.getElement(0)))
        self.attrArr.append(Attribute('nG', 9, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('ASID', 10, 8, cl.getElement(0), 1))
        self.attrArr.append(Attribute('ASID', 18, 4, cl.getElement(0), 1))
        self.attrArr.append(Attribute('ASID', 22, 4, cl.getElement(0), 1))
        self.attrArr.append(Attribute('VMID', 26, 16, (cl.getElement(1)<<32)|cl.getElement(0), 1))
        self.attrArr.append(Attribute('VA', 42, 74-42+1, (cl.getElement(2)<<32)|cl.getElement(1), 1))
        self.attrArr.append(Attribute('Valid', 75, 1, cl.getElement(2)))
        self.attrArr.append(Attribute('Parity', 76, 2, cl.getElement(2)))
    def __str__(self):
        return '{:>4}'.format(self.set) + ' ' + '{:>2}'.format(self.way) + ' ' + ' '.join(str(a) for a in self.attrArr)

class Kryo3GoldL2TLBDataLine(CacheLine):
    def __init__(self, cl):
        self.setWay(cl.getWay())
        self.setSet(cl.getSet())
        self.attrArr = []
        self.attrArr.append(Attribute('PA', 0, 32, cl.getElement(0), 1))
        self.attrArr.append(Attribute('NS', 32, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('AP', 33, 3, cl.getElement(1)))
        self.attrArr.append(Attribute('S2UXN', 36, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('S2PXN', 37, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('S1UXN', 38, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('S1PXN', 39, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('S2Level', 40, 2, cl.getElement(1)))
        self.attrArr.append(Attribute('HAP', 42, 2, cl.getElement(1)))
        self.attrArr.append(Attribute('S2DBM', 44, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('MemAttr', 45, 4, cl.getElement(1), 1))
        self.attrArr.append(Attribute('ITH', 49, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('Split', 50, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('Parity', 53, 1, cl.getElement(1)))
    def __str__(self):
        return '{:>4}'.format(self.set) + ' ' + '{:>2}'.format(self.way) + ' ' + ' '.join(str(a) for a in self.attrArr)

#-------------------------------------------------------------------------------
# Kryo-4 Gold cache descriptor data structures
#-------------------------------------------------------------------------------
class Kryo4GoldL2TLBLine(CacheLine):
    def __init__(self, cl):
        self.setWay(cl.getWay())
        self.setSet(cl.getSet())
        self.attrArr = []
        self.attrArr.append(Attribute('Valid', 2, 4, cl.getElement(0), 1))
        self.attrArr.append(Attribute('Coalesced', 6, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('PageSize', 17, 3, cl.getElement(0), 1))
        self.attrArr.append(Attribute('PA', 20, 28, cl.getElement(0, 1), 1))
        self.attrArr.append(Attribute('MemAttr', 52, 3, cl.getElement(1)))
        self.attrArr.append(Attribute('InnerShared', 56, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('OuterShared', 57, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('nonGlobal', 58, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('NS', 6, 1, cl.getElement(2)))
        self.attrArr.append(Attribute('VA', 7, 29, cl.getElement(2, 3), 1))
        self.attrArr.append(Attribute('Prefetched', 36, 1, cl.getElement(3), 1))
        self.attrArr.append(Attribute('walkCache', 37, 1, cl.getElement(3), 1))
        self.attrArr.append(Attribute('PBHA', 38, 1, cl.getElement(3), 1))
        self.attrArr.append(Attribute('ASID', 40, 16, cl.getElement(3), 1))
        self.attrArr.append(Attribute('VMID', 56, 16, cl.getElement(3, 4), 1))
        self.attrArr.append(Attribute('TxlnRegime', 8, 2, cl.getElement(4), 1))
    def __str__(self):
        return '{:>4}'.format(self.set) + ' ' + '{:>2}'.format(self.way) + ' ' + ' '.join(str(a) for a in self.attrArr)

class Kryo4GoldL1DTLBLine(CacheLine):
    def __init__(self, cl):
        self.setWay(cl.getWay())
        self.setSet(cl.getSet())
        self.attrArr = []
        self.attrArr.append(Attribute('Valid', 0, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('VMID', 1, 16, cl.getElement(0), 1))
        self.attrArr.append(Attribute('ASID', 17, 16, cl.getElement(0, 1), 1))
        self.attrArr.append(Attribute('TxlnRegime', 33, 2, cl.getElement(1), 1))
        self.attrArr.append(Attribute('NS', 35, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('PageSize', 36, 3, cl.getElement(1)))
        self.attrArr.append(Attribute('MemAttr', 50, 3, cl.getElement(1)))
        self.attrArr.append(Attribute('InnerShared', 57, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('OuterShared', 58, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('VA', 62, 37, (cl.getElement(3) << 64 ) | cl.getElement(1, 2), 1))
        self.attrArr.append(Attribute('PA', 35, 28, cl.getElement(3), 1))
        self.attrArr.append(Attribute('PBHA', 63, 2, cl.getElement(3, 4)))
    def __str__(self):
        return '{:>4}'.format(self.set) + ' ' + '{:>2}'.format(self.way) + ' ' + ' '.join(str(a) for a in self.attrArr)

class Kryo4GoldL1ITLBLine(CacheLine):
    def __init__(self, cl):
        self.setWay(cl.getWay())
        self.setSet(cl.getSet())
        self.attrArr = []
        self.attrArr.append(Attribute('Valid', 0, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('Attr1', 1, 4, cl.getElement(0), 1))
        self.attrArr.append(Attribute('TxlnRegime', 5, 2, cl.getElement(0), 1))
        self.attrArr.append(Attribute('VMID', 7, 16, cl.getElement(0), 1))
        self.attrArr.append(Attribute('ASID', 23, 16, cl.getElement(0, 1), 1))
        self.attrArr.append(Attribute('Attr2', 39, 5, cl.getElement(1), 1))
        self.attrArr.append(Attribute('InnerShared', 44, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('OuterShared', 45, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('Attr3', 46, 4, cl.getElement(1)))
        self.attrArr.append(Attribute('PageSize', 50, 3, cl.getElement(1)))
        self.attrArr.append(Attribute('MemAttr', 53, 3, cl.getElement(1)))
        self.attrArr.append(Attribute('Attr4', 56, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('PBHA', 57, 2, cl.getElement(1)))
        self.attrArr.append(Attribute('VA', 59, 37, cl.getElement(1, 2), 1))
        self.attrArr.append(Attribute('PA', 0, 28, cl.getElement(3), 1))
        self.attrArr.append(Attribute('NS', 60, 1, cl.getElement(3)))
    def __str__(self):
        return '{:>4}'.format(self.set) + ' ' + '{:>2}'.format(self.way) + ' ' + ' '.join(str(a) for a in self.attrArr)

class Kryo4GoldL1DCacheTagLine(CacheLine):
    def __init__(self, cl):
        self.setWay(cl.getWay())
        self.setSet(cl.getSet())
        self.attrArr = []
        self.attrArr.append(Attribute('MESI', 0, 2, cl.getElement(0)))
        self.attrArr.append(Attribute('WBNA', 2, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('PA', 5, 28, cl.getElement(0, 1), 1))
        self.attrArr.append(Attribute('NS', 33, 1, cl.getElement(1)))
        self.attrArr.append(Attribute('ECC', 34, 7, cl.getElement(1)))
    def __str__(self):
        return '{:>4}'.format(self.set) + ' ' + '{:>2}'.format(self.way) + ' ' + ' '.join(str(a) for a in self.attrArr)

class Kryo4GoldL1ICacheTagLine(CacheLine):
    def __init__(self, cl):
        self.setWay(cl.getWay())
        self.setSet(cl.getSet())
        self.attrArr = []
        self.attrArr.append(Attribute('Parity', 0, 1, cl.getElement(0), 0))
        self.attrArr.append(Attribute('ISA', 1, 2, cl.getElement(0)))
        self.attrArr.append(Attribute('PA', 3, 28, cl.getElement(0), 1))
        self.attrArr.append(Attribute('NS', 31, 1, cl.getElement(0), 0))
    def __str__(self):
        return '{:>4}'.format(self.set) + ' ' + '{:>2}'.format(self.way) + ' ' + ' '.join(str(a) for a in self.attrArr)

class Kryo4GoldL2DCache256KBTagLine(CacheLine):
    def __init__(self, cl):
        self.setWay(cl.getWay())
        self.setSet(cl.getSet())
        self.attrArr = []
        self.attrArr.append(Attribute('MESI', 0, 3, cl.getElement(0), 1))
        self.attrArr.append(Attribute('Valid', 3, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('OA', 4, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('Shareable', 5, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('VirtIndex', 9, 2, cl.getElement(0)))
        self.attrArr.append(Attribute('NS', 11, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('PA', 12, 25, cl.getElement(0, 1), 1))
        self.attrArr.append(Attribute('PBHA', 37, 2, cl.getElement(1)))
        self.attrArr.append(Attribute('ECC', 39, 7, cl.getElement(1)))
    def __str__(self):
        return '{:>4}'.format(self.set) + ' ' + '{:>2}'.format(self.way) + ' ' + ' '.join(str(a) for a in self.attrArr)

class Kryo4GoldL2DCache512KBTagLine(CacheLine):
    def __init__(self, cl):
        self.setWay(cl.getWay())
        self.setSet(cl.getSet())
        self.attrArr = []
        self.attrArr.append(Attribute('MESI', 0, 3, cl.getElement(0), 1))
        self.attrArr.append(Attribute('Valid', 3, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('OA', 4, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('Shareable', 5, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('VirtIndex', 9, 2, cl.getElement(0)))
        self.attrArr.append(Attribute('NS', 11, 1, cl.getElement(0)))
        self.attrArr.append(Attribute('PA', 12, 24, cl.getElement(0, 1), 1))
        self.attrArr.append(Attribute('PBHA', 36, 2, cl.getElement(1)))
        self.attrArr.append(Attribute('ECC', 38, 7, cl.getElement(1)))
    def __str__(self):
        return '{:>4}'.format(self.set) + ' ' + '{:>2}'.format(self.way) + ' ' + ' '.join(str(a) for a in self.attrArr)

#-------------------------------------------------------------------------------
# This is a generic function to extract words(4B) from a file
# It is assumed that 'inputFileName' is valid and exists
# The words are extracted in a nested for loop as follows:
# for i in (0: loopLimitA-1)
#     for j in (0: loopLimitB-1)
#         for k in (0: loopLimitC-1)
#              read 4B word
#-------------------------------------------------------------------------------
def extractFileContents(inputFileName, offset, loopLimitA, loopLimitB, loopLimitC, cr):
    cacheLineArr = []
    inFile = open(inputFileName, 'rb')
    inFile.seek(offset)
    for iterWay in range(loopLimitA):
        for iterSet in range(loopLimitB):
            cacheLine = CacheLine()
            cacheLine.setWay(iterWay)
            cacheLine.setSet(iterSet)
            for iterOffset in range(loopLimitC):
                word = inFile.read(4)
                cacheLine.addElement(unpack('I', word)[0])
            cacheLineArr.append(cacheLine)
    inFile.close()
    cr.sets = loopLimitB
    cr.ways = loopLimitA
    cr.offsets = loopLimitC
    cr.lines = cacheLineArr
    return

def main(argv):
    # Parse the command line args
    inputFileName=''
    outputFileName=''
    dumpType = 'DCD'
    offset = 0
    cpu = ''

    try:
        opts, args = getopt.getopt(argv,"hi:o:t:s:c:",["help","ifile=","ofile=","type=","seek=","cpu="])
    except getopt.GetoptError:
        print('ERROR: Incorrect arguments. Run with -h for help')
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-h", "--help"):
            print('parse.py -i <inputfile> -c <cpu> -t <cachedump type> -s <offset>')
            print('    -c can take kryo3gold, kryo3silver')
            print("    -t can take DCD(for D$ Data), DCT(for D$ Tag), ICD(for I$ Data), ICT(for I$ Tag), TLBD(for TLB Data), TLBT(for TLB Tag)")
            print('    -s is a byte offset into the input file')
            print('All output gets written to stdout')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputFileName = arg
        elif opt in ("-o", "--ofile"):
            outputFileName = arg
        elif opt in ("-t", "--type"):
            dumpType = arg.upper()
        elif opt in ("-c", "--cpu"):
            cpu = arg.lower()
        elif opt in ("-s", "--seek"):
            offset = ast.literal_eval(arg)
    if inputFileName == '' or not os.path.isfile(inputFileName):
        sys.stderr.write('Error: You need to provide a valid input file name\n')
        sys.exit(2)

    sys.stderr.write("Input file  :" + inputFileName + '\n')
    sys.stderr.write("Output file :" + outputFileName + '\n')
    sys.stderr.write("CPU         :" + cpu + '\n')
    sys.stderr.write("Dump type   :" + dumpType + '\n')
    sys.stderr.write("Offset      :" + str(offset) + '\n')

    if (cpu == 'kryo3silver' or cpu == 'kryo4silver'):
        if (dumpType == 'DCD'):
            cache = CacheRam(None, None, None, None, None)
            extractFileContents(inputFileName, offset, 4, 128, 16, cache)
            sys.stdout.write(getHeaderStr((cache.lines[0])))
            sys.stdout.write(str(cache))
        elif (dumpType == 'DCT'):
            cache = CacheRam(None, None, None, None, Kryo3SilverL1DCacheTagLine)
            extractFileContents(inputFileName, offset, 4, 128, 2, cache)
            sys.stdout.write(getHeaderStr(Kryo3SilverL1DCacheTagLine(cache.lines[0])))
            sys.stdout.write(str(cache.morph()))
        elif (dumpType == 'ICD'):
            cache = CacheRam(None, None, None, None, None)
            extractFileContents(inputFileName, offset, 4, 128, 16, cache)
            sys.stdout.write(getHeaderStr((cache.lines[0])))
            sys.stdout.write(str(cache))
        elif (dumpType == 'ICT'):
            cache = CacheRam(None, None, None, None, Kryo3SilverL1ICacheTagLine)
            extractFileContents(inputFileName, offset, 4, 128, 2, cache)
            sys.stdout.write(getHeaderStr(Kryo3SilverL1ICacheTagLine(cache.lines[0])))
            sys.stdout.write(str(cache.morph()))
        elif (dumpType == 'TLBT'):
            cache = CacheRam(None, None, None, None, Kryo3SilverL2TLBTagLine)
            extractFileContents(inputFileName, offset, 4, 256, 3, cache)
            sys.stdout.write(getHeaderStr(Kryo3SilverL2TLBTagLine(cache.lines[0])))
            sys.stdout.write(str(cache.morph()))
        elif (dumpType == 'TLBD'):
            cache = CacheRam(None, None, None, None, Kryo3SilverL2TLBDataLine)
            extractFileContents(inputFileName, offset, 4, 256, 2, cache)
            sys.stdout.write(getHeaderStr(Kryo3SilverL2TLBDataLine(cache.lines[0])))
            sys.stdout.write(str(cache.morph()))
    elif (cpu == 'kryo3gold'):
        if (dumpType == 'DCD'):
            cache = CacheRam(None, None, None, None, None)
            extractFileContents(inputFileName, offset, 16, 64, 16, cache)
            sys.stdout.write(getHeaderStr((cache.lines[0])))
            sys.stdout.write(str(cache))
        elif (dumpType == 'DCT'):
            cache = CacheRam(None, None, None, None, Kryo3GoldL1DCacheTagLine)
            extractFileContents(inputFileName, offset, 16, 64, 2, cache)
            sys.stdout.write(getHeaderStr(Kryo3GoldL1DCacheTagLine(cache.lines[0])))
            sys.stdout.write(str(cache.morph()))
        elif (dumpType == 'ICD'):
            cache = CacheRam(None, None, None, None, None)
            extractFileContents(inputFileName, offset, 4, 256, 16, cache)
            sys.stdout.write(getHeaderStr((cache.lines[0])))
            sys.stdout.write(str(cache))
        elif (dumpType == 'ICT'):
            cache = CacheRam(None, None, None, None, Kryo3GoldL1ICacheTagLine)
            extractFileContents(inputFileName, offset, 4, 256, 2, cache)
            sys.stdout.write(getHeaderStr(Kryo3GoldL1ICacheTagLine(cache.lines[0])))
            sys.stdout.write(str(cache.morph()))
        elif (dumpType == 'TLBT'):
            cache = CacheRam(None, None, None, None, Kryo3GoldL2TLBTagLine)
            extractFileContents(inputFileName, offset, 4, 256, 3, cache)
            sys.stdout.write(getHeaderStr(Kryo3GoldL2TLBTagLine(cache.lines[0])))
            sys.stdout.write(str(cache.morph()))
        elif (dumpType == 'TLBD'):
            cache = CacheRam(None, None, None, None, Kryo3GoldL2TLBDataLine)
            extractFileContents(inputFileName, offset, 4, 256, 3, cache)
            sys.stdout.write(getHeaderStr(Kryo3GoldL2TLBDataLine(cache.lines[0])))
            sys.stdout.write(str(cache.morph()))
    elif (cpu == 'kryo4gold'):
        if (dumpType == 'DCD'):
            cache = CacheRam(None, None, None, None, None)
            extractFileContents(inputFileName, offset, 4, 256, 16, cache)
            sys.stdout.write(getHeaderStr((cache.lines[0])))
            sys.stdout.write(str(cache))
        elif (dumpType == 'DCT'):
            cache = CacheRam(None, None, None, None, Kryo4GoldL1DCacheTagLine)
            extractFileContents(inputFileName, offset, 4, 256, 2, cache)
            sys.stdout.write(getHeaderStr(Kryo4GoldL1DCacheTagLine(cache.lines[0])))
            sys.stdout.write(str(cache.morph()))
        elif (dumpType == 'ICD'):
            cache = CacheRam(None, None, None, None, None)
            extractFileContents(inputFileName, offset, 4, 256, 16, cache)
            sys.stdout.write(getHeaderStr((cache.lines[0])))
            sys.stdout.write(str(cache))
        elif (dumpType == 'ICT'):
            cache = CacheRam(None, None, None, None, Kryo4GoldL1ICacheTagLine)
            extractFileContents(inputFileName, offset, 4, 256, 1, cache)
            sys.stdout.write(getHeaderStr(Kryo4GoldL1ICacheTagLine(cache.lines[0])))
            sys.stdout.write(str(cache.morph()))
        elif (dumpType == 'L2256CD'):
            cache = CacheRam(None, None, None, None, None)
            extractFileContents(inputFileName, offset, 8, 512, 16, cache)
            sys.stdout.write(getHeaderStr((cache.lines[0])))
            sys.stdout.write(str(cache))
        elif (dumpType == 'L2512CD'):
            cache = CacheRam(None, None, None, None, None)
            extractFileContents(inputFileName, offset, 8, 1024, 16, cache)
            sys.stdout.write(getHeaderStr((cache.lines[0])))
            sys.stdout.write(str(cache))
        elif (dumpType == 'L2256CT'):
            cache = CacheRam(None, None, None, None, Kryo4GoldL2DCache256KBTagLine)
            extractFileContents(inputFileName, offset, 8, 512, 2, cache)
            sys.stdout.write(getHeaderStr(Kryo4GoldL2DCache256KBTagLine(cache.lines[0])))
            sys.stdout.write(str(cache.morph()))
        elif (dumpType == 'L2512CT'):
            cache = CacheRam(None, None, None, None, Kryo4GoldL2DCache512KBTagLine)
            extractFileContents(inputFileName, offset, 8, 1024, 2, cache)
            sys.stdout.write(getHeaderStr(Kryo4GoldL2DCache512KBTagLine(cache.lines[0])))
            sys.stdout.write(str(cache.morph()))
        elif (dumpType == 'L1ITLB'):
            cache = CacheRam(None, None, None, None, Kryo4GoldL1ITLBLine)
            extractFileContents(inputFileName, offset, 1, 48, 4, cache)
            sys.stdout.write(getHeaderStr(Kryo4GoldL1ITLBLine(cache.lines[0])))
            sys.stdout.write(str(cache.morph()))
        elif (dumpType == 'L1DTLB'):
            cache = CacheRam(None, None, None, None, Kryo4GoldL1DTLBLine)
            extractFileContents(inputFileName, offset, 1, 48, 6, cache)
            sys.stdout.write(getHeaderStr(Kryo4GoldL1DTLBLine(cache.lines[0])))
            sys.stdout.write(str(cache.morph()))
        elif (dumpType == 'TLBD'):
            cache = CacheRam(None, None, None, None, Kryo4GoldL2TLBLine)
            extractFileContents(inputFileName, offset, 5, 256, 6, cache)
            sys.stdout.write(getHeaderStr(Kryo4GoldL2TLBLine(cache.lines[0])))
            sys.stdout.write(str(cache.morph()))


    else:
        sys.stderr.write("Incorrect CPU\n")
        sys.exit(2)

if __name__ == "__main__":
    main(sys.argv[1:])
