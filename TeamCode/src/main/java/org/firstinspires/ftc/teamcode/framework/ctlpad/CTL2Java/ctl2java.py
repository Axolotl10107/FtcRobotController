# CTL2Java CLI
# File last updated 8-1-25

import traceback
import os
import sys
from argparse import ArgumentParser

from ctlconv import CTLConv
import assets

version = "1.0-0"

# Define command-line arguments
argparser = ArgumentParser()
argparser.add_argument("-ver", "--version", help="Print version then exit", action="store_true")
argparser.add_argument("-if1", "--infile1", help="Input file for gamepad1, otherwise use stdin", action="store")
argparser.add_argument("-if2", "--infile2", help="Input file for gamepad2", action="store")
argparser.add_argument("-of", "--outfile", help="Output file, otherwise use stdout", action="store")
argparser.add_argument("-op", "--outpackage", help="Package that the output file resides in, otherwise assume TeamCode", action="store")
argparser.add_argument("-dt", "--drivetype", help="'Indy' (traditional) or 'Fieldy' (field-oriented). Assume Indy if this isn't set.", action="store")
argparser.add_argument("-f", "--verify", help="Only verify infile, do not generate control scheme", action="store_true")
argparser.add_argument("-d", "--debug", help="Enable some extra long debug output.", action="store_true")
args = argparser.parse_args()

# Handle --version immediately
if args.version:
    print(version)
    sys.exit(0)

# Utility methods
def error(self, msg, exc=False):
    if exc:
        print(msg + " More detailed info. below.")
        print(traceback.format_exc())
    else:
        print(msg)
    sys.exit(1)

# Process arguments
if args.outpackage:
    outpackage = args.outpackage
else:
    outpackage = "org.firstinspires.ftc.teamcode"

if args.drivetype:
    low = args.drivetype.lower()
    if low in ["indy", "fieldy"]:
        drivetype = low
    else:
        error("Invalid drivetype specified. Must be 'Indy' or 'Fieldy'.")
else:
    drivetype = "indy"

if args.infile1:
    try:
        infile1 = open(args.infile1, encoding="utf-8-sig")
    except:
        error("Could not open input file 1.", True)
else:
    infile1 = sys.stdin

if args.infile2:
    try:
        infile2 = open(args.infile2, encoding="utf-8-sig")
    except:
        error("Could not open input file 2.", True)
else:
    infile2 = None

# Convert files
converter1 = CTLConv(infile1, assets.gamepadRequiredFields)
print("Converting file for gamepad1:")
outdict1 = converter1.getVerifiedDict()
print("\n\n")
if infile2:
    converter2 = CTLConv(infile2, assets.gamepadRequiredFields)
    print("Converting file for gamepad2:")
    outdict2 = converter2.getVerifiedDict()
    print("\n\n")

# Create dict with needed categories but empty lists for values
libNameDict = assets.libDirs
for key in libNameDict.keys():
    libNameDict[key] = []

# Populate dict with filenames
for libType in assets.libDirs.keys():
    for filename in os.listdir(assets.libDirs[libType]):
        if filename[-5:] == ".json":
            libNameDict[libType].append(filename)

# Create dict to sort converted libs into
libDict = assets.libDirs
for key in libDict.keys():
    libDict[key] = {}

# Convert all libs
for libType in libNameDict.keys():
    for libName in libType:
        try:
            libFile = open(assets.libDirs[libName] + "/" + libName)
        except:
            error("Could not open Library file '" + libName + "'.")
        conv = CTLConv(libFile)
        newName = libName[:-5] # Remove file extension
        newLib = conv.getVerifiedDict()
        libDict[libType].update({newName : newLib})

# Debug output, if enabled
if args.debug:
    print("gamepad1:")
    assets.prettydict(outdict1)
    print("gamepad2:")
    assets.prettydict(outdict2)

    for libType in libDict.keys():
        print("Library type: " + libType)
        for libName in libType.keys():
            print(libName)
            assets.prettydict(libType[libName])
