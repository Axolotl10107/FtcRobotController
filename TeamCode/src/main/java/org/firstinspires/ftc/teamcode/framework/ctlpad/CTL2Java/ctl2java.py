# CTL2Java CLI
# File last updated 7-24-25

import traceback
import sys
from argparse import ArgumentParser

from ctlconv import CTLConv
import assets

version = "1.0-0"

# Define command-line arguments
argparser = ArgumentParser()
argparser.add_argument("-v", "--ver", help="Print version then exit", action="store_true")
argparser.add_argument("-if1", "--infile1", help="Specify input file for gamepad1, otherwise use stdin", action="store")
argparser.add_argument("-if2", "--infile2", help="Specify input file for gamepad2", action="store")
argparser.add_argument("-of", "--outfile", help="Specify output file, otherwise use stdout", action="store")
argparser.add_argument("-f", "--verify", help="Only verify infile, do not generate control scheme", action="store_true")
args = argparser.parse_args()

if args.ver:
    print(version)
    sys.exit(0)

def error(self, msg, exc=False):
    if exc:
        print(msg + " More detailed info. below.")
        print(traceback.format_exc())
    else:
        print(msg)
    sys.exit(1)

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

converter1 = CTLConv(infile1, assets.gamepadRequiredFields)
print("Converting file for gamepad1:")
outdict1 = converter1.getVerifiedDict()
print("\n\n")
if infile2:
    converter2 = CTLConv(infile2, assets.gamepadRequiredFields)
    print("Converting file for gamepad2:")
    outdict2 = converter2.getVerifiedDict()
    print("\n\n")

print("gamepad1:")
out = str(outdict1)
idl = 0
for letter in out:
    if letter == "{":
       idl += 1
       print(letter + "\n" + ("\t" * idl), end="")
    elif letter == "}":
        idl -= 1
        print(letter + "\n" + ("\t" * idl), end="")
    elif letter == ",":
        print(letter + "\n" + ("\t" * idl), end="")
    else:
        print(letter, end="")
print("\n\n")
print("gamepad2:")
out = str(outdict2)
idl = 0
for letter in out:
    if letter == "{":
        idl += 1
        print(letter + "\n" + ("\t" * idl), end="")
    elif letter == "}":
        idl -= 1
        print(letter + "\n" + ("\t" * idl), end="")
    elif letter == ",":
        print(letter + "\n" + ("\t" * idl), end="")
    else:
        print(letter, end="")
print("\n\n")