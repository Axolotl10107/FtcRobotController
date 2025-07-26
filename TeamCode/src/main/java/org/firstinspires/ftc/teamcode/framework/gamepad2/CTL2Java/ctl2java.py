# CTL2Java CLI
# File last updated 7-24-25

import traceback
import sys
from argparse import ArgumentParser

from ctlconv import CTLConv

version = "1.0-0"

# Define command-line arguments
argparser = ArgumentParser()
argparser.add_argument("-v", "--ver", help="Print version then exit", action="store_true")
argparser.add_argument("-if", "--infile", help="Specify input file, otherwise use stdin", action="store")
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

if args.infile:
    try:
        infile = open(args.infile, "r")
    except:
        error("Could not open input file.", True)
else:
    infile = sys.stdin

converter = CTLConv(infile)
outdict = converter.getVerifiedDict()
print("Output:")
out = str(outdict)
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