# Common methods used by multiple files.
# File last updated 8-1-25

import sys
import traceback

class Common:
    @staticmethod
    def error(msg, exc=False):
        if exc:
            print(msg + " More detailed info. below.")
            print(traceback.format_exc())
        else:
            print(msg)
        sys.exit(1)

    @staticmethod
    def prettydict(dict):
        out = str(dict)
        idl = 0
        for letter in out:
            if letter in ["{", "["]:
                idl += 1
                print(letter + "\n" + ("\t" * idl), end="")
            elif letter in ["}", "]"]:
                idl -= 1
                print(letter + "\n" + ("\t" * idl), end="")
            elif letter == ",":
                print(letter + "\n" + ("\t" * idl), end="")
            else:
                print(letter, end="")
        print("\n\n")