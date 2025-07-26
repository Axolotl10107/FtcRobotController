# CTL File Parser
# File last updated 7-24-25

import assets

# Our own Exception that we can raise
class CTLParseError(Exception):
    def __init__(self, message="Something went wrong while parsing the CTL file"):
        self.message = message
        super().__init__(message)

# Return False instead of raising an exception if list is empty
def getLastOf(list):
    try:
        return list[-1]
    except IndexError:
        return False

# If it should probably be an int, make it an int
def tryInt(item):
    try:
        return int(item)
    except:
        return item

# Turns a line into something a little more friendly for us
def parseOneLine(line, accept=False):
    # Strip indentation
    line = line.lstrip()

    # Modifiers
    if accept == "Modifiers":
        if line in assets.validButtons:
            return {"Type" : "Modifier",
                    "Button" : line}

    # Line is too short (*after* the Modifiers check to let one-letter Modifiers through)
    if len(line) < 2:
        raise CTLParseError("Line (outside of Modifiers) is too short.")

    # Comment
    if len(line) == 0 or line[0:1] == "//":
        return {"Type" : "Comment"}

    # Category Open
    if line[0] == "[":
        if accept in ["Modifiers", "Setters"]:
            raise CTLParseError("Sub-categories not allowed within " + accept)
        return {"Type" : "Category Open",
                "Name" : line[1:]}

    # Category Close
    if line[0] == "]":
        if len(line) > 1:
            raise CTLParseError("Category Close is not the only item on one of these lines! One item per line, please.")
        return {"Type" : "Category Close"}

    # If accepting Setters or Methods, we never treat these lines as Pairs.
    if accept == "Setters":
        return {"Type" : "Setter",
                "Name" : line}

    if accept == "Method Lines":
        return {"Type" : "Method Line",
                "Line" : line}

    # Pair
    colonPos = line.find(":")
    if colonPos > -1:
        # Checks
        if colonPos == 0:
            raise CTLParseError("Pair has no key!")
        elif colonPos == len(line)-1:
            raise CTLParseError("Pair has no value!")
        else:
            # tryInt(), because these often store coordinates and such which should be ints
            return {"Type" : "Pair",
                    "Pair" : {tryInt(line[0:colonPos]) : tryInt(line[colonPos+1:])} }
    else:
        raise CTLParseError("Couldn't make anything of one of the lines.")

def ctlToDict(filename):
    # Open file, set stuff up
    f = open(filename, "r")
    lines = f.readlines()
    catNameStack = []
    catDictStack = [{}] # starting dict. at top will become the return dict.

    # Verify file version
    if not lines[0] == "<#>CTL1<#>":
        raise CTLParseError("Bad file version line, or file is too new for this parser")

    # Parse the rest of the file
    for line in lines[1:]:
        parsed = parseOneLine(line, getLastOf(catNameStack))

        if parsed["Type"] == "Category Open":
            catNameStack.append(parsed["Name"])
            if parsed["Name"] in ["Modifiers", "Setters"]:
                catDictStack.append([])
            else:
                catDictStack.append({})

        elif parsed["Type"] == "Category Close":
            if len(catDictStack) < 2:
                raise CTLParseError("Category Close with no matching Category Open")
            catDictStack[-2].update({catNameStack[-1] : catDictStack[-1]})
            catDictStack.pop(-1)
            catNameStack.pop(-1)

        elif parsed["Type"] == "Pair":
            if len(catDictStack) < 2:
                raise CTLParseError("Pair outside of a category")
            catDictStack[1].update(parsed["Pair"])

        elif parsed["Type"] in ["Modifier", "Setter"]:
            catDictStack[-1].append(parsed["Button"])

        elif parsed["Type"] == "Method Line":
            catDictStack[-1][catNameStack[-1]] += parsed["Line"]

    if len(catDictStack) > 1:
        raise CTLParseError("Incomplete parse - probably an unclosed category somewhere")

    return catDictStack[-1]
