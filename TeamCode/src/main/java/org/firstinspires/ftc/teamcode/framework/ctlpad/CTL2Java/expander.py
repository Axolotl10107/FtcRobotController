# Expand expander tags
# File last updated 8-1-25

import assets
from common import Common

class Expander:
    def __init__(self, expandedLibDict, sortedMappings):
        self.libs = expandedLibDict
        self.actMaps = sortedMappings


    def getAndExpandMethod(self, methodName):
        out = ""
        method = self.libs["Methods"][methodName]
        for line in method:
            out += self.expandMethodLine(line)

    # Meant to be called from other files
    def expandMethodLine(self, line):
        # Do we have to run at all?
        if line.count("<~") == 0:
            return line

        if not line.count("<~") == line.count(">"):
            Common.error("Unmatched expander tag brackets somewhere in this line from somewhere:\n" + line)

        out = ""
        while line.count("<~") > 0:
            tagStart = line.index("<~")
            tagEnd = line.index(">")

            out += line[ : tagStart ]
            line = line[ tagStart : ]
            tag = line[ tagStart+1 : tagEnd ]

            if tag.count(":") > 0:
                if tag.count(":") > 1:
                    Common.error("Can't have more than 1 colon per tag! Found in this line from somewhere:\n" + line)
                out += self.processMethodColonTag(tag)
            else:
                out += self.processMethodNormalTag(tag)

        out += line # Append whatever's left after the last tag
        return out


    def processMethodColonTag(self, tag):
        colonIdx = tag.index(":")
        tagType = tag[ : colonIdx ]
        tagContent = tag[ colonIdx+1 : ]

        if tagType == "param":
            Common.error("There seems to be a 'param:' expander tag outside of an Action.")

        elif tagType == "all":
            # Action exists
            if not (tagContent in self.libs["BaseActions"].keys() or tagContent in self.libs["ExtensionActions"].keys()):
                Common.error("'all:' tag refers to Action '" + tagContent + "' that doesn't seem to exist?")

            # Gather all Mappings mapped to this Action
            mappingsToAdd = []
            for mapping in self.actMaps:
                if mapping["Action"]["Name"] == tagContent:
                    mappingsToAdd.append(mapping)

            # Expand and gather them all
            out = ""
            for mapping in mappingsToAdd:
                out += self.getAndExpandActionForMapping(mapping)

            # Return expanded line (which will now be many lines)
            return out

        else:
           Common.error("Invalid tag type in this tag from somewhere: '" + tag + "'")


    def processMethodNormalTag(self, tag):
        Common.error("Only 'all:' tags are currently supported in Methods.")



    # Meant to be called from other files
    # This takes actions from *Mappings*, not from Libraries!
    def getAndExpandActionForMapping(self, mapping, mappingName):
        actionName = mapping["Action"]["Name"]
        out = ""

        if actionName in self.libs["BaseActions"]:
            for line in self.libs["BaseActions"][actionName]["Code"]:
                out += self.expandBaseActionLine(line, mapping, mappingName)
        elif actionName in self.libs["ExtensionActions"]:
            for line in self.libs["ExtensionActions"][actionName]["Code"]:
                out += self.expandExtensionActionLine(line, mapping, mappingName)
        else:
            Common.error("Tried to get code for Action '" + actionName + "' that doesn't seem to exist?")

        return out


    def expandBaseActionLine(self, line, mapping, mappingName):
        # Do we have to run at all?
        if line.count("<~") == 0:
            return line

        if not line.count("<~") == line.count(">"):
            Common.error("Unmatched expander tag brackets somewhere in this line from somewhere:\n" + line)

        out = ""
        while line.count("<~") > 0:
            tagStart = line.index("<~")
            tagEnd = line.index(">")

            out += line[ : tagStart ]
            line = line[ tagStart : ]
            tag = line[ tagStart+1 : tagEnd ]

            if tag.count(":") > 0:
                if tag.count(":") > 1:
                    Common.error("Can't have more than 1 colon per tag! Found in this line from somewhere:\n" + line)
                out += self.processActionColonTag(tag, mapping)
            else:
                out += self.processActionNormalTag(tag, mapping)

        out += line # Append whatever's left after the last tag
        return out

    def expandExtensionActionLine(self, line, mapping, mappingName):
        Common.error("Extension Actions not supported quite yet!")


    def processActionColonTag(self, tag, mapping, mappingName):
        colonIdx = tag.index(":")
        tagType = tag[ : colonIdx ]
        tagContent = tag[ colonIdx+1 : ]

        if tagType == "all":
            Common.error("There seems to be an 'all:' tag outside of a Method.")

        elif tagType == "param":
            if not tagContent in mapping["Action"]["Parameters"].keys():
                Common.error("Found expander tag for Parameter '" + tagContent + "' that doesn't exist in Action '" + mapping["Action"]["Name"] + "'.")
            return str( mapping["Action"]["Parameters"][tagContent]["Value"] )


    def processActionNormalTag(self, tag, mapping, mappingName):
        if tag == "myname":
            return mappingName
        else:
            Common.error("Only 'myname' normal tags are currently supported in Actions.")
