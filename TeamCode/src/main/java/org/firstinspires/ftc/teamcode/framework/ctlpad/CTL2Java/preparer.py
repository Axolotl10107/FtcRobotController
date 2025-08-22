# Prepare all the imported stuff (Action & Method libraries & Mappings) in a way
# that will be convenient later. Sort stuff and do some higher-level checks than
# ctlconv did.
# File last updated 8-1-25

import assets
from common import Common

class Preparer:
    def __init__(self, mappings1, mappings2, libDict):
        self.mappings1 = mappings1
        self.mappings2 = mappings2
        self.libDict = libDict

        # Some initial verification
        if not self.mappings1["Season"] == self.mappings2["Season"]:
            Common.error("Mismatched Seasons between gamepads.")
        if not self.settersLib["Season"] == self.mappings1["Season"]:
            Common.error("Gamepads' Season does not match Setters' Season.")

        self.season = self.settersLib["Season"]

        # Libraries are valid for this Season
        for category in self.libDict.keys():
            for libName in self.libDict[category]:
                if not self.libDict[category][libName]["Season"] == self.season:
                    Common.error("Library '" + libName + "' is not valid for this Season (mismatched with Setters' Season).")

        # Gamepads were given as expected
        if not self.mappings1["Gamepad"] == 1:
            Common.error("Got gamepad2 where I expected gamepad1")
        if not self.mappings2["Gamepad"] == 2:
            Common.error("Got gamepad1 where I expected gamepad2")


        # Expand libDict and combine all Libraries in each category together
        self.baseLibs = libDict["BaseActions"]
        self.base = {}
        for libName in self.baseLibs.keys():
            for actionName in self.baseLibs[libName].keys():
                self.base.update( {actionName : self.baseLibs[libName]["Actions"][actionName]} )

        self.extensionLibs = libDict["ExtensionActions"]
        self.extension = {}
        for libName in self.extensionLibs.keys():
            for actionName in self.extensionLibs[libName].keys():
                self.extension.update( {actionName : self.extensionLibs[libName]["Actions"][actionName]} )

        self.methodLibs = libDict["Methods"]
        self.methods = {}
        for libName in self.methodLibs.keys():
            for methodName in self.methodLibs[libName].keys():
                self.methods.update( {methodName : self.methodLibs[libName]["Methods"][methodName]} )

        self.settersLib = libDict["Setters"]
        self.setters = self.settersLib["Setters"]


        # Method names are valid
        for methodName in self.methods.keys():
            if methodName.count("<~") > 0:
                if methodName.count("<~") > 1:
                    Common.error("Too many tags in name of Method '" + methodName + "'.")
                elif methodName.count(">") != 1:
                    Common.error("Unclosed tag or too many '>'s in name of Method '" + methodName + "'.")

                tagStart = methodName.index("<~")
                tagEnd = methodName.index(">")
                if tagEnd < tagStart:
                    Common.error("Backwards tag brackets in name of Method '" + methodName + "'.")
                if tagStart != 0:
                    Common.error("The drive type tag in a Method name should be at the start. It is not for Method '" + methodName + "'.")

                tagContents = methodName[tagStart+1 : tagEnd]
                if tagContents not in assets.validDriveTypes:
                    Common.error("Invalid drive type for Method '" + methodName + "'. Must be in this list: " + str(assets.validDriveTypes))


        # All Setters are implemented by Methods
        happySetters = []
        for methodName in self.methods.keys():
            # Strip drive type tag, if it's there
            tagEnd = methodName.find(">")
            if tagEnd > 0:
                methodName = methodName[tagEnd+1:]

            if methodName not in happySetters:
                happySetters.append(methodName)

        unhappySetters = []
        for setter in self.setters:
            if setter not in happySetters:
                unhappySetters.append(setter)

        if len(unhappySetters) > 0:
            Common.error("The following Setters are not implemented by Methods: " + str(unhappySetters))


    def sortMappingsByAction(self):
        primitives = self.mappings1["Buttons"]
        primitives.update(self.mappings1["Axes"])
        primitives.update(self.mappings2["Buttons"])
        primitives.update(self.mappings2["Axes"])

        actionDict = {}
        for primitiveName in primitives.keys():
            primitive = primitives[primitiveName]
            action = primitive["Action"]
            if action["Name"] not in actionDict.keys():
                actionDict.update( { action["Name"] : {primitiveName : primitive } } )
            actionDict[action["Name"]].update( {primitiveName : primitive} )

        return actionDict

    def expandLibDict(self):
        # We already did it in __init__() - we're just returning the results
        return {"BaseActions": self.base,
                "ExtensionActions": self.extension,
                "Methods": self.methods,
                "Setters": self.setters}
