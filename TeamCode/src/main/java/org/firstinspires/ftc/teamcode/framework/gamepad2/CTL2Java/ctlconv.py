# File updated 7-26-25

# Convert CTL file to dictionary, verify integrity and fix known bugs from graphical editors
# A rather lenient checker that lets graphical editors hide their own data all over the place if they want to.
# This will just remove anything that CTL2Java doesn't need.
# TODO: Not quite; there's a few spots that fail when extra keys are present instead of just skipping them. I'd like to change these.

# Big picture outline of what this does:
# - Use json.load() to convert an exported Snap! list to a Python dictionary
# - Thoroughly check the dictionary to make sure it is formatted the way we expect.
#    - Along the way, we'll turn things that should be ints rather than strings into ints.
#    - Along the way, we'll fix any known export bugs from graphical editors

# Remember while reading through this code:
#
# >>> dictionary = {"a": {"b": "c"}, "1": "2"}
# >>> a = dictionary["a"]
# >>> a["b"] = "hello"
# >>> dictionary["a"]["b"]
# "hello"
#
# Try it, it really works!
# If you don't understand references, you might be confused!

import json
import traceback
import sys

import assets

class CTLConv:
    def __init__(self, infile):
        self.version = "1.0-0"
        self.infile = infile
        self.convertedDict = {} # just json.load(), for getConverted()
        try:
            self.convertedDict = json.load(infile)
        except:
            self.error("Initial file conversion failed!", True)
        self.outdict = self.convertedDict # outdict will be updated by getVerified()
        self.warnings = []

    # --- Setup ---
    # Define utility methods
    def error(self, msg, exc=False):
        if exc:
            print(msg + " More detailed info. below.")
            print(traceback.format_exc())
        else:
            print(msg)
        sys.exit(1)


    def getConvertedDict(self):
        return self.outdict

    # --- Verify / Fix dict ---
    def getVerifiedDict(self):
        # Setup
        self.warnings = []

        print("Pass 1: Verify Fields Overall")

        fields = list(self.outdict.keys())
        toPop = []
        empty = []
        print("Fields:")
        for idx, field in enumerate(fields):

            # Field is valid
            if not field in assets.validCTLFields:
                toPop.append(idx)
                print("[unrecognized, ignoring] ", end="")

            # Field is not empty
            elif self.outdict[field] in [None, "", [], {}]:
                if field in assets.requiredCTLFields:
                    empty.append(field)
                    print("[empty, required, will skip later] ", end="")
                else:
                    toPop.append(idx)
                    print("[empty, skipping] ", end="")

            elif field not in assets.usableCTLFields:
                toPop.append(idx)
                print("[not used by CTL2Java, skipping] ", end="")

            else:
                # Value is of correct type
                if not isinstance(self.outdict[field], assets.ctlFieldTypes[field]):
                    self.error("Value for field '" + str(field) + "' is of wrong type.")

            print(field)
        toPop.reverse() # pop from end to start, otherwise the first pop will make all later indices invalid
        for idx in toPop:
            del self.outdict[fields[idx]]
            fields.pop(idx)

        # After pruning, we still have all required fields
        for item in assets.requiredCTLFields:
            if not item in fields:
                self.error("Missing required field '" + item + "'.")

        # Further inspect metadata fields
        if self.outdict["Version"] < 1:
            print("That's an interesting version number. I'm not going to try working with this one. Exiting now.")
            sys.exit(1)
        if self.outdict["Version"] > 1:
            print("What's up, future people? Do you know what's up? I don't, but I do know it's not my version number (" + self.version + ")! I don't know what to do with a file this new! You want me to try anyway?")
            user = input("[y/n]")
            if user.lower() in ["y", "yes"]:
                print("Alright! Let's try it...")
            else:
                print("Yeah, let's look for a newer version of me instead. Exiting now.")
                sys.exit(0)

        if self.outdict["Gamepad"] < 1 or self.outdict["Gamepad"] > 2:
            print("'Gamepad' should be either 1 or 2.")
            sys.exit(1)

        print("Completed Pass 1\n")


        # Pass 2: Verify Modifiers
        print("Pass 2: Verify Modifiers")
        modifiers = []
        if "Modifiers" in fields:
            for modifier in self.outdict["Modifiers"]: # Field type already verified in Pass 1
                if modifier in assets.validButtons:
                    print(modifier)
                    modifiers.append(modifier)
                else:
                    self.error("Invalid modifier '" + str(modifier) + "'.")
        else:
            print("No 'Modifiers' field, so skipping Pass 2\n")


        # Pass 3: Verify Buttons
        print("Pass 3: Verify Buttons")
        if "Buttons" in fields:
            buttons = self.outdict["Buttons"]
            buttonsToDel = []
            for idx, buttonName in enumerate(buttons.keys()):

                # Button exists
                if buttonName not in assets.validButtons:
                    buttonsToDel.append(buttonName)
                    print("[unrecognized, ignoring] ", end="")

                # Button is not already a modifier
                elif buttonName in modifiers: # 'modifiers' was set in Pass 2
                    self.error("Modifier button '" + buttonName + "' should not have Action mappings!")

                else:
                    button = buttons[buttonName]

                    # Button is a dict
                    if not isinstance(button, dict):
                        self.error("Button '" + buttonName + "' is not valid.")

                    # Check modifier mappings
                    buttonModifiers = list(button.keys())
                    modifiersToDel = []
                    for modifier in buttonModifiers:
                        # Modifier is valid
                        if modifier not in (modifiers + ["Default"]):
                            modifiersToDel.append(modifier)
                            print("\t[unusable; that button isn't a modifier] ", end="")
                            self.warnings.append("Button mapping '" + buttonName + "'/'" + str(modifier) + "' is unusable; that button is not mapped as a modifier.")
                        else:
                            # Action mappings are valid
                            mapping = button[modifier]

                            # Mapping is a dict
                            if not isinstance(mapping, dict):
                                self.error("Action mapping for button '" + buttonName + "' modifier '" + modifier + "' is not a dict!")

                            # Mapping contains the correct keys
                            if not mapping.keys() == assets.buttonMappingTypes.keys():
                                self.error("Malformed mapping for button '" + buttonName + "' modifier '" + modifier + "'.")

                            # Values for keys are of correct types
                            for key in assets.buttonMappingTypes.keys():
                                if not isinstance(mapping[key], assets.buttonMappingTypes[key]):
                                    if mapping[key] == "":
                                        # An empty field here should be None, but Snap! exports it as ""
                                        mapping[key] = None
                                    elif mapping[key] != None: # Empty fields are OK, just not malformed ones
                                        # Check for CTLedit 1.0 known bug
                                        if key == "Action" and isinstance(mapping["Action"], list):
                                            action = mapping["Action"]
                                            name = action[0]
                                            action[1].update({"Name" : name})
                                            mapping["Action"] = action[1]
                                        else:
                                            self.error("Value for key '" + str(key) + "' in modifier mapping '" + modifier + "' in button '" + buttonName + "' is of the wrong type.")


                            # Verify Action mapping
                            action = mapping["Action"]

                            # Action mapping contains correct keys
                            if not action.keys() == assets.actionTypes.keys():
                                # CTLedit 1.0 known bug: saves its Action library metadata in exported mapping
                                if "Type" in action.keys():
                                    del action["Type"]
                                if "Description" in action.keys():
                                    del action["Description"]
                                elif "Parameters" not in action.keys():
                                    # If Action has no Parameters, field can be set to None or not set at all. Either is fine.
                                    action.update({"Parameters" : None})
                                else:
                                    self.error("Malformed action mapping for button '" + buttonName + "' modifier '" + modifier + "'.")

                            # Values are of correct types
                            for key in action.keys():
                                if not isinstance(action[key], assets.actionTypes[key]):
                                    if not (key == "Parameters" and action[key] == None): # Empty Parameters are OK
                                        self.error("Value for key '" + str(key) + "' of action mapping for button '" + buttonName + "' modifier '" + modifier + "' is of the wrong type.")

                            # Verify Parameters
                            parameters = action["Parameters"]
                            if parameters:
                                for paramName in parameters.keys():
                                    param = parameters[paramName]

                                    # Parameter is a dictionary
                                    if not isinstance(param, dict):
                                        self.error("Parameter '" + paramName + "' of action mapped to button '" + buttonName + "' modifier '" + modifier + "' is not a dictionary.")

                                    # Parameter contains the correct keys
                                    if not param.keys() == assets.parametersTypes.keys():
                                        if (list(param.keys()) + ["Range"]) == list(assets.parametersTypes.keys()):
                                            param.update({"Range" : None}) # Empty Range is OK (means unlimited range)
                                        else:
                                            self.error("Malformed parameter '" + paramName + "' of action mapped to button '" + buttonName + "' modifier '" + modifier + "'.")

                                    # Type is a valid Java data type
                                    type = param["Type"]
                                    if type not in assets.validDataTypes:
                                        self.error("Invalid data type for parameter '" + paramName + "' of action mapped to button '" + buttonName + "' modifier '" + modifier + "'.")

                                    # Range is properly formatted
                                    range = param["Range"]
                                    fail = False
                                    if range:
                                        if range.count("/") == 1:
                                            slashidx = range.index("/")
                                            if range[:slashidx].isnumeric() and range[slashidx+1:].isnumeric():
                                                fail = False
                                            else:
                                                fail = True
                                        else:
                                            fail = True
                                    if fail:
                                        print("Malformed range for parameter '" + paramName + "of action mapped to button '" + buttonName + "' modifier '" + modifier + "'.")

                                    # Value is of correct type
                                    value = param["Value"]
                                    failMsg = "Failed to convert value for parameter '" + paramName + "of action mapped to button '" + buttonName + "' modifier '" + modifier + "' to correct type."
                                    if type in ["byte", "short", "int", "long"]:
                                        if not isinstance(value, int):
                                            try:
                                                param["Value"] = int(value)
                                            except:
                                                self.error(failMsg)
                                    elif type in ["float", "double"]:
                                        if not isinstance(value, float):
                                            try:
                                                param["Value"] = float(value)
                                            except:
                                                self.error(failMsg)
                                    elif type == "boolean":
                                        if not isinstance(value, bool):
                                            try:
                                                param["Value"] = bool(value)
                                            except:
                                                self.error(failMsg)
                                    elif type in ["char", "String"]:
                                        if not isinstance(value, str):
                                            try:
                                                param["Value"] = str(value)
                                            except:
                                                self.error(failMsg)

                        print("\t" + modifier)
                    for key in modifiersToDel:
                        del button[key]
                print(buttonName)
            for key in buttonsToDel:
                del buttons[key]
        else:
            print("No 'Buttons' field, so skipping Pass 3\n")

        print("File fully converted and verified.")
        return self.outdict