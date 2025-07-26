# Some data for CTL2Java
# File last updated 7-26-25

validButtons = ["A",
                "B",
                "X",
                "Y",
                "Start",
                "Select",
                "R1",
                "L1",
                "DPadUp",
                "DPadDown",
                "DPadLeft",
                "DPadRight",
                "LSC",
                "RSC",
                "PS",]

# Somewhat confusing naming, sorry. Not which buttons can be used as modifiers - all buttons can - this is
# for actions mapped to modifiers. An action can be mapped to the "Default" modifier as well as when any
# modifier button is active.
validModifiers = validButtons + ["Default"]

validAxes = ["LSX",
             "LSY",
             "RSX",
             "RSY",
             "R2",
             "L2",]

validCTLFields = ["Version",
                  "Gamepad",
                  "Season",
                  "Positions",
                  "Modifiers",
                  "Buttons",
                  "Axes",
                  "Methods",
                  "Actions",
                  "Setters",]

requiredCTLFields = ["Version",
                     "Gamepad",
                     "Season",
                     "Buttons",
                     "Axes"]

usableCTLFields = ["Version",
                   "Gamepad",
                   "Season",
                   "Modifiers",
                   "Buttons",
                   "Axes",
                   "Actions",
                   "Methods",
                   "Setters"]

ctlFieldTypes = {"Version" : int,
                 "Gamepad" : int,
                 "Season" : str,
                 "Positions" : dict,
                 "Modifiers" : list,
                 "Buttons" : dict,
                 "Axes" : dict,
                 "Methods" : dict,
                 "Actions" : dict,
                 "Setters" : list}

validModifierKeys = ["Type", "Action"]

buttonMappingTypes = {"Type" : str,
                      "Action" : dict}

actionTypes = {"Name" : str,
               "Parameters" : dict}

parametersTypes = {"Type" : str,
                   "Range" : str,
                   "Value" : int}

validDataTypes = ["byte",
                  "short",
                  "int",
                  "long",
                  "float",
                  "double",
                  "boolean",
                  "char",
                  "String"]