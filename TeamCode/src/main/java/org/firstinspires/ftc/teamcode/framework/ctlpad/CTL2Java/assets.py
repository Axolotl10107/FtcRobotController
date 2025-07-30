# Some data for CTL2Java
# File last updated 7-28-25

# Update this first item to match your TeamCode layout
# You may need to add things to support additional Method libraries. See their READMEs for what they need.
# Don't update anything else in here unless you're familiar with how CTL2Java works!
# <~tc> -> "org.firstinspires.ftc.teamcode"
importLocations = {"primitives" : "<~tc>.framework.ctlpad.primitives",
                   "subsystems" : "<~tc>.framework.subsystems",
                   "DTS" : "<~tc>.framework.units.DTS",}


# The following expander tags are available to the start template:
# <~tc> - location of TeamCode package
# <~package> - destination package of the output file
# <~className> - name of the output file
# <~seasonInterface> - the Scheme interface used for the selected season
# <~seasonState> - the State class used for the selected season
# anything specified in importLocations above (e.g. <~primitives>)

# Additional tags are available to the end template:
# <~getStateInteriorLines> - created by the code generator as it goes

startTemplateLines = ["package <package>;",
                      "",
                      "import com.qualcomm.robotcore.hardware.Gamepad;",
                      "import <~primitives>;",
                      "",
                      "/** This control scheme was generated using CTL2Java. */",
                      "public class <~className> implements <~seasonInterface> {",
                      "\t\n\t"]

endTemplateLines = ["\n\t",
                    "\t@Override",
                    "\tpublic <~seasonState> getState() {",
                    "\t\t<~getStateInteriorLines>\n\t\t",
                    "\t\treturn state;",
                    "\t}",
                    "}"]


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

gamepadRequiredFields = ["Version",
                     "Gamepad",
                     "Season",
                     "Buttons",
                     "Axes"]

methodLibRequiredFields = ["Version",
                           "Season",
                           "Bases",
                           "Extensions",
                           "Methods"]

actionLibRequiredFields = ["Version",
                           "Season",
                           "Actions"]

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
                 "Bases" : list,
                 "Extensions" : list,
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

validButtonTypes = ["Momentary",
                    "Toggle",
                    "Trigger",
                    "Axis"]

axisMappingTypes = {"Type" : str,
                    "Scaling" : float,
                    "Action" : dict}

validAxisTypes = ["Linear",
                  "Exponential",
                  "Merged",
                  "Button"]

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