# The code generator itself! Thanks to all the prepwork, this just strings
# already prepared sections together.
# File last updated 8-23-25

import assets
from common import Common


class Generator:
    def __init__( self, sortedMappings, expander, constructorLines, classLines, setters, importLines, driveType, debug ):
        self.sortedMappings = sortedMappings
        self.expander = expander
        self.constructorLines = constructorLines
        self.classLines = classLines
        self.setters = setters
        self.importLines = importLines
        self.driveType = driveType
        self.debug = debug

        if self.debug:
            print( "\nGenerator!" )
            print( "sortedMappings:" )
            print( self.sortedMappings )
            print( "\nexpander:" )
            print( self.expander )
            print( "\nconstructorLines:" )
            print( self.constructorLines )
            print( "\nclassLines:" )
            print( self.classLines )
            print( "\nsetters:" )
            print( self.setters )
            print( "\nimportLines:" )
            print( self.importLines )
            print( "\ndriveType:" )
            print( self.driveType )


    def stringify( self, lines ):
        out = ""
        for line in lines:
            out += line + "\n"
        return out


    def getPrimitiveClassLines(self):
        outLines = []
        for mappingName in self.sortedMappings.keys():
            mapping = self.sortedMappings[mappingName]
            for modifierName in mapping.keys():
                modifier = mapping[modifierName]
                if modifier["Type"] in assets.validButtonTypes:
                    type = "Button"
                elif modifier["Type"] in assets.validAxisTypes:
                    type = "Axis"
                outLines.append( "private " + type + " " + modifierName + ";" )
        return outLines


    def mappingTypeToCTLPadType(self, type):
        if type == "Momentary":
            return "MomentaryButton"
        elif type == "Toggle":
            return "ToggleButton"
        elif type == "Trigger":
            return "TriggerButton"
        elif type == "Axis":
            return "AxisAsButton"
        elif type == "Button":
            return "ButtonAxAxis"
        elif type == "Exponential":
            return "ExponentialAxis"
        elif type == "Linear":
            return "LinearAxis"
        elif type == "Merged":
            return "MergedAxis"

    def stripGamepadNumber(self, modifierName):
        return modifierName[3:]

    def modifierNameToGamepadField(self, modifierName):
        gamepadNumber = modifierName[:3]
        if gamepadNumber == "One":
            gamepadNumber = "1"
        else:
            gamepadNumber = "2"
        primitiveName = modifierName[3:].lower()
        if primitiveName in assets.validButtons or primitiveName in assets.validAxes:
            return "gamepad" + gamepadNumber + "." + primitiveName

    def splitMergedAxisName(self, mergedAxisName):
        gamepadNumber = mergedAxisName[:3]
        for mappingName in self.sortedMappings:
            if mergedAxisName.count(mappingName) > 0:
                firstAxis = mappingName
                break
        secondAxis = gamepadNumber + mergedAxisName[ 3+len(firstAxis) : ]
        return [firstAxis, secondAxis]

    def createPrimitiveConstructorLine(self, type, name, modifierName, gamepadField):
        # TODO: adding modifierName here changes basically everything else...
        return name + modifierName + " = new " + type + "( () -> " + gamepadField + " );"

    def getPrimitiveConstructorLines(self):
        outLines = []
        fakeAxes = []
        fakeButtons = []

        # Process real primitives and sort out fake ones for later
        for mappingName in self.sortedMappings.keys():
            mapping = self.sortedMappings[mappingName]
            for modifierName in mapping.keys():
                modifier = mapping[modifierName]
                modifierType = self.mappingTypeToCTLPadType( modifier["Type"] )
                # Split merged Axes
                if modifierType == "Merged":
                    splitNames = self.splitMergedAxisName( mappingName )
                else:
                    splitNames = [ mappingName ]
                # Whether it was split or not, we can iterate over it just the same
                for name in splitNames:
                    modifier = self.sortedMappings[ name ]
                    # Sort out fakes
                    if modifierType in assets.validButtonTypes and self.stripGamepadNumber( mappingName ) not in assets.validButtons:
                        fakeButtons.append( [ modifier, name, mappingName ] )
                    elif modifierType in assets.validAxisTypes and self.stripGamepadNumber( mappingName ) not in assets.validAxes:
                        fakeAxes.append( [ modifier, name, mappingName ] )
                    else:
                        # Process reals
                        gamepadField = self.modifierNameToGamepadField( name )
                        outLines.append( self.createPrimitiveConstructorLine( modifier["Type"], name, modifierName, gamepadField ) )


        # Process fake Buttons
        for fake in fakeButtons:
            fakeModifier = fake[0]
            fakeModifierName = fake[1]
            fakeMappingName = fake[2]

            # Find real Modifier and name
            success = False
            for mappingName in self.sortedMappings.keys():
                mapping = self.sortedMappings[ mappingName ]
                for modifierName in mapping.keys():
                    modifier = self.sortedMappings[ modifierName ]
                    if modifier["Action"] == "Button":
                        if modifier["Action"]["Parameters"]["Button"]["Value"] == self.stripGamepadNumber( fakeMappingName ):
                            realModifierName = modifierName
                            realModifier = modifier
                            success = True
                            break
            if not success:
                Common.error("Generator found a fake Button that seems to have no matching real Button.")

            # Now we can set up the Primitives, knowing where we're actually pulling from
            gamepadField = self.modifierNameToGamepadField( realModifierName )
            outLines.append( self.createPrimitiveConstructorLine( fakeModifier["Type"], fakeModifierName, gamepadField ) )

        return outLines



    def getConstructorInteriorLines( self ):
        returnList = self.getPrimitiveConstructorLines()
        returnList.extend( self.constructorLines )
        return returnList


    def getSetterMethods( self ):
        expandedMethods = []
        for setter in self.setters:
            if setter[:5] == "<~dt>":
                if self.driveType == "fieldy":
                    setter = "<~fieldy>" + setter[5:]
                else:
                    setter = "<~indy>" + setter[5:]
            methodCode = self.expander.getAndExpandMethod( setter )
            expandedMethods.append(methodCode)
        return expandedMethods


    def getStateInteriorLines( self ):
        outLines = []
        for setter in self.setters:
            if setter[:5] == "<~dt>":
                setter = setter[5:]
            outLines.append( setter + "();" )
        return outLines


    def expandTemplate( self, lines ):
        outLines = []
        for line in lines:
            expandedLine = self.expander.expandLine( line, "Template" )
            outLines.append( expandedLine )
        return outLines

    def indentLines( self, lines, by ):
        outLines = []
        for line in lines:
            outLines.append( ("\t" * by) + line )
        return outLines

    def getFile( self ):
        outLines = []

        if len(self.importLines) > 0:
            lines = [ "import " + self.expander.expandLine( line, "FilePath" ) + ";" for line in self.importLines ]
            self.expander.setImportLines( self.stringify( lines ) )
        else:
            self.expander.setImportLines( "\n" )

        self.classLines.extend( self.getPrimitiveClassLines() )
        # if len(self.classLines) > 0:
        self.expander.setClassLines( self.stringify( self.indentLines (self.classLines, 1 ) ) )
        # else:
        #     self.expander.setClassLines( "\n" )

        if self.driveType == "fieldy":
            outLines.extend( self.expandTemplate( assets.fieldyStartTemplateLines ) )
        else:
            outLines.extend( self.expandTemplate( assets.indyStartTemplateLines ) )

        outLines.extend( self.indentLines( self.getConstructorInteriorLines(), 2) )
        outLines.append( "}" )
        outLines.append( "" )

        setterMethods = self.getSetterMethods()
        for method in setterMethods:
            outLines.append( method )
            # outLines.append( "" )

        self.expander.setStateInteriorLines( self.stringify( self.indentLines( self.getStateInteriorLines(), 2 ) ) )

        if self.driveType == "fieldy":
            outLines.extend( self.expandTemplate( assets.fieldyEndTemplateLines ) )
        else:
            outLines.extend( self.expandTemplate( assets.indyEndTemplateLines ) )

        outLines.append( "" )

        return outLines
