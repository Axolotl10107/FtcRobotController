# The code generator itself! Thanks to all the prepwork, this just strings
# already prepared sections together.
# File last updated 8-23-25

import assets


class Generator:
    def __init__( self, sortedMappings, expander, constructorLines, classLines, setters, driveType, debug ):
        self.sortedMappings = sortedMappings
        self.expander = expander
        self.constructorLines = constructorLines
        self.classLines = classLines
        self.setters = setters
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
            print( "\ndriveType:" )
            print( self.driveType )


    def stringify( self, lines ):
        out = ""
        for line in lines:
            out += line + "\n"
        return out


    def getConstructorInteriorLines( self ):
        return self.constructorLines


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

        if len(self.classLines) > 0:
            self.expander.setClassLines( self.stringify( self.indentLines (self.classLines, 1 ) ) )
        else:
            self.expander.setClassLines( "\n" )

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
