CTL2Java is currently [in development]. There will be a version number here when the first release is named.

This is for use with CTLedit, a graphical editor for these control schemes.
CTLedit was made with Snap!, so it is accessed through your web browser:
https://snap.berkeley.edu/project?username=michaelrw&projectname=CTLedit

CTLedit will generate a .ctl file, which defines the physical appearance of the controller in that program as well as all of the button mappings. CTL2Java, this program, takes a CTL file and generates a Java control scheme from it. Assuming you don't run into any bugs, this scheme should be able to be used as-is.

I'm more likely to port CTLedit to something else than I am CTL2Java, so I gave this program some extra smarts that should make .ctl files pretty easy to write. The graphical editor doesn't have to keep track of much of the technical stuff - this will figure that out later.