A Setters file lists the setter methods of the controls state in use and a bit of other season information. There should be one file for each season.
The code generator will use this to know which methods to try to create. Not all Method Libraries need to implement all Setters, but all Setters for the selected Season need to be implemented by Methods. It can be covered by multiple Method Libraries.

See "fy23.py" as an example of how one of these files is formatted.

The following expander tags are available within values for "Interface" and "State":
- <~tc> - location of TeamCode package

No expander tags are available within "Season" or "Setters".