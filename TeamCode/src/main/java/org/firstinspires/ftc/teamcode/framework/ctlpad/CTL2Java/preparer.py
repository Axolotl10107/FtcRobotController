# Prepare all the imported stuff (Action & Method libraries & Mappings) in a way
# that will be convenient later. Sort stuff and do some higher-level checks than
# ctlconv did.
# File last updated 8-1-25

import os
import assets

class Preparer:
    def __init__(self, mappings1, mappings2, libDict):
        self.mappings1 = mappings1
        self.mappings2 = mappings2

        self.libDict = {"BaseActions":[], "ExtensionActions":[], "Methods":[]}
