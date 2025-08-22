# The code generator itself! Thanks to all the prepwork, this just strings
# already prepared sections together.
# File last updated 8-1-25

import os
import sys
import traceback

import assets

class Generator:
    def __init__(self, actionMappings, libDict):
        self.actionMappings = actionMappings
        self.libDict = libDict

