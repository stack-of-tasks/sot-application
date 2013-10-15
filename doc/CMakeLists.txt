#
#  Copyright 2010 CNRS
#  Author: Florent Lamiraux
#

# get the python script install path
INCLUDE(../cmake/python.cmake)
FINDPYTHON()

INCLUDE(../cmake/sphinx.cmake)
SPHINX_SETUP()
SPHINX_FINALIZE()
