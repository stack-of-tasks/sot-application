# Copyright 2013, Florent Lamiraux, CNRS

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/sphinx.cmake)

FINDPYTHON()

SET(PROJECT_NAME sot-application)
SET(PROJECT_DESCRIPTION "Initialization of control graph dependending on applications")
SET(PROJECT_URL "https://github.com/stack-of-tasks/sot-application")

SET(DOXYGEN_USE_MATHJAX YES)
STRING(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})

SETUP_PROJECT()

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("sot-dynamic-pinocchio >= 2.8")

# Add subdirectories.
ADD_SUBDIRECTORY(${PY_NAME})
ADD_SUBDIRECTORY(doc)

SETUP_PROJECT_FINALIZE()
