# Copyright 2013, Florent Lamiraux, CNRS

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(PROJECT_NAMESPACE stack-of-tasks)
SET(PROJECT_NAME sot-application)
SET(PROJECT_DESCRIPTION "Initialization of control graph dependending on applications")
SET(PROJECT_URL "https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME}")

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/sphinx.cmake)

FINDPYTHON()

SET(DOXYGEN_USE_MATHJAX YES)
STRING(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})

CMAKE_POLICY(SET CMP0048 OLD)
PROJECT(${PROJECT_NAME} CXX)

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("sot-dynamic-pinocchio >= 2.8")

# Add subdirectories.
ADD_SUBDIRECTORY(${PY_NAME})
ADD_SUBDIRECTORY(doc)
