# Copyright 2013, 2020, Florent Lamiraux, Guilhem Saurel, CNRS

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

# Project properties
SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME sot-application)
SET(PROJECT_DESCRIPTION "Initialization of control graph dependending on applications")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project configuration
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(DOXYGEN_USE_MATHJAX YES)

# JRL-cmakemodule setup
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
ADD_PROJECT_DEPENDENCY(sot-dynamic-pinocchio REQUIRED)
FINDPYTHON()
STRING(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})

ADD_LIBRARY(${PROJECT_NAME} INTERFACE)
INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)


PYTHON_INSTALL_ON_SITE(${PY_NAME} __init__.py)

PYTHON_INSTALL_ON_SITE(${PY_NAME}/velocity __init__.py)
PYTHON_INSTALL_ON_SITE(${PY_NAME}/velocity precomputed_tasks.py)
PYTHON_INSTALL_ON_SITE(${PY_NAME}/velocity precomputed_meta_tasks.py)

PYTHON_INSTALL_ON_SITE(${PY_NAME}/acceleration __init__.py)
PYTHON_INSTALL_ON_SITE(${PY_NAME}/acceleration precomputed_meta_tasks.py)

INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})
