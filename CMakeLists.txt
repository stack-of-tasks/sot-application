# Copyright 2013, 2020, Florent Lamiraux, Guilhem Saurel, CNRS

cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME sot-application)
set(PROJECT_DESCRIPTION
    "Initialization of control graph dependending on applications")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project configuration
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(DOXYGEN_USE_MATHJAX YES)

# JRL-cmakemodule setup
include(cmake/base.cmake)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
add_project_dependency(sot-dynamic-pinocchio REQUIRED)
string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})

add_library(${PROJECT_NAME} INTERFACE)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

python_install_on_site(${PY_NAME} __init__.py)

python_install_on_site(${PY_NAME}/velocity __init__.py)
python_install_on_site(${PY_NAME}/velocity precomputed_tasks.py)
python_install_on_site(${PY_NAME}/velocity precomputed_meta_tasks.py)

python_install_on_site(${PY_NAME}/acceleration __init__.py)
python_install_on_site(${PY_NAME}/acceleration precomputed_meta_tasks.py)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
