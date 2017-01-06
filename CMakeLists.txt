# Copyright 2013, Florent Lamiraux, CNRS
#
# This file is part of sot-application.
# sot-application is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-application is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-application. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/python.cmake)

FINDPYTHON()

SET(PROJECT_NAME sot-application)
SET(PROJECT_DESCRIPTION "Initialization of control graph dependending on applications")
SET(PROJECT_URL "https://github.com/stack-of-tasks/sot-application")

SETUP_PROJECT()

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("sot-dynamic-pinocchio >= 2.8")

# Add subdirectories.
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(doc)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
