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

INSTALL(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/application/__init__.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/application
  )

INSTALL(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/application/__init__.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/application/velocity/precomputed_tasks.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/application/velocity
  )

INSTALL(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/application/__init__.py
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/application/acceleration/precomputed_tasks.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/application/acceleration
  )
