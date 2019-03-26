# Copyright 2013, Florent Lamiraux, CNRS

INSTALL(FILES
  __init__.py
  DESTINATION ${PYTHON_SITELIB}/${PY_NAME}
  )

INSTALL(FILES
  __init__.py
  velocity/precomputed_tasks.py
  velocity/precomputed_meta_tasks.py
  DESTINATION ${PYTHON_SITELIB}/${PY_NAME}/velocity
  )

INSTALL(FILES
  __init__.py
  acceleration/precomputed_meta_tasks.py
  DESTINATION ${PYTHON_SITELIB}/${PY_NAME}/acceleration
  )
