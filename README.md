# SoT Application


[![Pipeline status](https://gitlab.laas.fr/stack-of-tasks/sot-application/badges/master/pipeline.svg)](https://gitlab.laas.fr/stack-of-tasks/sot-application/commits/master)
[![Coverage report](https://gitlab.laas.fr/stack-of-tasks/sot-application/badges/master/coverage.svg?job=doc-coverage)](https://gepettoweb.laas.fr/doc/stack-of-tasks/sot-application/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/stack-of-tasks/sot-application/master.svg)](https://results.pre-commit.ci/latest/github/stack-of-tasks/sot-application)

This package provides python initializations scripts for the Stack of
Tasks.

These scripts are aimed at initializing control graphs dependending on
the application:

 - type of control variable (velocity, acceleration, torque)
 - type of solver (equality only inequality and equality).
