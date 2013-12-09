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

#!/usr/bin/env python

from dynamic_graph.sot.core.feature_position import FeaturePosition
from dynamic_graph.sot.core import FeatureGeneric, FeatureJointLimits, Task, \
    JointLimitator, SOT
from dynamic_graph import plug

class Solver:

    def __init__(self, robot, solverType=SOT):
        self.robot = robot

        # Make sure control does not exceed joint limits.
        self.jointLimitator = JointLimitator('joint_limitator')
        plug(self.robot.dynamic.position, self.jointLimitator.joint)
        plug(self.robot.dynamic.upperJl, self.jointLimitator.upperJl)
        plug(self.robot.dynamic.lowerJl, self.jointLimitator.lowerJl)

        # Create the solver.
        self.sot = solverType('solver')
        self.sot.signal('damping').value = 1e-6
        self.sot.setSize(self.robot.dimension)


        # Plug the solver control into the filter.
        plug(self.sot.control, self.jointLimitator.controlIN)

        # Important: always use 'jointLimitator.control'
        # and NOT 'sot.control'!

        if robot.device:
            plug(self.jointLimitator.control, robot.device.control)

    def push (self, task):
        """
        Proxy method to push a task in the sot
        """
        self.sot.push (task.name)

    def remove (self, task):
        """
        Proxy method to remove a task from the sot
        """
        self.sot.remove (task.name)

    def __str__ (self):
        return self.sot.display ()

def initializeSignals (robot):
    """
    For portability, make some signals accessible as attributes.
    """
    robot.comRef = robot.featureComDes.errorIN
    robot.zmpRef = robot.device.zmp
    robot.com = robot.dynamic.com
    robot.comSelec = robot.featureCom.selec
    robot.comdot = robot.featureComDes.errordotIN

def createCenterOfMassFeatureAndTask(robot,
                                     featureName, featureDesName,
                                     taskName,
                                     selec = '111',
                                     gain = 1.):
    robot.dynamic.com.recompute(0)
    robot.dynamic.Jcom.recompute(0)
    
    featureCom = FeatureGeneric(featureName)
    plug(robot.dynamic.com, featureCom.errorIN)
    plug(robot.dynamic.Jcom, featureCom.jacobianIN)
    featureCom.selec.value = selec
    featureComDes = FeatureGeneric(featureDesName)
    featureComDes.errorIN.value = robot.dynamic.com.value
    featureCom.setReference(featureComDes.name)
    comTask = Task(taskName)
    comTask.add(featureName)
    comTask.controlGain.value = gain
    return (featureCom, featureComDes, comTask)

def createOperationalPointFeatureAndTask(robot,
                                         operationalPointName,
                                         featureName,
                                         taskName,
                                         gain = .2):
    jacobianName = 'J{0}'.format(operationalPointName)
    robot.dynamic.signal(operationalPointName).recompute(0)
    robot.dynamic.signal(jacobianName).recompute(0)
    feature = \
        FeaturePosition(featureName,
                        robot.dynamic.signal(operationalPointName),
                        robot.dynamic.signal(jacobianName),
                        robot.dynamic.signal(operationalPointName).value)
    task = Task(taskName)
    task.add(featureName)
    task.controlGain.value = gain
    return (feature, task)

def createBalanceTask (robot, taskName, gain = 1.):
    task = Task (taskName)
    task.add (robot.featureCom.name)
    task.add (robot.leftAnkle.name)
    task.add (robot.rightAnkle.name)
    task.controlGain.value = gain
    return task


def initialize (robot, solverType=SOT):
    """
    Tasks are stored into 'tasks' dictionary.

    For portability, some signals are accessible as attributes:
      - zmpRef: input (vector),
      - comRef: input (vector).
      - com:    output (vector)
      - comSelec input (flag)
      - comdot: input (vector) reference velocity of the center of mass

    """
    
    # --- center of mass ------------
    (robot.featureCom, robot.featureComDes, robot.comTask) = \
        createCenterOfMassFeatureAndTask(robot,
        '{0}_feature_com'.format(robot.name),
        '{0}_feature_ref_com'.format(robot.name),
        '{0}_task_com'.format(robot.name))
        
    # --- operational points tasks -----
    robot.features = dict()
    robot.tasks = dict()
    for op in robot.OperationalPoints:
        (robot.features[op], robot.tasks[op]) = \
            createOperationalPointFeatureAndTask(robot,
            op, '{0}_feature_{1}'.format(robot.name, op),
            '{0}_task_{1}'.format(robot.name, op))
        # define a member for each operational point
        w = op.split('-')
        memberName = w[0]
        for i in w[1:]:
            memberName += i.capitalize()
        setattr(robot, memberName, robot.features[op])
    robot.tasks ['com'] = robot.comTask
    robot.waist.selec.value = '011100'

    # --- balance task --- #
    robot.tasks ['balance'] =\
        createBalanceTask (robot, '{0}_task_balance'.format (robot.name))

    initializeSignals (robot)

    # --- create solver --- #
    solver = Solver (robot, solverType)

    # --- push balance task --- #
    solver.push (robot.tasks ['com'])
    solver.push (robot.tasks ['left-ankle'])
    solver.push (robot.tasks ['right-ankle'])

    return solver
