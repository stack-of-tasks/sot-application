#!/usr/bin/env python

# Copyright 2013, Florent Lamiraux, CNRS

import warnings

from numpy import identity

from dynamic_graph import plug
from dynamic_graph.sot.core.feature_generic import FeatureGeneric
from dynamic_graph.sot.core.feature_position import FeaturePosition
from dynamic_graph.sot.core.gain_adaptive import GainAdaptive
from dynamic_graph.sot.core.joint_limitator import JointLimitator
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.sot import SOT, Task


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

    def push(self, task):
        """
        Proxy method to push a task in the sot
        """
        self.sot.push(task.name)

    def remove(self, task):
        """
        Proxy method to remove a task from the sot
        """
        self.sot.remove(task.name)

    def __str__(self):
        return self.sot.display()


def initializeSignals(application, robot):
    """
    For portability, make some signals accessible as attributes.
    """
    application.comRef = application.featureComDes.errorIN
    application.zmpRef = robot.device.zmp
    application.com = robot.dynamic.com
    application.comSelec = application.featureCom.selec
    application.comdot = application.featureComDes.errordotIN


def createCenterOfMassFeatureAndTask(robot, featureName, featureDesName, taskName, selec='111', ingain=1.):
    robot.dynamic.com.recompute(0)
    robot.dynamic.Jcom.recompute(0)

    featureCom = FeatureGeneric(featureName)
    plug(robot.dynamic.com, featureCom.errorIN)
    plug(robot.dynamic.Jcom, featureCom.jacobianIN)
    featureCom.selec.value = selec
    featureComDes = FeatureGeneric(featureDesName)
    featureComDes.errorIN.value = robot.dynamic.com.value
    featureCom.setReference(featureComDes.name)
    taskCom = Task(taskName)
    taskCom.add(featureName)
    gainCom = GainAdaptive('gain' + taskName)
    gainCom.setConstant(ingain)
    plug(gainCom.gain, taskCom.controlGain)
    plug(taskCom.error, gainCom.error)
    return (featureCom, featureComDes, taskCom, gainCom)


def createOperationalPointFeatureAndTask(robot, operationalPointName, featureName, taskName, ingain=.2):
    operationalPointMapped = operationalPointName
    jacobianName = 'J{0}'.format(operationalPointMapped)
    robot.dynamic.signal(operationalPointMapped).recompute(0)
    robot.dynamic.signal(jacobianName).recompute(0)
    feature = \
        FeaturePosition(featureName,
                        robot.dynamic.signal(operationalPointMapped),
                        robot.dynamic.signal(jacobianName),
                        robot.dynamic.signal(operationalPointMapped).value)
    task = Task(taskName)
    task.add(featureName)
    gain = GainAdaptive('gain' + taskName)
    gain.setConstant(ingain)
    plug(gain.gain, task.controlGain)
    plug(task.error, gain.error)
    return (feature, task, gain)


def createBalanceTask(robot, application, taskName, ingain=1.):
    task = Task(taskName)
    task.add(application.featureCom.name)
    task.add(application.leftAnkle.name)
    task.add(application.rightAnkle.name)
    gain = GainAdaptive('gain' + taskName)
    gain.setConstant(ingain)
    plug(gain.gain, task.controlGain)
    plug(task.error, gain.error)
    return (task, gain)


def createPostureTask(robot, taskName, ingain=1.):
    robot.dynamic.position.recompute(0)
    feature = FeatureGeneric('feature' + taskName)
    featureDes = FeatureGeneric('featureDes' + taskName)
    featureDes.errorIN.value = robot.halfSitting
    plug(robot.dynamic.position, feature.errorIN)
    feature.setReference(featureDes.name)
    robotDim = len(robot.dynamic.position.value)
    feature.jacobianIN.value = matrixToTuple(identity(robotDim))
    task = Task(taskName)
    task.add(feature.name)
    gain = GainAdaptive('gain' + taskName)
    plug(gain.gain, task.controlGain)
    plug(task.error, gain.error)
    gain.setConstant(ingain)
    return (feature, featureDes, task, gain)


def initialize(robot, solverType=SOT):
    """
    Tasks are stored into 'tasks' dictionary.

    WARNING: deprecated, use Application instead

    For portability, some signals are accessible as attributes:
      - zmpRef: input (vector),
      - comRef: input (vector).
      - com:    output (vector)
      - comSelec input (flag)
      - comdot: input (vector) reference velocity of the center of mass

    """

    warnings.warn("""The function dynamic_graph.sot.application.velocity.precomputed_tasks.initialize is deprecated.
        Use dynamic_graph.sot.application.velocity.precomputed_tasks.Application""")

    # --- center of mass ------------
    (robot.featureCom, robot.featureComDes, robot.comTask,
     robot.gainCom) = createCenterOfMassFeatureAndTask(robot, '{0}_feature_com'.format(robot.name),
                                                       '{0}_feature_ref_com'.format(robot.name),
                                                       '{0}_task_com'.format(robot.name))

    # --- operational points tasks -----
    robot.features = dict()
    robot.tasks = dict()
    robot.gains = dict()
    for op in robot.OperationalPoints:
        (robot.features[op], robot.tasks[op],
         robot.gains[op]) = createOperationalPointFeatureAndTask(robot, op, '{0}_feature_{1}'.format(robot.name, op),
                                                                 '{0}_task_{1}'.format(robot.name, op))
        # define a member for each operational point
        w = op.split('-')
        memberName = w[0]
        for i in w[1:]:
            memberName += i.capitalize()
        setattr(robot, memberName, robot.features[op])
    robot.tasks['com'] = robot.comTask
    robot.gains['com'] = robot.gainCom
    robot.waist.selec.value = '011100'

    # --- balance task --- #
    (robot.tasks['balance'], robot.gains['balance']) = createBalanceTask(robot, robot,
                                                                         '{0}_task_balance'.format(robot.name))

    initializeSignals(robot, robot)

    # --- create solver --- #
    solver = Solver(robot, solverType)

    # --- push balance task --- #
    solver.push(robot.tasks['com'])
    solver.push(robot.tasks['left-ankle'])
    solver.push(robot.tasks['right-ankle'])

    return solver


class Application(object):
    """
    Generic application with most used tasks

    For portability, some signals are accessible as attributes:
      - zmpRef: input (vector),
      - comRef: input (vector).
      - com:    output (vector)
      - comSelec input (flag)
      - comdot: input (vector) reference velocity of the center of mass

    """

    def __init__(self, robot, solverType=SOT):

        self.robot = robot

        # --- center of mass ------------
        (self.featureCom, self.featureComDes, self.taskCom,
         self.gainCom) = createCenterOfMassFeatureAndTask(robot, '{0}_feature_com'.format(robot.name),
                                                          '{0}_feature_ref_com'.format(robot.name),
                                                          '{0}_task_com'.format(robot.name))

        # --- operational points tasks -----
        self.features = dict()
        self.tasks = dict()
        self.gains = dict()
        for op in robot.OperationalPoints:
            (self.features[op], self.tasks[op],
             self.gains[op]) = createOperationalPointFeatureAndTask(robot, op,
                                                                    '{0}_feature_{1}'.format(robot.name, op),
                                                                    '{0}_task_{1}'.format(robot.name, op))
            # define a member for each operational point
            w = op.split('-')
            memberName = w[0]
            for i in w[1:]:
                memberName += i.capitalize()
            setattr(self, memberName, self.features[op])

        self.tasks['com'] = self.taskCom
        self.features['com'] = self.featureCom
        self.gains['com'] = self.gainCom

        self.features['waist'].selec.value = '011100'

        # --- balance task --- #
        (self.tasks['balance'], self.gains['balance']) = createBalanceTask(robot, self,
                                                                           '{0}_task_balance'.format(robot.name))

        (self.featurePosture, self.featurePostureDes, self.taskPosture,
         self.gainPosture) = createPostureTask(robot, "posture")
        self.tasks['posture'] = self.taskPosture
        self.features['posture'] = self.featurePosture
        self.gains['posture'] = self.gainPosture

        initializeSignals(self, robot)

        # --- create solver --- #
        self.solver = Solver(robot, solverType)
        self.initDefaultTasks()

    def initDefaultTasks(self):
        self.solver.sot.clear()
        self.solver.push(self.tasks['com'])
        self.solver.push(self.tasks['left-ankle'])
        self.solver.push(self.tasks['right-ankle'])
