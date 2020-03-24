#!/usr/bin/env python

# Copyright 2013, Florent Lamiraux, Francesco Morsillo CNRS

from numpy import eye

from dynamic_graph import plug
from dynamic_graph.sot.core.feature_generic import FeatureGeneric
from dynamic_graph.sot.core.joint_limitator import JointLimitator
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom
from dynamic_graph.sot.dyninv import SolverKine, TaskInequality, TaskJointLimits


class Solver:
    def __init__(self, robot):
        self.robot = robot

        # Make sure control does not exceed joint limits.
        self.jointLimitator = JointLimitator('joint_limitator')
        plug(self.robot.dynamic.position, self.jointLimitator.joint)
        plug(self.robot.dynamic.upperJl, self.jointLimitator.upperJl)
        plug(self.robot.dynamic.lowerJl, self.jointLimitator.lowerJl)

        # Create the solver.
        self.sot = SolverKine('solver')
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
        Proxy method to push a task (not a MetaTask) in the sot
        """
        self.sot.push(task.name)
        if task.name != "taskposture" and "taskposture" in self.toList():
            self.sot.down("taskposture")

    def rm(self, task):
        """
        Proxy method to remove a task from the sot
        """
        self.sot.rm(task.name)

    def pop(self):
        """
        Proxy method to remove the last (usually posture) task from the sot
        """
        self.sot.pop()

    def __str__(self):
        return self.sot.display()

    def toList(self):
        """
        Creates the list of the tasks in the sot
        """
        return map(lambda x: x[1:-1], self.sot.dispStack().split('|')[1:])

    def clear(self):
        """
        Proxy method to remove all tasks from the sot
        """
        self.sot.clear()


def setTaskLim(taskJL, robot):
    """
    Sets the parameters for the 'joint-limits'
    """
    robot.dynamic.upperJl.recompute(0)
    robot.dynamic.lowerJl.recompute(0)
    plug(robot.dynamic.position, taskJL.position)
    taskJL.controlGain.value = 10
    taskJL.referenceInf.value = robot.dynamic.lowerJl.value
    taskJL.referenceSup.value = robot.dynamic.upperJl.value
    taskJL.dt.value = robot.timeStep
    taskJL.selec.value = toFlags(range(6, 22) + range(22, 28) + range(29, 35))


def createTasks(robot):

    # MetaTasks dictonary
    robot.mTasks = dict()
    robot.tasksIne = dict()

    # Foot contacts
    robot.contactLF = MetaTaskKine6d('contactLF', robot.dynamic, 'LF', robot.OperationalPointsMap['left-ankle'])
    robot.contactLF.feature.frame('desired')
    robot.contactLF.gain.setConstant(10)
    robot.contactRF = MetaTaskKine6d('contactRF', robot.dynamic, 'RF', robot.OperationalPointsMap['right-ankle'])
    robot.contactRF.feature.frame('desired')
    robot.contactRF.gain.setConstant(10)

    # MetaTasksKine6d for other operational points
    robot.mTasks['waist'] = MetaTaskKine6d('waist', robot.dynamic, 'waist', robot.OperationalPointsMap['waist'])
    robot.mTasks['chest'] = MetaTaskKine6d('chest', robot.dynamic, 'chest', robot.OperationalPointsMap['chest'])
    robot.mTasks['rh'] = MetaTaskKine6d('rh', robot.dynamic, 'rh', robot.OperationalPointsMap['right-wrist'])
    robot.mTasks['lh'] = MetaTaskKine6d('lh', robot.dynamic, 'lh', robot.OperationalPointsMap['left-wrist'])

    for taskName in robot.mTasks:
        robot.mTasks[taskName].feature.frame('desired')
        robot.mTasks[taskName].gain.setConstant(10)

    handMgrip = eye(4)
    handMgrip[0:3, 3] = (0, 0, -0.14)
    robot.mTasks['rh'].opmodif = matrixToTuple(handMgrip)
    robot.mTasks['lh'].opmodif = matrixToTuple(handMgrip)

    # CoM Task
    robot.mTasks['com'] = MetaTaskKineCom(robot.dynamic)
    robot.dynamic.com.recompute(0)
    robot.mTasks['com'].featureDes.errorIN.value = robot.dynamic.com.value
    robot.mTasks['com'].task.controlGain.value = 10
    robot.mTasks['com'].feature.selec.value = '011'

    # Posture Task
    robot.mTasks['posture'] = MetaTaskKinePosture(robot.dynamic)
    robot.mTasks['posture'].ref = robot.halfSitting
    robot.mTasks['posture'].gain.setConstant(5)

    # TASK INEQUALITY

    # Task Height
    featureHeight = FeatureGeneric('featureHeight')
    plug(robot.dynamic.com, featureHeight.errorIN)
    plug(robot.dynamic.Jcom, featureHeight.jacobianIN)
    robot.tasksIne['taskHeight'] = TaskInequality('taskHeight')
    robot.tasksIne['taskHeight'].add(featureHeight.name)
    robot.tasksIne['taskHeight'].selec.value = '100'
    robot.tasksIne['taskHeight'].referenceInf.value = (0., 0., 0.)  # Xmin, Ymin
    robot.tasksIne['taskHeight'].referenceSup.value = (0., 0., 0.80771)  # Xmax, Ymax
    robot.tasksIne['taskHeight'].dt.value = robot.timeStep


def createBalance(robot, solver):

    solver.clear()

    # Task Limits
    robot.taskLim = TaskJointLimits('taskLim')
    setTaskLim(robot.taskLim, robot)

    # --- push tasks --- #
    solver.sot.addContact(robot.contactLF)
    solver.sot.addContact(robot.contactRF)
    solver.push(robot.taskLim)
    solver.push(robot.mTasks['com'].task)
    # solver.push(robot.mTasks['posture'].task)


def initialize(robot):

    # --- create solver --- #
    solver = Solver(robot)

    # --- create tasks --- #
    createTasks(robot)

    createBalance(robot, solver)

    return solver
