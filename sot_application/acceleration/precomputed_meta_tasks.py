#!/usr/bin/env python

# Copyright 2013, Florent Lamiraux, Francesco Morsillo CNRS

from numpy import eye

from dynamic_graph import plug
from dynamic_graph.sot.core.feature_generic import FeatureGeneric
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.dyninv import SolverKine, TaskDynInequality, TaskDynLimits
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import MetaTaskDynCom, MetaTaskDynPosture


class Solver:

    def __init__(self, robot):
        self.robot = robot
        """
        # Make sure control does not exceed joint limits.
        self.jointLimitator = JointLimitator('joint_limitator')
        plug(self.robot.dynamic.position, self.jointLimitator.joint)
        plug(self.robot.dynamic.upperJl, self.jointLimitator.upperJl)
        plug(self.robot.dynamic.lowerJl, self.jointLimitator.lowerJl)
        """

        # Create the solver.
        self.sot = SolverKine('solver')
        self.sot.signal('damping').value = 1e-6
        self.sot.setSize(self.robot.dimension)

        # Set second order computation
        self.robot.device.setSecondOrderIntegration()
        self.sot.setSecondOrderKinematics()
        plug(self.robot.device.velocity, self.robot.dynamic.velocity)
        plug(self.robot.dynamic.velocity, self.sot.velocity)
        """
        # Plug the solver control into the filter.
        plug(self.sot.control, self.jointLimitator.controlIN)

        # Important: always use 'jointLimitator.control'
        # and NOT 'sot.control'!

        if robot.device:
            plug(self.jointLimitator.control, robot.device.control)
        """

        # Plug the solver control into the robot.
        plug(self.sot.control, robot.device.control)

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


def setTaskLim(taskLim, robot):
    """
    Sets the parameters for teh 'task-limits'
    """
    # Angular position and velocity limits
    plug(robot.dynamic.position, taskLim.position)
    plug(robot.dynamic.velocity, taskLim.velocity)
    taskLim.dt.value = robot.timeStep
    robot.dynamic.upperJl.recompute(0)
    robot.dynamic.lowerJl.recompute(0)
    taskLim.referencePosInf.value = robot.dynamic.lowerJl.value
    taskLim.referencePosSup.value = robot.dynamic.upperJl.value
    # dqup = (0, 0, 0, 0, 0, 0, 200, 220, 250, 230, 290, 520, 200, 220, 250, 230, 290, 520, 250, 140, 390, 390, 240,
    # 140, 240, 130, 270, 180, 330, 240, 140, 240, 130, 270, 180, 330)
    dqup = (1000, ) * robot.dimension
    taskLim.referenceVelInf.value = tuple([-val * 3.14 / 180 for val in dqup])
    taskLim.referenceVelSup.value = tuple([val * 3.14 / 180 for val in dqup])
    taskLim.controlGain.value = 0.3


def setContacts(contactLF, contactRF):
    """
    Sets the parameters for teh contacts
    """
    # Left foot
    contactLF.featureDes.velocity.value = (0, 0, 0, 0, 0, 0)
    contactLF.feature.frame('desired')
    contactLF.name = "LF"

    # Right foot
    contactRF.featureDes.velocity.value = (0, 0, 0, 0, 0, 0)
    contactRF.feature.frame('desired')
    contactRF.name = "RF"

    contactRF.support = ((0.11, -0.08, -0.08, 0.11), (-0.045, -0.045, 0.07, 0.07), (-0.105, -0.105, -0.105, -0.105))
    contactLF.support = ((0.11, -0.08, -0.08, 0.11), (-0.07, -0.07, 0.045, 0.045), (-0.105, -0.105, -0.105, -0.105))

    # Imposed errordot = 0
    contactLF.feature.errordot.value = (0, 0, 0, 0, 0, 0)
    contactRF.feature.errordot.value = (0, 0, 0, 0, 0, 0)


def createTasks(robot):

    # MetaTasks dictonary
    robot.mTasks = dict()
    robot.tasksIne = dict()

    # Foot contacts
    robot.contactLF = MetaTaskDyn6d('contactLF', robot.dynamic, 'lf', 'left-ankle')
    robot.contactRF = MetaTaskDyn6d('contactRF', robot.dynamic, 'rf', 'right-ankle')
    setContacts(robot.contactLF, robot.contactRF)

    # MetaTasksDyn6d for other operational points
    robot.mTasks['waist'] = MetaTaskDyn6d('waist', robot.dynamic, 'waist', 'waist')
    robot.mTasks['chest'] = MetaTaskDyn6d('chest', robot.dynamic, 'chest', 'chest')
    robot.mTasks['rh'] = MetaTaskDyn6d('rh', robot.dynamic, 'rh', 'right-wrist')
    robot.mTasks['lh'] = MetaTaskDyn6d('lh', robot.dynamic, 'lh', 'left-wrist')

    for taskName in robot.mTasks:
        robot.mTasks[taskName].feature.frame('desired')
        robot.mTasks[taskName].gain.setConstant(10)
        robot.mTasks[taskName].task.dt.value = robot.timeStep
        robot.mTasks[taskName].featureDes.velocity.value = (0, 0, 0, 0, 0, 0)

    handMgrip = eye(4)
    handMgrip[0:3, 3] = (0, 0, -0.14)
    robot.mTasks['rh'].opmodif = matrixToTuple(handMgrip)
    robot.mTasks['lh'].opmodif = matrixToTuple(handMgrip)

    # CoM Task
    robot.mTasks['com'] = MetaTaskDynCom(robot.dynamic, robot.timeStep)
    robot.dynamic.com.recompute(0)
    robot.mTasks['com'].featureDes.errorIN.value = robot.dynamic.com.value
    robot.mTasks['com'].task.controlGain.value = 10
    robot.mTasks['com'].feature.selec.value = '011'

    # Posture Task
    robot.mTasks['posture'] = MetaTaskDynPosture(robot.dynamic, robot.timeStep)
    robot.mTasks['posture'].ref = robot.halfSitting
    robot.mTasks['posture'].gain.setConstant(5)

    # TASK INEQUALITY

    # Task Height
    featureHeight = FeatureGeneric('featureHeight')
    plug(robot.dynamic.com, featureHeight.errorIN)
    plug(robot.dynamic.Jcom, featureHeight.jacobianIN)
    robot.tasksIne['taskHeight'] = TaskDynInequality('taskHeight')
    plug(robot.dynamic.velocity, robot.tasksIne['taskHeight'].qdot)
    robot.tasksIne['taskHeight'].add(featureHeight.name)
    robot.tasksIne['taskHeight'].selec.value = '100'
    robot.tasksIne['taskHeight'].referenceInf.value = (0., 0., 0.)  # Xmin, Ymin
    robot.tasksIne['taskHeight'].referenceSup.value = (0., 0., 0.80771)  # Xmax, Ymax
    robot.tasksIne['taskHeight'].dt.value = robot.timeStep


def createBalanceAndPosture(robot, solver):

    solver.clear()

    # Task Limits
    robot.taskLim = TaskDynLimits('taskLim')
    setTaskLim(robot.taskLim, robot)

    # --- push tasks --- #
    solver.sot.addContact(robot.contactLF)
    solver.sot.addContact(robot.contactRF)
    solver.push(robot.taskLim)
    solver.push(robot.mTasks['com'].task)
    solver.push(robot.mTasks['posture'].task)


def initialize(robot):

    # --- create solver --- #
    solver = Solver(robot)

    # --- create tasks --- #
    createTasks(robot)

    createBalanceAndPosture(robot, solver)

    return solver
