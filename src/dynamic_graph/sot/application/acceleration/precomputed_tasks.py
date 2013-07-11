# Copyright 2013, Florent Lamiraux, Francesco Morsillo CNRS
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
    JointLimitator
from dynamic_graph.sot.dyninv import SolverKine, TaskDynLimits
from dynamic_graph.sot.dyninv.meta_tasks_dyn import gotoNd, MetaTaskDynCom, \
    MetaTaskDynPosture
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph import plug

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
        plug(self.robot.device.velocity,self.robot.dynamic.velocity)
	plug(self.robot.dynamic.velocity,self.sot.velocity)

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


    def push (self, task):
        """
        Proxy method to push a task (not a MetaTask) in the sot
        """
        self.sot.push (task.name)
        if task.name!="taskposture" and "taskposture" in self.toList():
            self.sot.down("taskposture")

    def remove (self, task):
        """
        Proxy method to remove a task from the sot
        """
        self.sot.remove (task.name)

    def pop (self):
        """
        Proxy method to remove the last (usually posture) task from the sot
        """
        self.sot.pop ()

    def __str__ (self):
        return self.sot.display ()

    def toList(self):
        """
        Creates the list of the tasks in the sot
        """
        return map(lambda x: x[1:-1],self.sot.dispStack().split('|')[1:])

    def clear(self):
        """
        Proxy method to remove all tasks from the sot
        """
        self.sot.clear ()


def setTaskLim(taskLim,robot):
    """
    Sets the parameters for teh 'task-limits'
    """
    # Angular position and velocity limits
    plug(robot.dynamic.position,taskLim.position)
    plug(robot.dynamic.velocity,taskLim.velocity)
    taskLim.dt.value = robot.timeStep
    robot.dynamic.upperJl.recompute(0)
    robot.dynamic.lowerJl.recompute(0)
    taskLim.referencePosInf.value = robot.dynamic.lowerJl.value
    taskLim.referencePosSup.value = robot.dynamic.upperJl.value
    #dqup = (0, 0, 0, 0, 0, 0, 200, 220, 250, 230, 290, 520, 200, 220, 250, 230, 290, 520, 250, 140, 390, 390, 240, 140, 240, 130, 270, 180, 330, 240, 140, 240, 130, 270, 180, 330)
    dqup = (1000,)*robot.dimension     ########################
    taskLim.referenceVelInf.value = tuple([-val*3.14/180 for val in dqup])
    taskLim.referenceVelSup.value = tuple([ val*3.14/180 for val in dqup])
    taskLim.controlGain.value = 0.3


def setContacts(contactLF,contactRF):
    """
    Sets the parameters for teh contacts
    """
    # Left foot
    contactLF.featureDes.velocity.value=(0,0,0,0,0,0)
    contactLF.feature.frame('desired')
    contactLF.name = "LF"
    
    # Right foot
    contactRF.featureDes.velocity.value=(0,0,0,0,0,0)
    contactRF.feature.frame('desired')
    contactRF.name = "RF"
    
    contactRF.support = ((0.11,-0.08,-0.08,0.11),(-0.045,-0.045,0.07,0.07),(-0.105,-0.105,-0.105,-0.105))
    contactLF.support = ((0.11,-0.08,-0.08,0.11),(-0.07,-0.07,0.045,0.045),(-0.105,-0.105,-0.105,-0.105))
    
    # Imposed errordot = 0
    contactLF.feature.errordot.value=(0,0,0,0,0,0)
    contactRF.feature.errordot.value=(0,0,0,0,0,0)
    

def createBalanceAndPosture(robot,solver):
    
    # Task Limits
    robot.taskLim = TaskDynLimits('taskLim')
    setTaskLim(robot.taskLim,robot)
    
    # Foot contacts
    robot.contactLF = MetaTaskDyn6d('contact_lleg',robot.dynamic,'lf','left-ankle')
    robot.contactRF = MetaTaskDyn6d('contact_rleg',robot.dynamic,'rf','right-ankle')
    setContacts(robot.contactLF,robot.contactRF)
    
    # CoM Task
    robot.taskCom = MetaTaskDynCom(robot.dynamic,robot.timeStep)
    robot.dynamic.com.recompute(0)
    robot.taskCom.featureDes.errorIN.value = robot.dynamic.com.value
    robot.taskCom.task.controlGain.value = 10

    # Posture Task
    robot.taskPosture = MetaTaskDynPosture(robot.dynamic,robot.timeStep)
    robot.taskPosture.ref = robot.halfSitting
    robot.taskPosture.gain.setConstant(5)


    # --- push tasks --- #
    solver.clear()
    solver.sot.addContact(robot.contactLF)
    solver.sot.addContact(robot.contactRF)
    solver.push(robot.taskLim)
    solver.push(robot.taskCom.task)
    solver.push(robot.taskPosture.task)



def  initialize (robot):

    # --- create solver --- #
    solver = Solver (robot)


    # --- create tasks --- #
    createBalanceAndPosture(robot,solver)


    return solver
