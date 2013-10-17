#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tut')
import rospy
import smach
import smach_ros
from action_tut.msg import ExampleAction,ExampleGoal
from smach_ros import SimpleActionState

from smach_tut import example_states
from actionlib_msgs.msg import *

# Give time in seconds
class sleep(smach.State):
   def __init__(self,sleep_time):
          smach.State.__init__(self, outcomes=['succeeded','aborted'])
          self.sleep_time = sleep_time
   def execute(self,data):
        self.timer = 0
        while(self.timer/10 < self.sleep_time):
            rospy.sleep(.1)
            self.timer = self.timer + 1
            if (self.preempt_requested()):
                self.service_preempt()
                return 'aborted'
        return 'succeeded'

# main
def main():
    rospy.init_node('example_action_smach_state_machine')

#TEST CONCURRENCE STATES WITH ACTIONS--------------------------------------------------------------------------------------------------
    test_con1 = smach.Concurrence(outcomes=['test_done'],
                                          default_outcome = 'test_done',
                                          input_keys = ['test_order'],
                                          output_keys= ['test_output_order'],
                                          child_termination_cb = lambda outcome_map : True,
                                          outcome_cb = lambda outcome_map : 'test_done')

    # Test Concurrence with goal and result callbacks that grab data from
    # the userdata input key passed into the state machine
    with test_con1:
        def test_con1_result_cb(userdata, status, result):
           if status == GoalStatus.SUCCEEDED:
              rospy.loginfo(rospy.get_name() + ": The following data was returned from server %s" % str(result.sequence))
              userdata.test_output_order = 6
              return 'succeeded'

        def test_con1_goal_cb(userdata, goal):
            test_goal = ExampleGoal()
            test_goal.order = userdata.test_order
            return test_goal

        smach.Concurrence.add('TestCon1', SimpleActionState('example_server',
                                      ExampleAction,
                                      input_keys=['test_order'],
                                      output_keys=['test_output_order'],
                                      goal_cb=test_con1_goal_cb,
                                      result_cb=test_con1_result_cb))
                                 
        smach.Concurrence.add('testTimeout',sleep(20))


    test_con2 = smach.Concurrence(outcomes=['test_done'],
                                          default_outcome = 'test_done',
                                          input_keys = ['test_order'],
                                          child_termination_cb = lambda outcome_map : True,
                                          outcome_cb = lambda outcome_map : 'test_done')
    with test_con2:
        def test_con2_result_cb(userdata, status, result):
           if status == GoalStatus.SUCCEEDED:
              rospy.loginfo(rospy.get_name() + ": The following data was returned from server %s" % str(result.sequence))
              return 'succeeded'

        def test_con2_goal_cb(userdata, goal):
            test_goal = ExampleGoal()
            test_goal.order = userdata.test_order
            return test_goal

        smach.Concurrence.add('TestCon2', SimpleActionState('example_server',
                                          ExampleAction,
                                          input_keys=['test_order'],
                                          goal_cb=test_con2_goal_cb,
                                          result_cb=test_con2_result_cb))

        smach.Concurrence.add('testTimeout',sleep(20))


    test_con3 = smach.Concurrence(outcomes=['test_done'],
                                          default_outcome = 'test_done',
                                          child_termination_cb = lambda outcome_map : True,
                                          outcome_cb = lambda outcome_map : 'test_done')

    with test_con3:
        def test_con3_result_cb(userdata, status, result):
           if status == GoalStatus.SUCCEEDED:
              rospy.loginfo(rospy.get_name() + ": The following data was returned from server %s" % str(result.sequence))
              return 'succeeded'

        test_goal = ExampleGoal()
        test_goal.order = 7

        smach.Concurrence.add('TestCon3', SimpleActionState('example_server',
                                          ExampleAction,
                                          goal=test_goal,
                                          result_cb=test_con3_result_cb))

        smach.Concurrence.add('testTimeout',sleep(20)) 

#SETUP ITERATOR STATE MACHINE--------------------------------------------------------------------------
    sm_it = smach.StateMachine(outcomes=['outcome6'])
    sm_it.userdata.nums = range(25, 88, 3)
    sm_it.userdata.even_nums = []
    sm_it.userdata.odd_nums = []

    with sm_it:
        tutorial_it = smach.Iterator(outcomes = ['succeeded','aborted'],
                                       input_keys = ['nums', 'even_nums', 'odd_nums'],
                                       it = lambda: range(0, len(sm_it.userdata.nums)),
                                       output_keys = ['even_nums', 'odd_nums'],
                                       it_label = 'index',
                                       exhausted_outcome = 'succeeded')

        with tutorial_it:
            container_sm = smach.StateMachine(outcomes = ['succeeded','preempted','aborted','continue'],
                                        input_keys = ['nums', 'index', 'even_nums', 'odd_nums'],
                                        output_keys = ['even_nums', 'odd_nums'])

            with container_sm:
                #test wether even or odd with a conditional state
                smach.StateMachine.add('EVEN_OR_ODD',
                                 smach_ros.ConditionState(cond_cb = lambda ud:ud.nums[ud.index]%2, 
                                                input_keys=['nums', 'index']),
                                 {'true':'ODD',
                                  'false':'EVEN' })

                #add even state using a callback state
                @smach.cb_interface(input_keys=['nums', 'index', 'even_nums'],
                                    output_keys=['odd_nums'], 
                                    outcomes=['succeeded'])

                def even_cb(ud):
                    rospy.sleep(1)
                    ud.even_nums.append(ud.nums[ud.index])
                    return 'succeeded'


                smach.StateMachine.add('EVEN', smach.CBState(even_cb), 
                                 {'succeeded':'continue'})

                #add odd state using a callback state
                @smach.cb_interface(input_keys=['nums', 'index', 'odd_nums'], 
                                    output_keys=['odd_nums'], 
                                    outcomes=['succeeded'])

                def odd_cb(ud):
                    rospy.sleep(1)
                    ud.odd_nums.append(ud.nums[ud.index])
                    return 'succeeded'


                smach.StateMachine.add('ODD', smach.CBState(odd_cb), 
                                 {'succeeded':'continue'})

            #close container_sm
            smach.Iterator.set_contained_state('CONTAINER_STATE', 
                                         container_sm, 
                                         loop_outcomes=['continue'])

        smach.StateMachine.add('TUTORIAL_IT',tutorial_it,
                             {'succeeded':'outcome6',
                              'aborted':'outcome6'})

#STATE MACHINE-----------------------------------------------------------------------------------------
    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome5'])

    # Create userdata for top level state machine
    sm_top.userdata.first_goal = 5
    sm_top.userdata.pass_data = 0

    # Open the top container
    with sm_top:
        # Add Concurrent States
        smach.StateMachine.add('TestCon1', test_con1,
                                transitions={'test_done':'TestCon2'},
                                remapping={'test_order':'first_goal',
                                           'test_output_order':'pass_data'})

        smach.StateMachine.add('TestCon2', test_con2,
                                transitions={'test_done':'TestCon3'},
                                remapping={'test_order':'pass_data'})

        smach.StateMachine.add('TestCon3', test_con3,
                                transitions={'test_done':'SUB'})

        # Create and add a low tier statemachine
        sm_sub = smach.StateMachine(outcomes=['outcome4'])

        with sm_sub:
            smach.StateMachine.add('FOO', example_states.Foo(), 
                                   transitions={'outcome1':'BAR', 
                                                'outcome2':'outcome4'})
            smach.StateMachine.add('BAR', example_states.Bar(), 
                                   transitions={'outcome2':'FOO'})

        # Add the low tier statemachine
        smach.StateMachine.add('SUB', sm_sub,
                               transitions={'outcome4':'TUT_IT'})

        # Add the low tier statemachine
        smach.StateMachine.add('TUT_IT', sm_it,
                               transitions={'outcome6':'TestCon1'})

    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
