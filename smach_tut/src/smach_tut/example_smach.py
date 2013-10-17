#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tut')
import rospy
import smach
import smach_ros

from smach_tut import example_states
#from congregators_smach import test_states

# main
def main():
    rospy.init_node('example_smach_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', example_states.Foo(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', example_states.Bar(), 
                               transitions={'outcome2':'FOO'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
