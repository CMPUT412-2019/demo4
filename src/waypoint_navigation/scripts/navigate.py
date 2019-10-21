#!/usr/bin/env python

import rospy
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy
from smach import Sequence, State
from typing import List
from smach_ros import IntrospectionServer
from kobuki_msgs.msg import Sound

rospy.init_node('navigate')

waypoints_to_coords = {
    1: (-8.651, -1.3739, 0),
    2: (-6.616, -0.22274, 0),
    3: (-6.2549, -1.5073, 0),
    4: (-5.8715, 0.23429, 0),
}

joy_btn_codes = {
    'X': 2,
    'A': 0,
    'B': 1,
    'Y': 3,
}

joy_buttons_to_waypoints = {
    joy_btn_codes['X']: 1,
    joy_btn_codes['A']: 2,
    joy_btn_codes['B']: 3,
    joy_btn_codes['Y']: 4,
}


def play_sound():
    if play_sound.publisher is None:
        play_sound.publisher = rospy.Publisher('sound', Sound, queue_size=10)
    sound = Sound()
    sound.value = sound.ON
    play_sound.publisher.publish(sound)
play_sound.publisher = None


class GetWaypointsState(State):
    def __init__(self):
        State.__init__(self, outcomes=['ok'], output_keys=['waypoints'])
        self.joy_subscriber = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.waypoints = []  # type: List[int]
        self.last_time = None

    def execute(self, ud):
        self.waypoints = []
        rate = rospy.Rate(10)
        while len(self.waypoints) < 4:
            rate.sleep()
        ud.waypoints = tuple(self.waypoints)
        return 'ok'

    def joy_callback(self, message):  # type: (Joy) -> None
        try:
            button_pressed = next(index_status[0] for index_status in enumerate(message.buttons) if index_status[1] == 1)
        except StopIteration:
            return
        if self.last_time is None:
            self.last_time = rospy.get_time()
        else:
            duration = rospy.get_time() - self.last_time
            self.last_time = rospy.get_time()
            if duration < 0.1:
                return

        waypoint = joy_buttons_to_waypoints[button_pressed]
        print('Pressed button for waypoint {}'.format(waypoint))
        play_sound()
        self.waypoints.append(waypoint)


class NavigateToWaypointState(State):
    def __init__(self, waypoint):  # type: (int) -> None
        State.__init__(self, outcomes=['ok', 'err'])
        self.waypoint = waypoint
        self.client = SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, ud):
        print('NAVIGATING TO {}'.format(self.waypoint))
        coord = waypoints_to_coords[self.waypoint]
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = coord[0]
        goal.target_pose.pose.position.y = coord[1]
        goal.target_pose.pose.position.z = coord[2]
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)
        result = self.client.wait_for_result()
        play_sound()
        return 'ok' if result else 'err'


class NavigateToWaypointsState(State):
    def __init__(self):
        State.__init__(self, outcomes=['ok', 'err'], input_keys=['waypoints'])

    def execute(self, ud):
        seq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok')
        with seq:
            for index, waypoint in enumerate(ud.waypoints):  # type: int
                seq.add('NavigateTo{}'.format(index), NavigateToWaypointState(waypoint))
        return seq.execute()


seq = Sequence(outcomes=['ok', 'err'], connector_outcome='ok')
with seq:
    Sequence.add('GetWaypoints', GetWaypointsState())
    Sequence.add('Navigate', NavigateToWaypointsState())

sis = IntrospectionServer('smach_server', seq, '/SM_ROOT')
sis.start()

seq.execute()
