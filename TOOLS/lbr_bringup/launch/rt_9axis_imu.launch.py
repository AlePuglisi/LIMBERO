import launch_ros.events
import launch
import lifecycle_msgs.msg

from launch_ros.actions import LifecycleNode

from launch import LaunchDescription
from launch.actions import (EmitEvent, RegisterEventHandler)
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    driver = LifecycleNode(
        name='rt_usb_9axisimu_driver',
        namespace='',
        package='rt_usb_9axisimu_driver',
        executable='rt_usb_9axisimu_driver',
        output='screen'
    )

    return LaunchDescription([
      driver,
      # Transition state unconfigured -> configured
      RegisterEventHandler(
          OnProcessStart(
              target_action=driver,  # Target node
              on_start=[
                  EmitEvent(
                      event=launch_ros.events.lifecycle.ChangeState(
                          lifecycle_node_matcher=launch.events.matches_action(
                              driver),
                          transition_id=(lifecycle_msgs.msg.Transition.
                                         TRANSITION_CONFIGURE),
                      )
                  ),
              ],
          )
      ),
      RegisterEventHandler(
          launch_ros.event_handlers.OnStateTransition(
              target_lifecycle_node=driver,
              start_state='configuring',
              goal_state='inactive',
              entities=[
                  EmitEvent(
                      event=launch_ros.events.lifecycle.ChangeState(
                          lifecycle_node_matcher=launch.events.matches_action(
                              driver),
                          transition_id=(lifecycle_msgs.msg.Transition.
                                         TRANSITION_ACTIVATE),
                      )
                  )
              ]
          )
      )
    ])
