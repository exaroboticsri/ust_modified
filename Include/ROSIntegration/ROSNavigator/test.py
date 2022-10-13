#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from robot_navigator import BasicNavigator, NavigationResult

# Shelf positions for picking
shelf_positions = {
    "Door": [1., -2.]
    }

# Shipping destination for picked products
shipping_destinations = {
    "yh": [0., 0.],
    "sy": [0., 0.],
    "table1": [1., 2.],
    "table2": [3., 7.]
    }


def main():
    ####################
    request_item_location = 'Door'
    request_destination1 = 'table1'
    request_destination2 = 'table2'
    ####################

    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.
    initial_pose.pose.position.y = 0.
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    # initial_pose.pose.orientation = navigator.quaternion_from_euler(0,0,0)

    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = 1.0
    shelf_item_pose.pose.orientation.w = 0.0
    #shelf_item_pose.pose.orientation = navigator.quaternion_from_euler(0,0,0)

    print('Received request for item picking at ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)

    # Do something during our route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Simply print information for workers on the robot's ETA for the demonstation
    i = 0
    while not navigator.isNavComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for worker: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == NavigationResult.SUCCEEDED:
        print('NAVIGATION SUCCEEDED')
        print('Got product from ' + request_item_location +
              '! Bringing product to shipping destination (' + request_destination1 + ')...')
        shipping_destination = PoseStamped()
        shipping_destination.header.frame_id = 'map'
        shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
        shipping_destination.pose.position.x = shipping_destinations[request_destination1][0]
        shipping_destination.pose.position.y = shipping_destinations[request_destination1][1]
        shipping_destination.pose.orientation.z = 1.0
        shipping_destination.pose.orientation.w = 0.0
        #shipping_destination.pose.orientation = navigator.quaternion_from_euler(0,0,0)
        navigator.goToPose(shipping_destination)

    elif result == NavigationResult.CANCELED:
        print('Task at ' + request_item_location + ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == NavigationResult.FAILED:
        print('NAVIGATION FAILED')
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)

    result = navigator.getResult()
    if result == NavigationResult.SUCCEEDED:
        print('NAVIGATION SUCCEEDED')
        print('Got product from ' + request_item_location +
              '! Bringing product to shipping destination (' + request_destination2 + ')...')
        shipping_destination = PoseStamped()
        shipping_destination.header.frame_id = 'map'
        shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
        shipping_destination.pose.position.x = shipping_destinations[request_destination2][0]
        shipping_destination.pose.position.y = shipping_destinations[request_destination2][1]
        shipping_destination.pose.orientation.z = 1.0
        shipping_destination.pose.orientation.w = 0.0
        #shipping_destination.pose.orientation = navigator.quaternion_from_euler(0,0,0)
        navigator.goToPose(shipping_destination)

    elif result == NavigationResult.CANCELED:
        print('Task at ' + request_item_location + ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == NavigationResult.FAILED:
        print('NAVIGATION FAILED')
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)
    while not navigator.isNavComplete():
        pass

    print(result)
    exit(0)

if __name__ == '__main__':
    main()