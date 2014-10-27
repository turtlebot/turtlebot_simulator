#!/usr/bin/env python

import rospy

import world_canvas_client

if __name__ == '__main__':
    rospy.init_node('world_canvas_client')

    world = rospy.get_param('~world')

    # Get the 2D map for the given world
    map_ac = world_canvas_client.AnnotationCollection(world, types=['nav_msgs/OccupancyGrid'])
    map_ac.load_data()

    # Publish the map on server side; topic type is get from the annotation info
    map_ac.publish('/map', None, True, False)   # i.e. by_server=True, as_list=False

    # Get all walls for the given world
    walls_ac = world_canvas_client.AnnotationCollection(world, types=['yocs_msgs/Wall'])
    walls_ac.load_data()

    # Publish annotations' visual markers on client side
    walls_ac.publish_markers('wall_marker')

    # Publish annotations on client side
    walls_ac.publish('wall_pose_list', 'yocs_msgs/WallList', by_server=False, as_list=True)

    # Request server to also publish the same annotations, just for verification
    walls_ac.publish('wall_pose_list_server', 'yocs_msgs/WallList', by_server=True, as_list=True)


    # Get all columns for the given world and repeat the same operations
    columns_ac = world_canvas_client.AnnotationCollection(world, types=['yocs_msgs/Column'])
    columns_ac.load_data()

    # Publish annotations' visual markers on client side
    columns_ac.publish_markers('column_marker')

    # Publish annotations on client side
    columns_ac.publish('column_pose_list', 'yocs_msgs/ColumnList', by_server=False, as_list=True)

    # Request server to also publish the same annotations, just for verification
    columns_ac.publish('column_pose_list_server', 'yocs_msgs/ColumnList', by_server=True, as_list=True)

    rospy.loginfo("Done")
    rospy.spin()
