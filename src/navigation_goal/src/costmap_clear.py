#!/usr/bin/env python

import rospy

from std_srvs.srv import Empty

def main():
    r = rospy.Rate(1.0/30.0)
    while not rospy.is_shutdown():

        rospy.wait_for_service('/move_base/clear_costmaps')
        try:
            rospy.loginfo("Clearing costmaps")
            clear_cost = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
            clear_cost()
        except rospy.ServiceException as e:
            print("Service call to clear costmaps failed: %s" %e)
        
        r.sleep()

if __name__ == "__main__":
    rospy.init_node('costmap_clear')
    main()
    rospy.spin()


