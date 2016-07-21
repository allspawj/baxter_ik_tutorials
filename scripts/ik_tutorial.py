#!/usr/bin/env python

import rospy
import tf
import struct
from geometry_msgs.msg import (
        PoseStamped,
        Pose,
        Point,
        Quaternion
        )

from std_msgs.msg import Header
import time
from baxter_core_msgs.srv import (
        SolvePositionIK,
        SolvePositionIKRequest,
        )
from baxter_interface import Limb

def solve_ik():
    right_arm = Limb('right') #initialize baxter hand to control
    iksvc = rospy.ServiceProxy("ExternalTools/right/PositionKinematicsNode/IKService", SolvePositionIK) #setup our service call
    ikreq = SolvePositionIKRequest()
    
     #create our message. If you leave the orientation as is and only change the position=Point(0,0,0) line you can move the arm around
    hdr = Header(stamp = rospy.Time.now(), frame_id='right_hand') #notice frame_id='right_hand', this means our position and rotation will be relative to the current right hand position, rather than relative to the global world frame often used. 
    pose = PoseStamped(header=hdr,
            pose=Pose(
                position=Point( 0.0, 0.0, 0.1
                    ),orientation=Quaternion(0,0,0,1)))
    ikreq.pose_stamp.append(pose)
    try:
        rospy.wait_for_service("ExternalTools/right/PositionKinematicsNode/IKService",3.0)
        resp = iksvc(ikreq) #make the service call and handle any errors
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("service call failed: %s" % (e,))
        return 1
        
    #most of this is taken directly from the iksample provided by rethink, its largely unneccessary but I'll leave it in for output
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp
        for i in range(0,30):#set the positions of the arm to our new goal positions. If you set_joint_positions, it only activates for a very short period(much less than a second) so in order to move where you want you need to repeatedly send the same goal position. For our other stuff we use a limbMover thing that manages this, but for simplicity I just threw it in a for loop and assume I will be down within 3s.
            right_arm.set_joint_positions(limb_joints)
            time.sleep(0.1)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
    

if __name__ == '__main__':
    rospy.init_node("sample_ik_mover")
    solve_ik()
