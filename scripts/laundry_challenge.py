#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ROS library
import time
import logging
import rospy
import smach
import smach_ros
import math
import tf
import tf2_ros
import traceback

import hsrb_interface

#Time initialization in ROS
rospy.init_node('laundry_challenge')

from common import speech
from common import action_robot
from common.wait_hand_pushed import WaitHandPushed
from common.wait_door_open import WaitDoorOpen
from common.yolo_commands import restart_yolo, kill_yolo

from hsr_vision.simple_object_tf import MoveHand2TF, DetectSpaceOnPlaneTF, FindMiddlePointTF, FindPointTF, static_tf_publish
from hsrb_interface import geometry as hsr_geo
from hsr_vision.modules import FunctionParams, MyDetectionV8

import cv2
import numpy as np
from cv_bridge import CvBridge
import geometry_msgs.msg
from sensor_msgs.msg import CameraInfo

from navigation_tools.nav_tool_lib import nav_module
import dynamic_reconfigure.client as reconf_client

# GPSR info
from gpsr_modules.general_states import MyObject, location_obj
from gpsr_modules import gpsr_info

##################################################

# Main
robot = hsrb_interface.Robot()

whole_body = robot.get("whole_body")
omni_base = nav_module("pumas")
gripper = robot.get('gripper')
tf_buffer = robot._get_tf2_buffer()

a_robot = action_robot.RobotWithAction()

collision = robot.get('global_collision_world')

# OBSTACLE RECONFIGURE SETTINGS
reconf_laser_obstacle_enable = reconf_client.Client(
    '/tmc_map_merger/inputs/base_scan')
reconf_depth_obstacle_enable = reconf_client.Client(
    '/tmc_map_merger/inputs/head_rgbd_sensor')
# disable xtion obstacle avoidance
reconf_depth_obstacle_enable.update_configuration({"enable": False})

default_tts = speech.DefaultTTS()
console = speech.Console()
SAY = default_tts.say

####DICTIONARIES
room_info = gpsr_info.load_room_info(location="wrsarena")

# Load task locations
loc_washing_machine = location_obj("washing_machine", room_info)
loc_laundry_basket_left   = location_obj("laundry_basket_left",  room_info)
loc_laundry_basket_right   = location_obj("laundry_basket_right",  room_info)
loc_folding_table    = location_obj("folding_table",   room_info)
####

# Fixed hand pose for reaching into the washing machine drum (map frame)
# Format: [[x, y, z], [qx, qy, qz, qw]]
# TODO: calibrate with real robot
basket_hand_pose = [[0.325, -3.322, 0.215], [0.058, 0.958, -0.001, -0.282]]
washing_machine_hand_pose = [[0.065, -2.815, 0.518], [-0.058, -0.856, 0.011, 0.514]]

# Variables
_TASKTIMEOUT     = 100000
_NAVTIMEOUT      = 60.0

_HIST_DIFF_THRESHOLD = 0.40   # Bhattacharyya distance above which two different items are detected


#Start
whole_body.move_to_neutral()
collision.remove_all()

rospy.loginfo('initializing...')
rospy.sleep(3)

restart_yolo(is_yolo=True, is_docker=True, weights="yolo11s-fashionpedia.pt", conf_thres=0.30)
rospy.on_shutdown(lambda: kill_yolo(is_yolo=True, is_docker=True))
_yolo = MyDetectionV8()
_bridge = CvBridge()
_cam_info = rospy.wait_for_message(
    '/hsrb/head_rgbd_sensor/rgb/camera_info', CameraInfo, timeout=10.0)

# FashionPedia YOLO11 classes that can appear in a laundry basket/washing machine
_LAUNDRY_CLASSES = {
    'shirt/blouse', 'top/t-shirt/sweatshirt', 'sweater', 'cardigan',
    'jacket', 'vest', 'pants', 'shorts', 'skirt', 'coat', 'dress',
    'jumpsuit', 'cape', 'tights/stockings', 'leg warmer', 'sock',
}

def get_clothing_histogram(yolo_det, bridge, label='', max_retries=5, retry_delay=0.4):
    """Capture frames until a clothing item is detected, return a normalised HSV histogram.
    Retries up to max_retries times with retry_delay seconds between attempts.
    Within each frame the largest-area laundry detection is used.
    bb_list format: [Xmin, Xmax, Ymin, Ymax]
    Returns None if no clothing is detected after all retries."""
    prefix = '[{}] '.format(label) if label else ''

    for attempt in range(1, max_retries + 1):
        _, color_msg, _, bb_list, obj_list, _ = yolo_det.detection()

        if not bb_list:
            rospy.loginfo('{}attempt {}/{}: no detections'.format(prefix, attempt, max_retries))
            rospy.sleep(retry_delay)
            continue

        rospy.loginfo('{}attempt {}/{}: {} detections'.format(prefix, attempt, max_retries, len(bb_list)))
        clothing_idx = []
        for i, obj in enumerate(obj_list):
            cls = obj[0][1]
            conf = obj[0][2]
            bb = bb_list[i]
            area = (bb[1] - bb[0]) * (bb[3] - bb[2])
            rospy.loginfo('{}  det[{}] class={} conf={:.2f} bb=[{},{},{},{}] area={}'.format(
                prefix, i, cls, conf, bb[0], bb[1], bb[2], bb[3], area))
            if cls in _LAUNDRY_CLASSES:
                clothing_idx.append(i)

        if not clothing_idx:
            rospy.loginfo('{}attempt {}/{}: no laundry-class items'.format(prefix, attempt, max_retries))
            rospy.sleep(retry_delay)
            continue

        best = max(clothing_idx,
                   key=lambda i: (bb_list[i][1] - bb_list[i][0]) * (bb_list[i][3] - bb_list[i][2]))
        xmin, xmax, ymin, ymax = bb_list[best]
        best_area = (xmax - xmin) * (ymax - ymin)
        rospy.loginfo('{}attempt {}/{}: selected det[{}] class={} area={}'.format(
            prefix, attempt, max_retries, best, obj_list[best][0][1], best_area))

        bgr = bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        roi = bgr[ymin:ymax, xmin:xmax]
        if roi.size == 0:
            rospy.logwarn('{}ROI is empty after crop'.format(prefix))
            rospy.sleep(retry_delay)
            continue

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [36, 32], [0, 180, 0, 256])
        cv2.normalize(hist, hist, alpha=1.0, norm_type=cv2.NORM_L1)
        return hist

    rospy.logwarn('{}no clothing detected after {} attempts'.format(prefix, max_retries))
    return None


def _fold_point_px(bb, point_type):
    """Return pixel (u, v) for the fold point within bb = [Xmin, Xmax, Ymin, Ymax].
    Edge points use a larger inward margin (20%) to avoid depth readings from
    the floor below a hanging garment edge. Left/right margins stay at 5%."""
    xmin, xmax, ymin, ymax = bb
    u_c = (xmin + xmax) // 2
    v_c = (ymin + ymax) // 2
    margin_x = int((xmax - xmin) * 0.05)
    margin_y = int((ymax - ymin) * 0.10)
    if point_type == 'bottom_center':
        return u_c, ymax - margin_y
    elif point_type == 'center_left':
        return xmin + margin_x, v_c
    elif point_type == 'right_center':
        return xmax - margin_x, v_c
    return u_c, v_c


def _px_to_world(u, v, depth_img, cam_info, tf_buf, target_frame='odom', min_z=0.20):
    """Project pixel (u, v) + depth image to a PointStamped in target_frame.
    Depth image may be uint16 (mm) or float32 (m) — both are handled.
    Searches an expanding patch (up to radius 10) for the nearest pixel whose
    depth is valid (non-zero, finite) AND whose projected world z >= min_z.
    The z check rejects floor-level readings from hanging garment edges.
    Returns None if no valid candidate is found."""
    K = cam_info.K
    h, w = depth_img.shape[:2]

    def _project(pu, pv, pd):
        if pd > 100.0:          # uint16 mm → float m
            pd /= 1000.0
        pt = geometry_msgs.msg.PointStamped()
        pt.header.frame_id = cam_info.header.frame_id
        pt.header.stamp = rospy.Time(0)
        pt.point.x = (pu - K[2]) * pd / K[0]
        pt.point.y = (pv - K[5]) * pd / K[4]
        pt.point.z = pd
        return tf_buf.transform(pt, target_frame)

    for r in range(0, 11):
        v0, v1 = max(0, v - r), min(h, v + r + 1)
        u0, u1 = max(0, u - r), min(w, u + r + 1)
        patch = depth_img[v0:v1, u0:u1].astype(np.float64)
        valid = (patch != 0.0) & np.isfinite(patch)
        if not valid.any():
            continue
        ys, xs = np.where(valid)
        dist = (ys - (v - v0)) ** 2 + (xs - (u - u0)) ** 2
        for idx in np.argsort(dist):
            pu, pv_ = int(u0 + xs[idx]), int(v0 + ys[idx])
            pd = float(depth_img[pv_, pu])
            pt = _project(pu, pv_, pd)
            if pt.point.z >= min_z:
                return pt
    return None


def create_sm():

    sm = smach.StateMachine(outcomes=['success', 'failure'])

    sm.userdata.start_time = None
    sm.userdata.stop_time  = _TASKTIMEOUT
    sm.userdata.hand_pose  = washing_machine_hand_pose
    sm.userdata.fun_params = FunctionParams()
    sm.userdata.hsr        = 28
    sm.userdata.obj_name   = "unknown"

    with sm:

        ##########
        # START: TASK INITIALISATION
        ##########
        smach.StateMachine.add('WAIT_HAND', WaitHandPushed(timeout=300., 
                                #threshold=-15.0,
                                say_fn=SAY,
                                prompt_msg="Push the hand to start",
                                success_msg=""),
                                transitions={'success': 'WAIT_DOOR', #'FOLDCLOTHES_BASKET', #'WAIT_DOOR', 
                                             'timeout': 'WAIT_HAND', #'FOLDCLOTHES_BASKET', #'WAIT_HAND',
                                             'failure': 'failure'})

        smach.StateMachine.add('WAIT_DOOR', WaitDoorOpen(1.5, timeout=120000.,
                                                         knock_msg="Waiting for the door to be opened.",
                                                         success_msg="Thank you.",
                                                         say_fn=SAY),
                               transitions={'success': 'START',
                                            'timeout': 'failure'})

        @smach.cb_interface(outcomes=['success', 'failure'],
                            input_keys=['start_time'],
                            output_keys=['start_time'])
        def start_cb(userdata):
            try:
                userdata.start_time = time.time()
                try:
                    reconf_laser_obstacle_enable.update_configuration({"enable": False})
                    SAY('I am ready to start the laundry task.')
                    #SAY('I am ready to start the laundry task. Please, open the dishwasher door.')
                    omni_base.go_rel(0.8, 0, 0, _NAVTIMEOUT, "hsr")
                    #omni_base.go_rel(0.0, 0.0, 0.0, 0, "pumas")
                    reconf_laser_obstacle_enable.update_configuration({"enable": True})
                except:
                    traceback.print_exc()
                return 'success'
            except:
                rospy.logerr(traceback.format_exc())
                return 'failure'

        smach.StateMachine.add('START', smach.CBState(start_cb),
                               transitions={'success': 'MOVE2LAUNDRYBASKETLEFT', 
                                            'failure': 'failure'})
        ##########
        # END: TASK INITIALISATION
        ##########


        ##########
        # START: WASHING MACHINE STATES
        ##########
        @smach.cb_interface(outcomes=['success', 'failure'],
                            input_keys=['hand_pose'],
                            output_keys=['hand_pose'])
        def move_to_washing_machine_cb(userdata):
            try:
                _pos = loc_washing_machine.pose
                SAY('I am going to the washing machine.', wait_result=False)
                a_robot.move_arm(0.0, 0.0, -1.57, -1.57, 0.0, False)
                rospy.loginfo("Moving to washing_machine at {}, {}, {}".format(
                        _pos[0], _pos[1], _pos[3]))
                
                userdata.hand_pose  = washing_machine_hand_pose
                try:
                    omni_base.go_abs(_pos[0], _pos[1], _pos[3], _NAVTIMEOUT)
                except:
                    rospy.logerr(traceback.print_exc())
                    omni_base.go_abs(_pos[0], _pos[1], _pos[3], 0, "hsr")
                return 'success'
            except:
                rospy.logerr(traceback.format_exc())
                return 'failure'

        smach.StateMachine.add('MOVE2WASHINGMACHINE', smach.CBState(move_to_washing_machine_cb),
                               transitions={'success': 'TAKECLOTHESFROMWASHING',
                                            'failure': 'failure'})

        smach.StateMachine.add('TAKECLOTHESFROMWASHING',
                               MoveHand2TF(robot, grasp=True, neutral=True),
                               transitions={'success': 'CHECKGRASPINGWASHING',
                                            'failure': 'CHECKGRASPINGWASHING',
                                            'timeout': 'MOVE2STANDBY'})

        @smach.cb_interface(outcomes=['success', 'drop', 'failure'])
        def check_grasping_washing_cb(userdata):
            try:
                try:
                    omni_base.go_rel(-0.15, 0.0, 0.0, _NAVTIMEOUT, "hsr")
                    omni_base.go_rel(-0.15, 0.0, 0.0, _NAVTIMEOUT, "hsr")
                except:
                    rospy.logerr(traceback.print_exc())

                gripper.apply_force(1.0)

                # RIGHT VIEW
                whole_body.move_to_joint_positions({'arm_lift_joint': 0.3,
                                                    'arm_flex_joint': -1.0,
                                                    'arm_roll_joint': -1.57,
                                                    'wrist_flex_joint': -1.57,
                                                    'wrist_roll_joint': 0.0,
                                                    'head_pan_joint': -0.25,
                                                    'head_tilt_joint': -1.0})
                rospy.sleep(2.0)
                hist_right = get_clothing_histogram(_yolo, _bridge, label='RIGHT')

                # LEFT VIEW
                whole_body.move_to_joint_positions({'arm_lift_joint': 0.3,
                                                    'arm_flex_joint': -1.0,
                                                    'arm_roll_joint': 1.57,
                                                    'wrist_flex_joint': -1.57,
                                                    'wrist_roll_joint': 0.0,
                                                    'head_pan_joint': 0.40,
                                                    'head_tilt_joint': -1.0})
                rospy.sleep(2.0)
                hist_left = get_clothing_histogram(_yolo, _bridge, label='LEFT')

                rospy.loginfo('hist_right={} hist_left={}'.format(
                    'OK' if hist_right is not None else 'None',
                    'OK' if hist_left is not None else 'None'))
                if hist_right is None and hist_left is None:
                    SAY('Nothing grasped. Retrying.')
                    return 'drop'
                SAY('Grasping success.')
                return 'success'
            except:
                rospy.logerr(traceback.format_exc())
                return 'failure'

        smach.StateMachine.add('CHECKGRASPINGWASHING', smach.CBState(check_grasping_washing_cb),
                               transitions={'success': 'MOVE2FOLDINGTABLE_WM',
                                            'drop':    'MOVE2WASHINGMACHINE',
                                            'failure': 'failure'})
        ##########
        # END: WASHING MACHINE STATES
        ##########


        ##########
        # START: BASKET STATES
        ##########
        @smach.cb_interface(outcomes=['basket', 'bin', 'failure'],
                            input_keys=['fun_params', 'hand_pose'],
                            output_keys=['fun_params', 'hand_pose'])
        def move_to_laundry_basket_left_cb(userdata):
            try:
                _pos = loc_laundry_basket_left.pose
                SAY('I am going to the laundry basket.', wait_result=False)
                a_robot.move_arm(0.0, 0.0, -1.57, -1.57, 0.0, False)
                rospy.loginfo("Moving to laundry_basket at {}, {}, {}".format(
                        _pos[0], _pos[1], _pos[3]))
                try:
                    omni_base.go_abs(_pos[0], _pos[1], _pos[3], _NAVTIMEOUT)
                except:
                    rospy.logerr(traceback.print_exc())
                    omni_base.go_abs(_pos[0], _pos[1], _pos[3], 0, "hsr")

                userdata.hand_pose  = basket_hand_pose

                # Set vision parameters for GraspUnknownObjectTF
                # TODO: tune height_min/height_max with real robot
                userdata.fun_params.robot_pose.delay       = 1.0
                userdata.fun_params.robot_pose.tilt        = -0.96
                userdata.fun_params.robot_pose.height_diff = 0.25
                userdata.fun_params.robot_pose.wrist_roll  = 0.0
                userdata.fun_params.robot_pose.in_use      = True

                userdata.fun_params.objects_bb.depth_min   = 0.40
                userdata.fun_params.objects_bb.depth_max   = 0.80
                userdata.fun_params.objects_bb.width       = 0.40
                userdata.fun_params.objects_bb.height_min  = _pos[2] - 0.05
                userdata.fun_params.objects_bb.height_max  = _pos[2] + 0.05
                userdata.fun_params.objects_bb.in_use      = True

                userdata.fun_params.points_bb.maxx = 0.20
                userdata.fun_params.points_bb.miny = _pos[2] - 0.05
                userdata.fun_params.points_bb.maxy = _pos[2] + 0.05
                userdata.fun_params.points_bb.maxz = 0.80
                userdata.fun_params.points_bb.in_use = True

                a_robot.move_head(0, userdata.fun_params.robot_pose.tilt)

                if "basket" in loc_laundry_basket_right.name.lower():
                    return 'basket'
                else:
                    return "bin"
            except:
                rospy.logerr(traceback.format_exc())
                return 'failure'

        smach.StateMachine.add('MOVE2LAUNDRYBASKETLEFT', smach.CBState(move_to_laundry_basket_left_cb),
                               transitions={'basket' : 'TAKEBASKETPOINTLEFT',
                                            'bin'    : 'TAKEBINPOINTLEFT',
                                            'failure': 'failure'})

        smach.StateMachine.add('TAKEBASKETPOINTLEFT', 
                               FindMiddlePointTF(robot, 
                                           drift_x=0.15, 
                                           drift_y=0.0, 
                                           drift_z=-0.025, 
                                           delta_flex=-0.96,
                                           take=True,
                                           neutral=True),
                               transitions={'success': 'CHECKGRASPINGBASKETLEFT',
                                            'unknown': 'MOVE2LAUNDRYBASKETRIGHT',
                                            'timeout': 'MOVE2STANDBY',
                                            'failure': 'CHECKGRASPINGBASKETLEFT'})

        smach.StateMachine.add('TAKEBINPOINTLEFT', 
                               FindPointTF(robot, 
                                           drift_x=0.15, 
                                           drift_y=0.0, 
                                           drift_z=-0.025, 
                                           delta_flex=-0.96,
                                           take=True,
                                           neutral=True),
                               transitions={'success': 'CHECKGRASPINGBASKETLEFT',
                                            'unknown': 'MOVE2LAUNDRYBASKETRIGHT',
                                            'timeout': 'MOVE2STANDBY',
                                            'failure': 'CHECKGRASPINGBASKETLEFT'})


        @smach.cb_interface(outcomes=['success', 'drop', 'failure'])
        def check_grasping_basket_left_cb(userdata):
            try:
                gripper.apply_force(1.0)

                # RIGHT VIEW
                whole_body.move_to_joint_positions({'arm_lift_joint': 0.3,
                                                    'arm_flex_joint': -1.0,
                                                    'arm_roll_joint': -1.57,
                                                    'wrist_flex_joint': -1.57,
                                                    'wrist_roll_joint': 0.0,
                                                    'head_pan_joint': -0.25,
                                                    'head_tilt_joint': -1.0})
                rospy.sleep(2.0)
                hist_right = get_clothing_histogram(_yolo, _bridge, label='RIGHT')

                # LEFT VIEW
                whole_body.move_to_joint_positions({'arm_lift_joint': 0.3,
                                                    'arm_flex_joint': -1.0,
                                                    'arm_roll_joint': 1.57,
                                                    'wrist_flex_joint': -1.57,
                                                    'wrist_roll_joint': 0.0,
                                                    'head_pan_joint': 0.40,
                                                    'head_tilt_joint': -1.0})
                rospy.sleep(2.0)
                hist_left = get_clothing_histogram(_yolo, _bridge, label='LEFT')

                rospy.loginfo('hist_right={} hist_left={}'.format(
                    'OK' if hist_right is not None else 'None',
                    'OK' if hist_left is not None else 'None'))
                if hist_right is None and hist_left is None:
                    SAY('Nothing grasped. Retrying.')
                    return 'drop'
                if hist_right is not None and hist_left is not None:
                    diff = cv2.compareHist(hist_right, hist_left, cv2.HISTCMP_BHATTACHARYYA)
                    rospy.loginfo('histogram Bhattacharyya diff: {:.3f} (threshold={})'.format(
                        diff, _HIST_DIFF_THRESHOLD))
                    if diff > _HIST_DIFF_THRESHOLD:
                        SAY('More than one item detected. Releasing extra items.')
                        omni_base.go_rel(0.15, 0.0, 0.0, _NAVTIMEOUT, "hsr")
                        a_robot.move_arm(0.2, -1.57, 0.0, -1.57, 0.0, wait_result=True)
                        for _ in range(2):
                            gripper.command(1.2)
                            rospy.sleep(0.5)
                            gripper.apply_force(0.5)
                            rospy.sleep(0.5)
                        gripper.command(1.2)
                        a_robot.move_arm(0.0, 0.0, -1.57, -1.57, 0.0, wait_result=True)
                        return 'drop'
                SAY('Grasping success.')
                return 'success'
            except:
                rospy.logerr(traceback.format_exc())
                return 'failure'

        smach.StateMachine.add('CHECKGRASPINGBASKETLEFT', smach.CBState(check_grasping_basket_left_cb),
                               transitions={'success': 'MOVE2FOLDINGTABLE',
                                            'drop':    'MOVE2LAUNDRYBASKETRIGHT',
                                            'failure': 'failure'})

        @smach.cb_interface(outcomes=['basket', "bin", 'failure'],
                            input_keys=['fun_params', 'hand_pose'],
                            output_keys=['fun_params', 'hand_pose'])
        def move_to_laundry_basket_right_cb(userdata):
            try:
                _pos = loc_laundry_basket_right.pose
                SAY('I am going to the laundry basket.', wait_result=False)
                a_robot.move_arm(0.0, 0.0, -1.57, -1.57, 0.0, False)
                rospy.loginfo("Moving to laundry_basket at {}, {}, {}".format(
                        _pos[0], _pos[1], _pos[3]))
                try:
                    omni_base.go_abs(_pos[0], _pos[1], _pos[3], _NAVTIMEOUT)
                except:
                    rospy.logerr(traceback.print_exc())
                    omni_base.go_abs(_pos[0], _pos[1], _pos[3], 0, "hsr")

                userdata.hand_pose  = basket_hand_pose

                # Set vision parameters for GraspUnknownObjectTF
                # TODO: tune height_min/height_max with real robot
                userdata.fun_params.robot_pose.delay       = 1.0
                userdata.fun_params.robot_pose.tilt        = -0.96
                userdata.fun_params.robot_pose.height_diff = 0.25
                userdata.fun_params.robot_pose.wrist_roll  = 0.0
                userdata.fun_params.robot_pose.in_use      = True

                userdata.fun_params.objects_bb.depth_min   = 0.40
                userdata.fun_params.objects_bb.depth_max   = 0.80
                userdata.fun_params.objects_bb.width       = 0.40
                userdata.fun_params.objects_bb.height_min  = _pos[2] - 0.05
                userdata.fun_params.objects_bb.height_max  = _pos[2] + 0.05
                userdata.fun_params.objects_bb.in_use      = True

                userdata.fun_params.points_bb.maxx = 0.20
                userdata.fun_params.points_bb.miny = _pos[2] - 0.05
                userdata.fun_params.points_bb.maxy = _pos[2] + 0.05
                userdata.fun_params.points_bb.maxz = 0.80
                userdata.fun_params.points_bb.in_use = True

                a_robot.move_head(0, userdata.fun_params.robot_pose.tilt)

                if "basket" in loc_laundry_basket_right.name.lower():
                    return 'basket'
                else:
                    return "bin"
            except:
                rospy.logerr(traceback.format_exc())
                return 'failure'

        smach.StateMachine.add('MOVE2LAUNDRYBASKETRIGHT', smach.CBState(move_to_laundry_basket_right_cb),
                               transitions={'basket' : 'TAKEBASKETPOINTRIGHT', 
                                            'bin'    : 'TAKEBINPOINTRIGHT',
                                            'failure': 'failure'})

        smach.StateMachine.add('TAKEBASKETPOINTRIGHT', 
                               FindPointTF(robot, 
                                           drift_x=0.15, 
                                           drift_y=0.0, 
                                           drift_z=-0.025, 
                                           delta_flex=-0.96,
                                           take=True,
                                           neutral=True),
                               transitions={'success': 'CHECKGRASPINGBASKETRIGHT',
                                            'unknown': 'MOVE2LAUNDRYBASKETLEFT',
                                            'timeout': 'MOVE2STANDBY',
                                            'failure': 'CHECKGRASPINGBASKETRIGHT'})

        smach.StateMachine.add('TAKEBINPOINTRIGHT', 
                               FindPointTF(robot, 
                                           drift_x=0.15, 
                                           drift_y=0.0, 
                                           drift_z=-0.025, 
                                           delta_flex=-0.96,
                                           take=True,
                                           neutral=True),
                               transitions={'success': 'CHECKGRASPINGBASKETRIGHT',
                                            'unknown': 'MOVE2LAUNDRYBASKETLEFT',
                                            'timeout': 'MOVE2STANDBY',
                                            'failure': 'CHECKGRASPINGBASKETRIGHT'})

        @smach.cb_interface(outcomes=['success', 'drop', 'failure'])
        def check_grasping_basket_right_cb(userdata):
            try:
                gripper.apply_force(1.0)

                # RIGHT VIEW
                whole_body.move_to_joint_positions({'arm_lift_joint': 0.3,
                                                    'arm_flex_joint': -1.0, 
                                                    'arm_roll_joint': -1.57, 
                                                    'wrist_flex_joint': -1.57, 
                                                    'wrist_roll_joint': 0.0, 
                                                    'head_pan_joint': -0.25, 
                                                    'head_tilt_joint': -1.0})
                rospy.sleep(2.0)
                hist_right = get_clothing_histogram(_yolo, _bridge, label='RIGHT')

                # LEFT VIEW
                whole_body.move_to_joint_positions({'arm_lift_joint': 0.3,
                                                    'arm_flex_joint': -1.0, 
                                                    'arm_roll_joint': 1.57, 
                                                    'wrist_flex_joint': -1.57, 
                                                    'wrist_roll_joint': 0.0, 
                                                    'head_pan_joint': 0.40, 
                                                    'head_tilt_joint': -1.0})
                rospy.sleep(2.0)
                hist_left = get_clothing_histogram(_yolo, _bridge, label='LEFT')

                rospy.loginfo('hist_right={} hist_left={}'.format(
                    'OK' if hist_right is not None else 'None',
                    'OK' if hist_left is not None else 'None'))
                if hist_right is None and hist_left is None:
                    SAY('Nothing grasped. Retrying.')
                    return 'drop'
                if hist_right is not None and hist_left is not None:
                    diff = cv2.compareHist(hist_right, hist_left, cv2.HISTCMP_BHATTACHARYYA)
                    rospy.loginfo('histogram Bhattacharyya diff: {:.3f} (threshold={})'.format(
                        diff, _HIST_DIFF_THRESHOLD))
                    if diff > _HIST_DIFF_THRESHOLD:
                        SAY('More than one item detected. Releasing extra items.')
                        omni_base.go_rel(0.15, 0.0, 0.0, _NAVTIMEOUT, "hsr")
                        a_robot.move_arm(0.2, -1.57, 0.0, -1.57, 0.0, wait_result=True)
                        for _ in range(2):
                            gripper.command(1.2)
                            rospy.sleep(0.5)
                            gripper.apply_force(0.5)
                            rospy.sleep(0.5)
                        gripper.command(1.2)
                        a_robot.move_arm(0.0, 0.0, -1.57, -1.57, 0.0, wait_result=True)
                        return 'drop'
                SAY('Grasping success.')
                return 'success'
            except:
                rospy.logerr(traceback.format_exc())
                return 'failure'

        smach.StateMachine.add('CHECKGRASPINGBASKETRIGHT', smach.CBState(check_grasping_basket_right_cb),
                               transitions={'success': 'MOVE2FOLDINGTABLE', 
                                            'drop':    'MOVE2LAUNDRYBASKETRIGHT',
                                            'failure': 'failure'})
        ##########
        # END: BASKET STATES
        ##########


        ##########
        # START: FOLDING STATES
        ##########
        @smach.cb_interface(outcomes=['success', 'failure'],
                            input_keys=['fun_params'],
                            output_keys=['fun_params'])
        def move_to_folding_table_cb(userdata):
            try:
                _pos = loc_folding_table.pose
                SAY('I am going to the folding table.', wait_result=False)
                a_robot.move_arm(0.3, -0.30, 1.57, -1.57, 0.0, wait_result=True)

                rospy.loginfo("Moving to folding_table at {}, {}, {}".format(
                        _pos[0], _pos[1], _pos[3]))
                try:
                    omni_base.go_abs(_pos[0], _pos[1], _pos[3], _NAVTIMEOUT, "pumas", hold_pose=True)
                except:
                    rospy.logerr(traceback.print_exc())
                    omni_base.go_abs(_pos[0], _pos[1], _pos[3], 0, "pumas", hold_pose=True)

                # Set vision parameters for DetectSpaceOnPlaneTF
                # TODO: tune with real robot
                userdata.fun_params.robot_pose.tilt        = -0.76
                userdata.fun_params.robot_pose.height_diff = 0.35
                userdata.fun_params.robot_pose.in_use      = True

                userdata.fun_params.objects_bb.depth_min   = 0.40
                userdata.fun_params.objects_bb.depth_max   = 1.20
                userdata.fun_params.objects_bb.width       = 0.80
                userdata.fun_params.objects_bb.height_min  = _pos[2]
                userdata.fun_params.objects_bb.height_max  = _pos[2] + 0.20
                userdata.fun_params.objects_bb.in_use      = True

                if (_pos[2] > 0.695 and _pos[2] - 0.695 > 0.20):
                    userdata.fun_params.robot_pose.height_diff = 0.45
                    userdata.fun_params.robot_pose.tilt = -0.15
                elif (0.695 - _pos[2] < 0.10):
                    userdata.fun_params.robot_pose.height_diff = 0.25
                    userdata.fun_params.robot_pose.tilt = -0.45

                return 'success'
            except:
                rospy.logerr(traceback.format_exc())
                return 'failure'

        # Shared navigation callback; transitions differ to route basket vs washing-machine paths
        smach.StateMachine.add('MOVE2FOLDINGTABLE', smach.CBState(move_to_folding_table_cb),
                               transitions={'success': 'DEPOSIT2SURFACETF_BASKET',
                                            'failure': 'failure'})

        smach.StateMachine.add('MOVE2FOLDINGTABLE_WM', smach.CBState(move_to_folding_table_cb),
                               transitions={'success': 'DEPOSIT2SURFACETF_WM',
                                            'failure': 'failure'})


        smach.StateMachine.add('DEPOSIT2SURFACETF_BASKET',
                               DetectSpaceOnPlaneTF(robot,
                                                   drift_x=-0.05, drift_y=0.0, drift_z=0.10, side=0,
                                                   wside=0.20, dside=0.20,
                                                   split_front=True, back=True,
                                                   say_fn=SAY),
                               transitions={'success': 'FOLDCLOTHES_BASKET',
                                            'unknown': 'CHECKGRASPINGERROR_BASKET',
                                            'failure': 'CHECKGRASPINGERROR_BASKET',
                                            'timeout': 'MOVE2STANDBY'})

        smach.StateMachine.add('DEPOSIT2SURFACETF_WM',
                               DetectSpaceOnPlaneTF(robot,
                                                   drift_x=-0.05, drift_y=0.0, drift_z=0.10, side=0,
                                                   wside=0.20, dside=0.20,
                                                   split_front=True, back=True,
                                                   say_fn=SAY),
                               transitions={'success': 'FOLDCLOTHES_WM',
                                            'unknown': 'CHECKGRASPINGERROR_WM',
                                            'failure': 'CHECKGRASPINGERROR_WM',
                                            'timeout': 'MOVE2STANDBY'})

        @smach.cb_interface(outcomes=['success', 'repeat', 'failure'])
        def check_grasping_error_basket_cb(userdata):
            try:
                try:
                    omni_base.go_rel(-0.15, 0.0, 0.0, _NAVTIMEOUT, "hsr")
                    omni_base.go_rel(-0.15, 0.0, 0.0, _NAVTIMEOUT, "hsr")
                except:
                    rospy.logerr(traceback.print_exc())

                gripper.apply_force(1.0)

                # RIGHT VIEW
                whole_body.move_to_joint_positions({'arm_lift_joint': 0.3,
                                                    'arm_flex_joint': -1.0,
                                                    'arm_roll_joint': -1.57,
                                                    'wrist_flex_joint': -1.57,
                                                    'wrist_roll_joint': 0.0,
                                                    'head_pan_joint': -0.25,
                                                    'head_tilt_joint': -1.0})
                rospy.sleep(2.0)
                hist_right = get_clothing_histogram(_yolo, _bridge, label='RIGHT')

                # LEFT VIEW
                whole_body.move_to_joint_positions({'arm_lift_joint': 0.3,
                                                    'arm_flex_joint': -1.0,
                                                    'arm_roll_joint': 1.57,
                                                    'wrist_flex_joint': -1.57,
                                                    'wrist_roll_joint': 0.0,
                                                    'head_pan_joint': 0.40,
                                                    'head_tilt_joint': -1.0})
                rospy.sleep(2.0)
                hist_left = get_clothing_histogram(_yolo, _bridge, label='LEFT')

                rospy.loginfo('error_basket hist_right={} hist_left={}'.format(
                    'OK' if hist_right is not None else 'None',
                    'OK' if hist_left is not None else 'None'))
                if hist_right is None and hist_left is None:
                    return 'success'  # dropped it → go get more clothes from basket
                else:
                    return 'repeat'   # still holding cloth → retry deposit
            except:
                rospy.logerr(traceback.format_exc())
                return 'failure'

        smach.StateMachine.add('CHECKGRASPINGERROR_BASKET', smach.CBState(check_grasping_error_basket_cb),
                               transitions={'success': 'MOVE2LAUNDRYBASKETLEFT',
                                            'repeat':  'MOVE2FOLDINGTABLE',
                                            'failure': 'failure'})

        @smach.cb_interface(outcomes=['success', 'repeat', 'failure'])
        def check_grasping_error_wm_cb(userdata):
            try:
                try:
                    omni_base.go_rel(-0.15, 0.0, 0.0, _NAVTIMEOUT, "hsr")
                    omni_base.go_rel(-0.15, 0.0, 0.0, _NAVTIMEOUT, "hsr")
                except:
                    rospy.logerr(traceback.print_exc())

                gripper.apply_force(1.0)

                # RIGHT VIEW
                whole_body.move_to_joint_positions({'arm_lift_joint': 0.3,
                                                    'arm_flex_joint': -1.0,
                                                    'arm_roll_joint': -1.57,
                                                    'wrist_flex_joint': -1.57,
                                                    'wrist_roll_joint': 0.0,
                                                    'head_pan_joint': -0.25,
                                                    'head_tilt_joint': -1.0})
                rospy.sleep(2.0)
                hist_right = get_clothing_histogram(_yolo, _bridge, label='RIGHT')

                # LEFT VIEW
                whole_body.move_to_joint_positions({'arm_lift_joint': 0.3,
                                                    'arm_flex_joint': -1.0,
                                                    'arm_roll_joint': 1.57,
                                                    'wrist_flex_joint': -1.57,
                                                    'wrist_roll_joint': 0.0,
                                                    'head_pan_joint': 0.40,
                                                    'head_tilt_joint': -1.0})
                rospy.sleep(2.0)
                hist_left = get_clothing_histogram(_yolo, _bridge, label='LEFT')

                rospy.loginfo('error_wm hist_right={} hist_left={}'.format(
                    'OK' if hist_right is not None else 'None',
                    'OK' if hist_left is not None else 'None'))
                if hist_right is None and hist_left is None:
                    return 'success'  # dropped it → go get more clothes from washing machine
                else:
                    return 'repeat'   # still holding cloth → retry deposit
            except:
                rospy.logerr(traceback.format_exc())
                return 'failure'

        smach.StateMachine.add('CHECKGRASPINGERROR_WM', smach.CBState(check_grasping_error_wm_cb),
                               transitions={'success': 'MOVE2WASHINGMACHINE',
                                            'repeat':  'MOVE2FOLDINGTABLE_WM',
                                            'failure': 'failure'})

        @smach.cb_interface(outcomes=['success', 'failure'])
        def fold_clothes_cb(userdata):
            try:
                SAY('Please lay the shirt flat on the table.')
                # TODO: replace sleep with inline WaitHan     

dPushed when ready
                rospy.sleep(10.0)

                _pos = loc_folding_table.pose
                _drift      = 0.06
                _delta_h    = 0.02
                _delta_flex = -0.96

                # Three fold strokes — deltas in head_pan_link frame:
                #   x = forward (toward table), y = left, z = up
                # TODO: calibrate deltas for actual robot/table orientation
                fold_steps = [
                    ('bottom_center', ( 0.15,  0.00,  0.00)),  # fold bottom hem toward robot
                    ('center_left',   ( 0.00, -0.15,  0.00)),  # fold left flap to center
                    ('right_center',  ( 0.00,  0.15,  0.00)),  # fold right flap to center
                ]

                _surface = None  # collision box created on first detection, removed after loop

                for point_type, (dx, dy, dz) in fold_steps:
                    # Return to canonical folding-table position, re-rotate, and re-set arm/head
                    omni_base.go_abs(_pos[0], _pos[1], _pos[3], _NAVTIMEOUT, "hsr")
                    omni_base.go_rel(0.0, 0.0, 0.6, _NAVTIMEOUT, "hsr")
                    whole_body.move_to_joint_positions({
                        'arm_lift_joint': 0.4,
                        'arm_flex_joint': -0.6,
                        'arm_roll_joint': 1.57,
                        'wrist_flex_joint': -1.57,
                        'wrist_roll_joint': 0.0,
                        'head_pan_joint': -0.6,
                        'head_tilt_joint': -0.96})
                    rospy.sleep(1.0)

                    # Recompute TF after repositioning (head_pan_link changes with head joints)
                    _pan_listener = tf.TransformListener()
                    _pan_listener.waitForTransform('map', 'head_pan_link', rospy.Time(0), rospy.Duration(3.0))
                    (head_trans, _rot) = _pan_listener.lookupTransform('map', 'head_pan_link', rospy.Time(0))
                    _q = tf.transformations.quaternion_multiply(
                        _rot, tf.transformations.quaternion_from_euler(0, 0, math.pi))
                    _q = tf.transformations.quaternion_multiply(
                        _q, tf.transformations.quaternion_from_euler(0, -math.pi / 2, 0))
                    _rot_mat = tf.transformations.quaternion_matrix(list(_rot))[:3, :3]
                    _, _, depth_msg, bb_list, obj_list, _ = _yolo.detection()
                    clothing_idx = [i for i, obj in enumerate(obj_list)
                                    if obj[0][1] in _LAUNDRY_CLASSES]
                    if not clothing_idx:
                        rospy.logwarn('fold: no clothing detected for {} step'.format(point_type))
                        continue

                    best = max(clothing_idx,
                               key=lambda i: (bb_list[i][1]-bb_list[i][0])*(bb_list[i][3]-bb_list[i][2]))
                    u, v = _fold_point_px(bb_list[best], point_type)
                    depth_img = _bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

                    pt = _px_to_world(u, v, depth_img, _cam_info, tf_buffer, target_frame='map')
                    if pt is None:
                        rospy.logwarn('fold: invalid depth for {} at ({},{})'.format(point_type, u, v))
                        continue

                    # If the projected point is below the table surface the pixel came from
                    # the hanging part of the shirt — snap z to the known table height.
                    if pt.point.z < loc_folding_table.pose[2] - 0.01:
                        rospy.loginfo(
                            'fold {}: z {:.3f} < table {:.3f}, shirt hanging — clamping'.format(
                                point_type, pt.point.z, loc_folding_table.pose[2]))
                    pt.point.z = loc_folding_table.pose[2]

                    rospy.loginfo('fold {}: world ({:.3f},{:.3f},{:.3f})'.format(
                        point_type, pt.point.x, pt.point.y, pt.point.z))

                    _px = pt.point.x
                    _py = pt.point.y
                    _pz = pt.point.z

                    # Rotate stroke delta from head_pan_link frame into map frame
                    _d = _rot_mat.dot(np.array([dx, dy, dz]))

                    # --- Table near-edge detection for collision box (first step only) ---
                    if _surface is None:
                        # Scan bbox centre column from bottom (large v = nearest) upward;
                        # first pixel projecting to z >= table_height - 0.10 is the near edge.
                        _table_height    = loc_folding_table.pose[2]
                        _min_z_table     = _table_height - 0.10
                        _K               = _cam_info.K
                        _h_img           = depth_img.shape[0]
                        _xmin_b, _xmax_b, _ymin_b, _ymax_b = bb_list[best]
                        _u_scan          = (_xmin_b + _xmax_b) // 2
                        _table_near_edge = None

                        for _v_scan in range(min(_ymax_b, _h_img - 1), max(_ymin_b - 1, 0), -5):
                            _d_raw = float(depth_img[_v_scan, _u_scan])
                            if _d_raw == 0.0 or not np.isfinite(_d_raw):
                                continue
                            if _d_raw > 100.0:
                                _d_raw /= 1000.0
                            _scan_pt = geometry_msgs.msg.PointStamped()
                            _scan_pt.header.frame_id = _cam_info.header.frame_id
                            _scan_pt.header.stamp    = rospy.Time(0)
                            _scan_pt.point.x = (_u_scan - _K[2]) * _d_raw / _K[0]
                            _scan_pt.point.y = (_v_scan - _K[5]) * _d_raw / _K[4]
                            _scan_pt.point.z = _d_raw
                            _scan_pt_map = tf_buffer.transform(_scan_pt, 'map')
                            if _scan_pt_map.point.z >= _min_z_table:
                                _table_near_edge = _scan_pt_map
                                break

                        if _table_near_edge is None:
                            rospy.logwarn('fold {}: table near edge not found, skipping collision box'.format(point_type))
                        else:
                            _box_z_size = _table_height - 0.05
                            _s_depth, _s_width = 1.0, 1.5

                            _edge_pos = np.array([_table_near_edge.point.x,
                                                  _table_near_edge.point.y,
                                                  _table_near_edge.point.z])
                            _head_x   = _rot_mat[:, 0]
                            _d_edge   = float(np.dot(_edge_pos - np.array(head_trans), _head_x))
                            _DELTA    = 0.075  # shift front face toward robot for safe clearance

                            _box_local = np.array([_d_edge - _DELTA + _s_depth / 2.0,
                                                   0.0,
                                                   _box_z_size / 2.0 - head_trans[2]])
                            _box_map   = np.array(head_trans) + _rot_mat.dot(_box_local)
                            _bi, _bj, _bk = tf.transformations.euler_from_quaternion(list(_rot))

                            _surface = collision.add_box(
                                x=_s_depth, y=_s_width, z=_box_z_size,
                                pose=hsr_geo.pose(x=_box_map[0], y=_box_map[1], z=_box_map[2],
                                                  ei=_bi, ej=_bj, ek=_bk),
                                frame_id='map')
                            rospy.loginfo('fold: table edge d={:.3f} m, box map ({:.3f},{:.3f},{:.3f})'.format(
                                _d_edge, _box_map[0], _box_map[1], _box_map[2]))

                    # Publish 3 frames at once: pre-hover, contact, stroke endpoint
                    static_tf_publish(
                        [
                            [_px, _py, _pz + _drift + _delta_h + 0.175],                        # target_object_0 pre-hover
                            [_px, _py, _pz + _drift + _delta_h],                                # target_object_1 contact
                            [_px - _d[0], _py - _d[1], _pz + _drift + _delta_h + _d[2] + 0.10], # target_object_2 stroke pull
                            [_px + _d[0], _py + _d[1], _pz + _drift + _delta_h + _d[2] + 0.10], # target_object_3 stroke pre-endpoint
                            [_px + _d[0], _py + _d[1], _pz + _drift + _delta_h + _d[2]],        # target_object_4 stroke endpoint
                        ],
                        [list(_q), list(_q), list(_q), list(_q), list(_q)]
                    )

                    # Move end effector to target_object_0 frame
                    linear_weight = whole_body.linear_weight
                    angular_weight = whole_body.angular_weight
                    
                    whole_body.linear_weight = 10
                    whole_body.angular_weight = 10
                    whole_body.end_effector_frame = u'hand_palm_link'

                    if _surface is not None:
                        whole_body.collision_world = collision

                    gripper.command(1.2)
                    try:
                        gripper.command(1.2) # open before approach
                    except hsrb_interface.exceptions.GripperError:
                        rospy.logwarn('fold {}: gripper release against surface — continuing'.format(point_type))

                    # Step 1: pre-hover
                    whole_body.move_end_effector_pose(hsr_geo.pose(), 'target_object_0')

                    # Step 2: contact + wrist flex
                    whole_body.move_end_effector_pose(
                        hsr_geo.pose(ej=_delta_flex), 'target_object_1')

                    gripper.apply_force(1.0)  # grasp shirt

                    # Step 3: pull back the tshit (gripper closed, dragging shirt)
                    whole_body.move_end_effector_pose(
                        hsr_geo.pose(ej=_delta_flex), 'target_object_2')

                    # Step 4: fold stroke above surface (gripper closed, dragging shirt)
                    whole_body.move_end_effector_pose(
                        hsr_geo.pose(ej=_delta_flex), 'target_object_3')

                    # Step 5: fold stroke contact to surface (gripper closed, dragging shirt)
                    whole_body.move_end_effector_pose(
                        hsr_geo.pose(ej=_delta_flex), 'target_object_4')

                    # Release in place. GripperError can occur when fabric resists full
                    # opening against the table — gripper still opens partially, so continue.
                    try:
                        gripper.command(0.8)
                    except hsrb_interface.exceptions.GripperError:
                        rospy.logwarn('fold {}: gripper release against surface — continuing'.format(point_type))

                    # Step 6: return to safe position
                    whole_body.move_end_effector_pose(hsr_geo.pose(), 'target_object_0')

                    whole_body.collision_world = None  # free for navigation/repositioning

                    whole_body.linear_weight = linear_weight
                    whole_body.angular_weight = angular_weight

                if _surface is not None:
                    try:
                        collision.remove(_surface)
                    except Exception:
                        pass

                omni_base.go_rel(-0.20, 0.0, 0.0, _NAVTIMEOUT, "hsr")
                whole_body.move_to_go()
                return 'success'
            except:
                rospy.logerr(traceback.format_exc())
                return 'failure'

        # After folding a basket cloth → go to washing machine
        smach.StateMachine.add('FOLDCLOTHES_BASKET', smach.CBState(fold_clothes_cb),
                               transitions={'success': 'MOVE2WASHINGMACHINE', #'success', #'MOVE2WASHINGMACHINE',
                                            'failure': 'failure'})

        # After folding a washing-machine cloth → repeat cycle from basket
        smach.StateMachine.add('FOLDCLOTHES_WM', smach.CBState(fold_clothes_cb),
                               transitions={'success': 'MOVE2LAUNDRYBASKETLEFT',
                                            'failure': 'failure'})
        ##########
        # END: FOLDING STATES
        ##########


        ##########
        # START: FINISH
        ##########
        @smach.cb_interface(outcomes=['success', 'failure'])
        def move_to_standby_cb(userdata):
            try:
                whole_body.move_to_neutral()
                return 'success'
            except:
                return 'failure'

        smach.StateMachine.add('MOVE2STANDBY', smach.CBState(move_to_standby_cb),
                               transitions={'success': 'success',
                                            'failure': 'failure'})
        ##########
        # END: FINISH
        ##########

    return sm


if __name__ == '__main__':
    sm = create_sm()

    sis = smach_ros.IntrospectionServer('laundry_challenge', sm, '/SM_ROOT')
    sis.start()

    try:
        outcome = sm.execute()

        if outcome == 'success':
            SAY('I finished the laundry task.')
            rospy.loginfo('I finished the laundry task.')
        else:
            SAY('Sorry, something happened. I did not finish the task.')

    except:
        traceback.print_exc()

    finally:
        sis.stop()
        logging.disable(logging.CRITICAL)
