import time
import json
from constrained_solver import solve_motion, init_curobo
import rospy
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
from scipy.spatial.transform import Rotation as R
from frankapy.utils import min_jerk
import argparse
import pyrealsense2 as rs
import cv2
import numpy as np
import os
from ros_toolkit import Ros_listener, Ros_publisher, run_publisher
import threading
from std_msgs.msg import Float64MultiArray
from tqdm import tqdm
from termcolor import cprint
stop_event = threading.Event()
POSE = [
    [-0.35471786,  0.67136656, -0.12932039, -1.96341863,  0.79877985,  2.14604293,  1.27579102],
    [-1.12162436,  0.99415506,  0.60818567, -1.74134301,  0.3579409,   1.95144463,  1.53270908],
]
K_GAINS = [400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0]
D_GAINS = [320.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0]
def init_realsense(fps=30):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, fps)  # Configure color stream
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, fps)  # Configure depth stream
    profile = pipeline.start(config)
    return pipeline, profile

def control_thread(fa, joint_state, joints_traj, init_time, dir_name):
    joints_cmd = []
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
    time.sleep(0.2)
    fa.goto_joints(joint_state, duration=5, dynamic=True, buffer_time=10, ignore_virtual_walls=True, 
                     k_gains=K_GAINS,
                     d_gains=D_GAINS,
                   )
    rate = rospy.Rate(20)
    tss = []
    joints_traj.append(joints_traj[-1])
    joints_traj.append(joints_traj[-1])
    joints_traj.append(joints_traj[-1])

    for i in range(0, len(joints_traj)):
        pose = fa.get_pose().translation
        if pose[1]>-0.05:
            flag=1
        else:
            flag=0
        timestamp = rospy.Time.now().to_time() - init_time
        tss.append(timestamp)

        if i == 0:
            cmd = joints_traj[i] 
        else:
            #cmd = joints_traj[len(joints_traj)//2] if flag else joints_traj[-1]
            cmd = joints_traj[i] 
        # cmd[-2] = 1.9522560834884644
        # cmd[-1] = 1.5329617261886597
        traj_gen_proto_msg = JointPositionSensorMessage(
            id=i, timestamp=rospy.Time.now().to_time() - init_time, 
            joints=cmd,
        )
        joints_cmd.append(cmd)
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION)
        )
        
        pub.publish(ros_msg)

        rospy.loginfo(f"Published control command ID {traj_gen_proto_msg.id} {cmd}")
        rate.sleep()
    time.sleep(1)
    cmd_timestamps = []
    for i, (ts, cmd) in enumerate(zip(tss, joints_cmd)):
        
        cmd_timestamps.append({
            "id": i,
            "ros_timestamp": ts,
            "cmd": cmd
        })
    with open(f"{dir_name}/control.json", "w") as f:
        json.dump(cmd_timestamps, f, indent=4)

    #print("Timestamps saved to timestamps.json")
def generate_cmd(motion_gen, kin_model):
    trans_goals = []
    rot_goals = []

    X = 0.05
    for i in range(5):
        trans_goals.append(ee_translation + z_proj * X * i)
        rot_goals.append(ee_quaternion)
    new_joint_state = joint_state
    joints_traj = []
    ee_translation_traj = []
    for i in range(0, len(trans_goals)-1):
        
        j_traj, ee_traj, new_joint_state = solve_motion(
            args,
            new_joint_state,
            ee_translation_goal=trans_goals[i+1],
            ee_orientation_goal=rot_goals[i+1],
            motion_gen=motion_gen,
            kin_model=kin_model
        )
        joints_traj += j_traj
        ee_translation_traj += ee_traj

    joints_traj = joints_traj[::40]
    cprint(f"len of cmd: {len(joints_traj)}", "green")
    return joints_traj

def get_index(args):
    os.makedirs(args.save_dir, exist_ok = True)
    all_entries = os.listdir(args.save_dir)
    all_entries.sort()
    if len(all_entries) >= 1:
        cnt = int(all_entries[-1].split("_")[-1]) + 1
    else: 
        cnt = 0
    return cnt

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Pushing with Franka')
    parser.add_argument('--robot', type=str, default="dynamic_alignment/franka.yml", help="robot configuration to load")
    parser.add_argument('--debug', type=int, default=0, help='debug level')
    parser.add_argument('--cmd_num', type=int, default=20, help='')
    parser.add_argument('--save_dir', type=str, default='records/ovaltine', help='index of the saved data')
    parser.add_argument("--len", type=int, default=200)
    args = parser.parse_args()

    
    

    motion_gen, kin_model = init_curobo(args)
    pipeline, profile = init_realsense()
    cnt = get_index(args)
    
    fa = FrankaArm()
    publisher = threading.Thread(target=run_publisher, args=(fa, ))
    publisher.start()
    ros_listener = Ros_listener()
    INIT_POSE = POSE[1]
    init_info = {
        "init_pose": INIT_POSE,
        "k_gains": K_GAINS,
        "d_gains": D_GAINS,
    }


    for idx in range(cnt, args.len + 1):
        dir_name = os.path.join(args.save_dir, f"traj_{idx:05d}")
        depth_dir = os.path.join(dir_name, "depth")
        color_dir = os.path.join(dir_name, "rgb")
        vis_dir = os.path.join(dir_name, "vis")   
        os.makedirs(depth_dir, exist_ok=True)
        os.makedirs(color_dir, exist_ok=True)
        os.makedirs(vis_dir, exist_ok=True)
        
        with open(os.path.join(dir_name, "init.json"), "w") as f:
            json.dump(init_info, f, indent=4)

        cprint("="*60, "cyan")
        cprint(f"Recording traj {idx} to {dir_name}", "cyan")
        cprint("reset franka", "green")
        try:
            fa.stop_skill()
        except:
            raise EnvironmentError
        fa.goto_joints(INIT_POSE, ignore_virtual_walls=True)
        fa.close_gripper()
        
        joint_state = fa.get_joints().astype('float32')
        ee_translation = fa.get_pose().translation.astype('float32')
        ee_quaternion = fa.get_pose().quaternion.astype('float32')
        rotation = R.from_quat([ee_quaternion[3], ee_quaternion[0], ee_quaternion[1], ee_quaternion[2]])
        rotation_matrix = rotation.as_matrix()
        #print("rotation_matrix: ", rotation_matrix)
        print("ee_translation: ", ee_translation)
        print("ee_quaternion: ", ee_quaternion)
        print("joint_state: ", joint_state)
        #ee_translation[1] +=0.1

        z_axis = rotation_matrix[:, 2]
        z_proj = -z_axis
        z_proj[2] = 0.0
        z_proj = z_proj / np.linalg.norm(z_proj)
        print(f"projection of z axis of ee on XoY plain {z_proj}")
        
        joints_traj = generate_cmd(motion_gen, kin_model)
        input("Press enter to start moving")
        #control_thread(fa, joint_state, joints_traj, init_time, args)
        timestamps = []
        
        # Get the stream profiles for depth and color
        depth_profile = profile.get_stream(rs.stream.depth)
        color_profile = profile.get_stream(rs.stream.color)

        # Get the intrinsics for depth and color streams
        depth_intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()
        color_intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
        cam_K = np.array([
            [color_intrinsics.fx, 0, color_intrinsics.ppx],
            [0, color_intrinsics.fy, color_intrinsics.ppy],
            [0, 0, 1]
        ])
        with open(os.path.join(dir_name, "cam_K.txt"), "w") as f:
            for row in cam_K:
                f.write(" ".join(f"{x:.10f}" for x in row) + "\n")
        align_to = rs.stream.color
        align = rs.align(align_to)

        #start controlling
        init_time = rospy.Time.now().to_time()
        control_thread_obj = threading.Thread(target=control_thread, args=(fa, joint_state, joints_traj, init_time, dir_name))
        control_thread_obj.start()
        #start recording
        depth_images = []
        color_images = []
        for i in range(90):
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                cprint("No camera!", "red")
                continue
            camera_timestamp = frames.get_timestamp() / 1000.0
            joint_state = ros_listener.joint_state
            ee_trans = ros_listener.ee_pose.translation
            ee_trans = [ee_trans.x, ee_trans.y, ee_trans.z]
            # ee_quat = ros_listener.ee_pose.quartanion
            ee_quat = ros_listener.ee_pose.rotation
            ee_quat = [ee_quat.w, ee_quat.x, ee_quat.y, ee_quat.z]
            ros_timestamp = rospy.Time.now().to_time() - init_time
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())


            depth_images.append((depth_image.astype(np.float32) / 1000.0).copy())
            color_images.append(color_image.copy())
            
            timestamps.append({
                    "id": i,
                    "ros_timestamp": ros_timestamp,
                    "camera_timestamp": camera_timestamp,
                    "joint_state": joint_state,
                    "ee_trans": ee_trans,
                    # "ee_quat": ee_quat,
                    "ee_quat_wxyz": ee_quat
                })

        control_thread_obj.join()
        for idx, (depth_image, color_image) in enumerate(tqdm(zip(depth_images, color_images), desc='saving...')):
            np.savez_compressed(os.path.join(depth_dir, f"{idx:05d}.npz"), depth=depth_image)
            cv2.imwrite(os.path.join(color_dir, f"{idx:05d}.png"), color_image)

        with open(f"{dir_name}/frame.json", "w") as f:
            json.dump(timestamps, f, indent=4)

    stop_event = threading.Event()
    publisher.join()