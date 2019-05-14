import argparse
 
import rospy
 
import baxter_interface
from baxter_examples import JointRecorder
 
from baxter_interface import CHECK_VERSION

def main():
    """RSDK Joint Recorder Example
 
   Record timestamped joint and gripper positions to a file for
   later play back.
 
   Run this example while moving the robot's arms and grippers
   to record a time series of joint and gripper positions to a
   new csv file with the provided *filename*. This example can
   be run in parallel with any other example or standalone
   (moving the arms in zero-g mode while pressing the cuff
   buttons to open/close grippers).
 
   You can later play the movements back using one of the
   *_file_playback examples.
   """
    epilog = """
Related examples:
 joint_position_file_playback.py; joint_trajectory_file_playback.py.
   """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--file', dest='filename', required=True,
        help='the file name to record to'
    )
    parser.add_argument(
        '-r', '--record-rate', type=int, default=100, metavar='RECORDRATE',
        help='rate at which to record (default: 100)'
    )
    args = parser.parse_args(rospy.myargv()[1:])


    print("Initializing node... ")
    rospy.init_node("rsdk_joint_recorder")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
 
    recorder = JointRecorder(args.filename, args.record_rate)
    rospy.on_shutdown(recorder.stop)
 
    print("Recording. Press Ctrl-C to stop.")
    recorder.record()
 
    print("\nDone.")
 
if __name__ == '__main__':
    main()