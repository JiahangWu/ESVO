import argparse
import pathlib
import rosbag
import rospy
from tqdm import tqdm
from typing import List
import numpy as np

import sys

from bimvee.exportIitYarp import exportIitYarp


def convertDvsrosbag2yarplog(args: argparse.Namespace) -> None:
    # Params
    bag_path: pathlib.Path = pathlib.Path(args.bagfile_path)
    event_timestamp_init: float = None
    container: dict = {}
    xs_left: List[int] = []
    ys_left: List[int] = []
    pols_left: List[bool] = []
    tss_left: List[float] = []
    xs_right: List[int] = []
    ys_right: List[int] = []
    pols_right: List[bool] = []
    tss_right: List[float] = []
    start_time: rospy.Time
    end_time: rospy.Time

    # Store events msgs rosbag into Lists
    with rosbag.Bag(bag_path.__str__(), "r") as bag:
        print("Check total messages...")
        # Set start and end times
        if args.start_time_sec is not None:
            start_time = rospy.Time.from_sec(bag.get_start_time() + args.start_time_sec)
        else:
            start_time = rospy.Time.from_sec(bag.get_start_time())
        if args.end_time_sec is not None:
            end_time = rospy.Time.from_sec(bag.get_start_time() + args.end_time_sec)
        else:
            end_time = rospy.Time.from_sec(bag.get_end_time())

        # Get total message of /dvs/events
        total_messages = 0
        if args.start_time_sec is None and args.end_time_sec is None:
            topic_info = bag.get_type_and_topic_info()
            total_messages = topic_info.topics["/davis/left/events"].message_count
        else:
            total_messages = sum(
                1 for _ in bag.read_messages(topics=["/davis/left/events"], start_time=start_time, end_time=end_time)
            )

        # Store events in Lists
        for _, msg, _ in tqdm(
            bag.read_messages(topics=["/davis/left/events"], start_time=start_time, end_time=end_time),
            desc="Storing left events from bag...",
            total=total_messages,
        ):
            if event_timestamp_init is None:
                event_timestamp_init = msg.events[0].ts.secs + msg.events[0].ts.nsecs * 1e-9
            x_values = [event.x for event in msg.events]
            y_values = [event.y for event in msg.events]
            pol_values = [event.polarity for event in msg.events]
            ts_values = [(event.ts.secs + event.ts.nsecs * 1e-9) for event in msg.events]
            xs_left.extend(x_values)
            ys_left.extend(y_values)
            pols_left.extend(pol_values)
            tss_left.extend(ts_values)
        
        for _, msg, _ in tqdm(
            bag.read_messages(topics=["/davis/right/events"], start_time=start_time, end_time=end_time),
            desc="Storing right events from bag...",
            total=total_messages,
        ):
            if event_timestamp_init is None:
                event_timestamp_init = msg.events[0].ts.secs + msg.events[0].ts.nsecs * 1e-9
            x_values = [event.x for event in msg.events]
            y_values = [event.y for event in msg.events]
            pol_values = [event.polarity for event in msg.events]
            ts_values = [(event.ts.secs + event.ts.nsecs * 1e-9) for event in msg.events]
            xs_right.extend(x_values)
            ys_right.extend(y_values)
            pols_right.extend(pol_values)
            tss_right.extend(ts_values)

    #Create distionary

    tss_left = np.array(tss_left).astype(np.float64)
    xs_left = np.array(xs_left).astype(np.uint16)
    ys_left = np.array(ys_left).astype(np.uint16)
    pols_left = np.array(pols_left).astype(bool)
    
    tss_right = np.array(tss_right).astype(np.float64)
    xs_right = np.array(xs_right).astype(np.uint16)
    ys_right = np.array(ys_right).astype(np.uint16)
    pols_right = np.array(pols_right).astype(bool)
    
    output_dict = {
        "info": {
            "filePathOrName": args.outputdir_path
        },
        "data": {
            "left": {
                "dvs": {
                    "ts": tss_left,
                    "x": xs_left,
                    "y": ys_left,
                    "pol": pols_left
                }
            },
            "right": {
                "dvs": {
                    "ts": tss_right,
                    "x": xs_right,
                    "y": ys_right,
                    "pol": pols_right
                }
            }
        } 
    }

    # Delete variables
    del x_values, xs_left, xs_right, y_values, ys_left, ys_right, pol_values, pols_left, pols_right, ts_values, tss_left, tss_right

    # Export yarp .log file
    exportIitYarp(
        output_dict,
        exportFilePath=args.outputdir_path,
        exportAsEv2=True,
        exportTimestamps=False,
        minTimeStepPerBottle=5e-4,
        viewerApp=False,
        protectedWrite=False,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Converter from rosbag .bag to YARP .log format. Rosbag files has to include a topic "/dvs/events".'
    )
    parser.add_argument("-p", "--bagfile_path", help="Absolute or relative Path to .bag file", type=str, required=True)
    parser.add_argument("-o", "--outputdir_path", help="Output folder path", type=str, required=True)
    parser.add_argument("-s", "--start_time_sec", help="start time [s]", type=int, default=None)
    parser.add_argument("-e", "--end_time_sec", help="end time [s]", type=int, default=None)
    args = parser.parse_args()

    convertDvsrosbag2yarplog(args)