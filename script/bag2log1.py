import rosbag
import rospy
from sensor_msgs.msg import Image

bag_path = "/app/Datasets/RPG/desk1.bag"

with rosbag.Bag(bag_path, "r") as bag:
    for topic, msg, t in bag.read_messages():
        # print(f"Topic: {topic}, Time: {t.to_sec()}")

        if topic == "/dvs_msgs/EventArray ":
            print("Message Type:", msg)
        #     # for event in msg.events[:5]:
        #     #     print(f"X: {event.x}, Y: {event.y}, Timestamp: {event.ts.secs}.{event.ts.nsecs}, Polarity: {event.polarity}")
        #     break


# import argparse
# import pathlib
# import rosbag
# import rospy
# from tqdm import tqdm
# from typing import List
# import numpy as np

# import sys

# sys.path.append("/app/SCARF_analysis/submodule/bimvee")
# from bimvee.exportIitYarp import exportIitYarp


# def convertDvsrosbag2yarplog(args: argparse.Namespace) -> None:
#     # Params
#     bag_path: pathlib.Path = pathlib.Path(args.bagfile_path)
#     event_timestamp_init: float = None
#     container: dict = {}
#     xs: List[int] = []
#     ys: List[int] = []
#     pols: List[bool] = []
#     tss: List[float] = []
#     start_time: rospy.Time
#     end_time: rospy.Time

#     # Store events msgs rosbag into Lists
#     with rosbag.Bag(bag_path.__str__(), "r") as bag:
#         print("Check total messages...")
#         # Set start and end times
#         if args.start_time_sec is not None:
#             start_time = rospy.Time.from_sec(bag.get_start_time() + args.start_time_sec)
#         else:
#             start_time = rospy.Time.from_sec(bag.get_start_time())
#         if args.end_time_sec is not None:
#             end_time = rospy.Time.from_sec(bag.get_start_time() + args.end_time_sec)
#         else:
#             end_time = rospy.Time.from_sec(bag.get_end_time())

#         # Get total message of /dvs/events
#         total_messages = 0
#         if args.start_time_sec is None and args.end_time_sec is None:
#             topic_info = bag.get_type_and_topic_info()
#             total_messages = topic_info.topics["/davis/right/events"].message_count
#         else:
#             total_messages = sum(
#                 1 for _ in bag.read_messages(topics=["/davis/right/events"], start_time=start_time, end_time=end_time)
#             )

#         # Store events in Lists
#         for _, msg, _ in tqdm(
#             bag.read_messages(topics=["/davis/right/events"], start_time=start_time, end_time=end_time),
#             desc="Storing events from bag...",
#             total=total_messages,
#         ):
#             if event_timestamp_init is None:
#                 event_timestamp_init = msg.events[0].ts.secs + msg.events[0].ts.nsecs * 1e-9
#             x_values = [event.x for event in msg.events]
#             y_values = [event.y for event in msg.events]
#             pol_values = [event.polarity for event in msg.events]
#             ts_values = [(event.ts.secs + event.ts.nsecs * 1e-9) for event in msg.events]
#             xs.extend(x_values)
#             ys.extend(y_values)
#             pols.extend(pol_values)
#             tss.extend(ts_values)

#     # Create container as distionary
#     container["data"] = {"right": {"dvs": {"ts": np.array(tss).astype(np.float64)}}}
#     container["data"]["right"]["dvs"]["x"] = np.array(xs).astype(np.uint16)
#     container["data"]["right"]["dvs"]["y"] = np.array(ys).astype(np.uint16)
#     container["data"]["right"]["dvs"]["pol"] = np.array(pols).astype(bool)
#     container["info"] = {}
#     container["info"]["filePathOrName"] = args.outputdir_path

#     # Delete variables
#     del x_values, xs, y_values, ys, pol_values, pols, ts_values, tss

#     # Export yarp .log file
#     exportIitYarp(
#         container,
#         exportFilePath=args.outputdir_path,
#         exportAsEv2=True,
#         exportTimestamps=False,
#         minTimeStepPerBottle=5e-4,
#         viewerApp=False,
#         protectedWrite=False,
#     )


# if __name__ == "__main__":
#     parser = argparse.ArgumentParser(
#         description='Converter from rosbag .bag to YARP .log format. Rosbag files has to include a topic "/dvs/events".'
#     )
#     parser.add_argument("-p", "--bagfile_path", help="Absolute or relative Path to .bag file", type=str, required=True)
#     parser.add_argument("-o", "--outputdir_path", help="Output folder path", type=str, required=True)
#     parser.add_argument("-s", "--start_time_sec", help="start time [s]", type=int, default=None)
#     parser.add_argument("-e", "--end_time_sec", help="end time [s]", type=int, default=None)
#     args = parser.parse_args()

#     convertDvsrosbag2yarplog(args)