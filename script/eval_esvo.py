import numpy as np
import os
import h5py
import cv2
import glob
import argparse
from decimal import Decimal


def binary_search(dset, x, l=None, r=None, side='left'):
    l = 0 if l is None else l
    r = len(dset) - 1 if r is None else r
    while l <= r:
        mid = l + (r - l) // 2
        midval = dset[mid]
        if midval == x:
            return mid
        elif midval < x:
            l = mid + 1
        else:
            r = mid - 1
    if side == 'left':
        return l
    return r

def find_ts_index(events_t, timestamp):
    idx = binary_search(events_t, timestamp, side='right')
    return idx


# int imuLoader::findCorrespondTimestamp(double time, double threshold)
# {
#     vector<double> tmp;
#     for(auto it = _timestamps.begin(); it != _timestamps.end(); it++)
#     {
#         tmp.push_back(abs(*it - time));
#     }

#     auto smallest = std::min_element(std::begin(tmp), std::end(tmp));

#     int index = distance(begin(tmp), smallest);
#     if (abs(time - _timestamps[index]) < threshold)
#         return index;
#     else{
#         cout << "ERROR: wrong timestamp!" << endl;
#     }
#     return -1;
# }

def findClosestTimestamp(time, ts, threshold):
    idx = -1
    min = threshold  +1
    for i in range(len(ts)):
        dst = abs(float(ts[i]) - time)
        if dst > threshold:
            continue
        if dst < min:
            min = dst
            idx = i
        
    return idx
    
    
def saveVideo(est_depth, gt_depth, video_writer):
    
    est_color = cv2.normalize(est_depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    est_color = cv2.applyColorMap(est_color, cv2.COLORMAP_JET)
    gt_color = cv2.normalize(gt_depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    gt_color = cv2.applyColorMap(gt_color, cv2.COLORMAP_JET)
    
    mask_est = (est_depth == 0)
    est_color[mask_est] = [0, 0, 0]
    mask_gt = np.isnan(gt_depth)
    gt_color[mask_gt] = [0, 0, 0]
    merged_img = np.hstack((est_color, gt_color))

    video_writer.write(merged_img)



parser = argparse.ArgumentParser()
parser.add_argument('--est_depth', type=str, help="base path where to look for the depth image")
parser.add_argument('--gt_depth', type=str, help="base path where to look for the gt h5 file")
parser.add_argument('--ev_data', type=str, help="base path where to look for the gt h5 file")
parser.add_argument('--dataset', type=str, help="dataset name (MVSEC or M3ED)")
parser.add_argument('--fps', type=int, help="fps of estimation")
parser.add_argument('--output_path', type=str, help=".mp4 path")


args = parser.parse_args()


if __name__ == "__main__":
    est_files = sorted(glob.glob(os.path.join(args.est_depth, "*.png")))
    
    
    timestamps = []
    for file in est_files:
        filename = os.path.basename(file)
        timestamp_str = os.path.splitext(filename)[0]
        timestamp = Decimal(timestamp_str) 
        timestamps.append(timestamp)

    timestamps.sort()

    gt = h5py.File(args.gt_depth, 'r')
    ev = h5py.File(args.ev_data, 'r')
    DATASET = args.dataset
    fps = args.fps
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(args.output_path, fourcc, 1, (692, 260))
    
    if DATASET == "MVSEC":
        gt_frames = gt['/davis/left/depth_image_rect']
        gt_tss = gt['/davis/left/depth_image_rect_ts']
        event = ev['/davis/left/events']
        event_start_ts = event[0][2]
    if DATASET == "M3ED":
        gt_frames = gt['/depth/prophesee/left']
        ts_map = np.array(gt['/ts_map_prophesee_left'])
        event_t = ev['/prophesee/left/t']
        gt_tss = []
        for i in range(len(gt_frames)):
            gt_tss.append(event_t[ts_map[i]])
        gt_tss = np.array(gt_tss) * 1e-6
        event_start_ts = event_t[0] * 1e-6
    
    # print(timestamps - event_start_ts)


    # gt_tss = gt_tss - event_start_ts
    gt_depth_consolidated = []
    est_depth_consolidated = []

    # for i, est_depth in enumerate(depth_frames):
    #     ts = tss[i]
    #     idx = find_ts_index(gt_tss, ts)
    #     gt_depth = np.array(gt_frames[idx])
    #     gt_depth[np.isinf(gt_depth)] = np.nan
    #     gt_depth[gt_depth == 0] = np.nan
    #     est_depth[est_depth == 0] = np.nan
        
    #     mask = np.isnan(est_depth) | np.isnan(gt_depth)
        
    #     masked_gt_depth = np.ma.array(gt_depth, mask=mask)
    #     gt_depth_consolidated.append(masked_gt_depth)
    #     est_depth_consolidated.append(est_depth)
        
    #     masked_gt_img = np.where(masked_gt_depth.mask, np.nan, masked_gt_depth.data)
    #     # cv2.imshow("masked_gt", masked_gt_img)
    #     # cv2.imshow("est_depth", est_depth)
    #     # cv2.waitKey(1)
        
    for i, gt_depth in enumerate(gt_frames):
        gt_t = gt_tss[i]
        idx = findClosestTimestamp(gt_t, timestamps, 0.01)
        
        if(idx == -1):
            continue
        
        img_path = args.est_depth + str(timestamps[idx]) + ".png"
        evo_depth = cv2.imread(img_path,  cv2.IMREAD_UNCHANGED)
        est_depth = np.array(evo_depth).astype(np.float32) * 1e-3
    
        saveVideo(est_depth, gt_depth, video_writer)
        
     
        
        
        gt_depth[np.isinf(gt_depth)] = np.nan
        gt_depth[gt_depth == 0] = np.nan
        est_depth[est_depth == 0] = np.nan
        
        mask = np.isnan(est_depth) | np.isnan(gt_depth)
        
        masked_gt_depth = np.ma.array(gt_depth, mask=mask)
        gt_depth_consolidated.append(masked_gt_depth)
        est_depth_consolidated.append(est_depth)
        
        masked_gt_img = np.where(masked_gt_depth.mask, np.nan, masked_gt_depth.data)
        # cv2.imshow("masked_gt", masked_gt_img)
        # cv2.imshow("est_depth", est_depth)
        # cv2.waitKey(1)


    gt_depth_consolidated = np.ma.array(gt_depth_consolidated)
    est_depth_consolidated = np.ma.array(est_depth_consolidated)
    error = np.absolute(gt_depth_consolidated - est_depth_consolidated)
    scene_depth = np.max(gt_depth_consolidated[~ np.isnan(gt_depth_consolidated)])
    
    print("Dataset: ", DATASET, " From ", float(timestamps[0]) - event_start_ts, " to ", float(timestamps[-1]) - event_start_ts)
    print("Mean " + str(np.ma.mean(error)))
    print("Median " + str(np.ma.median(error)))
    print('Max depth: ' + str(scene_depth))
    print('Number of error pixels: ' + str(np.ma.count(error) / len(error)))