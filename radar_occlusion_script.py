import os
import random
import numpy as np
from functools import reduce
from pyquaternion import Quaternion
from nuscenes.utils.geometry_utils import transform_matrix
from nuscenes.utils.data_classes import RadarPointCloud

"""
Radar Occlusion Script for Occluded nuScenes Dataset
---------------------------------------------------

This function is provided as a DROP-IN REPLACEMENT for the original "get_radar_data" function in the nuScenes dataset loader script.

Paper-aligned occlusion parameters used in the manuscript:
- drop_percentage: 25
- noise_std: 0.1
- radars_to_drop: ['RADAR_BACK_RIGHT']
- seed: 42

All stochastic operations use a fixed random seed to ensure reproducible radar occlusions.
"""

# ---------------------- Reproducibility ----------------------
def set_seed(seed=42):
    random.seed(seed)
    np.random.seed(seed)


# ---------------------- Radar Occlusion Function ----------------------
def get_radar_data(
        nusc,
        sample_rec,
        nsweeps,
        min_distance,
        use_radar_filters,
        dataroot,
        radar_info=None,
        radars_to_drop=["RADAR_BACK_RIGHT"],
        radar_to_reduce=None,
        drop_percentage=0,
        drop_from_all=False,
        add_gaussian_noise=False,
        noise_std=0,
        radar_to_noise=None,
        add_noise_to_all=False,
        modify_rcs=False,
        rcs_scale=0,
        random_drop_one_radar=False,
        seed=42
    ):
    """
    Returns radar data with optional occlusion and noise applied.

    Supported occlusions:
    1. Dropping one or more radar sensors.
    2. Dropping a percentage of radar points from one or all radars.
    3. Adding Gaussian noise to one or all radar sensors.
    4. Scaling RCS values for signal strength degradation.
    """

    # Ensure deterministic behavior
    set_seed(seed)

    points = np.zeros((19, 0))
    radar_chan_list = [
        'RADAR_BACK_RIGHT',
        'RADAR_BACK_LEFT',
        'RADAR_FRONT',
        'RADAR_FRONT_LEFT',
        'RADAR_FRONT_RIGHT'
    ]

    # ---------------------- Random radar drop ----------------------
    if random_drop_one_radar:
        selected = random.choice(radar_chan_list)
        radars_to_drop = [selected]

    # ---------------------- Drop selected radars ----------------------
    if radars_to_drop:
        for radar_name in radars_to_drop:
            if radar_name in radar_chan_list:
                radar_chan_list.remove(radar_name)

    # ---------------------- Reference frame ----------------------
    if radar_info is None:
        ref_sd_token = sample_rec['data']['RADAR_BACK_RIGHT']
        ref_sd_rec = nusc.get('sample_data', ref_sd_token)
        ref_pose_rec = nusc.get('ego_pose', ref_sd_rec['ego_pose_token'])
        ref_time = 1e-6 * ref_sd_rec['timestamp']
        car_from_global = transform_matrix(
            ref_pose_rec['translation'],
            Quaternion(ref_pose_rec['rotation']),
            inverse=True
        )
    else:
        nsweeps = len(radar_info['RADAR_BACK_RIGHT']['time_lag'])

    RadarPointCloud.default_filters() if use_radar_filters else RadarPointCloud.disable_filters()

    # ---------------------- Radar sweep processing ----------------------
    for radar_name in radar_chan_list:

        if radar_info is None:
            sample_data_token = sample_rec['data'][radar_name]
            current_sd_rec = nusc.get('sample_data', sample_data_token)

        for i in range(nsweeps):

            if radar_info is None:
                path_to_radar_data = current_sd_rec['filename']
                current_pose_rec = nusc.get('ego_pose', current_sd_rec['ego_pose_token'])
                global_from_car = transform_matrix(
                    current_pose_rec['translation'],
                    Quaternion(current_pose_rec['rotation']),
                    inverse=False
                )
                current_cs_rec = nusc.get(
                    'calibrated_sensor',
                    current_sd_rec['calibrated_sensor_token']
                )
                car_from_current = transform_matrix(
                    current_cs_rec['translation'],
                    Quaternion(current_cs_rec['rotation']),
                    inverse=False
                )
                trans_matrix = reduce(
                    np.dot,
                    [car_from_global, global_from_car, car_from_current]
                )
                time_lag = ref_time - 1e-6 * current_sd_rec['timestamp']
            else:
                path_to_radar_data = radar_info[radar_name]['radar_paths'][i]
                trans_matrix = radar_info[radar_name]['trans_matrix'][i]
                time_lag = radar_info[radar_name]['time_lag'][i]

            current_pc = RadarPointCloud.from_file(
                os.path.join(dataroot, path_to_radar_data)
            )
            current_pc.remove_close(min_distance)
            current_pc.transform(trans_matrix)

            # ---------------------- Point Dropout ----------------------
            drop_this_radar = (radar_name == radar_to_reduce) if not drop_from_all else True
            if drop_this_radar and drop_percentage > 0:
                total_points = current_pc.nbr_points()
                num_to_drop = int(total_points * (drop_percentage / 100))
                if num_to_drop > 0:
                    drop_indices = np.random.choice(
                        total_points,
                        num_to_drop,
                        replace=False
                    )
                    current_pc.points = np.delete(
                        current_pc.points,
                        drop_indices,
                        axis=1
                    )

            # ---------------------- Gaussian Noise ----------------------
            if add_gaussian_noise:
                apply_noise = add_noise_to_all or (radar_name == radar_to_noise)
                if apply_noise and noise_std > 0:
                    noise = np.random.normal(
                        0,
                        noise_std,
                        size=current_pc.points[:3, :].shape
                    )
                    current_pc.points[:3, :] += noise

            # ---------------------- RCS Scaling ----------------------
            if modify_rcs and current_pc.points.shape[0] > 5:
                current_pc.points[5, :] *= rcs_scale

            # ---------------------- Time channel ----------------------
            times = time_lag * np.ones((1, current_pc.nbr_points()))
            new_points = np.concatenate((current_pc.points, times), axis=0)
            points = np.concatenate((points, new_points), axis=1)

            if radar_info is None:
                if current_sd_rec['prev'] == '':
                    break
                current_sd_rec = nusc.get('sample_data', current_sd_rec['prev'])

    return points
