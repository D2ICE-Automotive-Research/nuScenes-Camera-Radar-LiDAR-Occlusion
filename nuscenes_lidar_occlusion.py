##--------------Use this function when you want to drop points in spatial region--------------------##
##--------------Comment the function "drop_angle_based_region" when using this----------------------##
def drop_spatial_region(points, region="left", drop_percentage=100):
    """
    Drops points based on spatial region.
    Args:
        points: (6, N) lidar points (x,y,z,intensity,time,ring_index)
        region: "front", "back", "left", "right"
        drop_percentage: % of points to drop from selected region
    Returns:
        points after dropping
    """
    x = points[0, :]
    y = points[1, :]

    if region == "front":
        mask = x > 0
    elif region == "back":
        mask = x < 0
    elif region == "left":
        mask = y > 0
    elif region == "right":
        mask = y < 0
    else:
        raise ValueError("Region must be front, back, left, or right!")

    region_indices = np.where(mask)[0]
    num_to_drop = int(len(region_indices) * (drop_percentage / 100))
    if num_to_drop > 0:
        drop_indices = np.random.choice(region_indices, num_to_drop, replace=False)
        keep_indices = np.setdiff1d(np.arange(points.shape[1]), drop_indices)
        points = points[:, keep_indices]
    
    return points



##--------------Use this function when you want to drop points in angle based region--------------------##
##--------------Comment the function "drop_spatial_region" when using this----------------------##
def drop_angle_based_region(points, region="right", drop_percentage=100, angle_range=90):
    """
    Drops points based on spatial region, optionally constrained to a specific angle range.

    Args:
        points: (6, N) lidar points (x,y,z,intensity,time,ring_index)
        region: "front", "back", "left", "right"
        drop_percentage: % of points to drop from selected region
        angle_range: angle in degrees around the central axis

    Returns:
        points after dropping
    """
    x = points[0, :]
    y = points[1, :]
    theta = np.degrees(np.arctan2(y, x))  # angle in degrees

    if region == "front":
        mask = (x > 0) & (np.abs(theta) <= angle_range / 2)
    elif region == "back":
        mask = (x < 0) & (
            (np.abs(theta - 180) <= angle_range / 2) |
            (np.abs(theta + 180) <= angle_range / 2)
        )
    elif region == "left":
        mask = (y > 0) & (np.abs(theta - 90) <= angle_range / 2)
    elif region == "right":
        mask = (y < 0) & (np.abs(theta + 90) <= angle_range / 2)
    else:
        raise ValueError("Region must be front, back, left, or right!")

    region_indices = np.where(mask)[0]
    num_to_drop = int(len(region_indices) * (drop_percentage / 100))
    if num_to_drop > 0:
        drop_indices = np.random.choice(region_indices, num_to_drop, replace=False)
        keep_indices = np.setdiff1d(np.arange(points.shape[1]), drop_indices)
        points = points[:, keep_indices]

    return points


###------------Replace your original get_lidar_data function with this get_lidar_data with occlusion ----------------------
###------------When you wants to drop random point clouds then just define the drop_percentage value e.g.60------------###
###------------Comment both functions "drop_spatial_region" and "drop_angle_based_region" when using random drop point occlusion----------------------##
def get_lidar_data(nusc, sample_rec, nsweeps, min_distance, dataroot, lidar_info=None, drop_percentage=0):
    """
    Returns at most nsweeps of lidar in the ego frame.
    Supports random point dropout to simulate occlusion.

    Returned tensor is 6(x, y, z, reflectance, dt, ring_index) x N

    Args:
    - drop_percentage (float): Percentage of points to randomly drop (0 = no drop)
    """

    points = np.zeros((6, 0))  # Initialize empty point cloud

    if lidar_info is None:
        # Get reference pose and timestamp.
        ref_sd_token = sample_rec['data']['LIDAR_TOP']
        ref_sd_rec = nusc.get('sample_data', ref_sd_token)
        ref_pose_rec = nusc.get('ego_pose', ref_sd_rec['ego_pose_token'])
        ref_time = 1e-6 * ref_sd_rec['timestamp']

        car_from_global = transform_matrix(ref_pose_rec['translation'], Quaternion(ref_pose_rec['rotation']), inverse=True)

        sample_data_token = sample_rec['data']['LIDAR_TOP']
        current_sd_rec = nusc.get('sample_data', sample_data_token)
    else:
        nsweeps = len(lidar_info['lidar_paths'])

    for i in range(nsweeps):
        if lidar_info is None:
            path_to_lidar_data = current_sd_rec['filename']
            current_pose_rec = nusc.get('ego_pose', current_sd_rec['ego_pose_token'])
            global_from_car = transform_matrix(current_pose_rec['translation'], Quaternion(current_pose_rec['rotation']), inverse=False)
            current_cs_rec = nusc.get('calibrated_sensor', current_sd_rec['calibrated_sensor_token'])
            car_from_current = transform_matrix(current_cs_rec['translation'], Quaternion(current_cs_rec['rotation']), inverse=False)
            trans_matrix = reduce(np.dot, [car_from_global, global_from_car, car_from_current])
            timestamp = current_sd_rec['timestamp']
            time_lag = ref_time - 1e-6 * timestamp
        else:
            path_to_lidar_data = lidar_info['lidar_paths'][i]
            trans_matrix = lidar_info['trans_matrix'][i]
            time_lag = lidar_info['time_lag'][i]

        current_pc = LidarPointCloud.from_file(os.path.join(dataroot, path_to_lidar_data))
        current_pc.remove_close(min_distance)


        # Transform into the ego vehicle frame
        current_pc.transform(trans_matrix)

        # ✨ Drop random points if requested
        if drop_percentage > 0:
            total_points = current_pc.nbr_points()
            num_to_drop = int(total_points * (drop_percentage / 100))
            if num_to_drop > 0:
                drop_indices = np.random.choice(total_points, num_to_drop, replace=False)
                current_pc.points = np.delete(current_pc.points, drop_indices, axis=1)
                print(f"Dropped {num_to_drop} points ({drop_percentage}%) in sweep {i}")


        # Add time vector
        times = time_lag * np.ones((1, current_pc.nbr_points()))
        new_points = np.concatenate((current_pc.points, times), axis=0)
        points = np.concatenate((points, new_points), axis=1)

        if lidar_info is None:
            if current_sd_rec['prev'] == '':
                break
            else:
                current_sd_rec = nusc.get('sample_data', current_sd_rec['prev'])

    # Spatially drop points if needed
    #points = drop_spatial_region(points, region="left", drop_percentage=100)

    # Angle based points drop if needed
    #points = drop_angle_based_region(points, region="left", drop_percentage=100, angle_range=90)

    return points
