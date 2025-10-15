---

### Radar Occlusions (Scripts in this Repo)
The radar occlusions are implemented in a modified `get_radar_data()` function.  
Replace the original function in `nuscenes_dataset.py` with the one provided here.  

### Radar Occlusion Types
1. **Drop entire radar sensors** (fixed list or randomly selected one)  
2. **Random Point Drop** → drop percentage of radar points from one or all radars  
3. **Gaussian Noise** → add Gaussian noise to radar points (per sensor or all sensors)  

### Example Usage
```python
# Drop a specific radar sensor
radar_points = get_radar_data(nusc, sample_rec, nsweeps=5, min_distance=1.0,
                              dataroot="data/nuscenes", radars_to_drop=["RADAR_FRONT"])

# Randomly drop one radar sensor
radar_points = get_radar_data(nusc, sample_rec, nsweeps=5, min_distance=1.0,
                              dataroot="data/nuscenes", random_drop_one_radar=True)

# Drop 30% of radar points from all sensors
radar_points = get_radar_data(nusc, sample_rec, nsweeps=5, min_distance=1.0,
                              dataroot="data/nuscenes", drop_percentage=30, drop_from_all=True)

# Add Gaussian noise to all radar sensors
radar_points = get_radar_data(nusc, sample_rec, nsweeps=5, min_distance=1.0,
                              dataroot="data/nuscenes", add_gaussian_noise=True,
                              add_noise_to_all=True, noise_std=0.05)

```

### LiDAR Occlusions (Scripts in this Repo)


The LiDAR occlusions are implemented in a modified `get_lidar_data()` function.  
Replace the original function in `nuscenes_dataset.py` with the one provided in this repository.  

### LiDAR Occlusion Types
1. **Random Point Drop** → randomly drops a percentage of LiDAR points across the whole cloud.  
2. **Spatial Region-based Occlusion** → removes points from a fixed region of space (`front`, `back`, `left`, `right`).  
3. **Angle-based Region Occlusion** → removes points within a specified angular field of view (e.g., ±45° around `front`, `back`, `left`, or `right`).  

---

### Example Usage

```python
# 1. Random Point Drop: drop 50% of LiDAR points
lidar_points = get_lidar_data(nusc, sample_rec, nsweeps=10, min_distance=1.0,
                              dataroot="data/nuscenes", drop_percentage=50)

# 2. Spatial Region Occlusion: remove all points in the left region
lidar_points = drop_spatial_region(points, region="left", drop_percentage=100)

# 3. Angle-based Occlusion: drop 70% of points in the front (±45 degrees)
lidar_points = drop_angle_based_region(points, region="front", drop_percentage=70, angle_range=90)
```

