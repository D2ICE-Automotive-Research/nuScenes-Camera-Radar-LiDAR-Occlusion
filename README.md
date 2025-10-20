# nuScenes-Camera-Radar-LiDAR-Occlusion

This repository provides **occlusion datasets and scripts** for the [nuScenes](https://www.nuscenes.org/) dataset.  
It includes:  

- **Camera occlusion dataset** (Dirt, Water-blur, Scratches, Woodscape soiling patterns) hosted on **IEEE DataPort**.  
- **Radar and LiDAR occlusion scripts** (Python functions) for simulating sensor degradations.  

The goal is to simulate realistic sensor failures and degradations for **robust autonomous driving perception research**.  

![Occlusion Example](https://github.com/sanjaychowdhry/nuScenes-Camera-Radar-LiDAR-Occlusion/blob/main/thumbnail.png)



---

## Camera Occlusions (Dataset on IEEE DataPort)

Camera occlusion images are provided directly as a dataset.  
Each occlusion is applied to the **six nuScenes camera views**:  

- `CAM_FRONT`  
- `CAM_FRONT_LEFT`  
- `CAM_FRONT_RIGHT`  
- `CAM_BACK`  
- `CAM_BACK_LEFT`  
- `CAM_BACK_RIGHT`  

### Occlusion Types
1. **Dirt** → severity levels `0.1`, `0.2`, `0.3`  
2. **Water-blur** → severity levels `0.1`, `0.2`, `0.3`  
3. **Scratches**  
4. **Woodscape Soiled Effect** 

Both **mini** and **trainval** versions are available with the same folder structure.  

📌 **Download from IEEE DataPort:**  
👉 [Camera Occlusion Dataset for nuScenes (IEEE DataPort)](https://ieee-dataport.org/documents/occluded-nuscenes-multi-sensor-dataset-evaluating-perception-robustness-automated-0)  

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
## Acknowledgement

This dataset is built upon the publicly available [nuScenes](https://www.nuscenes.org/) dataset by Motional. We thank the nuScenes team for providing a high-quality multi-sensor dataset that made the development of the Occluded nuScenes Dataset possible.


## Citation

If you use this dataset or scripts in your research, please cite:
```bibtex

@data{fd64-0p49-25,
doi = {10.21227/fd64-0p49},
url = {https://dx.doi.org/10.21227/fd64-0p49},
author = {Sanjay Kumar and Tim Brophy and Reenu Mohandas and Eoin Martino Grua and Ganesh Sistu and Valentina Donzella and Ciaran Eising},
publisher = {IEEE Dataport},
title = {Occluded nuScenes: A Multi-Sensor Dataset for Evaluating Perception Robustness in Automated Driving},
year = {2025} }

@ARTICLE{11204511,
  author={Kumar, Sanjay and Sharma, Sushil and Asghar, Rabia and Mohandas, Reenu and Brophy, Tim and Sistu, Ganesh and Grua, Eoin Martino and Donzella, Valentina and Eising, Ciaran},
  journal={IEEE Open Journal of Vehicular Technology}, 
  title={Exploring Sensor Impact and Architectural Robustness in Adverse Weather on BEV Perception}, 
  year={2025},
  volume={},
  number={},
  pages={1-22},
  keywords={Cameras;Radar;Laser radar;Robustness;Transformers;Computer architecture;Snow;Spaceborne radar;Rain;Degradation;Multi-Sensor Fusion;Sensor Level Occlusion;Camera Features Projection;Bird's Eye View;Vehicle Segmentation;Map Segmentation;Transformer-Based Architectures;Geometric-Based Architectures},
  doi={10.1109/OJVT.2025.3621862}}

