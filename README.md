# DR-REMOVER (IEEE Transactions on Intelligent Vehicles 2024)
- TITILE: An Efficient Dynamic Object Remover Using Dual-Resolution Occupancy Grids for Constructing Static Point Cloud Maps
- Description:DR-REMOVER is an offline dynamic point removal method, which removes dynamic points more accurately, avoids more false removal of static points (including ground points), and achieves better results on KITTI and Apollo datasets.
- Limits: Limited by the selected height interval, it is more suitable for wide road scenes.

---

## Test Environment.

The code is tested successfully at

- Linux 20.04 LTS
- ROS noetic
- python 3.8
## Requirements

- pcl >=1.7
- python >=3.0
- Eigen 
## Build Our Package

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/zhongbusishaonianyou/DR-REMOVER.git
cd .. && catkin make 
```
## Prepared dataset

- You can download the rosbag we created directly.
  
- Apollo dataset(seq 00,seq 01,seq 02,seq 03,seq 04)
   - we downloaded apollo dataset from here: https://github.com/PRBonn/MapMOS.Please read dataset_description.yml(The file is in the compressed package after download) if you download Apollo dataset from above link.
   - Paper: "L3-net: Towards learning based lidar localization for autonomous driving"
   ```bash
   - rawseq: start-id - end-id -> LiDAR-MOS-seq: start-id - end-id   
  - 1: 6500  - 7500  -> 01: 000000 - 001500
  - 2: 18500 - 20000 -> 02: 000000 - 001500 
  - 2: 22300 - 24300 -> 00: 000000 - 002000
  - 3: 3100  - 3600  -> 03: 000000 - 000500
  - 4: 1500  - 3100  -> 04: 000000 - 000600
   ```
- semanticPOSS dataset(seq 00,seq 01,seq 02,seq 04,seq 05)

  - Paper:"SemanticPOSS: A Point Cloud Dataset with Large Quantity of Dynamic Instances"
  - dataset description: The data is collected in a highly congested environment, and the point cloud data contains a large number of dynamic pedestrians.

- you can download files frome here :https://drive.google.com/drive/my-drive?dmr=1&ec=wgc-drive-globalnav-goto
- For semanticKITTI,we use ERASOR rosbag files directly. you can run following command to download them

```bash
wget https://urserver.kaist.ac.kr/publicdata/erasor/rosbag/00_4390_to_4530_w_interval_2_node.bag
wget https://urserver.kaist.ac.kr/publicdata/erasor/rosbag/01_150_to_250_w_interval_1_node.bag
wget https://urserver.kaist.ac.kr/publicdata/erasor/rosbag/02_860_to_950_w_interval_2_node.bag
wget https://urserver.kaist.ac.kr/publicdata/erasor/rosbag/05_2350_to_2670_w_interval_2_node.bag
wget https://urserver.kaist.ac.kr/publicdata/erasor/rosbag/07_630_to_820_w_interval_2_node.bag
```

## Description of Preprocessed Rosbag Files

- Note that each label of the point is assigned in `intensity`.
- Note：After point cloud drop sampling, there are a few points with inaccurate labels in the map.

   - SemanticKITTI dataset
```
# 252: "moving-car"
# 253: "moving-bicyclist"
# 254: "moving-person"
# 255: "moving-motorcyclist"
# 256: "moving-on-rails"
# 257: "moving-bus"
# 258: "moving-truck"
# 259: "moving-other-vehicle"

DYNAMIC_CLASSES = {252, 253, 254, 255, 256, 257, 258, 259};
```
  - Apollo dataset
    - Semantic labels are the same as SemanticKITTI datasets.These labels were manually annotated by Xieyuanli Chen et al.
    - Paper: "Static map generation from 3D LiDAR point clouds exploiting ground segmentation"
    - DYNAMIC_CLASSES = {252, 253, 254, 255, 256, 257, 258, 259};
  - SemanticPOSS dataset
```
# 04: "person"
# 05: "persons"
# 06: "rider"
# 07: "car"
# 08: "trunk"
# 09: "plants"
# 10: "road-sign"
# 12: "traffic-sign"
# 13: "pole"
# 15: "building"
# 16: "stone"
# 17: "fence"
# 21: "bike"
# 22: "road"

DYNAMIC_CLASSES = {4,5,6};
```
- NOTE：The semanticPOSS dataset does not label the motion state of the object, so it is only used for qualitative evaluation experiments.

## How to Run DR-REMOVER

**Step 1. Build Raw Map**

- We used the map generation method of ERASOR and utilized their code. For specific details, you can find them here: https://github.com/LimHyungTae/ERASOR.
- The original map we established was also downsampled by 0.2m3.You can download our building raw map from here:https://drive.google.com/drive/my-drive?dmr=1&ec=wgc-drive-globalnav-goto

**Step 2. Run DR-REMOVER**

- Set the following parameters in `config/kitti_seq_00.yaml`.

  - `initial_map_path`: The path of built raw map
  - `save_path`: The save path for generating static maps.

- Set "load" file name in run_remover.launch

```bash
source devel/setup.bash
roslaunch remover run_remover.launch 
rosbag play xxxxxxxxxxxxxxxxxxx.bag
```
**Step 3. Save the static map**
-  run following command on another bash.

```bash
rostopic pub /saveflag std_msgs/Float32 "data: 0.2"
```
- "0.2" denotes voxelization size.

- The results will be saved under the `save_path` folder.

## Calculate PR/RR

- The experimental results of DR-REMOVER are in the "results" folder.
- We directly use the code of ERASOR. To run the Python code, you need to install the following packages:

```bash
pip install pypcd
pip install tqdm
pip install scikit-learn
pip install tabulate
```
- open analysis.py file,set "--gt " and "--est" path  in "load_pcd" function
- Then, run the analysis code as follows:

```bash
python analysis_py3.py 
```
## DR-REMOVER RESULTS

- We modified the code to make it more efficient and concise, and the experimental results are slightly different from the paper:
- SemanticKITTI dataset

  | Seq. | PR [%] | RR [%] | F1 score |
  | ---- | :----: | :----: | :------: |
  | 00   | 98.09  | 98.42  |  0.983   |
  | 01   | 99.35  | 94.78  |  0.970   | 
  | 02   | 95.33  | 97.39  |  0.963   |
  | 05   | 99.16  | 99.01  |  0.991   |
  | 07   | 95.49  | 98.70  |  0.971   |

- Apollo dataset

  | Seq. | PR [%] | RR [%] | F1 score |
  | ---- | :----: | :----: | :------: |
  | 00   | 99.11  | 97.81  |  0.985   |
  | 01   | 98.69  | 99.01  |  0.989   |
  | 02   | 99.11  | 99.18  |  0.991   | 
  | 03   | 99.04  | 98.27  |  0.987   |
  | 04   | 98.47  | 99.76  |  0.991   | 

## Description of the experimental dataset

- SemanticKITTI dataset

  - Sequence 00: 4,390~4,530
  - Sequence 01: 150~250
  - Sequence 02: 860~950
  - Sequence 05: 2,350~2,670
  - Sequence 07: 630~820

- Apollo dataset

  - Sequence 00: 00~350
  - Sequence 01: 700~950
  - Sequence 02: 860~950
  - Sequence 03: 150~450
  - Sequence 04: 900~1550

- POSS dataset

  - Sequence 00: 00~370
  - Sequence 01: 00~300
  - Sequence 02: 00~500
  - Sequence 04: 00~200
  - Sequence 05: 00~200

## Visualization of DR-REMOVER

- change files path in `launch/viz_map.launch` correctly
  
  - `/static_map_name`: building global static point cloud map
  - `/dynamic_map_name`: including the removing dynamic points

- After setting these parameters, run following command:

```bash
roslaunch remover viz_map.launch
```
## About source code

- Due to the sparsity of the point cloud, `L_max` should not be set too large. `min_h `and `max_h` need to be adjusted according to the environment. Generally speaking, min_h = -1m and max_h = 3m are completely acceptable.You can adjust the values of parameters such as `N_x` and `N_y` to better understand the code.

- `minimum_num_pts`,`Thres`,`Kappa`,`gf_dist_thr`,`gf_iter_times`,`gf_mini_num_points`,`gf_seeds_heigt_thr`;These parameters are set the same across all sequences, and we don't think it's necessary to adjust these parameters, even though changing them might give the method better results.It should be noted that if the number of radar lines is small, the `minimum_num_pts` value can be reduced appropriately.

- DR-REMOVER is relatively stable, and generally does not need to adjust parameters if there is no special road environment.

- According to results,you can choose whether to use `FOD_flag` and `overlap_flag` appropriately,you can also choose whether to run the `spatial distribution test` function based on the level of the ground.On all the experimental sequences, we used one code without accurate parameter and function tuning.

## Citation

    @ARTICLE{10540294,

    author={Zhang, Guangyi and Zhang, Tao and Wang, Rui and Hou, Lanhua},
    journal={IEEE Transactions on Intelligent Vehicles},
    title={DR-REMOVER: An Efficient Dynamic Object Remover Using Dual-Resolution Occupancy Grids for Constructing Static Point Cloud Maps},
    year={2024},
    volume={},
    number={},
    pages={1-13},
    }

    @article{lim2021erasor,
    title={ERASOR: Egocentric Ratio of Pseudo Occupancy-Based Dynamic Object Removal for Static 3D Point Cloud Map Building},
    author={Lim, Hyungtae and Hwang, Sungwon and Myung, Hyun},
    journal={IEEE Robotics and Automation Letters},
    volume={6},
    number={2},
    pages={2272--2279},
    year={2021},
    publisher={IEEE}
    }
