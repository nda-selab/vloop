# Multi Floorシーケンス

## 実計測rosbag
```text
duration:    1:49s (109s)
start:       Aug 19 2022 06:16:32.50 (1660857392.50)
end:         Aug 19 2022 06:18:22.50 (1660857502.50)
size:        2.4 GB
messages:    25679
compression: none [2638/2638 chunks]
types:       sensor_msgs/Image          [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/Imu            [6a62c6daae103f4ff57a132d6f95cec2]
             velodyne_msgs/VelodyneScan [50804fc9533a0e579e6322c04ae70566]
topics:      /cmu_sp1/camera_1/image_raw    2637 msgs    : sensor_msgs/Image         
             /imu/data                     21951 msgs    : sensor_msgs/Imu           
             /velodyne_packets              1091 msgs    : velodyne_msgs/VelodyneScan
```

## 仮想往復rosbag
```text
duration:     3:39s (219s)
start:        Aug 19 2022 06:16:32.50 (1660857392.50)
end:          Aug 19 2022 06:20:12.50 (1660857612.50)
size:         3.7 GB
messages:     51357
compression:  lz4 [5275/5275 chunks; 78.36%]
uncompressed: 4.7 GB @ 22.0 MB/s
compressed:   3.7 GB @ 17.3 MB/s (78.36%)
types:        sensor_msgs/Image          [060021388200f6f0f447d0fcd9c64743]
              sensor_msgs/Imu            [6a62c6daae103f4ff57a132d6f95cec2]
              velodyne_msgs/VelodyneScan [50804fc9533a0e579e6322c04ae70566]
topics:       /cmu_sp1/camera_1/image_raw    5274 msgs    : sensor_msgs/Image         
              /imu/data                     43901 msgs    : sensor_msgs/Imu           
              /velodyne_packets              2182 msgs    : velodyne_msgs/VelodyneScan
```

# Long Corridorシーケンス

## 実計測rosbag
```text
duration:    4:39s (279s)
start:       Jul 25 2023 12:01:52.82 (1690254112.82)
end:         Jul 25 2023 12:06:32.82 (1690254392.82)
size:        6.0 GB
messages:    65454
compression: none [6721/6721 chunks]
types:       sensor_msgs/Image          [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/Imu            [6a62c6daae103f4ff57a132d6f95cec2]
             velodyne_msgs/VelodyneScan [50804fc9533a0e579e6322c04ae70566]
topics:      /camera_1/image_raw    6720 msgs    : sensor_msgs/Image         
             /imu/data             55957 msgs    : sensor_msgs/Imu           
             /velodyne_packets      2777 msgs    : velodyne_msgs/VelodyneScan
```

## 仮想往復rosbag

```text
duration:     9:19s (559s)
start:        Jul 25 2023 12:01:52.82 (1690254112.82)
end:          Jul 25 2023 12:11:12.81 (1690254672.81)
size:         9.9 GB
messages:     130907
compression:  lz4 [13441/13441 chunks; 82.07%]
uncompressed: 12.1 GB @ 22.0 MB/s
compressed:    9.9 GB @ 18.1 MB/s (82.07%)
types:        sensor_msgs/Image          [060021388200f6f0f447d0fcd9c64743]
              sensor_msgs/Imu            [6a62c6daae103f4ff57a132d6f95cec2]
              velodyne_msgs/VelodyneScan [50804fc9533a0e579e6322c04ae70566]
topics:       /camera_1/image_raw    13440 msgs    : sensor_msgs/Image         
              /imu/data             111914 msgs    : sensor_msgs/Imu           
              /velodyne_packets       5553 msgs    : velodyne_msgs/VelodyneScan
```

# 修正前後のIMUトピック折返し部分の比較

## 修正前のIMUトピック出力（一部抜粋）

```text
header: 
  seq: 466954
  stamp: 
    secs: 1517157499
    nsecs: 161280000
  frame_id: "epson"
angular_velocity: 
  x: 0.03333069756627083
  y: -0.10018593817949295
  z: 0.006512821186333895
linear_acceleration: 
  x: 2.5856523513793945
  y: 0.007063014432787895
  z: 10.583958625793457
---
header: 
  seq: 466954
  stamp: 
    secs: 1690254392
    nsecs: 818357467
  frame_id: "epson"
angular_velocity: 
  x: -0.03333069756627083
  y: 0.10018593817949295
  z: -0.006512821186333895
linear_acceleration: 
  x: 2.5856523513793945
  y: 0.007063014432787895
  z: 10.583958625793457
```

## 修正前のIMUトピック出力（一部抜粋）

```text
header: 
  seq: 466954
  stamp: 
    secs: 1690254392
    nsecs: 818222500
  frame_id: "epson"
angular_velocity: 
  x: 0.03333069756627083
  y: -0.10018593817949295
  z: 0.006512821186333895
linear_acceleration: 
  x: 2.5856523513793945
  y: 0.007063014432787895
  z: 10.583958625793457
---
header: 
  seq: 466954
  stamp: 
    secs: 1690254392
    nsecs: 818357467
  frame_id: "epson"
angular_velocity: 
  x: -0.03333069756627083
  y: 0.10018593817949295
  z: -0.006512821186333895
linear_acceleration: 
  x: 2.5856523513793945
  y: 0.007063014432787895
  z: 10.583958625793457
```
