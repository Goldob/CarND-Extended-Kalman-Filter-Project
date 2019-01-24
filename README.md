# Extended Kalman Filter Project

The following is a solution for [EKF Project](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) from Term 2 of Udacity's Self Driving Car Nanodegree. Refer to the main repository for compilation and testing instructions.

The following tables show results (RMSE) achieved on both datasets provided in [the simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/v1.45).

 #### Dataset 1

|        | Sensor Fusion | Radar Only | Lidar Only    |
| -------| ------------- | ---------- | ------------- |
| **X**  | 0.1085        | 0.2452     | 0.1494        |
| **Y**  | 0.0880        | 0.3390     | 0.1164        |
| **VX** | 0.4689        | 0.6112     | 0.6571        |
| **VY** | 0.4436        | 0.6954     | 0.4497        |

 #### Dataset 2

|        | Sensor Fusion | Radar Only | Lidar Only    |
| -------| ------------- | ---------- | ------------- |
| **X**  | 0.0856        | 0.3091     | 0.1198        |
| **Y**  | 0.1059        | 0.3953     | 0.1278        |
| **VX** | 0.5236        | 0.6667     | 0.6346        |
| **VY** | 0.4384        | 0.7822     | 0.4589        |

The performance of the algorithm was tested in three operation modes:

* process both measurement types (radar & lidar)
* process radar measurements only, discard lidar
* process lidar measurements only, discard radar

As can be clearly seen from the tables, using EKF to combine measurements from multiple sources does indeed end in better results than could be achieved with any single one of them. Another observation is that lidar measurements provide better position estimation than radar (although are still outperformed by sensor fusion approach), which is consistent with the former having in general much better spatial resolution from the latter. Interestingly, lidar also provides comparable (and in some cases even better) velocity estimation, despite not being able to measure it explicitly.
