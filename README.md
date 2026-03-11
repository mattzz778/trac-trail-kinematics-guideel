# trac-trail-kinematics-guide
# Tractor–Trailer Trip Processing & ROS2 Bag Generation


## 1. Initialization

Run:

```
initialize.m
```

### What it does

* **Loads dataset**

  * Searches for the file:

```
saved-gps_trips_data_snapped.mat
```

* **Trip selection**

  * Prompts the user to enter a **Trip ID** from the dataset.

<img width="260" height="80" alt="image" src="https://github.com/user-attachments/assets/3ae11a2c-d42a-4d81-8add-9ad2115cf2ab" />


* **Configuration**

  * Automatically computes **kinematic offsets (`d`)**
  * Prepares **timeseries objects** required for the simulation.

---

## 2. Simulation & Analysis

Run:

```
runplot.m
```

### What it does

#### Model Execution

Runs the Simulink model:

```
tractor_trailer_kinematics.slx
```




<img width="1395" height="623" alt="image" src="https://github.com/user-attachments/assets/c1cb8fb8-aca0-4005-bb10-2688aa2f08ea" />







#### Kinematic Processing

* Calculates **articulation angles**
* Simulates trailer motion including:

  * **Swing velocity**
  * **Pull velocity**

#### Data Export

The processed data is saved as:

```
Full_Vehicle_Data_Trip_X.csv
Full_Vehicle_Data_Trip_X.mat
```

#### Visual Validation

The script generates **five validation plots**, including:

* Speed profiles
* Angular dynamics
* Local trajectory
* Geographic path
* Kinematic alignment validation

---

![5](https://github.com/user-attachments/assets/3cac4ada-79e7-4f4e-b7e6-4478c0d3017e)          ![4](https://github.com/user-attachments/assets/d9d895dd-059c-4e8e-8b51-0f8a3f295586)          ![3](https://github.com/user-attachments/assets/5dc4ca6e-dde6-41ce-8fae-0f4419d4886c)          ![2](https://github.com/user-attachments/assets/8715721e-763e-4171-8e8c-a0493e326f2d)          ![1](https://github.com/user-attachments/assets/a3592620-16d5-4d97-b569-d1cdbade6b8d)        ![filtered](https://github.com/user-attachments/assets/731959d3-79bd-4f55-b551-7185937fd9e7)






## 3. ROS2 Bag Export

Run:

```
ros_last_25.m
```

### What it does

#### Upsampling

Interpolates the processed data to:

```
100 Hz (dt = 0.01 s)
```

#### Filtering

Applies a **smoothing window filter** to reduce:

* GPS noise
* Signal jitter

#### ROS2 Bag Writing

Data is written to the topic:

```
/tractor_01/saved_vehicle_control
```

Using the message type:

```
saved_msgs/TractorTrailerState
```

#### Output

A **uniquely named ROS2 bag folder** is created in the current working directory.



<img width="376" height="132" alt="image" src="https://github.com/user-attachments/assets/bfd1b15b-4899-45b2-a518-cba4c71858ee" />

---
