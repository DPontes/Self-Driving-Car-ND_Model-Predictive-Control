# Model Predictive Control Project

## Overview

The project implements a **Model Predictive Control (MPC)** algorithm in C++ using **Ipopt** and **CppAD** automatic differentiation libraries to control steering and throttle while driving a simulated vehicle around a track following reference waypoints.

At each timestep, the vehicle's state (global position, heading, speed, steering and throttle) and the closest six waypoints are received by the controller.

Before applying the MPC control, the **vehicle's state is estimated after some expected latency** (processing time and ~100ms actuator delay) using the modeled **motion equations (bicycle model)**.

This estimated state position is then used to **convert the waypoints from global to vehicle coordinate systems**. The waypoints are then preprocessed with **waypoint interpolation** and **weighting** before applying a **3rd order polyfit** to use as the reference driving path.

The polyfit coefficients and estimated vehicle state after latency are then used by the MPC controller to **optimize a planned path over ~1sec horizon**.
