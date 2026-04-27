# Odometry Architecture

Two separate pipelines — they do not interfere with each other.

## Simulation

```
Gazebo p3d plugin ──→ /odom ──→ odom_tf_broadcaster ──→ TF: odom → base_link

stanford_controller → odom/raw   ← orphaned (no subscriber in sim)
```

## Real Robot

```
stanford_controller → odom/raw ──┐
imu/data ────────────────────────┴──→ EKF (baselink_to_odom_ekf)
                                            ├──→ /odom
                                            └──→ TF: odom → base_link
```

The EKF uses only the **velocity fields** from `odom/raw` (`vx`, `vy`, `vyaw`).
The dead-reckoned pose inside `odom/raw` is ignored.
