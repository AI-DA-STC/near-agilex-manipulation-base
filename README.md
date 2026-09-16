# Robot Manipulation Framework

## Overview
A modular framework for text-prompted robotic manipulation: GroundedSAM2 segments objects from an RGB-D point cloud via text prompt, Contact-GraspNet/AnyGrasp computes the grasp pose directly on the segmented cloud, MoveIt Task Constructor sequences the manipulation task into stages (approach, grasp, attach, lift, transit, place, retreat), and MoveIt executes the motion.

## Modules
 
| Stage | Component | Notes |
|---|---|---|
| Perception (segmentation) | GroundedSAM2 | Text prompt in, mask out on RGB-D point cloud. Zero-shot, slow. Top-down camera only. |
| Perception (grasp pose) |  AnyGrasp | Runs directly on segmented point cloud — no separate object pose detection step. Outputs full 6-DoF grasp pose. |
| Task Planning | MoveIt Task Constructor | Defines grasp as a sequence of stages (approach, grasp, attach, lift, transit, place, retreat) with fixed preset waypoints. |
| Motion Planning | MoveIt | uses poses from task planner to perform trajectory generation |

## Pipeline Flow
 
1. Capture RGB-D point cloud from top-down camera.
2. Send text prompt + point cloud to GroundedSAM2 → get segmented object mask/point cloud.
3. Run grasp pose detection (Contact-GraspNet / AnyGrasp) on segmented cloud → get 6-DoF grasp pose.
4. Pass grasp pose into MoveIt Task Constructor to build the staged task graph.
5. MoveIt executes motion plans for each stage.