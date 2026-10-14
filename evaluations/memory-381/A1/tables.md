## phase: window

sample t=1791999005.0 phase='window': MemAvailable 665 MB (window min/median over 81 samples: 665 / 683 MB); sampler cost 0.185 s

### MemTotal 7620 MB, by kernel category

| category | MB | share |
|---|---:|---:|
| free (MemFree) | 79 | 1.0% |
| page cache, file-backed (Buffers + Cached - Shmem) | 731 | 9.6% |
| shmem / tmpfs | 19 | 0.2% |
| anonymous, process heaps (AnonPages) | 2234 | 29.3% |
| GPU allocations (nvmap, all processes) | 4047 | 53.1% |
| nvmap page pool (freed GPU pages kept for reuse) | 0 | 0.0% |
| zram: compressed swap held in RAM | 86 | 1.1% |
| kernel slab (reclaimable) | 67 | 0.9% |
| kernel slab (unreclaimable) | 92 | 1.2% |
| kernel stacks + page tables + per-cpu | 29 | 0.4% |
| vmalloc | 63 | 0.8% |
| unexplained (MemTotal - the rows above) | 174 | 2.3% |

explained: 97.7% of MemTotal (criterion 1: >= 95%)

### Processes by component (PSS; GPU from nvmap)

| component | procs | anon | file | shmem | swapped | GPU | total in RAM |
|---|---:|---:|---:|---:|---:|---:|---:|
| VLM driver (VE, Edge-LLM) | 1 | 519 | 66 | 0 | 0 | 3382 | 3967 |
| perception (YOLOE + CLIP) | 1 | 1113 | 162 | 0 | 64 | 664 | 1939 |
| VLM bench client | 1 | 96 | 3 | 0 | 0 | 0 | 99 |
| simulator sensors | 2 | 58 | 2 | 0 | 9 | 0 | 60 |
| docker daemons | 3 | 33 | 24 | 0 | 25 | 0 | 57 |
| ROS container: python3 | 1 | 42 | 8 | 0 | 1 | 0 | 50 |
| system (other) | 65 | 26 | 14 | 9 | 70 | 0 | 49 |
| ROS container: async_slam_toolbox_node | 1 | 43 | 5 | 0 | 1 | 0 | 48 |
| robot server | 1 | 44 | 1 | 0 | 2 | 0 | 46 |
| brain | 1 | 37 | 1 | 0 | 7 | 0 | 38 |
| simulator body | 1 | 30 | 1 | 0 | 4 | 0 | 31 |
| ROS container: ros2 launch | 1 | 26 | 3 | 0 | 0 | 0 | 28 |
| tunnel proxy | 1 | 27 | 1 | 0 | 8 | 0 | 28 |
| nav load / harness python | 1 | 21 | 1 | 0 | 0 | 0 | 22 |
| ROS container: planner_server | 1 | 15 | 2 | 0 | 0 | 0 | 17 |
| ROS container: ros2_control_node | 1 | 11 | 4 | 0 | 1 | 0 | 15 |
| ROS container: lifecycle_manager | 2 | 13 | 2 | 0 | 1 | 0 | 15 |
| ROS container: foxglove_bridge | 1 | 11 | 3 | 0 | 2 | 0 | 14 |
| ROS container: bt_navigator | 1 | 9 | 4 | 0 | 0 | 0 | 14 |
| desktop (gdm, gnome) | 3 | 13 | 1 | 0 | 2 | 0 | 14 |
| ROS container: controller_server | 1 | 9 | 2 | 0 | 1 | 0 | 11 |
| measurement (sampler, tegrastats) | 4 | 7 | 2 | 0 | 2 | 0 | 9 |
| ROS container: behavior_server | 1 | 8 | 1 | 0 | 1 | 0 | 9 |
| ROS container: twist_mux | 1 | 8 | 1 | 0 | 1 | 0 | 9 |
| ROS container: collision_monitor | 1 | 7 | 1 | 0 | 1 | 0 | 8 |
| ROS container: robot_state_publisher | 1 | 6 | 1 | 0 | 1 | 0 | 7 |
| (components under 5 MB, 0) | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

### Perception's torch allocator (nearest log, t=1791999004.8)

* allocated_mb: 478.5
* reserved_mb: 550.0
* peak_allocated_mb: 527.4
* peak_reserved_mb: 550.0
* alloc_retries: 0

## phase: perception

sample t=1791998679.7 phase='perception': MemAvailable 4665 MB (window min/median over 6 samples: 4665 / 4675 MB); sampler cost 0.124 s

### MemTotal 7620 MB, by kernel category

| category | MB | share |
|---|---:|---:|
| free (MemFree) | 1598 | 21.0% |
| page cache, file-backed (Buffers + Cached - Shmem) | 3193 | 41.9% |
| shmem / tmpfs | 25 | 0.3% |
| anonymous, process heaps (AnonPages) | 1670 | 21.9% |
| GPU allocations (nvmap, all processes) | 664 | 8.7% |
| nvmap page pool (freed GPU pages kept for reuse) | 0 | 0.0% |
| zram: compressed swap held in RAM | 39 | 0.5% |
| kernel slab (reclaimable) | 103 | 1.4% |
| kernel slab (unreclaimable) | 94 | 1.2% |
| kernel stacks + page tables + per-cpu | 26 | 0.3% |
| vmalloc | 57 | 0.7% |
| unexplained (MemTotal - the rows above) | 151 | 2.0% |

explained: 98.0% of MemTotal (criterion 1: >= 95%)

### Processes by component (PSS; GPU from nvmap)

| component | procs | anon | file | shmem | swapped | GPU | total in RAM |
|---|---:|---:|---:|---:|---:|---:|---:|
| perception (YOLOE + CLIP) | 1 | 1176 | 246 | 0 | 0 | 664 | 2087 |
| simulator sensors | 2 | 66 | 3 | 0 | 0 | 0 | 69 |
| docker daemons | 3 | 35 | 26 | 0 | 23 | 0 | 61 |
| system (other) | 65 | 31 | 16 | 9 | 65 | 0 | 56 |
| ROS container: python3 | 1 | 41 | 10 | 0 | 0 | 0 | 51 |
| brain | 1 | 44 | 3 | 0 | 0 | 0 | 47 |
| robot server | 1 | 45 | 2 | 0 | 0 | 0 | 46 |
| tunnel proxy | 1 | 35 | 1 | 0 | 0 | 0 | 36 |
| simulator body | 1 | 33 | 2 | 0 | 0 | 0 | 35 |
| ROS container: async_slam_toolbox_node | 1 | 23 | 9 | 0 | 0 | 0 | 32 |
| ROS container: ros2 launch | 1 | 26 | 3 | 0 | 0 | 0 | 29 |
| ROS container: foxglove_bridge | 1 | 12 | 6 | 0 | 0 | 0 | 18 |
| ROS container: bt_navigator | 1 | 9 | 9 | 0 | 0 | 0 | 18 |
| ROS container: ros2_control_node | 1 | 11 | 6 | 0 | 0 | 0 | 17 |
| desktop (gdm, gnome) | 3 | 14 | 1 | 0 | 0 | 0 | 15 |
| ROS container: lifecycle_manager | 2 | 13 | 2 | 0 | 0 | 0 | 15 |
| ROS container: planner_server | 1 | 12 | 3 | 0 | 0 | 0 | 14 |
| ROS container: controller_server | 1 | 8 | 3 | 0 | 0 | 0 | 11 |
| measurement (sampler, tegrastats) | 4 | 9 | 2 | 0 | 0 | 0 | 11 |
| ROS container: behavior_server | 1 | 7 | 3 | 0 | 0 | 0 | 10 |
| ROS container: collision_monitor | 1 | 7 | 2 | 0 | 0 | 0 | 9 |
| ROS container: twist_mux | 1 | 8 | 1 | 0 | 0 | 0 | 9 |
| ROS container: robot_state_publisher | 1 | 6 | 2 | 0 | 0 | 0 | 8 |
| (components under 5 MB, 0) | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

### Perception's torch allocator (nearest log, t=1791998678.2)

* allocated_mb: 478.7
* reserved_mb: 550.0
* peak_allocated_mb: 527.4
* peak_reserved_mb: 550.0
* alloc_retries: 0

