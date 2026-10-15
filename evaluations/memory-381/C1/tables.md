sample t=1792025937.1 phase='window': MemAvailable 1089 MB (window min/median over 83 samples: 1089 / 1105 MB); sampler cost 0.146 s

### MemTotal 7620 MB, by kernel category

| category | MB | share |
|---|---:|---:|
| free (MemFree) | 81 | 1.1% |
| page cache, file-backed (Buffers + Cached - Shmem) | 1153 | 15.1% |
| shmem / tmpfs | 24 | 0.3% |
| anonymous, process heaps (AnonPages) | 2020 | 26.5% |
| GPU allocations (nvmap, all processes) | 3795 | 49.8% |
| nvmap page pool (freed GPU pages kept for reuse) | 0 | 0.0% |
| zram: compressed swap held in RAM | 118 | 1.6% |
| kernel slab (reclaimable) | 67 | 0.9% |
| kernel slab (unreclaimable) | 92 | 1.2% |
| kernel stacks + page tables + per-cpu | 28 | 0.4% |
| vmalloc | 63 | 0.8% |
| unexplained (MemTotal - the rows above) | 180 | 2.4% |

explained: 97.6% of MemTotal (criterion 1: >= 95%)

### Processes by component (PSS; GPU from nvmap)

| component | procs | anon | file | shmem | swapped | GPU | total in RAM |
|---|---:|---:|---:|---:|---:|---:|---:|
| VLM driver (VE, Edge-LLM) | 1 | 514 | 76 | 0 | 5 | 3382 | 3971 |
| perception (YOLOE + CLIP) | 1 | 962 | 182 | 0 | 102 | 412 | 1556 |
| VLM bench client | 1 | 88 | 3 | 0 | 9 | 0 | 91 |
| system (other) | 67 | 36 | 16 | 18 | 58 | 0 | 69 |
| docker daemons | 3 | 38 | 22 | 0 | 23 | 0 | 60 |
| ROS container: python3 | 1 | 45 | 9 | 0 | 0 | 0 | 53 |
| ROS container: async_slam_toolbox_node | 1 | 40 | 8 | 0 | 0 | 0 | 48 |
| simulator sensors | 2 | 45 | 2 | 0 | 23 | 0 | 47 |
| robot server | 1 | 39 | 1 | 0 | 7 | 0 | 40 |
| ROS container: ros2 launch | 1 | 26 | 3 | 0 | 0 | 0 | 29 |
| simulator body | 1 | 25 | 1 | 0 | 11 | 0 | 26 |
| brain | 1 | 23 | 1 | 0 | 22 | 0 | 24 |
| nav load / harness python | 1 | 21 | 1 | 0 | 0 | 0 | 22 |
| ROS container: ros2_control_node | 1 | 16 | 5 | 0 | 0 | 0 | 21 |
| tunnel proxy | 1 | 18 | 1 | 0 | 17 | 0 | 19 |
| ROS container: bt_navigator | 1 | 10 | 7 | 0 | 0 | 0 | 17 |
| ROS container: lifecycle_manager | 2 | 14 | 2 | 0 | 0 | 0 | 16 |
| ROS container: planner_server | 1 | 14 | 3 | 0 | 0 | 0 | 16 |
| ROS container: controller_server | 1 | 11 | 3 | 0 | 0 | 0 | 14 |
| ROS container: behavior_server | 1 | 9 | 2 | 0 | 0 | 0 | 12 |
| ROS container: collision_monitor | 1 | 7 | 2 | 0 | 0 | 0 | 9 |
| ROS container: robot_state_publisher | 1 | 6 | 1 | 0 | 0 | 0 | 7 |
| ROS container: twist_mux | 1 | 6 | 1 | 0 | 0 | 0 | 6 |
| measurement (sampler, tegrastats) | 4 | 4 | 1 | 0 | 4 | 0 | 6 |
| desktop (gdm, gnome) | 3 | 2 | 1 | 0 | 13 | 0 | 2 |
| (components under 5 MB, 0) | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

### Perception's torch allocator (nearest log, t=1792025935.5)

* allocated_mb: 234.6
* reserved_mb: 298.0
* peak_allocated_mb: 392.4
* peak_reserved_mb: 402.0
* alloc_retries: 0
