# Step 1: Eigen Basics (1 week)

## Install & run
```bash
sudo apt install libeigen3-dev        # ROS 2 install unte already untundi
cd eigen_basics
cmake -S . -B build && cmake --build build
./build/01_vectors_matrices           # prati lesson ni ilaa run cheyyandi
```

## Week plan (roju ki ~1 hour)

| Day | File | Em nerchukovali | Check yourself |
|---|---|---|---|
| 1 | `01_vectors_matrices` | Vector3d vs VectorXd, `<<` initializer, `block()` | `q.head(3)`, `A.col(2)` print cheyyagalara? |
| 2 | `02_multiply_inverse` | `*`, transpose, inverse vs `solve()`, pseudo-inverse | Singular matrix ki det = 0 enduku? |
| 3 | `03_rotations` | Rotation matrix, AngleAxis, Quaternion, RPY, slerp | R^-1 = R^T enduku? Paper meeda cheppandi |
| 4 | `04_transforms` | Isometry3d, frame naming `T_a_b`, chaining, inverse | Point vs direction difference |
| 5 | `05_mini_project_fk` | 2-link FK rendu methods | File chudakunda SOLO ga raayandi |
| 6–7 | Challenges | Workspace plot + numerical Jacobian | Step 4 ki ready |

## Rules
1. Run cheyyadaniki **mundu** output paper meeda guess cheyyandi. Taruvata run chesi compare cheyyandi.
2. Prati file chivari lo `TRY:` undi. Adi skip cheyyakandi.
3. `6.12e-17` lanti chinna numbers kanipisthe adi floating-point valla vachina zero. Bug kaadu.
   Compare cheyyadaniki `==` kaakunda `isApprox()` vaadandi.

