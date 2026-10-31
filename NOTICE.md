# Attribution and scope

This project contains original task code, documentation, measured simulation logs,
and a screen recording of ROBOTIS AIWorker-SG2 running in Gazebo.
The ROS-independent waypoint and Q-learning modules and the recording utilities
were reused from an earlier project by the same contributor; their original
module names are retained. Navigation missions and their deliverables are not
part of this repository.

The Dockerfile downloads the official `robotis/ai-worker:amd64-2.3.1` image,
pinned to SHA-256 `99748390fd747165625000ab1ee36184516f2d59f21f7fc931474e899f341e03`.
The source revision inside the tested image is
`691db567f873edca340eea01f190a28935782412`:
https://github.com/ROBOTIS-GIT/ai_worker/tree/691db567f873edca340eea01f190a28935782412

ROBOTIS models, meshes, robot names and logos shown in the demonstration remain
the property of their respective rights holders. This repository's Apache-2.0
license applies to its original contributions and does not relicense third-party
models, images, fonts or dependencies. No ROBOTIS source tree or binary image is
vendored here. This is an independent course project, without an endorsement claim.
