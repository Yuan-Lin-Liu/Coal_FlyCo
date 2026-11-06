# Third-party sources and redistribution

FlyCo originates from https://github.com/FC-Family/FlyCo. The ROS2 migration,
Isaac sensor bridge and warehouse integration are included in this project.
The maintainer has confirmed permission to publish this modified distribution.
The upstream license and required copyright notices remain in LICENSE.FlyCo
and third_party/flyco_ros2/vendor/LICENSE. This confirmation does not replace
the upstream license or grant additional commercial rights to downstream users.

EVF-SAM: https://github.com/hustvl/EVF-SAM (Apache-2.0; bundled LICENSE retained).
SAM2: vendored under FlyCo, with its upstream license and notices retained.
Pegasus Simulator: https://github.com/PegasusSimulator/PegasusSimulator
(BSD-3-Clause), downloaded at v5.1.0 and patched during image construction.
NKSR: https://github.com/nv-tlabs/NKSR, pinned in Dockerfile.flyco.
Embree, libigl, TBB and the solver libraries retain their bundled notices.

Model files are downloaded from their original publishers using
config/models.json. Their original terms apply independently of project code.
NVIDIA Isaac Sim is obtained from NVIDIA's official container registry; each
user must accept its EULA. No NVIDIA driver or installed simulator is in Git.

The checked-in native Embree/libigl libraries target Linux x86_64. CUDA
extensions are rebuilt in Docker for the GPU architecture detected by setup.sh.

SAM_AD_Perception: https://github.com/wentengsong627-sudo/SAM_AD_Perception.
The vendored A runtime includes the generic tracking/completion fixes developed
for this workspace. Upstream provenance, asset hashes and required third-party
notices remain in third_party/SAM_AD_Perception/provenance.json, assets.lock.json
and LICENSES/. Model weights and pinned dependency downloads are excluded from Git.

Native deployment sources include the Odin ROS2 driver and x86_64/ARM64 SDK
from the existing feat/orin-px4-native integration. Provenance and license are
preserved in third_party/drivers/odin_ros_driver/SOURCE.json and LICENSE.
PX4 ROS2 messages are pinned to release/1.16; provenance and BSD license are in
third_party/px4_msgs/SOURCE.json and LICENSE. Custom SITL builds must load their
matching message overlay; the PX4 external-mode API is used without relabeling
Offboard or patching QGroundControl.
