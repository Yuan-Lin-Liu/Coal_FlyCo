"""Run the planner's trajectory sampling block against real polynomial trajectories."""
from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
PLANNER = ROOT / 'third_party/flyco_ros2/vendor/planner_simulator/src'


@pytest.fixture(scope='module')
def sampler(tmp_path_factory):
    compiler = shutil.which('g++')
    if compiler is None:
        pytest.skip('g++ is required for the C++ trajectory regression')
    source = (PLANNER / 'hierarchical_coverage_planner/src/planner_node.cpp').read_text()
    global_thread = source.split('void FCPlanner_PP_Node::globalPlanThread()', 1)[1]
    # Exercise production selection and sampling, without starting ROS or flight control.
    sampling = global_thread.split('rclcpp::Time t_cur = node_->now();', 1)[1]
    sampling = 'rclcpp::Time t_cur = node_->now();' + sampling.split(
        'this->last_global_path_ = r_path;', 1,
    )[0]
    directory = tmp_path_factory.mktemp('planner-trajectory-start')
    cpp = directory / 'sampling.cpp'
    cpp.write_text(r'''
#include "gcopter/trajectory.hpp"
#include <cassert>
#include <memory>
#include <string>
using namespace std;

namespace rclcpp {
struct Time {
    double value;
    Time operator-(Time other) const { return {value - other.value}; }
    double seconds() const { return value; }
};
}
struct Clock {
    rclcpp::Time current;
    rclcpp::Time now() const { return current; }
};
struct Rate {
    int sleeps = 0;
    void sleep() { ++sleeps; }
};
Trajectory<7> line(double x) {
    Piece<7>::CoefficientMat coefficients = Piece<7>::CoefficientMat::Zero();
    coefficients(0, 7) = x;
    coefficients(0, 6) = 1.;
    return Trajectory<7>({2.}, {coefficients});
}
struct Sampler {
    shared_ptr<Clock> node_ = make_shared<Clock>();
    shared_ptr<Rate> global_planning_rate_ = make_shared<Rate>();
    Eigen::VectorXd cur_pos_ = Eigen::VectorXd::Constant(5, -99.);
    Eigen::Vector3d cur_vel_ = Eigen::Vector3d::Zero();
    bool sample(bool history, double now, bool position = true, bool orientation = true) {
        Trajectory<7> last_pos_traj, last_ori_traj;
        Trajectory<7> new_pos_traj, new_ori_traj;
        rclcpp::Time last_traj_start_time{history ? 100. : 0.};
        rclcpp::Time new_traj_start_time{101.};
        vector<Eigen::VectorXd> new_path, last_path;
        vector<bool> new_indi, last_indi;
        if (position) new_pos_traj = line(10.);
        if (orientation) new_ori_traj = line(0.);
        if (history) {
            last_pos_traj = line(1.);
            last_ori_traj = line(0.);
        }
        node_->current = {now};
        for (int attempt = 0; attempt < 1; ++attempt) {
''' + sampling + r'''
            return true;
        }
        return false;
    }
};
int main(int argc, char **argv) {
    assert(argc == 2);
    const string scenario = argv[1];
    Sampler planner;
    bool sampled = false;
    double expected = 0.;
    if (scenario == "first_pending") {
        assert(!planner.sample(false, 100.9));
        assert(planner.cur_pos_(0) == -99.);
        assert(planner.global_planning_rate_->sleeps == 1);
        return 0;
    }
    if (scenario == "missing_position" || scenario == "missing_orientation") {
        assert(!planner.sample(false, 101.1,
            scenario != "missing_position", scenario != "missing_orientation"));
        assert(planner.cur_pos_(0) == -99.);
        return 0;
    }
    if (scenario == "first_start") {
        sampled = planner.sample(false, 101.);
        expected = 10.;
    } else if (scenario == "first_running") {
        sampled = planner.sample(false, 101.25);
        expected = 10.25;
    } else if (scenario == "handover_pending") {
        sampled = planner.sample(true, 100.9);
        expected = 1.9;
    } else if (scenario == "handover_start") {
        sampled = planner.sample(true, 101.);
        expected = 10.;
    } else if (scenario == "completed") {
        sampled = planner.sample(false, 104.);
        expected = 12.;
    } else {
        return 2;
    }
    assert(sampled);
    assert(abs(planner.cur_pos_(0) - expected) < 1.e-9);
    assert(planner.global_planning_rate_->sleeps == 0);
}
''')
    executable = directory / 'sampling'
    subprocess.run(
        [compiler, '-std=c++17', '-O0', '-D_GLIBCXX_ASSERTIONS',
         '-I/usr/include/eigen3', '-I' + str(PLANNER), str(cpp), '-o', str(executable)],
        check=True, capture_output=True, text=True,
    )
    return executable


@pytest.mark.parametrize('scenario', [
    'first_pending', 'first_start', 'first_running', 'handover_pending',
    'handover_start', 'completed', 'missing_position', 'missing_orientation',
])
def test_global_planner_samples_only_available_active_trajectories(sampler, scenario):
    result = subprocess.run([str(sampler), scenario], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
