#include <drake/common/find_resource.h>
#include <drake/common/text_logging_gflags.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/piecewise_polynomial_trajectory.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsers/urdf_parser.h>
#include <drake/multibody/rigid_body_plant/drake_visualizer.h>
#include <drake/multibody/rigid_body_plant/rigid_body_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/systems/primitives/signal_logger.h>

#include <Algo/spline.h>

#include <random>

#include "drake.h"
#include "drake-internal.h"
#include "RAI_machine.h"

#include <Core/thread.h>

#include <memory>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "lcmtypes/robotlocomotion/robot_plan_t.hpp"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
//#include "state_machine_system.h"
#include "iiwa_common.h"
#include "iiwa_wsg_diagram_factory.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmtypes/drake/lcmt_contact_results_for_viz.hpp"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_status.hpp"
#include "drake/lcmtypes/drake/lcmt_schunk_wsg_command.hpp"
#include "robot_plan_interpolator.h"
#include "schunk_wsg_controller.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "world_sim_tree_builder.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_int32(target, 0, "ID of the target to pick.");
DEFINE_double(orientation, 2 * M_PI, "Yaw angle of the box.");
DEFINE_int32(start_position, 1, "Position index to start from");
DEFINE_int32(end_position, 2, "Position index to end at");
DEFINE_double(dt, 1e-3, "Integration step size");
DEFINE_double(realtime_rate, 0.0, "Rate at which to run the simulation, "
    "relative to realtime");
DEFINE_bool(quick, false, "Run only a brief simulation and return success "
    "without executing the entire task");

namespace drake {

double realtime_rate = 1.0; // "Rate at which to run the simulation, relative to realtime"
double simulation_time = 2.0; //"How long to simulate"

Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> conv_arr2eigen(const arr& X){
  if(X.nd==2) return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(X.p, X.d0, X.d1);
  return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(X.p, X.d0, 1);
}

arr conv_eigen2arr(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& X){
  arr x(X.data(), X.size());
  if(X.IsRowMajor) return x.reshape(X.rows(), X.cols()); //by reference!
  x.reshape(X.cols(), X.rows());
  return ~x; //by copy!

//  arr x(X.rows(), X.cols());
//  for(uint i=0;i<x.d0;i++) for(uint j=0;j<x.d1;j++) x(i,j) = X(i,j);
//  return x;
//  return .reshape(X.rows(), X.cols());
}


MyDrake::MyDrake(int argc, char *argv[]){
  s = new sMyDrake;
  gflags::SetUsageMessage(
        "Simulates the Kuka iiwa arm tracking a trajectory with an inverse "
        "dynamics controller.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();

  // Instantiate LCM interface and start receiving. This is needed for
  // communicating with drake-visualizer
  s->lcm = std::make_unique<drake::lcm::DrakeLcm>();
  s->lcm->StartReceiveThread();
}

MyDrake::~MyDrake(){
  delete s;
}

void MyDrake::addKukaPlant(){
  // Load the Kuka model from a URDF-file
  auto iiwa = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        "/opt/drake/share/drake/drake/manipulation/models/iiwa_description/urdf/"
        "iiwa14_polytope_collision.urdf",
        drake::multibody::joints::kFixed, iiwa.get());

  // Add the Kuka arm to the diagram. The RigidBodyPlant block has one input
  // (actuator torques) and one output (joint state).
  s->kukaPlant = s->builder.AddSystem<drake::systems::RigidBodyPlant<double>>(std::move(iiwa));

  // Add a visualizer to the diagram and connect its input to the output of the
  // plant.
  s->visualizer = s->builder.AddSystem<drake::systems::DrakeVisualizer>(
        s->kukaPlant->get_rigid_body_tree(), s->lcm.get());
  s->builder.Connect(s->kukaPlant->get_output_port(0), s->visualizer->get_input_port(0));
}

void MyDrake::addController(){
  const int kNumPositions{s->kukaPlant->get_num_positions()};

  // All the gains are for acceleration, not directly responsible for generating
  // torques. These are set to high values to ensure good tracking. These gains
  // are picked arbitrarily.
  drake::VectorX<double> kp{100 * drake::VectorX<double>::Ones(kNumPositions)};
  drake::VectorX<double> kd{2 * kp.cwiseSqrt()};
  drake::VectorX<double> ki{drake::VectorX<double>::Ones(kNumPositions)};
  s->controller = s->builder.AddSystem<
      drake::systems::controllers::InverseDynamicsController<double>>(s->kukaPlant->get_rigid_body_tree().Clone(), kp, ki, kd,
                                                                      true /*has_reference_acceleration*/);

  // Connect the controller's output to the plant's input
  drake::log()->debug("Connecting controller output to plant input");
  s->builder.Cascade(*s->controller, *s->kukaPlant);

  // State feedback from the plant to the controller.
  drake::log()->debug(
        "Connecting plant output to controller estimated state input");
  s->builder.Connect(s->kukaPlant->get_output_port(0),
                  s->controller->get_input_port_estimated_state());
}

void MyDrake::addLogger(){
  const int kNumPositions{s->kukaPlant->get_num_positions()};
  const int kNumVelocities{s->kukaPlant->get_num_velocities()};

  s->logger = s->builder.AddSystem<drake::systems::SignalLogger<double>>(kNumPositions+kNumVelocities);

  s->builder.Connect(s->kukaPlant->get_output_port(0),
                     s->logger->get_input_port());
}

void MyDrake::addReferenceTrajectory(const arr& knots){
  std::default_random_engine random_generator{1234};
  std::vector<double> t_values;

  if(&knots){
    const double dt = simulation_time / (knots.d0 - 1);
    for(uint i=0; i<knots.d0; i++) {
      t_values.push_back(i * dt);
      s->splineKnots.push_back( conv_arr2eigen(knots[i]) );
    }
  }else{
    const int kNumKnots = 5;
    const double dt = simulation_time / (kNumKnots - 1);
    for (int i = 0; i < kNumKnots; ++i) {
      t_values.push_back(i * dt);
      s->splineKnots.push_back(
            s->kukaPlant->get_rigid_body_tree().getRandomConfiguration(random_generator));
    }
  }

  drake::PiecewisePolynomialTrajectory reference_position_trajectory{
    PiecewisePolynomial<double>::Pchip(t_values, s->splineKnots,
                                       true /*zero_end_point_derivatives*/)};

  // This reference trajectory can used in the diagram by means of
  // TrajectorySource blocks.
  auto reference_state_source =
      s->builder.AddSystem<drake::systems::TrajectorySource<double>>(reference_position_trajectory, 1 /*output_derivative_order*/);
  auto reference_acceleration_source =
      s->builder.AddSystem<drake::systems::TrajectorySource<double>>(*reference_position_trajectory.derivative(2));

  // Finally, we can connect the trajectories to the inputs of the controller.
  drake::log()->debug("Connecting desired state trajectory to controller");
  s->builder.Connect(reference_state_source->get_output_port(),
                     s->controller->get_input_port_desired_state());
  drake::log()->debug(
        "Connecting desired acceleration trajectory to controller");
  s->builder.Connect(reference_acceleration_source->get_output_port(),
                     s->controller->get_input_port_desired_acceleration());

}

void MyDrake::simulate(){
  // Instantiate and configure simulator.
  drake::systems::Simulator<double> simulator{*s->system};
  simulator.set_target_realtime_rate(realtime_rate);
  for (int index = 0; index < s->kukaPlant->get_num_positions(); index++) {
    s->kukaPlant->set_position(simulator.get_mutable_context(), index,
                               s->splineKnots.front()(index));
  }
  simulator.Initialize();

  // Run simulation.
  simulator.StepTo(simulation_time);
}

arr MyDrake::getLog(){
  return conv_eigen2arr(s->logger->data().transpose());
}

void MyDrake::build(){
  s->system = s->builder.Build();
}


using robotlocomotion::robot_plan_t;

using drake::manipulation::schunk_wsg::SchunkWsgController;
using drake::manipulation::schunk_wsg::SchunkWsgStatusSender;
using drake::systems::RigidBodyPlant;
using drake::systems::RungeKutta2Integrator;
using drake::systems::Simulator;
using drake::manipulation::util::ModelInstanceInfo;
using drake::manipulation::planner::RobotPlanInterpolator;
using drake::manipulation::util::WorldSimTreeBuilder;

const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";
const char kIiwaEndEffectorName[] = "iiwa_link_ee";

// The `z` coordinate of the top of the table in the world frame.
// The quantity 0.736 is the `z` coordinate of the frame associated with the
// 'surface' collision element in the SDF. This element uses a box of height
// 0.057m thus giving the surface height (`z`) in world coordinates as
// 0.736 + 0.057 / 2.
const double kTableTopZInWorld = 0.736 + 0.057 / 2;

// Coordinates for kRobotBase originally from iiwa_world_demo.cc.
// The intention is to center the robot on the table.
// TODO(sam.creasey) fix this
const Eigen::Vector3d kRobotBase(0, 0, kTableTopZInWorld);
const Eigen::Vector3d kTableBase(0.243716, 0.625087, 0.);


Target GetTarget() {
  Target targets[] = {
    {"block_for_pick_and_place.urdf", Eigen::Vector3d(0.06, 0.06, 0.2)},
    {"black_box.urdf", Eigen::Vector3d(0.055, 0.165, 0.18)},
    {"simple_cuboid.urdf", Eigen::Vector3d(0.06, 0.06, 0.06)},
    {"simple_cylinder.urdf", Eigen::Vector3d(0.065, 0.065, 0.13)}
  };

  const int num_targets = 4;
  if ((FLAGS_target >= num_targets) || (FLAGS_target < 0)) {
    throw std::runtime_error("Invalid target ID");
  }
  return targets[FLAGS_target];
}

std::unique_ptr<drake::systems::RigidBodyPlant<double>> BuildCombinedPlant(
    const std::vector<Eigen::Vector3d>& post_positions,
    const Eigen::Vector3d& table_position,
    const std::string& target_model,
    const Eigen::Vector3d& box_position,
    const Eigen::Vector3d& box_orientation,
    ModelInstanceInfo<double>* iiwa_instance,
    ModelInstanceInfo<double>* wsg_instance,
    ModelInstanceInfo<double>* box_instance) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel("iiwa", kIiwaUrdf);
  tree_builder->StoreModel("table",
                           "drake/examples/kuka_iiwa_arm/models/table/"
                           "extra_heavy_duty_table_surface_only_collision.sdf");
  tree_builder->StoreModel(
      "target", "drake/examples/kuka_iiwa_arm/models/objects/" + target_model);
  tree_builder->StoreModel("yellow_post",
                           "drake/examples/kuka_iiwa_arm/models/objects/"
                           "yellow_post.urdf");
  tree_builder->StoreModel(
      "wsg",
      "drake/manipulation/models/wsg_50_description"
      "/sdf/schunk_wsg_50_ball_contact.sdf");


  // The main table which the arm sits on.
  tree_builder->AddFixedModelInstance("table",
                                      kTableBase,
                                      Eigen::Vector3d::Zero());
  tree_builder->AddFixedModelInstance("table",
                                      kTableBase + table_position,
                                      Eigen::Vector3d::Zero());
  for (const Eigen::Vector3d& post_location : post_positions) {
    tree_builder->AddFixedModelInstance("yellow_post",
                                        post_location,
                                        Eigen::Vector3d::Zero());
  }
  tree_builder->AddGround();
  // Chooses an appropriate box.
  int box_id = 0;
  int iiwa_id = tree_builder->AddFixedModelInstance("iiwa", kRobotBase);
  *iiwa_instance = tree_builder->get_model_info_for_instance(iiwa_id);

  box_id = tree_builder->AddFloatingModelInstance("target", box_position,
                                                  box_orientation);
  *box_instance = tree_builder->get_model_info_for_instance(box_id);

  int wsg_id = tree_builder->AddModelInstanceToFrame(
      "wsg", tree_builder->tree().findFrame("iiwa_frame_ee"),
      drake::multibody::joints::kFixed);
  *wsg_instance = tree_builder->get_model_info_for_instance(wsg_id);

  std::unique_ptr<RigidBodyTree<double>> tree = tree_builder->Build();

  //remove all collision groups
//  tree->removeCollisionGroupsIf([](const std::string& str)->bool{ return true; });

  return std::make_unique<drake::systems::RigidBodyPlant<double>>( std::move(tree) );
}

void MyDrake::mono_setupGeometry() {
  // Locations for the posts from physical pick and place tests with
  // the iiwa+WSG.
  // TODO(sam.creasey) this should be 1.10 in the Y direction.
  s->post_locations.push_back(Eigen::Vector3d(0.5, 0.5, 0));  // position A
  s->post_locations.push_back(Eigen::Vector3d(0.5, -0.5, 0));  // position B
  s->post_locations.push_back(Eigen::Vector3d(-0.5, -0.5, 0));  // position D
  s->post_locations.push_back(Eigen::Vector3d(-0.5, 0.5, 0));  // position E
  s->post_locations.push_back(Eigen::Vector3d(-0.47, -0.8, 0));  // position F

  // Position of the pick and place location on the table, relative to
  // the base of the arm.  In the original test, the position was
  // specified as 0.90m forward of the arm.  We change that to 0.86
  // here as the previous test grasped the target with the tip of the
  // fingers in the middle while this test places the fingertip
  // further forward.  The position is right at the edge of what we
  // can plan to, so this 4cm change does matter.

  // The offset from the top of the table to the top of the post, used for
  // calculating the place locations in iiwa relative coordinates.
  const Eigen::Vector3d post_height_offset(0, 0, 0.26);

  s->table_position = {0.86, -0.36, -0.07};

  // TODO(sam.creasey) select only one of these
  drake::Isometry3<double> place_location;
  place_location.translation() = s->post_locations[0] + post_height_offset;
  place_location.linear() = drake::Matrix3<double>(
        AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));
  s->place_locations.push_back(place_location);

  place_location.translation() = s->post_locations[1] + post_height_offset;
  place_location.linear().setIdentity();
  s->place_locations.push_back(place_location);

  place_location.translation() = s->table_position;
  place_location.linear().setIdentity();
  s->place_locations.push_back(place_location);

  place_location.translation() = s->post_locations[2] + post_height_offset;
  place_location.linear() = Matrix3<double>(
        AngleAxis<double>(-M_PI / 2., Vector3<double>::UnitZ()));
  s->place_locations.push_back(place_location);

  place_location.translation() = s->post_locations[3] + post_height_offset;
  place_location.linear() = Matrix3<double>(
        AngleAxis<double>(-M_PI / 2., Vector3<double>::UnitZ()));
  s->place_locations.push_back(place_location);

  place_location.translation() = s->post_locations[4] + post_height_offset;
  place_location.linear() = Matrix3<double>(
        AngleAxis<double>(-M_PI / 2., Vector3<double>::UnitZ()));
  s->place_locations.push_back(place_location);

  s->target = GetTarget();
  s->box_origin = {0, 0, kTableTopZInWorld};
  s->box_origin += s->place_locations[FLAGS_start_position].translation();
  Eigen::Vector3d half_target_height(0, 0, s->target.dimensions(2) * 0.5);
  s->box_origin += half_target_height;

  for (size_t i = 0; i < s->place_locations.size(); i++) {
    s->place_locations[i].translation() += half_target_height;
  }
}

void MyDrake::addMonoPlant(){
  // Offset from the center of the second table to the pick/place
  // location on the table.
  const Eigen::Vector3d table_offset(0.30, 0, 0);
  std::unique_ptr<drake::systems::RigidBodyPlant<double>> model_ptr =
      BuildCombinedPlant(s->post_locations, s->table_position + table_offset,
                         s->target.model_name,
                         s->box_origin, Vector3<double>(0, 0, FLAGS_orientation),
                         &s->iiwa_instance, &s->wsg_instance, &s->box_instance);

  s->plant = s->builder.AddSystem<drake::examples::kuka_iiwa_arm::IiwaAndWsgPlantWithStateEstimator<double>>(
                                                                                                              std::move(model_ptr), s->iiwa_instance, s->wsg_instance, s->box_instance);
  s->plant->set_name("plant");

  //-- add visualizers
  s->contact_viz =
      s->builder.AddSystem<systems::ContactResultsToLcmSystem<double>>(
          s->plant->get_tree());
  s->contact_results_publisher = s->builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_contact_results_for_viz>(
          "CONTACT_RESULTS", s->lcm.get()));
  // Contact results to lcm msg.
  s->builder.Connect(s->plant->get_output_port_contact_results(),
                  s->contact_viz->get_input_port(0));
  s->builder.Connect(s->contact_viz->get_output_port(0),
                  s->contact_results_publisher->get_input_port(0));

  s->visualizer = s->builder.AddSystem<systems::DrakeVisualizer>(
      s->plant->get_plant().get_rigid_body_tree(), s->lcm.get());

  s->builder.Connect(s->plant->get_output_port_plant_state(),
                  s->visualizer->get_input_port(0));
}

void MyDrake::addWsgController(){
  s->wsg_controller = s->builder.AddSystem<SchunkWsgController>();
  s->builder.Connect(s->plant->get_output_port_wsg_state(),
                     s->wsg_controller->get_state_input_port());
  s->builder.Connect(s->wsg_controller->get_output_port(0),
                     s->plant->get_input_port_wsg_command());

  s->wsg_status_sender = s->builder.AddSystem<SchunkWsgStatusSender>(
        s->plant->get_output_port_wsg_state().size(), 0, 0);
  s->builder.Connect(s->plant->get_output_port_wsg_state(),
                     s->wsg_status_sender->get_input_port(0));
}

void MyDrake::addPlanInterpolator(){
  s->iiwa_trajectory_generator = s->builder.AddSystem<RobotPlanInterpolator>(
        FindResourceOrThrow(kIiwaUrdf));
  s->builder.Connect(s->plant->get_output_port_iiwa_state(),
                     s->iiwa_trajectory_generator->get_state_input_port());
  s->builder.Connect(s->iiwa_trajectory_generator->get_state_output_port(),
                     s->plant->get_input_port_iiwa_state_command());
  s->builder.Connect(s->iiwa_trajectory_generator->get_acceleration_output_port(),
                     s->plant->get_input_port_iiwa_acceleration_command());
}

void MyDrake::addStateMachine(){
#if 0
  const Eigen::Vector3d robot_base(0, 0, kTableTopZInWorld);
  Isometry3<double> iiwa_base = Isometry3<double>::Identity();
  iiwa_base.translation() = robot_base;

  if (FLAGS_end_position >= 0) {
    if (FLAGS_end_position >= static_cast<int>(s->place_locations.size())) {
      throw std::runtime_error("Invalid end position specified.");
    }
    std::vector<Isometry3<double>> new_place_locations;
    new_place_locations.push_back(s->place_locations[FLAGS_end_position]);
    s->place_locations.swap(new_place_locations);
  }

  s->state_machine =
      s->builder.template AddSystem<drake::examples::kuka_iiwa_arm::monolithic_pick_and_place::PickAndPlaceStateMachineSystem>(
        FindResourceOrThrow(kIiwaUrdf), kIiwaEndEffectorName,
        iiwa_base, s->place_locations);

  //input to state machine: box_state, wsg_status, and iiwa_state
  s->builder.Connect(s->plant->get_output_port_box_robot_state_msg(),
                     s->state_machine->get_input_port_box_state());
  s->builder.Connect(s->wsg_status_sender->get_output_port(0),
                     s->state_machine->get_input_port_wsg_status());
  s->builder.Connect(s->plant->get_output_port_iiwa_robot_state_msg(),
                     s->state_machine->get_input_port_iiwa_state());

  //output of state machine: wsg_command, iiwa_plan
  s->builder.Connect(s->state_machine->get_output_port_wsg_command(),
                     s->wsg_controller->get_command_input_port());
  s->builder.Connect(s->state_machine->get_output_port_iiwa_plan(),
                     s->iiwa_trajectory_generator->get_plan_input_port());
#endif
}

void MyDrake::addRAIMachine(){
  const Eigen::Vector3d robot_base(0, 0, kTableTopZInWorld);
  Isometry3<double> iiwa_base = Isometry3<double>::Identity();
  iiwa_base.translation() = robot_base;

  if (FLAGS_end_position >= 0) {
    if (FLAGS_end_position >= static_cast<int>(s->place_locations.size())) {
      throw std::runtime_error("Invalid end position specified.");
    }
    std::vector<Isometry3<double>> new_place_locations;
    new_place_locations.push_back(s->place_locations[FLAGS_end_position]);
    s->place_locations.swap(new_place_locations);
  }

  s->rai_machine = s->builder.template AddSystem<drake::RAI_Machine>();

  //input to state machine: box_state, wsg_status, and iiwa_state
  s->builder.Connect(s->plant->get_output_port_box_robot_state_msg(),
                     s->rai_machine->get_input_port_box_state());
  s->builder.Connect(s->wsg_status_sender->get_output_port(0),
                     s->rai_machine->get_input_port_wsg_status());
  s->builder.Connect(s->plant->get_output_port_iiwa_robot_state_msg(),
                     s->rai_machine->get_input_port_iiwa_state());

  //output of state machine: wsg_command, iiwa_plan
  s->builder.Connect(s->rai_machine->get_output_port_wsg_command(),
                     s->wsg_controller->get_command_input_port());
  s->builder.Connect(s->rai_machine->get_output_port_iiwa_plan(),
                     s->iiwa_trajectory_generator->get_plan_input_port());
}

void MyDrake::simulate2(){
  Simulator<double> simulator(*s->system);
  simulator.Initialize();
  simulator.set_target_realtime_rate(0.); //FLAGS_realtime_rate);
  simulator.reset_integrator<RungeKutta2Integrator<double>>(*s->system,
      FLAGS_dt, simulator.get_mutable_context());
  simulator.get_mutable_integrator()->set_maximum_step_size(FLAGS_dt);
  simulator.get_mutable_integrator()->set_fixed_step_mode(true);

  auto& plan_source_context = s->system->GetMutableSubsystemContext(
      *s->iiwa_trajectory_generator, simulator.get_mutable_context());
  s->iiwa_trajectory_generator->Initialize(
      plan_source_context.get_time(),
      Eigen::VectorXd::Zero(7),
      plan_source_context.get_mutable_state());

  Access<double> timeToGo("timeToGo");

  // Step the simulator in some small increment.  Between steps, check
  // to see if the state machine thinks we're done, and if so that the
  // object is near the target.
  const double simulation_step = 0.1;
  while (true){
    simulator.StepTo(simulator.get_context().get_time() + simulation_step);
    timeToGo.writeAccess();
    if(timeToGo()>0.) timeToGo() -= simulation_step;
    timeToGo.deAccess();

    if (FLAGS_quick) {
      // We've run a single step, just get out now since we won't have
      // reached our destination.
      return;
    }
  }
}

int MyDrake::DoMain() {
  mono_setupGeometry();

  addMonoPlant();

  addPlanInterpolator();

  addWsgController();

//  addStateMachine();
  addRAIMachine();

  build();

  simulate2();

//  const drake::examples::kuka_iiwa_arm::pick_and_place::WorldState& world_state =
//      s->state_machine->world_state(
//          s->system->GetSubsystemContext(*s->state_machine,
//                                   simulator.get_context()));
//  const Isometry3<double>& object_pose = world_state.get_object_pose();
//  const Vector6<double>& object_velocity = world_state.get_object_velocity();
//  Isometry3<double> goal = s->place_locations.back();
//  goal.translation()(2) += kTableTopZInWorld;
//  Eigen::Vector3d object_rpy = math::rotmat2rpy(object_pose.linear());
//  Eigen::Vector3d goal_rpy = math::rotmat2rpy(goal.linear());

//  drake::log()->info("Pose: {} {}",
//                     object_pose.translation().transpose(),
//                     object_rpy.transpose());
//  drake::log()->info("Velocity: {}", object_velocity.transpose());
//  drake::log()->info("Goal: {} {}",
//                     goal.translation().transpose(),
//                     goal_rpy.transpose());

//  const double position_tolerance = 0.02;
//  Eigen::Vector3d position_error =
//      object_pose.translation() - goal.translation();
//  drake::log()->info("Position error: {}", position_error.transpose());
//  DRAKE_DEMAND(std::abs(position_error(0)) < position_tolerance);
//  DRAKE_DEMAND(std::abs(position_error(1)) < position_tolerance);
//  DRAKE_DEMAND(std::abs(position_error(2)) < position_tolerance);

//  const double angle_tolerance = 0.0873;  // 5 degrees
//  Eigen::Vector3d rpy_error = object_rpy - goal_rpy;
//  drake::log()->info("RPY error: {}", rpy_error.transpose());
//  DRAKE_DEMAND(std::abs(rpy_error(0)) < angle_tolerance);
//  DRAKE_DEMAND(std::abs(rpy_error(1)) < angle_tolerance);
//  DRAKE_DEMAND(std::abs(rpy_error(2)) < angle_tolerance);


//  const double linear_velocity_tolerance = 0.1;
//  DRAKE_DEMAND(std::abs(object_velocity(0)) < linear_velocity_tolerance);
//  DRAKE_DEMAND(std::abs(object_velocity(1)) < linear_velocity_tolerance);
//  DRAKE_DEMAND(std::abs(object_velocity(2)) < linear_velocity_tolerance);

//  const double angular_velocity_tolerance = 0.1;
//  DRAKE_DEMAND(std::abs(object_velocity(3)) < angular_velocity_tolerance);
//  DRAKE_DEMAND(std::abs(object_velocity(4)) < angular_velocity_tolerance);
//  DRAKE_DEMAND(std::abs(object_velocity(5)) < angular_velocity_tolerance);

  return 0;
}

}

