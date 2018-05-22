#include "drake.h"

#include <drake/systems/framework/diagram_builder.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/rigid_body_plant/rigid_body_plant.h>
#include <drake/multibody/rigid_body_plant/drake_visualizer.h>
#include <drake/systems/controllers/inverse_dynamics_controller.h>
#include <drake/systems/primitives/signal_logger.h>
#include "world_sim_tree_builder.h"
#include "iiwa_wsg_diagram_factory.h"
#include "drake/multibody/rigid_body_plant/contact_results_to_lcm.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "schunk_wsg_controller.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
//#include "state_machine_system.h"
#include "RAI_machine.h"
#include "robot_plan_interpolator.h"

struct Target {
  std::string model_name;
  Eigen::Vector3d dimensions;
};

namespace drake{
struct sMyDrake{
    drake::systems::DiagramBuilder<double> builder;
    std::unique_ptr<drake::lcm::DrakeLcm> lcm; //IPC with visualizer
    //three systems that are passed to the builder
    drake::systems::RigidBodyPlant<double>* kukaPlant = NULL;
    drake::systems::DrakeVisualizer* visualizer = NULL;
    drake::systems::SignalLogger<double>* logger = NULL;
    drake::systems::controllers::InverseDynamicsController<double>* controller = NULL;

    drake::manipulation::util::ModelInstanceInfo<double> iiwa_instance, wsg_instance, box_instance;


    std::unique_ptr<drake::systems::Diagram<double>> system;

    std::vector<drake::MatrixX<double>> splineKnots;

    //specific to mono?

    systems::ContactResultsToLcmSystem<double>* contact_viz=NULL;
    systems::lcm::LcmPublisherSystem* contact_results_publisher=NULL;
    drake::examples::kuka_iiwa_arm::IiwaAndWsgPlantWithStateEstimator<double>* plant=NULL;

    manipulation::schunk_wsg::SchunkWsgController* wsg_controller=NULL;
    manipulation::schunk_wsg::SchunkWsgStatusSender* wsg_status_sender=NULL;

//    drake::examples::kuka_iiwa_arm::monolithic_pick_and_place::PickAndPlaceStateMachineSystem* state_machine=NULL;
    drake::RAI_Machine* rai_machine=NULL;

    manipulation::planner::RobotPlanInterpolator* iiwa_trajectory_generator=NULL;

    std::vector<Eigen::Vector3d> post_locations;
    std::vector<drake::Isometry3<double>> place_locations;
    Eigen::Vector3d box_origin;
    Eigen::Vector3d table_position;
    Target target;
};
}
