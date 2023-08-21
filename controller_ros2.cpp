#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>
#include "controller.h"
// #include "./kdtree-cpp-master/kdtree.cpp"
#include <geometry_msgs/msg/twist.hpp>
#include "./kdtree-cpp-master/kdtree.hpp"
#include "plannerfunctions.h"

using namespace std;
using namespace Eigen;
using namespace CBFCirc;
using namespace std::chrono_literals;

// COMMUNICATION FUNCTIONS

// double getTime() {
//   return ros::Time::now().toSec() - Global::startTime;
// }

// void setTwist(VectorXd linearVelocity, double angularVelocity) {
//   double theta = getRobotPose().orientation;
//   double ctheta = cos(theta);
//   double stheta = sin(theta);

//   Vector2d velTransformed;
//   velTransformed << ctheta * linearVelocity[0] + stheta * linearVelocity[1],
//       -stheta * linearVelocity[0] + ctheta * linearVelocity[1];

//   geometry_msgs::msg::Twist twist;
//   twist.linear.x = velTransformed[0];
//   twist.linear.y = velTransformed[1];
//   twist.linear.z = 0;
//   twist.angular.x = 0;
//   twist.angular.y = 0;
//   twist.angular.z = angularVelocity;

//   Global::desLinVelocity = linearVelocity;
//   Global::desAngVelocity = angularVelocity;
//   // Global::pubBodyTwist->publish(twist);
// }

// void setLinearVelocity(VectorXd linearVelocity) {
//   double theta = getRobotPose().orientation;
//   Vector3d normVelocity = linearVelocity.normalized();
//   double angularVelocity = Global::param.gainRobotYaw * (cos(theta) * normVelocity[1]
//   -
//                                                          sin(theta) *
//                                                          normVelocity[0]);

//   setTwist(linearVelocity, angularVelocity);
// }

// vector<vector<VectorXd>> getFrontierPoints() {
//   vector<vector<VectorXd>> frontierPoints;
//   octomap_with_query::frontier_points srv;

//   srv.request.z_min = Global::param.constantHeight - 0.2;
//   srv.request.z_max = Global::param.constantHeight + 0.2;

//   if (Global::frontierClient->call(srv)) {
//     int idMax = 0;
//     int idMin = 1000;
//     for (int i = 0; i < srv.response.cluster_id.size(); i++) {
//       idMax = srv.response.cluster_id[i] > idMax ? srv.response.cluster_id[i] :
//       idMax; idMin = srv.response.cluster_id[i] < idMin ? srv.response.cluster_id[i]
//       : idMin;
//     }

//     for (int i = 0; i <= idMax - idMin; i++) {
//       vector<VectorXd> points = {};
//       frontierPoints.push_back(points);
//     }

//     for (int i = 0; i < srv.response.frontiers.size(); i++) {
//       VectorXd newPoint = VectorXd::Zero(3);
//       newPoint << srv.response.frontiers[i].x, srv.response.frontiers[i].y,
//           Global::param.constantHeight;
//       frontierPoints[srv.response.cluster_id[i] - idMin].push_back(newPoint);
//     }
//   }

//   // Filter frontier points
//   vector<vector<VectorXd>> frontierPointsFiltered = {};
//   Global::mutexUpdateKDTree.lock_shared();
//   for (int i = 0; i < frontierPoints.size(); i++) {
//     double maxDist = 0;
//     for (int j = 0; j < frontierPoints[i].size(); j++) {
//       vector<VectorXd> points =
//           getLidarPointsKDTree(frontierPoints[i][j], Global::param.sensingRadius);
//       double dist = VERYBIGNUMBER;
//       for (int k = 0; k < points.size(); k++)
//         dist = min(dist, (points[k] - frontierPoints[i][j]).norm());

//       maxDist = max(maxDist, dist);
//     }
//     if (maxDist > max(Global::param.boundingRadius, Global::param.boundingHeight /
//     2))
//       frontierPointsFiltered.push_back(frontierPoints[i]);
//   }
//   Global::mutexUpdateKDTree.unlock_shared();

//   return frontierPointsFiltered;
// }

// vector<VectorXd> getLidarPointsSource(VectorXd position, double radius) {
//   vector<VectorXd> points;

//   octomap_with_query::neighbor_points srv;

//   srv.request.radius = radius;
//   srv.request.query.x = position[0];
//   srv.request.query.y = position[1];
//   srv.request.query.z = position[2];

//   int fact = 5;

//   if (Global::neighborhClient->call(srv)) {
//     for (int i = 0; i < srv.response.neighbors.size() / fact; i++) {
//       double z = srv.response.neighbors[fact * i].z;
//       // if (z >= Global::param.constantHeight - 0.3 && z <=
//       // Global::param.constantHeight + 0.3)
//       if (z >= Global::measuredHeight - 0.10 && z <= Global::measuredHeight + 0.10) {
//         VectorXd newPoint = VectorXd::Zero(3);
//         newPoint << srv.response.neighbors[fact * i].x,
//             srv.response.neighbors[fact * i].y, Global::param.constantHeight;
//         points.push_back(newPoint);
//       }
//     }
//   }

//   return points;
// }

// vector<VectorXd> getLidarPointsKDTree(VectorXd position, double radius) {
//   vector<VectorXd> points = {};

//   if (Global::pointsKDTree.size() > 0) {
//     vector<double> positionV(3);
//     positionV[0] = position[0];
//     positionV[1] = position[1];
//     positionV[2] = position[2];

//     Kdtree::KdNodeVector result;
//     Global::kdTree->range_nearest_neighbors(positionV, radius, &result);

//     for (int i = 0; i < result.size(); ++i) {
//       VectorXd ptemp =
//           vec3d(result[i].point[0], result[i].point[1], result[i].point[2]);
//       points.push_back(ptemp);
//     }
//   }

//   return points;
// }

// MAIN FUNCTIONS

// void lowLevelMovement() {
//   while (ros::ok() && Global::continueAlgorithm) {
//     if (Global::measured && Global::firstPlanCreated) {
//       if (Global::planningState != MotionPlanningState::planning &&
//           Global::commitedPath.size() > 1) {
//         vector<VectorXd> obsPoints =
//             getLidarPointsSource(getRobotPose().position,
//             Global::param.sensingRadius);
//         VectorFieldResult vfr =
//             vectorField(getRobotPose(), Global::commitedPath, Global::param);
//         CBFControllerResult cccr =
//             CBFController(getRobotPose(), vfr.linearVelocity, vfr.angularVelocity,
//                           obsPoints, Global::param);

//         // Send the twist
//         setTwist(1.2 * cccr.linearVelocity, 1.2 * cccr.angularVelocity);

//         // Refresh some variables
//         Global::distance = cccr.distanceResult.distance;
//         Global::safety = cccr.distanceResult.safety;
//         Global::gradSafetyPosition = cccr.distanceResult.gradSafetyPosition;
//         Global::gradSafetyOrientation = cccr.distanceResult.gradSafetyOrientation;
//         Global::witnessDistance = cccr.distanceResult.witnessDistance;
//       } else {
//         setTwist(VectorXd::Zero(3), 0);
//       }
//     }
//   }
// }

// void replanCommitedPathCall() {
//   Global::mutexReplanCommitedPath.lock();
//   updateKDTreeCall();
//   Global::mutexUpdateKDTree.lock_shared();

//   Global::generateManyPathResult = CBFCircPlanMany(
//       getRobotPose(), Global::currentGoalPosition, getLidarPointsKDTree,
//       Global::param.maxTimePlanner, Global::param.plannerOmegaPlanReachError,
//       Global::param.deltaTimePlanner, Global::param);

//   if (Global::generateManyPathResult.atLeastOnePathReached)
//     Global::commitedPath = optimizePath(Global::generateManyPathResult.bestPath.path,
//                                         getLidarPointsKDTree, Global::param);

//   // DEBUG
//   int counter = Global::generalCounter;
//   debug_addMessage(counter, "Store event: replanning omega");
//   debug_generateManyPathsReport(counter);
//   debug_Store(counter);
//   // DEBUG

//   if (Global::generateManyPathResult.atLeastOnePathReached) {
//     double lengthCurrentOmega;
//     for (int j = 0; j < Global::generateManyPathResult.pathResults.size(); j++) {
//       if (getMatrixNumber(Global::generateManyPathResult.pathOmega[j]) ==
//           getMatrixNumber(Global::currentOmega)) {
//         if (Global::generateManyPathResult.pathResults[j].pathState ==
//             PathState::sucess)
//           lengthCurrentOmega = Global::generateManyPathResult.pathLenghts[j];
//         else
//           lengthCurrentOmega = VERYBIGNUMBER;
//       }
//     }
//     if (Global::generateManyPathResult.bestPathSize <
//         Global::param.acceptableRatioChangeCirc * lengthCurrentOmega) {
//       Global::currentOmega = Global::generateManyPathResult.bestOmega;
//       debug_addMessage(counter, "Circulation changed!");
//     } else
//       debug_addMessage(counter, "Circulation kept because of the ratio!");

//     Global::firstPlanCreated = true;
//   } else {
//     // Transition condition
//     ROS_INFO_STREAM("Failed to find path... plan to explore frontier!");
//     debug_addMessage(counter, "Failed to find path... plan to explore frontier!");

//     Global::planningState = MotionPlanningState::planning;
//     vector<vector<VectorXd>> frontierPoints = getFrontierPoints();
//     while (frontierPoints.size() == 0) {
//       ROS_INFO_STREAM("No frontier points found... trying again...");
//       std::this_thread::sleep_for(std::chrono::seconds(5));
//       frontierPoints = getFrontierPoints();
//     }

//     updateGraphCall(false);
//     Global::mutexUpdateGraph.lock();
//     NewExplorationPointResult nepr = Global::graph.getNewExplorationPoint(
//         getRobotPose(), getLidarPointsKDTree, frontierPoints, Global::param);
//     Global::mutexUpdateGraph.unlock();

//     if (nepr.success) {
//       // Algorithm succesful
//       debug_addMessage(counter,
//                        "Frontier point selection successful... replanning path");

//       Global::currentPath = nepr.pathToExplorationPoint;
//       Global::currentIndexPath = 0;
//       Global::explorationPosition = nepr.bestExplorationPosition;
//       Global::currentGoalPosition = Global::currentPath[0]->nodeOut->position;

//       // DEBUG
//       debug_addMessage(counter, "Store event: beginning to travel path");
//       debug_generateManyPathsReport(counter);
//       debug_Store(counter);
//       // DEBUG

//       Global::mutexReplanCommitedPath.unlock();
//       Global::mutexUpdateKDTree.unlock_shared();
//       replanCommitedPathCall();
//       Global::mutexReplanCommitedPath.lock();
//       Global::mutexUpdateKDTree.lock_shared();

//       Global::planningState = MotionPlanningState::pathToExploration;
//     } else {
//       // Algorithm failed
//       ROS_INFO_STREAM(
//           "Algorithm for finding new exploration points failed! Algorithm halted!");
//       Global::planningState = MotionPlanningState::failure;
//       Global::continueAlgorithm = false;
//     }
//   }
//   Global::mutexUpdateKDTree.unlock_shared();
//   Global::mutexReplanCommitedPath.unlock();
// }

// void replanCommitedPath() {
//   while (ros::ok() && Global::continueAlgorithm)
//     if (Global::measured &&
//         (Global::generalCounter % Global::param.freqReplanPath == 0))
//       replanCommitedPathCall();
// }

// void updateGraphCall(bool forceUpdate) {
//   Global::mutexUpdateGraph.lock();
//   Global::mutexUpdateKDTree.lock_shared();

//   VectorXd currentPoint = getRobotPose().position;
//   VectorXd correctedPoint = correctPoint(
//       currentPoint,
//       getLidarPointsKDTree(getRobotPose().position, Global::param.sensingRadius),
//       Global::param);

//   if (forceUpdate ||
//       (Global::graph.getNeighborNodes(correctedPoint, Global::param.radiusCreateNode)
//            .size() == 0)) {
//     vector<double> distances;
//     vector<int> indexes;
//     vector<Matrix3d> omegas;

//     int i = 0;
//     bool cont = true;

//     while (cont) {
//       RobotPose pose;
//       pose.position = Global::graph.nodes[i]->position;
//       pose.orientation = 0;

//       GenerateManyPathsResult gmpr = CBFCircPlanMany(
//           pose, correctedPoint, getLidarPointsKDTree,
//           Global::param.maxTimePlanConnectNode, Global::param.plannerReachError,
//           Global::param.deltaTimePlanner, Global::param);
//       if (gmpr.atLeastOnePathReached) {
//         indexes.push_back(i);
//         omegas.push_back(gmpr.bestOmega);
//         distances.push_back(gmpr.bestPathSize);
//         cont = gmpr.bestPathSize > Global::param.acceptableMinDist;
//       }

//       i++;
//       cont = cont && (i < Global::graph.nodes.size());
//     }

//     if (distances.size() > 0) {
//       vector<int> ind = sortGiveIndex(distances);
//       Node* newNode = Global::graph.addNode(correctedPoint);
//       Global::graph.connect(Global::graph.nodes[indexes[ind[0]]], newNode,
//                             distances[ind[0]], omegas[ind[0]]);

//       // DEBUG
//       debug_addMessage(Global::generalCounter, "Graph updates with a new node!");
//       //
//     } else {
//       // DEBUG
//       debug_addMessage(Global::generalCounter, "Graph was not updated");
//       //
//     }
//   }

//   Global::mutexUpdateKDTree.unlock_shared();
//   Global::mutexUpdateGraph.unlock();
// }

// void updateGraph() {
//   while (ros::ok() && Global::continueAlgorithm)
//     if (Global::measured &&
//         (Global::generalCounter % Global::param.freqUpdateGraph == 0))
//       updateGraphCall(false);
// }

// void updateKDTreeCall() {
//   Global::mutexUpdateKDTree.lock();

//   vector<VectorXd> pointsFromLidar =
//       getLidarPointsSource(getRobotPose().position, Global::param.sensingRadius);

//   int debug_pointsAdded = 0;

//   for (int i = 0; i < pointsFromLidar.size(); i++) {
//     double minDist = VERYBIGNUMBER;
//     for (int j = 0; j < Global::pointsKDTree.size(); j++)
//       minDist =
//           min(minDist, (Global::pointsKDTree[j] - pointsFromLidar[i]).squaredNorm());

//     if (minDist > pow(Global::param.minDistFilterKDTree, 2)) {
//       Global::pointsKDTree.push_back(pointsFromLidar[i]);
//       debug_pointsAdded++;
//     }
//   }

//   Kdtree::KdNodeVector nodes;

//   // Guarantee that it has at least one node
//   vector<double> point(3);
//   point[0] = VERYBIGNUMBER;
//   point[1] = VERYBIGNUMBER;
//   point[2] = VERYBIGNUMBER;
//   nodes.push_back(Kdtree::KdNode(point));

//   for (int i = 0; i < Global::pointsKDTree.size(); i++) {
//     vector<double> point(3);
//     point[0] = Global::pointsKDTree[i][0];
//     point[1] = Global::pointsKDTree[i][1];
//     point[2] = Global::pointsKDTree[i][2];
//     nodes.push_back(Kdtree::KdNode(point));
//   }

//   Global::kdTree = new Kdtree::KdTree(&nodes, 2);

//   Global::mutexUpdateKDTree.unlock();
//   debug_addMessage(
//       Global::generalCounter,
//       "Updated KD Tree with " + std::to_string(debug_pointsAdded) + " points");
// }

// void updateKDTree() {
//   while (ros::ok() && Global::continueAlgorithm)
//     if (Global::measured &&
//         (Global::generalCounter % Global::param.freqUpdateKDTree == 0))
//       updateKDTreeCall();
// }

// void transitionAlg() {
//   while (ros::ok() && Global::continueAlgorithm) {
//     if (Global::measured) {
//       bool pointReached =
//           (getRobotPose().position - Global::currentGoalPosition).norm() <=
//           Global::param.plannerReachError;

//       if ((Global::planningState == MotionPlanningState::goingToGlobalGoal) &&
//           pointReached) {
//         Global::planningState = MotionPlanningState::success;
//         Global::continueAlgorithm = false;

//         // DEBUG
//         debug_addMessage(Global::generalCounter, "Success!");
//         // DEBUG
//       }

//       if ((Global::planningState == MotionPlanningState::goingToExplore) &&
//           pointReached) {
//         Global::planningState = MotionPlanningState::goingToGlobalGoal;
//         Global::currentGoalPosition = Global::param.globalTargetPosition;

//         Global::currentPath = {};
//         Global::currentIndexPath = -1;
//         Global::explorationPosition = VectorXd::Zero(3);

//         replanCommitedPathCall();

//         // DEBUG
//         debug_addMessage(Global::generalCounter,
//                          "Reached exploration point. Going to global target!");
//         // DEBUG
//       }

//       if ((Global::planningState == MotionPlanningState::pathToExploration) &&
//           pointReached) {
//         if (Global::currentIndexPath == Global::currentPath.size() - 1) {
//           Global::planningState = MotionPlanningState::goingToExplore;
//           Global::currentGoalPosition = Global::explorationPosition;

//           replanCommitedPathCall();

//           // DEBUG
//           debug_addMessage(
//               Global::generalCounter,
//               "Reached last point on the path. Going to explore a frontier...");
//           // DEBUG
//         } else {
//           // DEBUG
//           debug_addMessage(Global::generalCounter,
//                            "Reached point " +
//                            std::to_string(Global::currentIndexPath));
//           // DEBUG

//           Global::currentIndexPath++;
//           Global::currentGoalPosition =
//               Global::currentPath[Global::currentIndexPath]->nodeIn->position;
//           Global::currentOmega =
//           Global::currentPath[Global::currentIndexPath]->omega;
//         }
//       }
//     }
//   }
// }

// void endCallback(const std_msgs::Int16::ConstPtr& msg) {
//   int16_t ind = msg.get()->data;
//   if (ind == 1)
//     Global::continueAlgorithm = false;
// }

class CBFControllerNode : public rclcpp::Node {
 public:
  CBFControllerNode() : Node("cbf_controller_node") {
    // Parameters
    this->declare_parameter("source_frame", "B1_154/odom_fast_lio");
    this->declare_parameter("target_frame", "B1_154/base_link");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    PoseCallbackTimer_ =
        this->create_wall_timer(100ms, std::bind(&CBFControllerNode::UpdatePose, this));

    VelocityPublisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    double radius = 2.0;
    size_t N = 1000;

    target_path_ = vector<RobotPose>();

    for (size_t i = 0; i < N; ++i) {
      double t_i = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(N);
      double x = radius * sin(t_i);
      double y = -radius * cos(t_i);
      // double t_i = M_PI / 4.0;
      // double x = 4.0 * static_cast<double>(i) / static_cast<double>(N);
      // double y = 4.0 * static_cast<double>(i) / static_cast<double>(N);

      RobotPose pose;
      VectorXd position(3);
      position << x, y, 0.0;
      pose.position = position;
      pose.orientation = t_i;
      target_path_.push_back(pose);
    }

    cbf_parameters_ = Parameters();
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Initialization successful. N:" << target_path_.size());
  }

 private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr PoseCallbackTimer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr VelocityPublisher_;
  vector<RobotPose> target_path_;
  Parameters cbf_parameters_;

  RobotPose getRobotPose() {
    RobotPose pose;
    pose.position = Global::position;
    pose.orientation = Global::orientation;
    return pose;
  }

  void setTwist(VectorXd linearVelocity, double angularVelocity) {
    double theta = getRobotPose().orientation;
    double ctheta = cos(theta);
    double stheta = sin(theta);

    Vector2d velTransformed;
    velTransformed << ctheta * linearVelocity[0] + stheta * linearVelocity[1],
        -stheta * linearVelocity[0] + ctheta * linearVelocity[1];

    geometry_msgs::msg::Twist twist;
    twist.linear.x = velTransformed[0];
    twist.linear.y = velTransformed[1];
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = angularVelocity;

    Global::desLinVelocity = linearVelocity;
    Global::desAngVelocity = angularVelocity;
    VelocityPublisher_->publish(twist);
  }

  void UpdatePose() {
    std::string source_frame = this->get_parameter("source_frame").as_string();
    std::string target_frame = this->get_parameter("target_frame").as_string();

    geometry_msgs::msg::TransformStamped t;

    try {
      t = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero);

      // double px = Global::transform->getOrigin().x();
      // double py = Global::transform->getOrigin().y();
      // // double pz = Global::transform->getOrigin().z();
      // double pz = Global::param.constantHeight;
      // double x = Global::transform->getRotation().x();
      // double y = Global::transform->getRotation().y();
      // double z = Global::transform->getRotation().z();
      // double w = Global::transform->getRotation().w();

      // double coshalfv = w;
      // double sinhalfv = sqrt(x * x + y * y + z * z);

      // if (z < 0)
      //   sinhalfv = -sinhalfv;

      // Global::position << px, py, pz;
      // Global::orientation = 2 * atan2(sinhalfv, coshalfv);
      // Global::measuredHeight = Global::transform->getOrigin().z();
      // Global::measured = true;

    } catch (const tf2::TransformException& ex) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Could not transform "
                                                 << source_frame << " to "
                                                 << target_frame
                                                 << ". Reason: " << ex.what());
      return;
    }

    double px = t.transform.translation.x;
    double py = t.transform.translation.y;
    // double pz = t.transform.translation.z;
    double pz = 0.0;
    // double pz = Global::param.constantHeight;
    // RCLCPP_INFO_STREAM(this->get_logger(), "px:" << px << " py:" << py << " pz:" <<
    // pz);

    double x = t.transform.rotation.x;
    double y = t.transform.rotation.y;
    double z = t.transform.rotation.z;
    double w = t.transform.rotation.w;
    // RCLCPP_INFO_STREAM(this->get_logger(),
    //                    "x:" << x << " y:" << y << " z:" << z << " w:" << w);

    double coshalfv = w;
    double sinhalfv = sqrt(x * x + y * y + z * z);
    if (z < 0)
      sinhalfv = -sinhalfv;
    double orientation = 2 * atan2(sinhalfv, coshalfv);

    // RCLCPP_INFO_STREAM(this->get_logger(),
    //                    "sinhalfv:" << sinhalfv << " coshalfv:" << coshalfv
    //                                << " orientation:" << orientation);

    VectorXd position(3);
    position << px, py, pz;

    Global::position = position;
    Global::orientation = orientation;

    // RCLCPP_INFO_STREAM(this->get_logger(), "Pose set successfully. Position: "
    //                                            << Global::position << " Orientation:"
    //                                            << Global::orientation);

    RobotPose current_pose;
    current_pose.position = Global::position;
    current_pose.orientation = Global::orientation;

    VectorFieldResult result = vectorField(current_pose, target_path_, cbf_parameters_);
    // RCLCPP_INFO_STREAM(this->get_logger(),
    //                    "Vector field result obtained. Distance: " <<
    //                    result.distance);

    setTwist(result.linearVelocity, result.angularVelocity);

    // // Publish velocity
    // geometry_msgs::msg::Twist command_velocity;
    // double factor = 1.0;
    // command_velocity.linear.x = result.linearVelocity(0) * factor;
    // command_velocity.linear.y = result.linearVelocity(1) * factor;
    // // command_velocity.linear.z = result.linearVelocity(2) * factor;
    // command_velocity.angular.z = result.angularVelocity * factor;
    // VelocityPublisher_->publish(command_velocity);
    // RCLCPP_WARN(this->get_logger(), "Published velocity");
  }
};

int main(int argc, char** argv) {
  // Initialize ROS variables
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CBFControllerNode>());
  rclcpp::shutdown();

  // ros::init(argc, argv, "manager");
  // ros::NodeHandle nodeHandler;
  // ros::Publisher aux_pubBodyTwist =
  //     nodeHandler.advertise<geometry_msgs::Twist>("b1_gazebo/cmd_vel", 1);
  // ros::ServiceClient neighborhClient =
  // nodeHandler.serviceClient<octomap_with_query::neighbor_points>("b1_gazebo/octomap_query_node/neighbor_points");
  // ros::ServiceClient frontierClient =
  // nodeHandler.serviceClient<octomap_with_query::frontier_points>("b1_gazebo/octomap_query_node/frontier_points");
  // tf::TransformListener tflistener;
  // tf::StampedTransform transform;
  // ros::Rate rate(100);
  // ros::Timer updatePoseTimer;
  // updatePoseTimer = nodeHandler.createTimer(ros::Duration(0.01), updatePose);

  // ros::Subscriber subEnd = nodeHandler.subscribe("endProgram", 1000, endCallback);
  // ros::Publisher pubEnd = nodeHandler.advertise<std_msgs::Int16>("endProgram", 1000);

  // Initialize some global variables
  // Global::pubBodyTwist = &aux_pubBodyTwist;
  // TODO // Global::startTime = ros::Time::now().toSec();
  // Global::currentGoalPosition = Global::param.globalTargetPosition;
  // Global::currentOmega = Matrix3d::Zero();
  // VectorXd startingPosition = vec3d(0, 0, Global::param.constantHeight);
  // Global::graph.addNode(startingPosition);
  // Global::neighborhClient = &neighborhClient;
  // Global::frontierClient = &frontierClient;
  // Global::tflistener = &tflistener;
  // Global::transform = &transform;

  // // Initialize some threads
  // std::thread lowLevelMovementThread = thread(lowLevelMovement);
  // std::thread replanOmegaThread = thread(replanCommitedPath);
  // std::thread updateGraphThread = thread(updateGraph);
  // std::thread updateKDTreeThread = thread(updateKDTree);
  // std::thread transitionAlgThread = thread(transitionAlg);

  // while (ros::ok() && Global::continueAlgorithm) {
  //   ros::spinOnce();

  //   // if (Global::measured)
  //   // {

  //   //     if (Global::generalCounter % Global::param.freqDisplayMessage == 0 &&
  //   //     (Global::planningState != MotionPlanningState::planning))
  //   //     {
  //   //         if (Global::planningState == MotionPlanningState::goingToGlobalGoal)
  //   //         {
  //   //             ROS_INFO_STREAM("-----GOING TO GLOBAL TARGET------");
  //   //         }
  //   //         if (Global::planningState == MotionPlanningState::pathToExploration)
  //   //         {
  //   //             ROS_INFO_STREAM("-------PATH TO EXPLORATION-------");
  //   //             ROS_INFO_STREAM("Point " << (Global::currentIndexPath + 1) << " of
  //   "
  //   //             << (Global::currentPath.size()));
  //   //         }
  //   //         if (Global::planningState == MotionPlanningState::goingToExplore)
  //   //         {
  //   //             ROS_INFO_STREAM("---------GOING TO EXPLORE--------");
  //   //         }
  //   //         ROS_INFO_STREAM("distobs = " << Global::distance);
  //   //         ROS_INFO_STREAM("distgoal = " << (getRobotPose().position -
  //   //         Global::currentGoalPosition).norm()); ROS_INFO_STREAM("omega = " <<
  //   //         getMatrixName(Global::currentOmega));
  //   //     }

  //   //     // DEBUG
  //   //     if (Global::firstPlanCreated && (Global::generalCounter %
  //   //     Global::param.freqStoreDebug == 0) && (Global::planningState !=
  //   //     MotionPlanningState::planning))
  //   //         debug_Store(Global::generalCounter);

  //   //     // DEBUG

  //   //     Global::generalCounter++;
  //   // }

  //   rate.sleep();
  // }

  // ofstream file;
  // debug_printAlgStateToMatlab(&file);

  // lowLevelMovementThread.join();
  // cout << "lowLevelMovementThread joined" << std::endl;
  // replanOmegaThread.join();
  // cout << "replanOmegaThread joined" << std::endl;
  // updateGraphThread.join();
  // cout << "updateGraphThread joined" << std::endl;
  // updateKDTreeThread.join();
  // cout << "updateKDTreeThread joined" << std::endl;
  // transitionAlgThread.join();
  // cout << "transitionAlgThread joined" << std::endl;

  return 0;
}
