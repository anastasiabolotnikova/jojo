#include "TeleoperationDemoTripod.h"
#include <mc_mori/devices/Led.h>
#include <mc_mori/devices/Debug.h>
#include <mc_tasks/MetaTaskLoader.h>

using Color = mc_rtc::gui::Color;
using Style = mc_rtc::gui::plot::Style;
using PolygonDescription = mc_rtc::gui::plot::PolygonDescription;

TeleoperationDemoTripod::TeleoperationDemoTripod(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  config_.load(config);

  // Set leds of all modules to green
  for(auto & d: robot().devices())
  {
    if(d->type() == "Led")
    {
      auto & led = robot().device<mc_mori::Led>(d->name());
      led.command("led 0 0 15");
    }
  }

  // Add closed-loop collision constraints
  std::vector<mc_rbdyn::Collision> collisions = config("closedLoopCollision", std::vector<mc_rbdyn::Collision>{});
  for(auto & c : collisions){

    if(robot().hasBody(c.body1) && robot().hasBody(c.body2))
    {
      collisionConstraints_.push_back(std::make_shared<clcoll::ClosedLoopCollisionConstraint>(robots(), robots().robotIndex(), c.sDist, c.iDist, solver().dt()));
      collisionConstraints_[collisionConstraints_.size()-1]->setName("clcoll" + c.body1 + c.body2);
      collisionConstraints_[collisionConstraints_.size()-1]->setBodies(c.body1, c.body2);
      collisionConstraints_[collisionConstraints_.size()-1]->addToGUI(*gui());
      collisionConstraints_[collisionConstraints_.size()-1]->addToLogger(logger());

      solver().addConstraint(collisionConstraints_[collisionConstraints_.size()-1].get());
      solver().updateConstrSize();
    }
  }

  if(useCoMCnstr_)
  {
    for(int i=0;i<3;i++)
    {
      comProjections_.push_back(Eigen::Vector3d::Zero());
      comProjectionsSpeed_.push_back(Eigen::Vector3d::Zero());
      planeNormals_.push_back(Eigen::Vector3d::Zero());
      planeNormalsDot_.push_back(Eigen::Vector3d::Zero());
    }

    comIncPlaneConstraint_.reset(new mc_solver::CoMIncPlaneConstr(robots(), robots().robotIndex(), dt) );
  }

  mc_rtc::log::success("TeleoperationDemoTripod init done ");
}

bool TeleoperationDemoTripod::run()
{

  t_ += solver().dt();

  if(useCoMCnstr_)
  {
    com_ = robot().com();
    com_[2] = 0;
    // get x,y location of the contacts
    int cnt1 = 0;
    for(const auto & c : contacts())
    {
      // contact point
      contactPoints_[cnt1] = robot().surfacePose(c.r1Surface).translation();
      //contactPoints_[cnt1] = realRobot().bodyPosW(tips[cnt1]).translation();
      cnt1++;
    }

    // triangle center point
    Eigen::Vector3d C = (contactPoints_[0] + contactPoints_[1] + contactPoints_[2]) / 3.0;
    int cnt2 = 0;
    for(auto & point : contactPoints_)
    {
      // margin point
      auto pb = (C - point).normalized();
      marginPoints_[cnt2] = point + pb*0.1;
      marginPoints_[cnt2][2] = 0;
      cnt2++;
    }

    // construct plane constraints from points p1 p2, p2 p3, p3 p1
    std::vector<mc_rbdyn::Plane> planes;
    for(int i = 0; i < marginPoints_.size(); i++)
    {
      auto p1 = marginPoints_[i];
      auto p2 =  (i+1 <= marginPoints_.size()-1) ? marginPoints_[i+1] : marginPoints_[0];

      // line normal
      double dy = p2[1] - p1[1]; //! p1[1] - p2[1]
      double dx = p2[0] - p1[0]; //! p2[0] - p1[0]
      Eigen::Vector3d n = Eigen::Vector3d({dy, -dx, 0}).normalized(); //! Eigen::Vector3d({dy, dx, 0}).normalized();

      //!
      double norm = n.norm();
      if(norm > 0)
      {
        n = n / norm;
      }
      else
      {
        n = Eigen::Vector3d::Zero();
      }
      //! end

      // distance from (0, 0) to the line ax + by + c = 0
      double a = dy / dx;
      double b = 1;
      double c = -a * p1[0] + p1[1];
      double off = std::abs(c) / std::sqrt(a*a + b*b);

      //! off = -1 * (n.x() * p1.x + n.y() * p.y); // TODO try out new planes computation for CoM constraints

      planes.push_back({n, off});

      if(!comPlanesSet_)
      {
        mc_rtc::log::info("Points {} {}, {} {}", p1[0], p1[1], p2[0], p2[1]);
        mc_rtc::log::info("Planes noraml {} offset {}\n", n.transpose(), off);
      }

      midLinePoints_[i] = (p1 + p2) / 2;
      normalEndPoints_[i] = midLinePoints_[i] + n * 0.03;

      // Plane normals
      planeNormals_[i] = n;

      // CoM projection
      comProjections_[i] = robot().com() - (robot().com() - p1).dot(n) * n;
      comDist2Planes_[i] = (robot().com() - comProjections_[i]).norm();

      // CoM projection point speed
      comProjectionsSpeed_[i] = (comProjections_[i] - comProjectionsPrev_[i]) / timeStep;
      comProjectionsPrev_[i] = comProjections_[i];

      // Plane normal derivative
      planeNormalsDot_[i] = (planeNormals_[i] - planeNormalsPrev_[i]) / timeStep;
      planeNormalsPrev_[i] = planeNormals_[i];
    }

    // Update the planes of the CoM constraints
    if(!comPlanesSet_)
    {
      //comIncPlaneConstraint_->set_planes(solver(), planes, comProjectionsSpeed_, planeNormalsDot_, 0.03, 0.01, 0.1, 0.5);
      comPlanesSet_ = true;
    }

    off_ = std::abs(robot().surfacePose("E_Tip1").translation()[0]); // Here try using realRobot leg instead
    std::vector<mc_rbdyn::Plane> plane = {{{1., 0., 0.}, off_}};
    comIncPlaneConstraint_->set_planes(solver(), plane, {}, {}, 0.06, 0.04);//, comProjectionsSpeed_, planeNormalsDot_, 0.03, 0.01, 0.1, 0.5);
  }

  //return mc_control::fsm::Controller::run(mc_solver::FeedbackType::Joints);
  return mc_control::fsm::Controller::run();
}

void TeleoperationDemoTripod::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  // contact points
  int cnt = 0;
  for(const auto & c : contacts())
  {
    contactPoints_.push_back(robot().surfacePose(c.r1Surface).translation());
    cnt++;
  }

  if(useCoMCnstr_)
  {
    solver().addConstraintSet(*comIncPlaneConstraint_);

    // center point
    Eigen::Vector3d C = (contactPoints_[0] + contactPoints_[1] + contactPoints_[2]) / 3.0;
    for(auto & point : contactPoints_)
    {
      // margin point
      auto pb = (C - point).normalized();
      Eigen::Vector3d mp =  point + pb*0.1;
      mp[2] = 0.0;
      marginPoints_.push_back(mp);
    }
    marginPointsInit_ = marginPoints_;

    // construct plane constraints from points p1 p2, p2 p3, p3 p1
    std::vector<mc_rbdyn::Plane> planes;
    for(int i = 0; i < marginPoints_.size(); i++)
    {
      auto p1 = marginPoints_[i];
      auto p2 =  (i+1 <= marginPoints_.size()-1) ? marginPoints_[i+1] : marginPoints_[0];

      // line normal
      double dy = p2[1] - p1[1];
      double dx = p2[0] - p1[0];
      Eigen::Vector3d n = Eigen::Vector3d({dy, -dx, 0}).normalized();

      // distance from (0, 0) to the line ax + by + c = 0
      double a = dy / dx;
      double b = 1;
      double c = -a * p1[0] + p1[1];
      double off = std::abs(c) / std::sqrt(a*a + b*b);

      planes.push_back({n, off});

      midLinePoints_.push_back((p1 + p2) / 2);
      normalEndPoints_.push_back(midLinePoints_[i] + n * 0.03);

      // CoM projection onto planes
      Eigen::Vector3d comProjection = robot().com() - (robot().com() - p1).dot(n) * n;
      comProjectionsPrev_.push_back(comProjection);
      comDist2Planes_.push_back((robot().com() - comProjection).norm());

      // Plane normal
      planeNormalsPrev_.push_back(n);
    }
  }

  // Base position task
  if(!config_.has("baseTask")){
    mc_rtc::log::error_and_throw<std::runtime_error>("TeleoperationDemoTripod | baseTask config entry missing");
  }
  baseTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(solver(), config_("baseTask"));
  solver().addTask(baseTask_);

  std::vector<tasks::qp::JointStiffness> stiffnesses = {{"E_Tip_joint", 0.},
                                                        {"E_Tip_X_Rot_joint", 0},
                                                        {"F_Tip_joint", 0.},
                                                        {"F_Tip_X_Rot_joint", 0},
                                                        {"F_Tip_Z_Rot_joint", 0},
                                                        {"D_Tip_joint", 0.},
                                                        {"D_Tip_X_Rot_joint", 0},
                                                        {"D_Tip_Z_Rot_joint", 0}};

  getPostureTask("mori_tripod")->jointStiffness(solver(), stiffnesses);

  // Log
  auto getMoriQ = [this]() -> const std::vector<double> &
  {
    static std::vector<double> q(robot().refJointOrder().size());
    auto & mbc = robot().mbc();
    const auto & rjo = robot().refJointOrder();
    int i=0;
    for(const auto & jn : rjo){
      for(auto & j : mbc.q[robot().jointIndexByName(jn)]){
        q[i] = j;
      }
      i++;
    }
    return q;
  };

  auto getMoriV = [this]() -> const std::vector<double> &
  {
    static std::vector<double> v(robot().refJointOrder().size());
    auto & mbc = robot().mbc();
    const auto & rjo = robot().refJointOrder();
    int i=0;
    for(const auto & jn : rjo){
      for(auto & j : mbc.alpha[robot().jointIndexByName(jn)]){
        v[i] = j;
      }
      i++;
    }
    return v;
  };

  auto getMoriT = [this]() -> const std::vector<double> &
  {
    static std::vector<double> t(robot().refJointOrder().size());
    auto & mbc = robot().mbc();
    const auto & rjo = robot().refJointOrder();
    int i=0;
    for(const auto & jn : rjo){
      for(auto & j : mbc.jointTorque[robot().jointIndexByName(jn)]){
        t[i] = j;
      }
      i++;
    }
    return t;
  };

  auto getMoriP = [this]() -> const std::vector<double> &
  {
    auto postureObjective = getPostureTask("mori_tripod")->posture();
    static std::vector<double> p(robot().refJointOrder().size());
    auto & mbc = robot().mbc();
    const auto & rjo = robot().refJointOrder();
    int i=0;
    for(const auto & jn : rjo){
      auto ji = robot().jointIndexByName(jn);
      p[i] = postureObjective[ji][0];
      i++;
    }
    return p;
  };

  auto getMoriD = [this](unsigned int id) -> const std::vector<double> &
  {
    static std::vector<double> d(robot().refJointOrder().size());
    const auto & rjo = robot().refJointOrder();
    if(robot().hasDevice<mc_mori::Debug>("debug"))
    {
      auto & debugDevice = robot().device<mc_mori::Debug>("debug");
      int i=0;
      for(const auto & jn : rjo){
        d[i] = debugDevice.getDebug(jn, id);
        i++;
      }
    }
    return d;
  };

  auto getInterfaceTime = [this]() -> const double &
  {
    static double t = 0;
    if(robot().hasDevice<mc_mori::Debug>("debug"))
    {
      auto & debugDevice = robot().device<mc_mori::Debug>("debug");
      t = debugDevice.getInterfaceTime();
    }
    return t;
  };

  auto getMoriA = [this]() -> const std::vector<double> &
  {
    static std::vector<double> a(robot().refJointOrder().size());
    auto & mbc = robot().mbc();
    const auto & rjo = robot().refJointOrder();
    int i=0;
    for(const auto & jn : rjo){
      for(auto & j : mbc.alphaD[robot().jointIndexByName(jn)]){
        a[i] = j;
      }
      i++;
    }
    return a;
  };

  auto getMoriCoM = [this]() -> const Eigen::Vector3d &
  {
    static Eigen::Vector3d com;
    com = robot().com();
    return com;
  };

  auto getCom2Plane = [this]() -> const double &
  {
    static double x;
    x = Eigen::Vector3d(1, 0, 0).dot(robot().com()) + off_;
    return x;
  };

  auto getMoriContacts = [this]() -> const std::vector<Eigen::Vector3d> &
  {
    static std::vector<Eigen::Vector3d> contacts;
    contacts = contactPoints_;
    return contacts;
  };

  logger().addLogEntry(
    "Mori_Joints", [getMoriQ]() -> const std::vector<double> & { return getMoriQ(); });

  logger().addLogEntry(
    "Mori_Speed", [getMoriV]() -> const std::vector<double> & { return getMoriV(); });

  logger().addLogEntry(
    "Mori_Torque", [getMoriT]() -> const std::vector<double> & { return getMoriT(); });

  logger().addLogEntry(
    "Mori_Target", [getMoriP]() -> const std::vector<double> & { return getMoriP(); });

  logger().addLogEntry(
    "Mori_Acceleration", [getMoriA]() -> const std::vector<double> & { return getMoriA(); });

  logger().addLogEntry(
    "Mori_Debug_RecievedCmd", [getMoriD]() -> const std::vector<double> & { return getMoriD(0); });

  logger().addLogEntry(
    "Mori_Debug_FilteredVel", [getMoriD]() -> const std::vector<double> & { return getMoriD(1); });

  logger().addLogEntry(
    "Mori_Debug_Position", [getMoriD]() -> const std::vector<double> & { return getMoriD(2); });

  logger().addLogEntry(
    "Mori_Debug_IntIntegralNeigh", [getMoriD]() -> const std::vector<double> & { return getMoriD(3); });

  logger().addLogEntry(
    "InterfaceTime", [getInterfaceTime]() -> const double & { return getInterfaceTime(); });

  logger().addLogEntry(
    "CoM2Plane", [getCom2Plane]() -> const double & { return getCom2Plane(); });

  logger().addLogEntry(
    "RobotCoM", [getMoriCoM]() -> const Eigen::Vector3d & { return getMoriCoM(); });

  logger().addLogEntry(
    "RobotContact1", [getMoriContacts]() -> const Eigen::Vector3d & { return getMoriContacts()[0]; });

  logger().addLogEntry(
    "RobotContact2", [getMoriContacts]() -> const Eigen::Vector3d & { return getMoriContacts()[1]; });

  logger().addLogEntry(
    "RobotContact3", [getMoriContacts]() -> const Eigen::Vector3d & { return getMoriContacts()[2]; });

  nonSlidingContactSurface1_ = "F_Tip1";
  nonSlidingContactSurface2_ = "D_Tip1";
  nonSlidingContactId1_ = getContactId(nonSlidingContactSurface1_);
  nonSlidingContactId2_ = getContactId(nonSlidingContactSurface2_);

  if(useCoMCnstr_)
  {
    gui()->addElement({},
        mc_rtc::gui::Button("FixContacts", [this]() {

          mc_rtc::log::info("Fixing contacts");

          removeContact({"mori_tripod", "ground", "F_Tip1", "AllGround"});
          removeContact({"mori_tripod", "ground", "D_Tip1", "AllGround"});

          addContact({"mori_tripod", "ground", "F_Tip1_", "AllGround"});
          addContact({"mori_tripod", "ground", "D_Tip1_", "AllGround"});

          //contactConstraint().contactConstr->removeDofContact(nonSlidingContactId1_);
          //contactConstraint().contactConstr->removeDofContact(nonSlidingContactId2_);
          //auto res1 = contactConstraint().contactConstr->addDofContact(nonSlidingContactId1_, Eigen::Vector6d::Ones().asDiagonal());
          //auto res2 = contactConstraint().contactConstr->addDofContact(nonSlidingContactId2_, Eigen::Vector6d::Ones().asDiagonal());
          //if(!res1)
          //{
          //  mc_rtc::log::error("Failed to set dof contact for {}", nonSlidingContactSurface1_);
          //}
          //if(!res2)
          //{
          //  mc_rtc::log::error("Failed to set dof contact for {}", nonSlidingContactSurface2_);
          //}
          //contactConstraint().contactConstr->updateDofContacts();
          ; }));

    gui()->addElement({},
        mc_rtc::gui::Button("Free Contacts", [this]() {

          mc_rtc::log::info("Freeing contacts");

          removeContact({"mori_tripod", "ground", "F_Tip1_", "AllGround"});
          removeContact({"mori_tripod", "ground", "D_Tip1_", "AllGround"});

          addContact({"mori_tripod", "ground", "F_Tip1", "AllGround"});
          addContact({"mori_tripod", "ground", "D_Tip1", "AllGround"});

          //contactConstraint().contactConstr->removeDofContact(nonSlidingContactId1_);
          //contactConstraint().contactConstr->removeDofContact(nonSlidingContactId2_);
          //auto res1 = contactConstraint().contactConstr->addDofContact(nonSlidingContactId1_, Eigen::Vector6d::Ones().asDiagonal());
          //auto res2 = contactConstraint().contactConstr->addDofContact(nonSlidingContactId2_, Eigen::Vector6d::Ones().asDiagonal());
          //if(!res1)
          //{
          //  mc_rtc::log::error("Failed to set dof contact for {}", nonSlidingContactSurface1_);
          //}
          //if(!res2)
          //{
          //  mc_rtc::log::error("Failed to set dof contact for {}", nonSlidingContactSurface2_);
          //}
          //contactConstraint().contactConstr->updateDofContacts();
          ; }));

    gui()->addPlot(
      "Distance to constraint",
      mc_rtc::gui::plot::X("t", [this]()
                           { return t_; }),
      mc_rtc::gui::plot::Y(
          "D", [this]() { return Eigen::Vector3d(1, 0, 0).dot(robot().com()) + off_; }, Color::Blue),
      mc_rtc::gui::plot::Y(
          "D", [this]() { return 0.04; }, Color::Red));

    // Conservative support polygon vertices
    gui()->addElement({"CoM", "com"},
                   mc_rtc::gui::Point3D("contact1", [this]() -> const Eigen::Vector3d & { return this->com_; }));

    // CoM projection
    gui()->addXYPlot(
            "Center of Mass projection",
            mc_rtc::gui::plot::XY("CoM", [this]() { return robot().com()[0]; },
                                              [this]() { return robot().com()[1]; }, Color::Red, Style::Point),
            mc_rtc::gui::plot::XY("CoMProj1", [this]() { return comProjections_[0][0]; },
                                              [this]() { return comProjections_[0][1]; }, Color::Green, Style::Point),
            mc_rtc::gui::plot::XY("CoMProj2", [this]() { return comProjections_[1][0]; },
                                              [this]() { return comProjections_[1][1]; }, Color::Green, Style::Point),
            mc_rtc::gui::plot::XY("CoMProj3", [this]() { return comProjections_[2][0]; },
                                              [this]() { return comProjections_[2][1]; }, Color::Green, Style::Point),
            mc_rtc::gui::plot::Polygon("SupportOut", [this]() { return PolygonDescription({{marginPoints_[0][0],
                                                                                            marginPoints_[0][1]},
                                                                                           {marginPoints_[1][0],
                                                                                            marginPoints_[1][1]},
                                                                                           {marginPoints_[2][0],
                                                                                            marginPoints_[2][1]}},
                                                                                            Color::Red); }),
            mc_rtc::gui::plot::Polygon("SupportOut", [this]() { return PolygonDescription({{marginPointsInit_[0][0],
                                                                                            marginPointsInit_[0][1]},
                                                                                           {marginPointsInit_[1][0],
                                                                                            marginPointsInit_[1][1]},
                                                                                           {marginPointsInit_[2][0],
                                                                                            marginPointsInit_[2][1]}},
                                                                                            Color::Yellow); }),
            mc_rtc::gui::plot::Polygon("Normal1", [this]() { return PolygonDescription({{midLinePoints_[0][0],
                                                                                            midLinePoints_[0][1]},
                                                                                           {normalEndPoints_[0][0],
                                                                                            normalEndPoints_[0][1]}},
                                                                                            Color::Green); }),
            mc_rtc::gui::plot::Polygon("Normal2", [this]() { return PolygonDescription({{midLinePoints_[1][0],
                                                                                            midLinePoints_[1][1]},
                                                                                           {normalEndPoints_[1][0],
                                                                                            normalEndPoints_[1][1]}},
                                                                                            Color::Green); }),
            mc_rtc::gui::plot::Polygon("Normal3", [this]() { return PolygonDescription({{midLinePoints_[2][0],
                                                                                            midLinePoints_[2][1]},
                                                                                           {normalEndPoints_[2][0],
                                                                                            normalEndPoints_[2][1]}},
                                                                                            Color::Green); }),
            mc_rtc::gui::plot::Polygon("SupportIn", [this]() { return PolygonDescription({{contactPoints_[0][0],
                                                                                            contactPoints_[0][1]},
                                                                                           {contactPoints_[1][0],
                                                                                            contactPoints_[1][1]},
                                                                                           {contactPoints_[2][0],
                                                                                            contactPoints_[2][1]}}, Color::Blue); })
          );
  }
}


tasks::qp::ContactId TeleoperationDemoTripod::getContactId(std::string s)
{
  for(const auto & c : solver().contacts())
  {
    if((c.r1Index() == 0 && c.r1Surface()->name() == s) || (c.r2Index() == 0 && c.r2Surface()->name() == s))
    {
      return c.contactId(robots());
    }
  }
  mc_rtc::log::error_and_throw<std::runtime_error>("Failed to find contact id for {}", s);
}
