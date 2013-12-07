#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>
#include <cmath>

#include "SMORES.hh"
using namespace std;

#define PI 3.1415926 

namespace gazebo
{
	typedef const boost::shared_ptr<const msgs::GzString> GzStringPtr;
	typedef const boost::shared_ptr<const msgs::Pose> PosePtr;
	struct JointPlus
	{
		physics::JointPtr JointX;
		math::Angle JointAngleNow;
		math::Angle JointAngleDesire;
		// Whether the angle of the joint needs to be set
		bool Need2BeSet;

		// Joint Properties
		double MaximiumForce;
		double MaximiumRotRate;

		// Variables need to be use to control the joint
	 	double JointErrorHis;
	 	double JointErrorAccu;
	};
	class ModelController : public ModelPlugin
	{
		public: ModelController() : ModelPlugin(),JointAngleKPID(1.5,0,0),ModelAngleKPID(1,0,0)
				{
					// Initialize variables
					WheelRadius =  0.045275;
					MaxiRotationRate = 2.4086;
					AccelerationRate = 8;
					PlanarMotionStopThreshold = 0.02;
					Driving2Angle.SetFromRadian(5.49778);
					Driving2Point.Set(1,1);
					// Location is test variable for dynamic joint generation
					Location.Set(1,1,0.05);
					// Rotmat is test variable for dynamic joint generation
					Rotmat.SetFromAxis(0,0,1,5.49778);
					Need2BeSet = 0;
					isModel3  = 0;
					// A hint of model been initialized
					printf("Model Initiated\n");
				}
		public: void Load (physics::ModelPtr _parent, sdf::ElementPtr _sdf)
				{
					string modelName = _parent->GetName();
					
					string nextElementName = _sdf->GetNextElement()->Get<string>("name");
					// Remove the name of the next element to get its scope within the model
					unsigned lastScope = nextElementName.rfind("::");
					if (lastScope == string::npos)
					{	// string "::" was not actually found in nextElementName
						this->withinModelPrefix = "";
						this->scopedPrefix = modelName + "::";
					} else {
						this->withinModelPrefix = nextElementName.substr(0,lastScope) + "::";
						this->scopedPrefix = modelName + "::" + this->withinModelPrefix;
					}
					
					/* Debugging output
					cout << "nextElementName[" << nextElementName << "]" << endl;
					cout << "withinModelPrefix[" << this->withinModelPrefix << "]" << endl;
					cout << "scopedPrefix[" << this->scopedPrefix << "]" << endl;
					/**/
					
					/* testing next elements
					cout << "Next 5 elements:" << endl;
					cout << _sdf->GetNextElement()->Get<string>("name") << endl;
					cout << _sdf->GetNextElement()->GetNextElement()->Get<string>("name") << endl;
					cout << _sdf->GetNextElement()->GetNextElement()->GetNextElement()->Get<string>("name") << endl;
					cout << _sdf->GetNextElement()->GetNextElement()->GetNextElement()->GetNextElement()->Get<string>("name") << endl;
					cout << _sdf->GetNextElement()->GetNextElement()->GetNextElement()->GetNextElement()->GetNextElement()->Get<string>("name") << endl;
					*/
														
					// Initialize the whole system
					SystemInitialization(_parent);
					// SMORES module management
					CreateSMORESModule(_parent);					

					// ************************************************************************************
					//gazebo::transport::NodePtr node(new gazebo::transport::Node());
					//node->Init();
					//this->sub = node->Subscribe("~/Welcome",&ModelController::welcomInfoProcessor, this);
					// commandSubscriber = node->Subscribe("~/collision_map/command", &CollisionMapCreator::create, this);
					//***************************************************************************************
					// Testing codes
					this->JointWR->SetVelocity(0,0);
					this->JointWL->SetVelocity(0,0);
					// math::Angle AngleInterested(0.5235987);
					// this->JointCB->SetAngle(0,AngleInterested);
					// Event register, which will make the function be executed in each iteration
					this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelController::OnSystemRunning, this, _1));
				}
				//########################################################
		private: void SystemInitialization(physics::ModelPtr parentModel)
				{
					// Get all the pointers point to right objects
					this->model = parentModel;
					this->JointWR = model->GetJoint(this->withinModelPrefix + "Right_wheel_hinge");
					this->JointWL = model->GetJoint(this->withinModelPrefix + "Left_wheel_hinge");
					this->JointWF = model->GetJoint(this->withinModelPrefix + "Front_wheel_hinge");
					this->JointCB = model->GetJoint(this->withinModelPrefix + "Center_hinge");
					JointWRP.JointX = JointWR;
					JointWRP.Need2BeSet = false;
					JointWRP.JointErrorHis = 0;
					JointWRP.JointErrorAccu = 0;
					JointWLP.JointX = JointWL;
					JointWLP.Need2BeSet = false;
					JointWLP.JointErrorHis = 0;
					JointWLP.JointErrorAccu = 0;
					JointWFP.JointX = JointWF;
					JointWFP.Need2BeSet = false;
					JointWFP.JointErrorHis = 0;
					JointWFP.JointErrorAccu = 0;
					JointCBP.JointX = JointCB;
					JointCBP.Need2BeSet = false;
					JointCBP.JointErrorHis = 0;
					JointCBP.JointErrorAccu = 0;
					// Setting the model states
					// Setting the maximium torque of the two wheels
					this->JointWR->SetMaxForce(0,JointWR->GetSDF()->GetElement("physics")->GetElement("ode")->GetElement("max_force")->GetValueDouble());
					this->JointWL->SetMaxForce(0,JointWL->GetSDF()->GetElement("physics")->GetElement("ode")->GetElement("max_force")->GetValueDouble());
					// Setting the maximium torque of the front wheel
					this->JointWF->SetMaxForce(0,JointWF->GetSDF()->GetElement("physics")->GetElement("ode")->GetElement("max_force")->GetValueDouble());
					// Setting the maximium torque of the body bending joint
					this->JointCB->SetMaxForce(0,JointCB->GetSDF()->GetElement("physics")->GetElement("ode")->GetElement("max_force")->GetValueDouble());
					// Set the angle of the hinge in the center to zero
					math::Angle InitialAngle(0.05);
					this->JointCB->SetAngle(0, InitialAngle);
					// math::Angle AngleNeed2Be(0.49778);
					// this->JointCB->SetAngle(0, AngleNeed2Be);

					
					// physics::ModelState CurrentModelState(model);
					// string TopicName = "~/";
					// TopicName += CurrentModelState.GetName();
					// TopicName += "_world";
					// gazebo::transport::NodePtr node(new gazebo::transport::Node());
					// 		node->Init();
					// 		this->PositionSub = node->Subscribe(TopicName,&ModelController::PositionDecoding, this);
					CollisionSubInitialization();
				}
		private: void CollisionSubInitialization(void)
				{
					gazebo::transport::NodePtr node1(new gazebo::transport::Node());

					node1->Init();
					string topicPrefix = SMORES::util::ColonToSlash(this->scopedPrefix);
					
					//cout<<"Mode: node name is '"<< this->scopedPrefix.substr(0,this->scopedPrefix.length()-2) <<"'"<<endl;
					string TopicName = "~/" + topicPrefix + "FrontWheel/front_contact";
					//cout<<"Mode: contact topic is '"<<TopicName<<"'"<<endl;
					this->LinkCollisonSub[0] = node1->Subscribe(TopicName,&ModelController::CollisionReceiverProcessor,this);
					
					TopicName = "~/" + topicPrefix + "UHolderBody/UHolder_contact";
					//cout<<"Mode: contact topic is '"<<TopicName<<"'"<<endl;
					this->LinkCollisonSub[1] = node1->Subscribe(TopicName,&ModelController::CollisionReceiverProcessor,this);
					
					TopicName = "~/" + topicPrefix + "LeftWheel/LeftWheel_contact";
					//cout<<"Mode: contact topic is '"<<TopicName<<"'"<<endl;
					this->LinkCollisonSub[2] = node1->Subscribe(TopicName,&ModelController::CollisionReceiverProcessor,this);
					
					TopicName = "~/" + topicPrefix + "RightWheel/RightWheel_contact";
					//cout<<"Mode: contact topic is '"<<TopicName<<"'"<<endl;
					this->LinkCollisonSub[3] = node1->Subscribe(TopicName,&ModelController::CollisionReceiverProcessor,this);
				}
		private: void CollisionReceiverProcessor(ConstContactsPtr &msg)
				{
					// Loop through all contacts, only process contacts that are not with the ground plane
					for (int i = 0; i < msg->contact_size(); ++i)
					{
						string coll1ModelName = SMORES::util::StripLastScope(SMORES::util::StripLastScope(msg->contact(i).collision1()));
						string coll2ModelName = SMORES::util::StripLastScope(SMORES::util::StripLastScope(msg->contact(i).collision2()));
						
						// short circuit will prevent us from searching for SMORES module most of the time
						if ( (coll1ModelName.find("ground_plane") == string::npos) &&
								 (coll2ModelName.find("ground_plane") == string::npos) && 
								 (SMORES::SMORESManager::Instance()->GetModuleByName(coll1ModelName)) &&
								 (SMORES::SMORESManager::Instance()->GetModuleByName(coll2ModelName)) )
						{
							//cout << coll1ModelName << " collided with " << coll2ModelName << "(this is [" << this->scopedPrefix << "])" << endl;
							
							SMORES::SMORESModulePtr module1 = SMORES::SMORESManager::Instance()->GetModuleByName(coll1ModelName);
							SMORES::SMORESModule::Port port1 = static_cast<SMORES::SMORESModule::Port>(
								SMORES::SMORESModule::ConvertPort(SMORES::util::GetLastScope(msg->contact(i).collision1())));
						
							SMORES::SMORESModulePtr module2 = SMORES::SMORESManager::Instance()->GetModuleByName(coll2ModelName);
							SMORES::SMORESModule::Port port2 = static_cast<SMORES::SMORESModule::Port>(
								SMORES::SMORESModule::ConvertPort(SMORES::util::GetLastScope(msg->contact(i).collision2())));
							
							// Only do the connection if the modules actually collided at ports!
							if (port1 <= SMORES::SMORESModule::PORT_COUNT && port2 <= SMORES::SMORESModule::PORT_COUNT)
							{
								//cout << "Sending off connection between[" << module1->GetName() << "::" << SMORES::PortCollisionNames[port1] << "] and [" << module2->GetName() << "::" << SMORES::PortCollisionNames[port2] << "]" << endl;
								SMORES::SMORESManager::Connect(module1,port1,module2,port2);
								break; // This mirrors the behavior of the old connecting code, but I don't think it makes sense
							}
						}
					}
				}
		// Position Command Decoding Function
		private: void PositionDecoding(PosePtr &msg)
				{
					// Location = msg->position();
					// Rotmat = msg->orientation();
					math::Pose tmpPose = gazebo::msgs::Convert(*msg);
					Location = tmpPose.pos;
					Rotmat = tmpPose.rot;
					Need2BeSet = 0;
					cout<<"Geting Location: "<< Location.x<<","<<Location.y<<","<<Location.z<<endl;
				}
		// Testing function
		private: void OnSystemRunning(const common::UpdateInfo & /*_info*/)
				{
					double AngleValue;
					double force;
					math::Pose CurrentPosition;
					math::Angle AngleNeed2Be(5.49778);  //0.78539, 2.3562, 3.9270, 5.49778
					// math::Angle AngleNeed2Be2(0);
					// force = JointCB->GetForce(0);
					force = this->JointCB->GetMaxForce(0);
					// force = JointCB->GetSDF()->GetElement("max_force")->GetValueDouble();
					// cout<<"The Maximium Force of the Joint:"<<force<<endl;
					AngleValue = this->RevolutionSpeedCal(JointWR,0);
					// cout<<"The real rotation rate is "<<AngleValue<<" rad/s"<<endl;
					CurrentPosition = GetModelCentralCoor();
					// cout<<"The Coordinates of the robot is: ["<<CurrentPosition.pos.x<<","<<CurrentPosition.pos.y<<","<<CurrentPosition.pos.z<<"];"<<endl;
					// cout<<"The direction of the robot is: ["<<CurrentPosition.rot.GetRoll()<<","<<CurrentPosition.rot.GetPitch()<<","<<CurrentPosition.rot.GetYaw()<<"];"<<endl;
					// cout<<"The direction of the robot is: ["<<CurrentPosition.pos.x<<","<<CurrentPosition.pos.y<<","<<CurrentPosition.pos.z<<"];"<<endl;
					//=========== Basic Controllers Need to Be Executed In Each Iteration ===========
					// JointAngleControl();
					JointWFP.JointAngleNow = GetJointAngle(JointWF,0);
					// JointPIDController(JointWFP,0,AngleNeed2Be);
					// math::Vector2d CurrentSpeed;
					// CurrentSpeed.x = 2;
					// CurrentSpeed.y = 2;
					// AnglePIDController(AngleNeed2Be, CurrentPosition.rot.GetYaw(), CurrentSpeed);
					math::Vector2d endPointTest;
					endPointTest.Set(0,0);
					// math::Vector2d startPointTest;
					// startPointTest.x = CurrentPosition.pos.x;
					// startPointTest.y = CurrentPosition.pos.y;
					// math::Angle desireAngle = AngleCalculation2Points(startPointTest, endPointTest);
					// cout<<"Calculated angle: " << desireAngle.Degree() << "," << desireAngle.Radian()<<endl;
					// AnglePIDController(desireAngle, CurrentPosition.rot.GetYaw(), CurrentSpeed);

					// Move2Point(endPointTest,AngleNeed2Be);
					Move2Point(Driving2Point,Driving2Angle);
					// math::Pose DesiredPos(Location,Rotmat);
					// if (Need2BeSet==0)
					// {
					// 	math::Pose DesiredPos(Location,Rotmat);
					// 	this->model->SetLinkWorldPose(DesiredPos,"CircuitHolder");
					// 	Need2BeSet += 1;
					// }else{
					// 	if (isModel3 == 0)
					// 	{
					// 	// 	Move2Point(endPointTest,AngleNeed2Be);
					// 	// 	SetJointSpeed(JointCB, 0, 0.05);
					// 	}
					// 	if(isModel3 == 1)
					// 	{
					// 		// this->DynamicJoint = this->model->GetWorld()->GetPhysicsEngine()->CreateJoint("revolute",  this->model);
					// 	 // 	this->DynamicJoint->Attach(model->GetLink("FrontWheel"), model->GetWorld()->GetModel("SMORES4Neel_0")->GetLink("FrontWheel"));
					// 	 // 	this->DynamicJoint->Load(model->GetLink("FrontWheel"), model->GetWorld()->GetModel("SMORES4Neel_0")->GetLink("FrontWheel"), math::Pose(model->GetLink("FrontWheel")->GetWorldPose().pos, math::Quaternion()));
					// 	 // 	math::Vector3 axis(0,1,0);
					// 	 // 	this->DynamicJoint->SetAxis(0, axis);
					// 	 	isModel3 += 1;
					// 	}
					// 	if (isModel3 >=2)
					// 	{
					// 		// SetJointSpeed(JointCB, 0, 0.05);
					// 		// JointPIDController(DynamicJoint,0,AngleNeed2Be2);
					// 		// Move2Point(endPointTest,AngleNeed2Be);
					// 	}
					// 	// cout<<"Model: The position of the Link: [ "<<model->GetLink("FrontWheel")->GetWorldPose().pos.x<<", "<<model->GetLink("FrontWheel")->GetWorldPose().pos.y<<", "<<model->GetLink("FrontWheel")->GetWorldPose().pos.z<<"]"<<endl;
					// 	// cout<<"Model: The y axis vector: ["<<  model->GetLink("FrontWheel")->GetWorldPose().rot.GetYAxis().x<<", "<<  model->GetLink("FrontWheel")->GetWorldPose().rot.GetYAxis().y<<", "<<  model->GetLink("FrontWheel")->GetWorldPose().rot.GetYAxis().z<<"]"<<endl;
					// 	// cout<<"Model: Euler angles are: ["<< model->GetLink("FrontWheel")->GetWorldPose().rot.GetAsEuler().x<<", "<< model->GetLink("FrontWheel")->GetWorldPose().rot.GetAsEuler().y<<", "<< model->GetLink("FrontWheel")->GetWorldPose().rot.GetAsEuler().z<<"]"<<endl;
					// 	// cout<<"Model: The Quaternion values: ["<< model->GetLink("FrontWheel")->GetWorldPose().rot.x<<", "<< model->GetLink("FrontWheel")->GetWorldPose().rot.y<<", "<< model->GetLink("FrontWheel")->GetWorldPose().rot.z<<", "<< model->GetLink("FrontWheel")->GetWorldPose().rot.w<<"]"<<endl;
					// }
					// this->model->SetLinkWorldPose(DesiredPos,"CircuitHolder");
					//===============================================================================
				}
		// The unit of the angle is radian
		// This function will only be used in simulation
		private: void SetJointAngleForce(physics::JointPtr CurrentJoint, int RotAxis, math::Angle AngleDesired)
				{
					CurrentJoint->SetAngle(RotAxis, AngleDesired);
				}
		// private: void SetJointAngle(JointPlus &CurrentJoint, int RotAxis, math::Angle AngleDesired, double DesireSpeed = 1)
		// 		{

		// 		}
		// This function is used to replace "SetJointAngle" when actually control the joint
		// This function will drive the joint to the desire angle according to the speed specified
		// The speed is the percentage of the maximum speed in the simulation, [0, 1]
		// The speed in the real worl depends on the external torque that actually applies on the joint
		private: void JointAngleControl(void)
				{

				}
		private: void JointPIDController(JointPlus &CurrentJoint, int RotAxis, math::Angle AngleDesired, double DesireSpeed = 1)
				{
					double AngleError, AngleDiffError;
					double SwingSpeed;
					AngleError = (AngleDesired - CurrentJoint.JointAngleNow).Radian();
					// cout<<"AngleError:"<<AngleError<<endl;
					AngleDiffError = AngleError - CurrentJoint.JointErrorHis;
					CurrentJoint.JointErrorAccu += AngleError;
					SwingSpeed = JointAngleKPID.x*AngleError + JointAngleKPID.y*CurrentJoint.JointErrorAccu + JointAngleKPID.z*AngleDiffError;
					// cout<<"SwingSpeed:"<<SwingSpeed<<endl;
					if (abs(SwingSpeed)> MaxiRotationRate*DesireSpeed)
					{
						SwingSpeed = SwingSpeed>0?MaxiRotationRate*DesireSpeed:(-MaxiRotationRate*DesireSpeed);
					}
					SetJointSpeed(CurrentJoint.JointX,RotAxis,SwingSpeed);

					CurrentJoint.JointErrorHis = AngleError;
				}
		// This function will return the angle of the specified joint
		// This function will be set to virtual in the future
		public: math::Angle GetJointAngle(physics::JointPtr CurrentJoint, int RotAxis)
				{
					math::Angle CurrentJointAngle;
					CurrentJointAngle = CurrentJoint->GetAngle(RotAxis);
					return CurrentJointAngle;
				}
		// This function will set the rotation rate of the joint
		// The unit of this speed is rad/s
		// In the future, this function should be defined as virtual
		public: void SetJointSpeed(physics::JointPtr CurrentJoint, int RotAxis, double SpeedDesired)
				{
					CurrentJoint->SetVelocity(RotAxis,SpeedDesired);
				}
		// This public function is used to calculate the angular velocity of a revolute joint
		// In the future, this function should be defined as virtual
		public: double RevolutionSpeedCal(physics::JointPtr JointNTBC, const int AxisIndex)
				{
					double ResSpeedRef;

					// In the API instruction, fucntion GetVelocity() returns the "rotation rate"
					// The unit of this rotation rate is "rad/s"
					ResSpeedRef = JointNTBC->GetVelocity(AxisIndex);
					//cout<<"ResSpeedRef = "<<ResSpeedRef<<" rad/s"<<endl;

					return ResSpeedRef;
				}
		// Get the coordinates and direction of the current model
		// Need to be virtual in the furture
		public: math::Pose GetModelCentralCoor(void)
				{
					/*
					math::Pose smorePose_in0;					
					
					math::Pose LinkPose_in0 = model->GetLink(this->withinModelPrefix + "CircuitHolder")->GetWorldPose();
					
					math::Pose LinkPose_inSMORE = math::Pose(0.0, 0.0, 0.05, 0.0, 0.0, 0.0); // Transformation from SMORE to link
					math::Pose smorePose_inLink = -LinkPose_inSMORE; // Transformation from link to SMORE
					
					smorePose_in0 = LinkPose_in0 + smorePose_inLink;
					return smorePose_in0; */
					return this->module->GetPose();
				}
		// This function is an old version function of RevolutionSpeedCal()
		// Which has the same functionality as RevolutionSpeedCal()
		private: double RevolutionSpeedCalOld(physics::JointPtr JointNTBC, const int AxisIndex)
				{
					math::Angle CurrentAngle;
					double CurrentAngleRadVal;
					static double CurrentAngleRadValHis = 0;
					common::Time TimeTmp;
					physics::JointState CurrentJointState(JointNTBC);
					double TimeStampNow;
					static double TimeStampNowHis = 0;
					double RevSpeed;

					CurrentAngle = JointNTBC->GetAngle(AxisIndex);
					CurrentAngleRadVal = CurrentAngle.Radian();

					TimeTmp = CurrentJointState.GetSimTime();
					TimeStampNow = TimeTmp.Double();

					// This method uses the change of the angle divided by the time interval
					RevSpeed = (CurrentAngleRadVal-CurrentAngleRadValHis)/(TimeStampNow-TimeStampNowHis); // The unit is rad/sec

					CurrentAngleRadValHis = CurrentAngleRadVal;
					TimeStampNowHis = TimeStampNow;

					return RevSpeed;
				}
		// Angle calculation base on the two points
		// This angle is in the static global frame
		// This function can be used when the robot is on the ground
		// And aim to move to another point on the ground
		private: math::Angle AngleCalculation2Points(math::Vector2d StartPoint, math::Vector2d EndPoint)
				{
					double displacementX, displacementY, angleC;
					displacementX = EndPoint.x - StartPoint.x;
					displacementY = EndPoint.y - StartPoint.y;
					// cout<<"direction vector is [ " << displacementX << "," <<displacementY<<"]"<<endl;
					angleC = atan2(displacementY,displacementX);
					math::Angle ReturnAngle(angleC+PI/2);
					return ReturnAngle;
				}
		// Angle PID Controller
		// DesiredAngleis in the static global frame
		// CurrentSpeed is the rotation rate of the wheel
		// This function can only be used when the robot drive on the ground plane
		private: void AnglePIDController(math::Angle DesiredAngle, math::Angle CurrentAngle, math::Vector2d CurrentSpeed)
				{
					double AngleError, AngleErrorDiff, DiffSpeedControl;
					static double AngleErrorHis = 0;
					static double AngleErrorAcu = 0;
					double LeftWheelSpeed, RightWheelSpeed;

					while (abs(DesiredAngle.Degree()) > 180)
					{
						double DegreeAngle;
						DegreeAngle = DesiredAngle.Degree()>0?DesiredAngle.Degree()-360:DesiredAngle.Degree()+360;
						DesiredAngle.SetFromDegree(DegreeAngle);
					}
					// cout<<"Desired Angle is "<< DesiredAngle.Degree()<<endl;

					if (abs((DesiredAngle-CurrentAngle).Degree())>180)
					{
						AngleError = (DesiredAngle-CurrentAngle).Degree()>0?(DesiredAngle-CurrentAngle).Radian()-2*PI:(DesiredAngle-CurrentAngle).Radian()+2*PI;
					}else{
						AngleError = (DesiredAngle-CurrentAngle).Radian();
					}
					AngleErrorAcu += AngleError;
					AngleErrorDiff = AngleError - AngleErrorHis;
					DiffSpeedControl = ModelAngleKPID.x*AngleError + ModelAngleKPID.y*AngleErrorAcu + ModelAngleKPID.z*AngleErrorDiff;
					LeftWheelSpeed = CurrentSpeed.x - DiffSpeedControl;
					RightWheelSpeed = CurrentSpeed.y + DiffSpeedControl;

					// Maximium speed checking
					// if (abs(RightWheelSpeed)>JointWRP.MaximiumRotRate)
					// {
					// 	RightWheelSpeed = RightWheelSpeed>0?JointWRP.MaximiumRotRate:-JointWRP.MaximiumRotRate;
					// 	LeftWheelSpeed = RightWheelSpeed - 2*DiffSpeedControl;
					// 	if (abs(LeftWheelSpeed)>JointWLP.MaximiumRotRate)
					// 	{
					// 		LeftWheelSpeed = LeftWheelSpeed>0?JointWLP.MaximiumRotRate:-JointWLP.MaximiumRotRate;
					// 	}
					// }
					// if (abs(LeftWheelSpeed)>JointWLP.MaximiumRotRate)
					// {
					// 	LeftWheelSpeed = LeftWheelSpeed>0?JointWLP.MaximiumRotRate:-JointWLP.MaximiumRotRate;
					// 	RightWheelSpeed = LeftWheelSpeed + 2*DiffSpeedControl;
					// 	if (abs(RightWheelSpeed)>JointWRP.MaximiumRotRate)
					// 	{
					// 		RightWheelSpeed = RightWheelSpeed>0?JointWRP.MaximiumRotRate:-JointWRP.MaximiumRotRate;
					// 	}
					// }

					SetJointSpeed(JointWR, 0, RightWheelSpeed);
					SetJointSpeed(JointWL, 0, LeftWheelSpeed);
					AngleErrorHis = AngleError;
				}
		private: void Move2Point(math::Vector2d DesiredPoint, math::Angle DesiredOrientation)
				{
					math::Pose CurrentPosition;
					math::Vector2d startPoint;
					math::Vector2d CurrentSpeed;
					CurrentPosition = GetModelCentralCoor();
					startPoint.x = CurrentPosition.pos.x;
					startPoint.y = CurrentPosition.pos.y;
					math::Angle desireAngle = AngleCalculation2Points(startPoint, DesiredPoint);
					double CurrentDistance = startPoint.Distance(DesiredPoint);
					if (CurrentDistance > PlanarMotionStopThreshold)
					{
						CurrentSpeed.x = AccelerationRate*CurrentDistance;
						if (CurrentSpeed.x > MaxiRotationRate)
						{
							CurrentSpeed.x = MaxiRotationRate;
						}
						CurrentSpeed.y = CurrentSpeed.x;
						AnglePIDController(desireAngle, CurrentPosition.rot.GetYaw(), CurrentSpeed);
					}else{
						CurrentSpeed.x = 0;
						CurrentSpeed.y = 0;
						AnglePIDController(DesiredOrientation, CurrentPosition.rot.GetYaw(), CurrentSpeed);
					}
				}
		// A complementary filter
		// The default coefficient of the filter is 0.9
		public: double ComplementaryFilter(double FilteringValue, double ComplementFilterPar = 0.9)
				{
					static double ValueRecorder = 0;
					double FiltedValue;
					FiltedValue = (1 - ComplementFilterPar)*FilteringValue + ComplementFilterPar*ValueRecorder;

					ValueRecorder = FiltedValue;

					return FiltedValue;
				}
			
		// Create a SMORESModule for the module this plugin controls and add it to the SMORESManager	
		private: void CreateSMORESModule(physics::ModelPtr parent)
		{
			this->module = SMORES::SMORESModulePtr(new SMORES::SMORESModule);
			
			this->module->SetName(this->scopedPrefix.substr(0,this->scopedPrefix.length()-2));
			this->module->SetParentModel(this->model);
			{
			// Add the links
			physics::LinkPtr circuitHolder = this->model->GetLink(this->withinModelPrefix + "CircuitHolder");
			physics::LinkPtr uHolderBody = this->model->GetLink(this->withinModelPrefix + "UHolderBody");
			physics::LinkPtr frontWheel = this->model->GetLink(this->withinModelPrefix + "FrontWheel");
			physics::LinkPtr leftWheel = this->model->GetLink(this->withinModelPrefix + "LeftWheel");
			physics::LinkPtr rightWheel = this->model->GetLink(this->withinModelPrefix + "RightWheel");
			
			this->module->SetLinks(circuitHolder, uHolderBody, frontWheel, leftWheel, rightWheel);
			}
						
			this->module->SetJoints(this->JointWR, this->JointWL, this->JointWF, this->JointCB);
			
			SMORES::SMORESManager::Instance()->AddModule(this->module);
		}
		
		private: SMORES::SMORESModulePtr module;
		
		// Current Model Pointer
		private: physics::ModelPtr model;
		private: string scopedPrefix;
		private: string withinModelPrefix;
		// Pointers to all the Joints the current model has
		private: physics::JointPtr JointWR;
		private: physics::JointPtr JointWL;
		private: physics::JointPtr JointWF;
		private: physics::JointPtr JointCB;
		private: JointPlus JointWRP;
		private: JointPlus JointWLP;
		private: JointPlus JointWFP;
		private: JointPlus JointCBP;
		// The event that will be refreshed in every iteration of the simulation
		private: event::ConnectionPtr updateConnection;
		// Default Joint Angle PID controller parameter
		public:  math::Vector3 JointAngleKPID;	// First digit is Kp, second digit is Ki and third digit is Kd
		// Default Joint Angle PID controller parameter
		public:  math::Vector3 ModelAngleKPID;	// First digit is Kp, second digit is Ki and third digit is Kd
		// Maximium rotation rate of each joint
		public:  double MaxiRotationRate;
		public:  double AccelerationRate;
		public:  double PlanarMotionStopThreshold;

		//private: transport::SubscriberPtr sub;
		private: transport::SubscriberPtr PositionSub;
		private: transport::SubscriberPtr LinkCollisonSub[4];
		private: transport::PublisherPtr CollisionInfoToServer;
		//################# Variables for testing ############################
		// In the future version, this radius value will be eliminate
		private: double WheelRadius;
		private: math::Angle Driving2Angle;
		private: math::Vector2d Driving2Point;
		private: math::Vector3 Location;
		private: math::Quaternion Rotmat;
		private: int Need2BeSet;
		private: physics::JointPtr DynamicJoint;
		private: int isModel3;
		//####################################################################
	};

	GZ_REGISTER_MODEL_PLUGIN(ModelController)
}
