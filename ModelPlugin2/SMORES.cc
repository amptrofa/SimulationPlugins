#include <SMORES.hh>

using namespace gazebo;

/******************************************************************************
/ SMORES::util
******************************************************************************/

std::string SMORES::util::ColonToSlash(const std::string &str)
{
  std::string replacedStr = str;
  unsigned colonPosition = replacedStr.find("::");
  // loop through, replacing the colon each time it is found
  while (colonPosition != std::string::npos)
  {
    replacedStr.replace(colonPosition,2,"/");
    colonPosition = replacedStr.find("::");
  };
  
  return replacedStr;
}

std::string SMORES::util::StripLastScope(const std::string &str)
{
  unsigned lastScope = str.rfind("::");
  std::string strippedStr = str.substr(0,lastScope);
  return strippedStr;  
}

std::string SMORES::util::GetLastScope(const std::string &str)
{
  unsigned lastScope = str.rfind("::");
  std::string theLastScope = str.substr(lastScope+2);
  return theLastScope;
}

using namespace SMORES;

/******************************************************************************
/ SMORESModule
******************************************************************************/

SMORESModule::SMORESModule()
{
  //TODO: create connection nodes
}

SMORESModule::~SMORESModule()
{
  //TODO: delete connection nodes
  //TODO: delete links
  //TODO: delete joints
}

void SMORESModule::SetName(const std::string &name)
{
  this->name = name;
}
const std::string &SMORESModule::GetName()
{
  return this->name;
}

void SMORESModule::SetParentModel(physics::ModelPtr model)
{
  this->parentModel = model;
}

physics::ModelPtr SMORESModule::GetParentModel()
{
  return this->parentModel;
}

void SMORESModule::SetLinks(physics::LinkPtr circuitHolder, physics::LinkPtr uHolderBody, physics::LinkPtr frontWheel, physics::LinkPtr leftWheel, physics::LinkPtr rightWheel)
{
  this->circuitHolderLink = circuitHolder;
  this->uHolderBodyLink = uHolderBody;
  this->frontWheelLink = frontWheel;
  this->leftWheelLink = leftWheel;
  this->rightWheelLink = rightWheel;
}

void SMORESModule::SetJoints(physics::JointPtr right, physics::JointPtr left, physics::JointPtr front, physics::JointPtr center)
{
  this->rightWheelHinge = right;
  this->leftWheelHinge = left;
  this->frontWheelHinge = front;
  this->centerHinge = center;
}

SMORESModule::Port SMORESModule::ConvertPort(const std::string &str)
{
  for (unsigned int i = 0; i < PORT_COUNT; ++ i)
    if (PortCollisionNames[i] == str)
      return static_cast<Port>(i);
  
  return UNKNOWN_PORT;
}

physics::LinkPtr SMORESModule::GetPortLink(SMORESModule::Port port)
{
  switch (port)
  {
    case SMORESModule::FRONT:
      return this->frontWheelLink;
    case SMORESModule::REAR:
      return this->uHolderBodyLink;
    case SMORESModule::LEFT:
      return this->leftWheelLink;
    case SMORESModule::RIGHT:
      return this->rightWheelLink;
    default:
      gzwarn << "Link could not be found for SMORESModule::Port[" << port << "]" << std::endl;
      physics::LinkPtr nothing;
      return nothing;
  }
}

math::Pose SMORESModule::GetPose()
{
  math::Pose LinkPose_in0 = this->circuitHolderLink->GetWorldPose();
  
  // Based off of the SMORE.sdf, I believe this is the proper transformation
  math::Pose LinkPose_inSMORES = math::Pose(0.0, 0.0, 0.05, 0.0, 0.0, 0.0); // Transformation from module-centered coordinates to link coordinates
  math::Pose SMORESPose_inLink = -LinkPose_inSMORES; // Transformation from link-centered coordinates to module-centered coordinates
  
  math::Pose SMORESPose_in0 = LinkPose_in0 + SMORESPose_inLink; // module pose, in world-centered coordinates
  return SMORESPose_in0;
}

void SMORESModule::SetPose(math::Pose poseTo)
{
  //TODO: Once the links are changed to being in a list, this should be turned into a loop
  math::Pose poseFrom = this->GetPose();
  
  // Circuit Holder link
  math::Pose linkRelPose = this->circuitHolderLink->GetWorldPose() - poseFrom; // Transformation from module to link
  math::Pose linkNewPose = linkRelPose * poseTo; // This magic from gazebo::Physics::Model::SetLinkWorldPose()
  this->circuitHolderLink->SetWorldPose(linkNewPose);
  
  // U Holder Body link
  linkRelPose = this->uHolderBodyLink->GetWorldPose() - poseFrom;
  linkNewPose = linkRelPose * poseTo;
  this->uHolderBodyLink->SetWorldPose(linkNewPose);
  
  // Front Wheel link
  std::cout << std::endl;
  //std::cout << "orgFrontWheelPose[" << this->frontWheelLink->GetWorldPose() << "]" << std::endl;  
  linkRelPose = this->frontWheelLink->GetWorldPose() - poseFrom;
  linkNewPose = linkRelPose * poseTo;
  this->frontWheelLink->SetWorldPose(linkNewPose);
  //std::cout << "newFrontWheelPose[" << this->frontWheelLink->GetWorldPose() << "]" << std::endl;
  
  // Left Wheel link
  linkRelPose = this->leftWheelLink->GetWorldPose() - poseFrom;
  linkNewPose = linkRelPose * poseTo;
  this->leftWheelLink->SetWorldPose(linkNewPose);
  
  // Right Wheel link
  linkRelPose = this->rightWheelLink->GetWorldPose() - poseFrom;
  linkNewPose = linkRelPose * poseTo;
  this->rightWheelLink->SetWorldPose(linkNewPose);
  
  //TODO: recursively move all connected modules to maintain alignment? This might not be necessary
  
  /* Old version, did not work
  //TODO: Once the links are changed to being in a list, this should be turned into a loop
  math::Pose poseFrom = this->GetPose();
  std::cout << "modulePoseNow[" << poseFrom << "]" << std::endl;
  math::Pose poseChange = poseTo-poseFrom;
  
  std::cout << "poseChange[" << poseChange << "]" << std::endl;
  
  this->circuitHolderLink->SetWorldPose(this->circuitHolderLink->GetWorldPose() + poseChange);
  this->uHolderBodyLink->SetWorldPose(this->uHolderBodyLink->GetWorldPose() + poseChange);
  this->frontWheelLink->SetWorldPose(this->frontWheelLink->GetWorldPose() + poseChange);
  
  std::cout << "newFrontWheelPose[" << this->frontWheelLink->GetWorldPose() + poseChange << "]" << std::endl;
  
  this->leftWheelLink->SetWorldPose(this->leftWheelLink->GetWorldPose() + poseChange);
  this->rightWheelLink->SetWorldPose(this->rightWheelLink->GetWorldPose() + poseChange);
  
  //TODO: recursively move all connected modules to maintain alignment? This might not be necessary
  */
}

void SMORESModule::SetLinkPose(math::Pose poseTo, const physics::LinkPtr specifiedLink)
{ // Implementation from gazebo::Physics::Model::SetLinkWorldPose()
  math::Pose linkPose = specifiedLink->GetWorldPose();
  math::Pose currentModulePose = this->GetPose();
  math::Pose linkRelPose = currentModulePose - linkPose;
  math::Pose targetModulePose = linkRelPose * poseTo;
  
  /*
  std::cout << std::endl;
  std::cout << "Module[" << this->name << "] -- " << std::endl;
  std::cout << "linkPose[" << linkPose << "](" << specifiedLink->GetName() << ")" << std::endl;
  std::cout << "modulePose[" << currentModulePose << "]" << std::endl;
  std::cout << "linkRelativePose[" << linkRelPose << "]" << std::endl;
  std::cout << "targetModulePose[" << targetModulePose << "]" << std::endl;
  */
  
  this->SetPose(targetModulePose);
}

/******************************************************************************
/ SMORESManager
******************************************************************************/

SMORESManager::SMORESManager(){}
SMORESManager::~SMORESManager(){}

SMORESModulePtr SMORESManager::GetModuleByName(const std::string &name)
{
  std::vector<SMORESModulePtr>::iterator iter;
  
  for (iter = this->modules.begin(); iter != this->modules.end(); ++iter)
  {
    if ((*iter)->GetName().compare(name) == 0)
      return *iter;
  }
  
  // No module with name==name found
  SMORESModulePtr nothing;
  return nothing;
}

void SMORESManager::AddModule(SMORESModulePtr modulePtr)
{
  this->modules.push_back(modulePtr);
}

void SMORESManager::RemoveModule(const std::string &name)
{
  //TODO: determine which module corresponds to name, if any
  //TODO: remove the module f rom this->modules
}

void SMORESManager::GetModuleNames(std::list<std::string> &nameList)
{
  std::vector<SMORESModulePtr>::iterator iter;
  
  for (iter = this->modules.begin(); iter != this->modules.end(); ++iter)
  {
    nameList.push_back((*iter)->GetName());
  }
}

void SMORESManager::GetModulePtrs(std::list<SMORESModulePtr> modulePtrs)
{
  std::vector<SMORESModulePtr>::iterator iter;
  
  for (iter = this->modules.begin(); iter != this->modules.end(); ++iter)
  {
    modulePtrs.push_back(*iter);
  }
}

void SMORESManager::Connect(SMORESModulePtr module1, SMORESModule::Port port1, SMORESModulePtr module2, SMORESModule::Port port2)
{
  // Unpack SMORESManager members for use within function
  std::vector<std::string> namesOfPendingRequest = SMORESManager::Instance()->namesOfPendingRequest;
  std::vector<math::Pose> PendingRequestPos = SMORESManager::Instance()->PendingRequestPos;
  std::vector<std::string> existConnections = SMORESManager::Instance()->existConnections;
  std::vector<std::string> existConnectionGroups = SMORESManager::Instance()->existConnectionGroups;
  std::vector<physics::JointPtr> DynamicConnections = SMORESManager::Instance()->DynamicConnections;
  std::vector<std::string> existConnectedPair = SMORESManager::Instance()->existConnectedPair; 
  
  physics::WorldPtr currentWorld = module1->GetParentModel()->GetWorld();
  
  // std::cout<<"World: First collision is : "<<msg->collision1()<<std::endl;
  std::string nameString1 = module1->GetName() + "::" + SMORES::PortCollisionNames[port1] + "," + module2->GetName() + "::" + SMORES::PortCollisionNames[port2];
  std::string nameString2 = module2->GetName() + "::" + SMORES::PortCollisionNames[port2] + "," + module1->GetName() + "::" + SMORES::PortCollisionNames[port1];
  std::string ModelOfCollision1 = module1->GetName();
  std::string ModelOfCollision2 = module2->GetName();
  std::string LinkOfCollision1 = module1->GetPortLink(port1)->GetName();
  std::string LinkOfCollision2 = module2->GetPortLink(port2)->GetName();
  std::string ModelOfModels1 = ModelOfCollision1+","+ModelOfCollision2;
  std::string ModelOfModels2 = ModelOfCollision2+","+ModelOfCollision1;

  math::Pose ContactLinkPos = module1->GetPortLink(port1)->GetWorldPose();
  int FoundPendingOne = 0;
  for (unsigned int i = 0; i < namesOfPendingRequest.size(); ++i)
  {
    // std::cout<<"World: Pending Array length: "<<namesOfPendingRequest.size()<<std::endl;
    // if (namesOfPendingRequest.at(i).compare(nameString1) ==0 || namesOfPendingRequest.at(i).compare(nameString2) ==0)
    if (namesOfPendingRequest.at(i).compare(nameString1) ==0)
    {
      int InAConnectionGroup = 0;
      std::cout<<"World: First model is: "<<ModelOfCollision1<<std::endl;
      for (unsigned int m = 0; m < existConnectionGroups.size(); ++m)
      {
        if (existConnectionGroups.at(m).find(ModelOfCollision1)!=std::string::npos)
        {
          InAConnectionGroup = 1;
          break;
        }
      }
      if (InAConnectionGroup != 1)
      {
        std::string tmpString = ModelOfCollision1;
        ModelOfCollision1 = ModelOfCollision2;
        ModelOfCollision2 = tmpString;
        
        tmpString = LinkOfCollision1;
        LinkOfCollision1 = LinkOfCollision2;
        LinkOfCollision2 = tmpString;
        
        math::Pose tmpPose = ContactLinkPos;
        ContactLinkPos = PendingRequestPos.at(i);
        PendingRequestPos.at(i) = tmpPose;
        
        /* I think the modules should be swapped too, if all of this is being swapped
        SMORESModulePtr tmpModule = module1;
        module1 = module2;
        module2 = tmpModule; */
      }
      std::cout<<"World: Now the first model became: "<<ModelOfCollision1<<std::endl;
      physics::LinkPtr Link1, Link2;
      math::Vector3 axis;
      math::Vector3 ZDirectionOffset(0,0,0.000);  //0.008
      math::Vector3 newCenterPoint = 0.5*(ContactLinkPos.pos + PendingRequestPos.at(i).pos)+ZDirectionOffset;
      math::Vector3 newPositionOfLink1;
      math::Vector3 newPositionOfLink2;
      math::Quaternion newDirectionofLink1;
      math::Vector3 newZAxis;
      double AngleBetweenZAxes;
      math::Quaternion FirstRotationOfLink2;
      math::Quaternion SecondRotationOfLink2;
      math::Quaternion newDirectionofLink2;

      if (LinkOfCollision1.find("FrontWheel")!=std::string::npos)
      {
      //   newPositionOfLink1 = newCenterPoint + 0.0495*ContactLinkPos.rot.GetYAxis();
      //   newPositionOfLink2 = newCenterPoint - 0.0495*ContactLinkPos.rot.GetYAxis();
        newPositionOfLink1 = ContactLinkPos.pos;
        newPositionOfLink2 = ContactLinkPos.pos - 0.0998*ContactLinkPos.rot.GetYAxis();
        newDirectionofLink1 = ContactLinkPos.rot;
        std::cout<<"World: 'newDirectionofLink1' Z axis [ "<<newDirectionofLink1.GetZAxis().x<<", "<<newDirectionofLink1.GetZAxis().y<<", "<<newDirectionofLink1.GetZAxis().z<<"]"<<std::endl;
        std::cout<<"World: 'newDirectionofLink1' 'Yaw' is : "<<newDirectionofLink1.GetYaw()<<std::endl;
        // std::cout<<"World: 'newDirectionofLink1' Y axis [ "<<newDirectionofLink1.GetYAxis().x<<", "<<newDirectionofLink1.GetYAxis().y<<", "<<newDirectionofLink1.GetYAxis().z<<"]"<<std::endl;
        // std::cout<<"World: 'newDirectionofLink1' 'Pitch' is : "<<newDirectionofLink1.GetPitch()<<std::endl;
        // std::cout<<"World: 'referenceQuaternion' Y axis [ "<<referenceQuaternion.GetYAxis().x<<", "<<referenceQuaternion.GetYAxis().y<<", "<<referenceQuaternion.GetYAxis().z<<"]"<<std::endl;
        // std::cout<<"World: 'referenceQuaternion' 'Pitch' is : "<<referenceQuaternion.GetPitch()<<std::endl;
        std::cout<<"World: 'PendingRequestPos' Z axis [ "<<PendingRequestPos.at(i).rot.GetZAxis().x<<", "<<PendingRequestPos.at(i).rot.GetZAxis().y<<", "<<PendingRequestPos.at(i).rot.GetZAxis().z<<"]"<<std::endl;
        std::cout<<"World: 'PendingRequestPos' 'Yaw' is : "<<PendingRequestPos.at(i).rot.GetYaw()<<std::endl;
        newZAxis = PendingRequestPos.at(i).rot.GetZAxis().Dot(newDirectionofLink1.GetZAxis())*newDirectionofLink1.GetZAxis() + PendingRequestPos.at(i).rot.GetZAxis().Dot(newDirectionofLink1.GetXAxis())*newDirectionofLink1.GetXAxis();
        newZAxis = newZAxis.Normalize();
        AngleBetweenZAxes = acos(newZAxis.Dot(newDirectionofLink1.GetZAxis()));
        FirstRotationOfLink2.SetFromEuler(0,0,PI);
        double DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetYAxis());
        // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
        if (DirectionReference>0)
        {
          SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
        }else{
          SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
        }
        if (LinkOfCollision2.find("UHolderBody")!=std::string::npos)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,0);
          std::cout<<"World: Calibrate angle: "<<AngleBetweenZAxes<<std::endl;
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetYAxis());
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          }else{
            SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
          }
        }
        if (LinkOfCollision2.find("LeftWheel")!=std::string::npos)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,PI/2);
          std::cout<<"World: Calibrate angle: "<<AngleBetweenZAxes<<std::endl;
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetXAxis());
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
          }else{
            SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
          }
        }
        if (LinkOfCollision2.find("RightWheel")!=std::string::npos)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,PI/2);
          std::cout<<"World: Calibrate angle: "<<AngleBetweenZAxes<<std::endl;
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetXAxis());
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
          }else{
            SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
          }
        }
        // math::Quaternion newDirectionofLink2(newZAxis, PI+newDirectionofLink1.GetYaw());
        newDirectionofLink2 = newDirectionofLink1*FirstRotationOfLink2*SecondRotationOfLink2;
        // std::cout<<"World: 'FirstRotationOfLink2' Y axis [ "<<FirstRotationOfLink2.GetYAxis().x<<", "<<FirstRotationOfLink2.GetYAxis().y<<", "<<FirstRotationOfLink2.GetYAxis().z<<"]"<<std::endl;
        // std::cout<<"World: 'FirstRotationOfLink2' 'Pitch' is : "<<FirstRotationOfLink2.GetPitch()<<std::endl;
        // std::cout<<"World: 'newDirectionofLink2' Y axis [ "<<newDirectionofLink2.GetYAxis().x<<", "<<newDirectionofLink2.GetYAxis().y<<", "<<newDirectionofLink2.GetYAxis().z<<"]"<<std::endl;
        // std::cout<<"World: 'newDirectionofLink2' 'Pitch' is : "<<newDirectionofLink2.GetPitch()<<std::endl;

        // std::cout<<"World: Need to be set position ["<<newPositionOfLink1.x<<", "<<newPositionOfLink1.y<<", "<<newPositionOfLink1.z<<"]"<<std::endl;
        // std::cout<<"World: Model name is : "<<msg->collision1().substr(0,msg->collision1().find("::"))<<" and link name is : "<<msg->collision1().substr(msg->collision1().find("::")+2,msg->collision1().rfind("::")-msg->collision1().find("::")-2)<<std::endl;
        axis.Set(0,1,0);
      }
      if (LinkOfCollision1.find("LeftWheel")!=std::string::npos)
      {
        newPositionOfLink1 = ContactLinkPos.pos;
        newPositionOfLink2 = ContactLinkPos.pos + 0.0998*ContactLinkPos.rot.GetXAxis();
        newDirectionofLink1 = ContactLinkPos.rot;
        newZAxis = PendingRequestPos.at(i).rot.GetZAxis().Dot(newDirectionofLink1.GetZAxis())*newDirectionofLink1.GetZAxis() + PendingRequestPos.at(i).rot.GetZAxis().Dot(newDirectionofLink1.GetYAxis())*newDirectionofLink1.GetYAxis();
        newZAxis = newZAxis.Normalize();
        AngleBetweenZAxes = acos(newZAxis.Dot(newDirectionofLink1.GetZAxis()));
        FirstRotationOfLink2.SetFromEuler(0,0,PI);
        double DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetXAxis());
        // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
        if (DirectionReference>0)
        {
          SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
        }else{
          SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
        }
        if (LinkOfCollision2.find("FrontWheel")!=std::string::npos)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,-PI/2);
          std::cout<<"World: Calibrate angle: "<<AngleBetweenZAxes<<std::endl;
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetYAxis());
          // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          }else{
            SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
          }
        }
        if (LinkOfCollision2.find("UHolderBody")!=std::string::npos)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,PI/2);
          // SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0 ,0);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetYAxis());
          // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          }else{
            SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
          }
        }
        if (LinkOfCollision2.find("RightWheel")!=std::string::npos)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,-PI);
          // SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0 ,0);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetXAxis());
          // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0 ,0);
          }else{
            SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0 ,0);
          }
        }
        newDirectionofLink2 = newDirectionofLink1*FirstRotationOfLink2*SecondRotationOfLink2;
        axis.Set(1,0,0);
      }
      if (LinkOfCollision1.find("RightWheel")!=std::string::npos)
      {
        newPositionOfLink1 = ContactLinkPos.pos;
        newPositionOfLink2 = ContactLinkPos.pos + 0.0998*ContactLinkPos.rot.GetXAxis();
        newDirectionofLink1 = ContactLinkPos.rot;
        newZAxis = PendingRequestPos.at(i).rot.GetZAxis().Dot(newDirectionofLink1.GetZAxis())*newDirectionofLink1.GetZAxis() + PendingRequestPos.at(i).rot.GetZAxis().Dot(newDirectionofLink1.GetYAxis())*newDirectionofLink1.GetYAxis();
        newZAxis = newZAxis.Normalize();
        AngleBetweenZAxes = acos(newZAxis.Dot(newDirectionofLink1.GetZAxis()));
        FirstRotationOfLink2.SetFromEuler(0,0,PI);
        double DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetXAxis());
        // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
        if (DirectionReference>0)
        {
          SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
        }else{
          SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
        }
        if (LinkOfCollision2.find("FrontWheel")!=std::string::npos)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,-PI/2);
          std::cout<<"World: Calibrate angle: "<<AngleBetweenZAxes<<std::endl;
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetYAxis());
          // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          }else{
            SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
          }
        }
        if (LinkOfCollision2.find("UHolderBody")!=std::string::npos)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,PI/2);
          // SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0 ,0);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetYAxis());
          // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          }else{
            SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
          }
        }
        if (LinkOfCollision2.find("LeftWheel")!=std::string::npos)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,-PI);
          // SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0 ,0);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetXAxis());
          // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0 ,0);
          }else{
            SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0 ,0);
          }
        }
        newDirectionofLink2 = newDirectionofLink1*FirstRotationOfLink2*SecondRotationOfLink2;
        axis.Set(1,0,0);
      }
      if (LinkOfCollision1.find("UHolderBody")!=std::string::npos)
      {
        newPositionOfLink1 = ContactLinkPos.pos;
        newPositionOfLink2 = ContactLinkPos.pos + 0.0998*ContactLinkPos.rot.GetYAxis();
        newDirectionofLink1 = ContactLinkPos.rot;
        newZAxis = PendingRequestPos.at(i).rot.GetZAxis().Dot(newDirectionofLink1.GetZAxis())*newDirectionofLink1.GetZAxis() + PendingRequestPos.at(i).rot.GetZAxis().Dot(newDirectionofLink1.GetXAxis())*newDirectionofLink1.GetXAxis();
        newZAxis = newZAxis.Normalize();
        AngleBetweenZAxes = acos(newZAxis.Dot(newDirectionofLink1.GetZAxis()));
        FirstRotationOfLink2.SetFromEuler(0,0,PI);
        double DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetYAxis());
        // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
        if (DirectionReference>0)
        {
          SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
        }else{
          SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
        }
        if (LinkOfCollision2.find("FrontWheel")!=std::string::npos)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,0);
          std::cout<<"World: Calibrate angle: "<<AngleBetweenZAxes<<std::endl;
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetYAxis());
          // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          }else{
            SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
          }
        }
        if (LinkOfCollision2.find("LeftWheel")!=std::string::npos)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,-PI/2);
          // SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0 ,0);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetXAxis());
          // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
          }else{
            SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
          }
        }
        if (LinkOfCollision2.find("RightWheel")!=std::string::npos)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,-PI/2);
          // SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0 ,0);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetXAxis());
          // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
          }else{
            SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
          }
        }
        newDirectionofLink2 = newDirectionofLink1*FirstRotationOfLink2*SecondRotationOfLink2;

        axis.Set(0,1,0);
      }
      
      Link1 = module1->GetPortLink(port1);
      Link2 = module2->GetPortLink(port2);      
      
      
      std::cout << "Current Pose for link1[" << module1->GetPortLink(port1)->GetWorldPose() << "]" << std::endl;
      std::cout << "Destination Pose link1[" << math::Pose(newPositionOfLink1,newDirectionofLink1) << "]" << std::endl;
      std::cout << "Current Pose for link2[" << module2->GetPortLink(port2)->GetWorldPose() << "]" << std::endl;
      std::cout << "Destination Pose link2[" << math::Pose(newPositionOfLink2,newDirectionofLink2) << "]" << std::endl;
      
      module1->SetLinkPose(math::Pose(newPositionOfLink1,newDirectionofLink1),Link1);
      module2->SetLinkPose(math::Pose(newPositionOfLink2,newDirectionofLink2),Link2);


      math::Pose newLink1PosReq = module1->frontWheelLink->GetWorldPose();
      // std::cout<< "World: Position set has done."<<std::endl;
      // std::cout<< "World: New Link1 Position ["<<newLink1PosReq.pos.x<<", "<<newLink1PosReq.pos.y<<", "<<newLink1PosReq.pos.z<<"]"<<std::endl;
      physics::JointPtr DynamicJoint;
      DynamicJoint = currentWorld->GetPhysicsEngine()->CreateJoint("revolute", module1->GetParentModel());
      DynamicJoint->Attach(Link1, Link2);
      DynamicJoint->Load(Link1, Link2, math::Pose(math::Vector3(0,-0.00,0),math::Quaternion()));
      DynamicJoint->SetAxis(0, axis);
      DynamicJoint->SetName("Dynamic_Joint");
      module1->GetParentModel()->GetJointController()->AddJoint(DynamicJoint);
      // DynamicJoint->Init();
      // std::cout<<"World: The parent of the new joint is '"<<DynamicJoint->GetParent()->GetName()<<"'"<<std::endl; 
      // std::cout<<"World: The children of the new joint is '"<<DynamicJoint->GetChild()->GetName()<<"'"<<std::endl; 
      DynamicJoint->SetAngle(0,math::Angle(0));
      DynamicJoint->SetHighStop(0,math::Angle(0.01));
      DynamicJoint->SetLowStop(0,math::Angle(-0.01));
      DynamicConnections.push_back(DynamicJoint);
      // std::cout<< "World: Dynamic joint has been generated."<<std::endl;

      existConnections.push_back(nameString1);
      existConnectedPair.push_back(ModelOfModels1);
      int InOneOfTheGroup = 0;
      for (unsigned int m = 0; m < existConnectionGroups.size(); ++m)
      {
        if (existConnectionGroups.at(m).find(ModelOfCollision1) != std::string::npos)
        {
          int InOtherGroup = 0;
          for (unsigned int j = 0; j < existConnectionGroups.size(); ++j)
          {
            if (existConnectionGroups.at(j).find(ModelOfCollision2) != std::string::npos)
            {
              existConnectionGroups.at(m) += ','+existConnectionGroups.at(j);
              existConnectionGroups.erase(existConnectionGroups.begin()+j);
              InOtherGroup = 1;
              break;
            }
          }
          if (InOtherGroup == 0)
          {
            existConnectionGroups.at(m) += ','+ModelOfCollision2;
          }
          InOneOfTheGroup = 1;
          break;
        }
        if (existConnectionGroups.at(m).find(ModelOfCollision2) != std::string::npos)
        {
          int InOtherGroup = 0;
          for (unsigned int j = 0; j < existConnectionGroups.size(); ++j)
          {
            if (existConnectionGroups.at(j).find(ModelOfCollision1) != std::string::npos)
            {
              existConnectionGroups.at(m) += ','+existConnectionGroups.at(j);
              existConnectionGroups.erase(existConnectionGroups.begin()+j);
              InOtherGroup = 1;
              break;
            }
          }
          if (InOtherGroup == 0)
          {
            existConnectionGroups.at(m) += ','+ModelOfCollision1;
          }
          InOneOfTheGroup = 1;
          break;
        }
      }
      if (InOneOfTheGroup==0)
      {
        existConnectionGroups.push_back(ModelOfModels1);
      }
      // +++++  Testing Script ++++++++++++++++++++++
      for (unsigned int m = 0; m < existConnectionGroups.size(); ++m)
      {
        std::cout<<"Wprld: One of the connected groups is: "<<existConnectionGroups.at(m)<<std::endl;
      }
      // ++++++++++++++++++++++++++++++++++++++++++++
      // std::cout<<"World: Connection between models: "
      namesOfPendingRequest.erase(namesOfPendingRequest.begin()+i);
      PendingRequestPos.erase(PendingRequestPos.begin()+i);
           
      FoundPendingOne = 1;
      // std::cout<<"World: Joint Generated!"<<std::endl;
      break;
    }
  }
  if (FoundPendingOne == 0)
  {
    for (unsigned int i = 0; i < namesOfPendingRequest.size(); ++i)
    {
      // Mistake in the condition
      if (namesOfPendingRequest.at(i).find(nameString1.substr(0,nameString1.find(","))) !=std::string::npos || namesOfPendingRequest.at(i).find(nameString1.substr(nameString1.find(",")+1,-1)) !=std::string::npos || (namesOfPendingRequest.at(i).find(ModelOfCollision1) != std::string::npos && namesOfPendingRequest.at(i).find(ModelOfCollision2) != std::string::npos))
      {
        // std::cout<<"World: Connection discard because of One part already in pending"<<std::endl;
        FoundPendingOne = 1;
        break;
      }
    }
    for (unsigned int i = 0; i < existConnections.size(); ++i)
    {
      if (existConnections.at(i).find(nameString1.substr(0,nameString1.find(","))) !=std::string::npos || existConnections.at(i).find(nameString1.substr(nameString1.find(",")+1,-1)) !=std::string::npos)
      {
        // std::cout<<"World: Connection discard because of One component has been connected"<<std::endl;
        FoundPendingOne = 1;
        break;
      }
    }
    for (unsigned int i = 0; i < existConnectedPair.size(); ++i)
    {
      if (existConnectedPair.at(i).compare(ModelOfModels1) ==0 || existConnectedPair.at(i).compare(ModelOfModels2) ==0)
      {
        // std::cout<<"World: Connection discard because of two model already connected"<<std::endl;
        FoundPendingOne = 1;
        break;
      }
      // if (existConnectedPair.at(i).substr(0,existConnectedPair.at(i).find(",")).compare(ModelOfCollision1)==0)
      // {

      // }else{
      //   if (existConnectedPair.at(i).substr(existConnectedPair.at(i).find(",")+1,-1).compare(ModelOfCollision1)==0)
      //   {
      //     /* code */
      //   }
      // }
    }
    if (FoundPendingOne==0)
    {
      // Check the distance between the center of the two models
      math::Vector3 CenterModel1 = module1->GetPose().pos;
      math::Vector3 CenterModel2 = module2->GetPose().pos;
      std::cout<<"World: Distance between centers: "<<(CenterModel1-CenterModel2).GetLength()<<std::endl;
      if((CenterModel1-CenterModel2).GetLength()<VALIDCONNECTIONDISUPPER && (CenterModel1-CenterModel2).GetLength()>VALIDCONNECTIONDISLOWER)
      {
      //
        std::cout<<"World: Distance between centers: "<<(CenterModel1-CenterModel2).GetLength()<<std::endl;
        namesOfPendingRequest.push_back(nameString1);
        PendingRequestPos.push_back(ContactLinkPos);
        std::cout<<"World: An pending entry has been established: '"<< nameString1<<"'"<<std::endl;
      }
    }
  }
  
  // Unpack SMORESManager members for use within function
  SMORESManager::Instance()->namesOfPendingRequest = namesOfPendingRequest;
  SMORESManager::Instance()->PendingRequestPos = PendingRequestPos;
  SMORESManager::Instance()->existConnections = existConnections;
  SMORESManager::Instance()->existConnectionGroups = existConnectionGroups;
  SMORESManager::Instance()->DynamicConnections = DynamicConnections;
  SMORESManager::Instance()->existConnectedPair = existConnectedPair;
}

void SMORESManager::Disconnect()
{
  //TODO
}
