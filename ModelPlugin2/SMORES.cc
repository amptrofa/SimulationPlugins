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
  //TODO: delet joints
}

void SMORESModule::SetName(const std::string &name)
{
  this->name = name;
}
const std::string &SMORESModule::GetName()
{
  return this->name;
}

void SMORESModule::SetLinks(sdf::ElementPtr circuitHolder, sdf::ElementPtr uHolderBody, sdf::ElementPtr frontWheel, sdf::ElementPtr leftWheel, sdf::ElementPtr rightWheel)
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

void SMORESModule::Connect(Port thisModulePort, SMORESModule moduleConnecting, Port connectingModulePort)
{
  //TODO: set up connection in node graph
  //TODO: move the module so that the connection is made
  //TODO: create a rigid dynamic joint
}

void SMORESModule::Disconnect(Port disconnectPort)
{
  //TODO
}

math::Pose SMORESModule::GetPose()
{
  math::Pose modulePose;
  //TODO: Need to establish how the pose of the entire module is defined
  //TODO: Need to return this definition of the module pose
  return modulePose;
}

void SMORESModule::SetPose(math::Pose poseTo)
{
  //TODO: Need to establish how the pose of the entire module is defined
  //TODO: recursively move all connected modules to maintain alignment
}

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

/******************************************************************************
/ SMORESManager
******************************************************************************/

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
