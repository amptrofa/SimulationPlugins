#include <vector>
#include <list>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include "gazebo/common/SingletonT.hh"

#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
namespace gazebo
{
  namespace SMORES
  {
    
    /// \brief Docking port enumeration
    enum Port
    {
      FRONT,
      RIGHT,
      LEFT,
      REAR
    };
    
    class SMORESModule : public boost::enable_shared_from_this<SMORESModule>
    {
      /// \brief constructor
      public: SMORESModule();
      /// \brief destructor
      public: ~SMORESModule();
      
      /// \brief Set the name of the module
      /// \param[in] name The string to set the name to
      public: void SetName(const std::string &name);
      
      /// \brief Get the module name
      public: const std::string &GetName();
      
      /// \brief Set the links of this module
      /// \param[in] circuitHolder The circuit holder link
      /// \param[in] uHolderBody The u-shaped holder link
      /// \param[in] frontWheel The front wheel link
      /// \param[in] leftWheel The left wheel link
      /// \param[in] rightWheel The right wheel link
      public: void SetLinks(sdf::ElementPtr circuitHolder, sdf::ElementPtr uHolderBody, sdf::ElementPtr frontWheel, sdf::ElementPtr leftWheel, sdf::ElementPtr rightWheel);
      
      /// \brief Set the joints of this module
      /// \param[in] right The joint connecting the right wheel
      /// \param[in] left The joint connecting the left wheel
      /// \param[in] front The joint connecting the front wheel
      /// \param[in] center The joint connecting the two halves of the body, forming the center hinge
      public: void SetJoints(physics::JointPtr right, physics::JointPtr left, physics::JointPtr front, physics::JointPtr center);
      
      /// \brief Connect two modules together
      /// \param[in] thisModulePort Port of this module to connect
      /// \param[in] moduleConnecting Module to connect this module to
      /// \param[in] connectingModulePort Port of the module this one is being connected to to connect
      public: void Connect(Port thisModulePort, SMORESModule moduleConnecting, Port connectingModulePort);
      
      /// \brief Disconnect this module from another module
      /// \param[in] disconnectPort Port of this module to disconnect
      /// TODO: disconnection is not yet supported
      public: void Disconnect(Port disconnectPort);
      
      /// \brief Get the module pose
      public: math::Pose GetPose();
      
      /// \brief Move this module to the specified pose. Maintain rigid connections
      /// \param[in] poseTo Pose to move the module to
      public: void SetPose(math::Pose poseTo);
      
      private: std::string name;
      
      private: sdf::ElementPtr circuitHolderLink;
      private: sdf::ElementPtr uHolderBodyLink;
      private: sdf::ElementPtr frontWheelLink;
      private: sdf::ElementPtr leftWheelLink;
      private: sdf::ElementPtr rightWheelLink;
      
      private: physics::JointPtr rightWheelHinge;
      private: physics::JointPtr leftWheelHinge;
      private: physics::JointPtr frontWheelHinge;
      private: physics::JointPtr centerHinge;
      
    };
    
    /// \def SMORESModulePtr
    /// \brief Shared_ptr to SMORESModule object
    typedef boost::shared_ptr<SMORESModule> SMORESModulePtr;
    
    class SMORESManager : public SingletonT<SMORESManager>
    {
      private: SMORESManager();
      private: virtual ~SMORESManager();

      /// \brief Find a SMORES Module object by topic
      /// \param[in] name The name of the module to search for
      /// \return Pointer to the SMORESModule object, if found (can be null)
      public: SMORESModulePtr GetModuleByName(const std::string &name);

      /// \brief Add a module to the manager
      /// \param[in] modulePtr The module to be added
      public: void AddModule(SMORESModulePtr modulePtr);

      /// \brief Remove a module by its name
      /// \param[in] name The name of the module to be removed
      public: void RemoveModule(const std::string &name);

      /// \brief Get all the module names 
      /// \param[out] nameList The list of names will be written here
      public: void GetModuleNames(std::list<std::string> &nameList);

      /// \brief Get all module pointers
      /// \param[out] moduleList The list of SMORESModulePtrs will be written here
      public: void GetModulePtrs(std::list<SMORESModulePtr> modulePtrs);

      private: std::vector<SMORESModulePtr> modules;

      // Singleton implementation
      private: friend class SingletonT<SMORESManager>;
    };
    
  }
}
