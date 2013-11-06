#include <tinyxml.h>
#include "gazebo/gazebo.hh"
#include <sdf/sdf.hh>
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include <cmath>
#include <string>
#include <iostream>
#include <stdio.h>
#include "boost/filesystem.hpp"

#include <exception>

#define SMORES_MANIFEST_FILENAME "model.config" // the same as GZ_MANIFEST_FILENAME
#define GZ_MODEL_MANIFEST_FILENAME "model.config" // I would think I should get this from some gazebo include

using namespace std;

namespace gazebo
{
/* Places a series of SMORES modules into the scene.
 * Allows the user to include a robot as a "single" model, while maintaining distinct model controllers for each bot */
class SMORESFactory : public WorldPlugin
{
	public: SMORESFactory() : WorldPlugin()
	{
		cout << "SMORESFactory plugin instantiated" << endl;
	}
	public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
	{
		// Get the name of this plugin
		// Note: Gazebo requires that this attribute exists and is unique
		string factoryName;
		_sdf->GetAttribute(string("name"))->Get(factoryName); 
		
		// Grab the directory that the SMORES description is in
		string modelPath;
		if (_sdf->HasElement("uri"))
		{
			modelPath = uri2FilePath(_sdf->GetElement("uri")->Get<string>());
		} else {
			gzerr << "Missing parameter <uri> in " << factoryName << endl;
		}
		
		// Check for the manifest file
		boost::filesystem::path manifestPath = modelPath;
		if (checkManifest(manifestPath / GZ_MODEL_MANIFEST_FILENAME))
		{
			manifestPath /= GZ_MODEL_MANIFEST_FILENAME;
		}		
		
		// Parse the manifest file
		string sdfFileName;
		TiXmlDocument manifestDoc;
		if (manifestDoc.LoadFile(manifestPath.string()))
		{
			TiXmlElement *modelXML = manifestDoc.FirstChildElement("model");
			if (modelXML)
			{
				TiXmlElement *sdfXML = modelXML->FirstChildElement("sdf");
				sdfFileName = modelPath + "/" + sdfXML->GetText();
			} else {
				gzerr << "No <model> element in manifest[" << manifestPath << "]" << endl;
			}
		} else {
			gzerr << "Error parsing XML in manifest[" << manifestPath << "]" << endl;
		}
		
		// Parse the SMORES file
		TiXmlDocument sdfDoc;
		TiXmlElement *modelXML;
		if (sdfDoc.LoadFile(sdfFileName))
		{
			TiXmlElement *sdfXML = sdfDoc.FirstChildElement("sdf");
			if (!sdfXML)
			{
				gzerr << "No <sdf> element in sdf[" << sdfFileName << "]" << endl;
			} else {
				modelXML = sdfXML->FirstChildElement("model");
				if (!modelXML)
				{
					gzerr << "No <model> element in sdf[" << sdfFileName << "]" << endl;
				}
			}
		} else {
			gzerr << "Error parsing XML in sdf[" << sdfFileName << "]" << endl;
		}
		
		
		// Check if this series of SMORES modules has an overall pose
		math::Pose globalPose;
		if (_sdf->HasElement("pose"))
		{
			sdf::Pose globalPoseSDF = _sdf->GetElement("pose")->Get<sdf::Pose>();
			// Get euler angles from the sdf::Pose.rot sdf::Quaternion
			sdf::Vector3 globalPoseSDFerot = globalPoseSDF.rot.GetAsEuler();
			// Convert everything to a gazebo::math::Pose
			globalPose = math::Pose(globalPoseSDF.pos.x,
									globalPoseSDF.pos.y,
									globalPoseSDF.pos.z,
									globalPoseSDFerot.x,
									globalPoseSDFerot.y,
									globalPoseSDFerot.z);
		} else {
			cout << "No <pose> identified in " << factoryName << ". Defaulting pose to the origin." << endl;
			globalPose = math::Pose::Zero;		
		}
		
		
		// Loop through all of the xml elements and add each SMORES module to the world
		unsigned int noNameIndex = 0;
		for (TiXmlElement *childElemXml = modelXML->FirstChildElement();
			childElemXml; childElemXml = childElemXml->NextSiblingElement())
		{
			string thisModelString;
			sdf::SDF thisModelSdf;
			if (string("include") != childElemXml->Value())
			{
				gzerr << "SMORES file should contain only include elements" << endl;
			}
			
			thisModelString = getSdfString(childElemXml,globalPose,factoryName,
				noNameIndex);
			
			thisModelSdf.SetFromString(thisModelString);
			_world->InsertModelSDF(thisModelSdf);
						
			cout << "Successful!" << endl;
		}
		
		//TODO: Once dynamic joint API is finalized, we will need to add creation of dynamic joints upon loading
	}
	
	// Parse one <include> statement and add the proper model to the world
	private: string getSdfString(const TiXmlElement *inclElemXml,
		 const math::Pose &parentPose, const string factoryName, 
		 unsigned int &namingIndex)
	{
		// Get this model's name
		string modelName = factoryName + string("::");
		if (inclElemXml->FirstChildElement("name"))
		{
			modelName += inclElemXml->FirstChildElement("name")->GetText();
		} else {
			char *namingIndexChar;
			sprintf(namingIndexChar, "%u", namingIndex);
			modelName += namingIndexChar;
			namingIndex += 1; // Increment for unique model names
		}
		
		// Get this model's pose
		math::Pose modelPose = parentPose;
		if (inclElemXml->FirstChildElement("pose"))
		{
			string poseString = inclElemXml->FirstChildElement("pose")->GetText();
			char *poseChar = (char *)poseString.c_str();
			double x = atof(strsep(&poseChar, " "));
			double y = atof(strsep(&poseChar, " "));
			double z = atof(strsep(&poseChar, " "));
			double r = atof(strsep(&poseChar, " "));
			double p = atof(strsep(&poseChar, " "));
			double yaw = atof(strsep(&poseChar, " "));
			
			/* TODO: This needs to be fixed to properly check if the parsing makes any sense
			try {
				cout << "code over here runs " << endl;
				double thisisnothing = atof(strsep(&poseChar, " "));
				cout << "i doubt this code runs " << endl;
			} catch (exception& e) {
				cout << "exception occurred" << endl;
			}

			cout << "this code runs " << endl;
				
			if (poseChar || (!x) || (!y) || (!z) || (!r) || (!p) || (!yaw) )
			{
				gzerr << "Error parsing <pose> element for " << modelName << endl;
			} */
			modelPose = modelPose + math::Pose(x,y,z,r,p,yaw);
		} else {
			gzerr << "No <pose> element for " << modelName << endl;
		}
		
		
		// Parse uri into a full file path
		string modelPath;
		if (inclElemXml->FirstChildElement("uri"))
		{
			modelPath = uri2FilePath(
				string(inclElemXml->FirstChildElement("uri")->GetText()));
		} else {
			gzerr << "Missing parameter <uri> in " << modelName << endl;
		}
		// Check for the manifest file
		boost::filesystem::path manifestPath = modelPath;
		if (checkManifest(manifestPath / SMORES_MANIFEST_FILENAME))
		{
			manifestPath /= SMORES_MANIFEST_FILENAME;
		}
		// Parse the manifest file
		string sdfFileName;
		TiXmlDocument manifestDoc;
		if (manifestDoc.LoadFile(manifestPath.string()))
		{
			TiXmlElement *modelXML = manifestDoc.FirstChildElement("model");
			if (modelXML)
			{
				TiXmlElement *sdfXML = modelXML->FirstChildElement("sdf");
				TiXmlElement *sdfSearch = sdfXML;
				
				// search for sdf that matches our version
				while (sdfSearch) 
				{
					if (sdfSearch->Attribute("version") && 
						string(sdfSearch->Attribute("version")) == sdf::SDF::version)
					{
						sdfXML = sdfSearch;
						break;
					}
					sdfSearch = sdfSearch->NextSiblingElement("sdf");
				}
				
				sdfFileName = modelPath + "/" + sdfXML->GetText();
			} else {
				gzerr << "No <model> element in manifest[" << manifestPath << "]" << endl;
			}
		} else {
			gzerr << "Error parsing XML in manifest[" << manifestPath << "]" << endl;
		}
		
		// Parse the sdf file
		TiXmlDocument sdfDoc;
		TiXmlElement *sdfModel;
		if (sdfDoc.LoadFile(sdfFileName))
		{
			if (sdfDoc.FirstChildElement("sdf"))
			{
				if (sdfDoc.FirstChildElement("sdf")->FirstChildElement("model"))
				{
					sdfModel = (TiXmlElement *) sdfDoc.FirstChildElement("sdf")->FirstChildElement("model");
				} else {
					gzerr << "No <model> element in sdf[" << sdfFileName << "]" << endl;
				}
			} else {
				gzerr << "No <sdf> element in sdf[" << sdfFileName << "]" << endl;
			}
		} else {
			gzerr << "Error parsing XML in sdf[" << sdfFileName << "]" << endl;
		} 
		
		// Change the model name
		sdfModel->SetAttribute(string("name"), modelName);
		
		// Update the model pose
		// Create the text to add to the element
		stringstream poseStream;
		poseStream << modelPose; // used to convert pose to string
		TiXmlText *poseText = new TiXmlText(poseStream.str());
		if (sdfModel->FirstChildElement("pose"))
		{
			// Delete all children - pose should only have one child (text)
			sdfModel->FirstChildElement("pose")->Clear();
			// Add our text child
			sdfModel->FirstChildElement("pose")->LinkEndChild(poseText);
		} else {
			// Create the pose element
			TiXmlElement *poseElement = new TiXmlElement("pose");
			// Add text to the pose element
			poseElement->LinkEndChild(poseText);
			// Insert the element
			sdfModel->LinkEndChild(poseElement);
		}
		
		
		// Prefix the plugin name
		if (sdfModel->FirstChildElement("plugin"))
		{
			const char *pluginName = sdfModel->FirstChildElement("plugin")->Attribute("name");
			string newPluginName = modelName + "::" + string(pluginName);
			sdfModel->FirstChildElement("plugin")->SetAttribute(
				string("name"), newPluginName);

		} else { 
			gzwarn << "No <plugin> element in sdf[" << sdfFileName << "]"
				<< ". This may not be an error." << endl;
		}
		
		
		// Import the model from a string
		stringstream fullSdfStringStream;
		fullSdfStringStream << sdfDoc;
		cout << "Adding " << modelName << " to simulation...";
		return fullSdfStringStream.str();
	}


	// Take the <uri> string value and convert it to a full file path, 
	private: static string uri2FilePath(const string uri)
	{
		string modelPath = common::SystemPaths::Instance()->FindFileURI(uri);
		
		if (modelPath.empty())
		{
			gzerr << "Unable to find uri[" << uri << "]" << endl;
		} else {
			boost::filesystem::path dir(modelPath);
			if (!boost::filesystem::exists(dir) || !boost::filesystem::is_directory(dir))
			{
				gzerr << "Directory doesn't exist[" << modelPath << "]" << endl;
			}
		}
		
		return modelPath;
	}
	
	// Check to see if a path exists
	private: static bool checkManifest(boost::filesystem::path pathToCheck)
	{
		if (!boost::filesystem::exists(pathToCheck))
		{
			gzerr << pathToCheck << " not found" << endl;
			// no support for Gazebo's depricated manifest.xml
			return false;
		} else {
			return true;
		}
	}
	
};
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(SMORESFactory);
}
