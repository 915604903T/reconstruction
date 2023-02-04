/**
 * spaintgui: CollaborativePipeline.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "CollaborativePipeline.h"
using namespace itmx;
using namespace spaint;

#include "itmx/imagesources/RemoteImageSourceEngine.h"

//#################### CONSTRUCTORS ####################

CollaborativePipeline::CollaborativePipeline(const Settings_Ptr &settings,
                                             const std::string &resourcesDir,
                                             const std::map<std::string, int> &scenesPoseCnt,
                                             const std::vector<CompositeImageSourceEngine_Ptr> &imageSourceEngines,
                                             const std::vector<std::string> &trackerConfigs,
                                             const std::vector<SLAMComponent::MappingMode> &mappingModes,
                                             const std::vector<SLAMComponent::TrackingMode> &trackingModes,
                                             bool detectFiducials,
                                             const MappingServer_Ptr &mappingServer,
                                             CollaborationMode collaborationMode,
                                             const std::map<std::string, std::string> &sceneDirs)
  // Note: A minimum of 2 labels is required (background and foreground).
  : MultiScenePipeline("collaborative", settings, resourcesDir, 2, mappingServer), 
    m_collaborationStarted(false), 
    m_detectFiducials(detectFiducials),
    // m_worldIsRemote(imageSourceEngines.empty()),
    m_worldIsRemote(false),
    m_sceneDirs(sceneDirs)
{
  // If local scenes were specified when the pipeline was instantiated, we add a SLAM component for each such scene.
  /* for(size_t i = 0, size = imageSourceEngines.size(); i < size; ++i)
  {
    const std::string sceneID = i == 0 ? Model::get_world_scene_id() : "Local" + boost::lexical_cast<std::string>(i);
    m_slamComponents[sceneID].reset(new SLAMComponent(m_model,
                                                      sceneID,
                                                      imageSourceEngines[i],
                                                      trackerConfigs[i],
                                                      mappingModes[i],
                                                      trackingModes[i],
                                                      detectFiducials));
  }*/
  std::map<std::string, TrackingController_Ptr> trackingControllers;
  for(size_t i = 0, size = imageSourceEngines.size(); i < size; ++i)
  {
    const std::string sceneID = i == 0 ? Model::get_world_scene_id() : "Local" + boost::lexical_cast<std::string>(i);
    m_slamComponents[sceneID].reset(new SLAMComponent(
        m_model, sceneID, imageSourceEngines[i], trackerConfigs[i], mappingModes[i], trackingModes[i], detectFiducials));
    load_models(m_slamComponents[sceneID], m_sceneDirs[sceneID]);
    trackingControllers[sceneID] = m_slamComponents[sceneID]->get_tracking_controller();
  }

  m_collaborativeComponent.reset(new CollaborativeComponent(m_model, collaborationMode, scenesPoseCnt, trackingControllers));
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

std::set<std::string> CollaborativePipeline::run_main_section()
{
  // If we're running a mapping server, add SLAM components for any newly-connected remote clients.
  /*
  if(m_model->get_mapping_server()) check_for_new_clients();

  // Run the main section of the pipeline.
  const std::set<std::string> scenesProcessed = MultiScenePipeline::run_main_section();

  // Provided at least one of the scenes has started fusion, run the collaborative pose estimation process.
  m_collaborationStarted = m_collaborationStarted || !scenesProcessed.empty();
  if(m_collaborationStarted) m_collaborativeComponent->run_collaborative_pose_estimation();
  */
  std::set<std::string> result;
  for(std::map<std::string,SLAMComponent_Ptr>::const_iterator it = m_slamComponents.begin(), iend = m_slamComponents.end(); it != iend; ++it)
  {
    std::cout << "[pipeline] this is slamcomponent first: " << it->first << "\n";
    bool isConsistent = m_collaborativeComponent->run_collaborative_pose_estimation();
    if (!isConsistent) 
    {
      result.insert(it->first);
    }
  }
  return result;
}

void CollaborativePipeline::set_mode(Mode mode)
{
  // The only supported mode.
  m_mode = MODE_NORMAL;
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

void CollaborativePipeline::add_remote_slam_component(const std::string &sceneID, int remoteClientID)
{
  std::cout << "Instantiating a SLAM component for client '" << remoteClientID << "', with local scene ID '" << sceneID
            << "'" << std::endl;

  // Note: We create remote clients that are voxel-only and have no support for fiducials.
  const MappingServer_Ptr &mappingServer = m_model->get_mapping_server();
  const ImageSourceEngine_Ptr imageSourceEngine(new RemoteImageSourceEngine(mappingServer, remoteClientID));
  const std::string trackerConfig =
      "<tracker type='remote'><params>" + boost::lexical_cast<std::string>(remoteClientID) + "</params></tracker>";
  const SLAMComponent::MappingMode mappingMode = SLAMComponent::MAP_VOXELS_ONLY;
  const SLAMComponent::TrackingMode trackingMode = SLAMComponent::TRACK_VOXELS;
  m_slamComponents[sceneID].reset(new SLAMComponent(
      m_model, sceneID, imageSourceEngine, trackerConfig, mappingMode, trackingMode, m_detectFiducials));
  mappingServer->set_scene_id(remoteClientID, sceneID);
}

void CollaborativePipeline::check_for_new_clients()
{
  // Check if there are new remote clients, and add SLAM components for them if so.
  const std::vector<int> activeClients = m_model->get_mapping_server()->get_active_clients();
  for(size_t i = 0, size = activeClients.size(); i < size; ++i)
  {
    const int clientID = activeClients[i];

    // Determine what the scene for this client should be called.
    const std::string sceneID = m_worldIsRemote && clientID == 0
                                    ? Model::get_world_scene_id()
                                    : "Remote" + boost::lexical_cast<std::string>(clientID);

    // If a SLAM component for this client does not yet exist, add one now.
    if(m_slamComponents.find(sceneID) == m_slamComponents.end())
    {
      add_remote_slam_component(sceneID, clientID);
    }
  }
}
