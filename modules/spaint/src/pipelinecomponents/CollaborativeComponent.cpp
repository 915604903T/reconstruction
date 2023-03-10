/**
 * spaint: CollaborativeComponent.cpp
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#include "pipelinecomponents/CollaborativeComponent.h"
using namespace ITMLib;
using namespace ORUtils;
using namespace itmx;

#include <algorithm>

#include <boost/bind.hpp>
using boost::bind;

#ifdef WITH_OPENCV
#include <itmx/ocv/OpenCVUtil.h>
#endif

#include <orx/geometry/GeometryUtil.h>
#include <orx/relocalisation/Relocaliser.h>
using namespace orx;

#define DEBUGGING 0

namespace spaint {

//#################### CONSTRUCTORS ####################

CollaborativeComponent::CollaborativeComponent(const CollaborativeContext_Ptr& context, CollaborationMode mode)
: m_context(context),
  m_frameIndex(0),
  m_mode(mode),
  m_reconstructionIsConsistent(false),
  m_rng(12345),
  m_stopRelocalisationThread(false),
  m_visualisationGenerator(new VisualisationGenerator(context->get_settings()))
{
  const Settings_CPtr& settings = context->get_settings();
  const std::string settingsNamespace = "CollaborativeComponent.";
  m_considerPoorRelocalisations = settings->get_first_value<bool>(settingsNamespace + "considerPoorRelocalisations", mode == CM_LIVE);
  m_stopAtFirstConsistentReconstruction = settings->get_first_value<bool>(settingsNamespace + "stopAtFirstConsistentReconstruction", false);
  m_timeCollaboration = settings->get_first_value<bool>(settingsNamespace + "timeCollaboration", false);

  m_relocalisationThread = boost::thread(boost::bind(&CollaborativeComponent::run_relocalisation, this));

  const std::string globalPosesSpecifier = settings->get_first_value<std::string>("globalPosesSpecifier", "");
  m_context->get_collaborative_pose_optimiser()->start(globalPosesSpecifier);
}

//#################### DESTRUCTOR ####################

CollaborativeComponent::~CollaborativeComponent()
{
  m_stopRelocalisationThread = true;
  m_readyToRelocalise.notify_one();
  m_relocalisationThread.join();

  // If we're computing the time spent collaborating:
  if(m_collaborationTimer)
  {
    // Stop the collaboration timer if it is still running (e.g. if we didn't stop at the first consistent reconstruction).
    m_collaborationTimer->stop();

    // Output the time spent collaborating.
    std::cout << "Time spent collaborating: " << m_collaborationTimer->format(3) << '\n';
  }

#if DEBUGGING
  output_results();
#endif
}

//#################### PUBLIC MEMBER FUNCTIONS ####################

void CollaborativeComponent::run_collaborative_pose_estimation()
{
  bool fusionMayStillRun = update_trajectories();
  if(!fusionMayStillRun) m_mode = CM_BATCH;
  if(m_frameIndex > 0 && (!fusionMayStillRun || (m_mode == CM_LIVE && m_frameIndex % 50 == 0)))
  {
    // Start the collaboration timer if required.
    if(m_timeCollaboration && !m_collaborationTimer)
    {
      std::cout << "Collaboration starting at frame: " << m_frameIndex << '\n';
      m_collaborationTimer.reset(boost::timer::cpu_timer());
    }

    // Check to see whether the reconstruction has just become consistent.
    if(!m_reconstructionIsConsistent)
    {
      const std::vector<std::string> sceneIDs = m_context->get_scene_ids();
      m_reconstructionIsConsistent = true;
      for(size_t sceneIdx = 0; sceneIdx < sceneIDs.size(); ++sceneIdx)
      {
        if(!m_context->get_collaborative_pose_optimiser()->try_get_estimated_global_pose(sceneIDs[sceneIdx]))
        {
          m_reconstructionIsConsistent = false;
          break;
        }
      }

      if(m_reconstructionIsConsistent) std::cout << "The reconstruction became consistent at frame: " << m_frameIndex << '\n';
    }

    // If the reconstruction is consistent and we're stopping at the first consistent reconstruction:
    if(m_reconstructionIsConsistent && m_stopAtFirstConsistentReconstruction)
    {
      // Stop the collaboration timer if necessary.
      if(m_collaborationTimer) m_collaborationTimer->stop();

      // Early out to prevent any more relocalisation attempts being scheduled.
      return;
    }

    // Otherwise, try to schedule a relocalisation attempt.
    try_schedule_relocalisation();
  }

  ++m_frameIndex;

#if defined(WITH_OPENCV) && 0
  cv::waitKey(1);
#endif
}

//#################### PRIVATE MEMBER FUNCTIONS ####################

std::list<CollaborativeRelocalisation> CollaborativeComponent::generate_random_candidates(size_t desiredCandidateCount) const
{
  static bool batchThisTime = true;
  batchThisTime = !batchThisTime;

  std::list<CollaborativeRelocalisation> candidates;

  const int sceneCount = static_cast<int>(m_trajectories.size());
  if(sceneCount < 2) return std::list<CollaborativeRelocalisation>();

  for(size_t i = 0; i < desiredCandidateCount; ++i)
  {
    // Randomly select the trajectories of two different scenes.
    int ki = m_rng.generate_int_from_uniform(0, sceneCount - 1);
    int kj = m_rng.generate_int_from_uniform(0, sceneCount - 2);
    if(kj >= ki) ++kj;

    std::map<std::string,std::deque<ORUtils::SE3Pose> >::const_iterator it = m_trajectories.begin();
    std::map<std::string,std::deque<ORUtils::SE3Pose> >::const_iterator jt = m_trajectories.begin();
    std::advance(it, ki);
    std::advance(jt, kj);

    const std::string sceneI = it->first;
    const std::string sceneJ = jt->first;

    SLAMState_CPtr slamStateI = m_context->get_slam_state(sceneI);
    SLAMState_CPtr slamStateJ = m_context->get_slam_state(sceneJ);

    // Randomly pick a frame from scene j.
    const int frameCountJ = static_cast<int>(jt->second.size());
    const int frameIndexJ = m_mode == CM_BATCH || slamStateJ->get_input_status() == SLAMState::IS_TERMINATED || batchThisTime
      ? m_rng.generate_int_from_uniform(0, frameCountJ - 1)
      : std::max(0, frameCountJ - 20);

    // Add a candidate to relocalise the selected frame of scene j against scene i.
    const Vector4f& depthIntrinsicsI = slamStateI->get_intrinsics().projectionParamsSimple.all;
    const ORUtils::SE3Pose& localPoseJ = jt->second[frameIndexJ];
    candidates.push_back(CollaborativeRelocalisation(sceneI, depthIntrinsicsI, sceneJ, frameIndexJ, localPoseJ));
  }

  return candidates;
}

std::list<CollaborativeRelocalisation> CollaborativeComponent::generate_sequential_candidate() const
{
  std::list<CollaborativeRelocalisation> candidates;

  const int sceneCount = static_cast<int>(m_trajectories.size());
  if(sceneCount < 2) return std::list<CollaborativeRelocalisation>();

  // Randomly select two different scenes.
  std::map<std::string,std::deque<ORUtils::SE3Pose> >::const_iterator it1 = m_trajectories.begin();
  std::map<std::string,std::deque<ORUtils::SE3Pose> >::const_iterator it2 = m_trajectories.begin();
  ++it2;

  const std::string scene1 = it1->first;
  const std::string scene2 = it2->first;

  for(int n = 0; n < 2; ++n)
  {
    const std::string& sceneI = n == 0 ? scene1 : scene2;
    const std::string& sceneJ = n == 0 ? scene2 : scene1;
    const std::deque<ORUtils::SE3Pose>& trajectoryJ = n == 0 ? it2->second : it1->second;

    // Try to add a candidate to relocalise the next untried frame (if any) of scene j against scene i.
    std::map<std::pair<std::string,std::string>,std::set<int> >::const_iterator kt = m_triedFrameIndices.find(std::make_pair(sceneI, sceneJ));
    const int frameIndexJ = kt != m_triedFrameIndices.end() ? *kt->second.rbegin() + 1 : 0;
    const int frameCountJ = static_cast<int>(trajectoryJ.size());
    if(frameIndexJ < frameCountJ)
    {
      const Vector4f& depthIntrinsicsI = m_context->get_slam_state(sceneI)->get_intrinsics().projectionParamsSimple.all;
      const ORUtils::SE3Pose& localPoseJ = trajectoryJ[frameIndexJ];
      candidates.push_back(CollaborativeRelocalisation(sceneI, depthIntrinsicsI, sceneJ, frameIndexJ, localPoseJ));
      return candidates;
    }
  }

  return candidates;
}

bool CollaborativeComponent::is_verified(const CollaborativeRelocalisation& candidate) const
{
#ifdef WITH_OPENCV
  // Note: In live mode, we use a slightly more tolerant threshold because the relocalisers have been less thoroughly trained.
  // FIXME: This is a bit hacky - we might want to improve this in the future.
  const float depthDiffThreshold = m_mode == CM_LIVE ? 10.0f : 5.0f;

  return candidate.m_meanDepthDiff(0) < depthDiffThreshold && candidate.m_targetValidFraction >= 0.5f;
#else
  return true;
#endif
}

void CollaborativeComponent::output_results() const
{
  int good = 0, bad = 0, unknown = 0;
  int failed = 0, rejected = 0, verified = 0;
  size_t firstVerification = 0;

  // First, output the results themselves.
  std::cout << "SceneI\tSceneJ\tFrameIndexJ\tInitialQuality\t"
#ifdef WITH_OPENCV
            << "MeanDepthDiff\tTargetValidFraction\t"
#endif
            << "FinalStatus\tGoodStatus\n\n";

  for(size_t i = 0, size = m_results.size(); i < size; ++i)
  {
    const CollaborativeRelocalisation& result = m_results[i];

    std::string finalStatus;
    if(is_verified(result))
    {
      finalStatus = "Verified";
      ++verified;
      if(firstVerification == 0)
      {
        firstVerification = i + 1;
      }
    }
    else if(result.m_initialRelocalisationQuality == Relocaliser::RELOCALISATION_GOOD)
    {
      finalStatus = "Rejected";
      ++rejected;
    }
    else
    {
      finalStatus = "Failed";
      ++failed;
    }

    std::string goodStatus;
    boost::optional<std::pair<ORUtils::SE3Pose,size_t> > relativeTransform = m_context->get_collaborative_pose_optimiser()->try_get_relative_transform(result.m_sceneI, result.m_sceneJ);
    if(!relativeTransform)
    {
      goodStatus = "Unknown";
      ++unknown;
    }
    else
    {
      bool similar;
      try        { similar = GeometryUtil::poses_are_similar(relativeTransform->first, *result.m_relativePose); }
      catch(...) { similar = false; }

      if(similar)
      {
        goodStatus = "Good";
        ++good;
      }
      else
      {
        goodStatus = "Bad";
        ++bad;
      }
    }

    std::cout << std::fixed << std::setprecision(2);
    std::cout << result.m_sceneI << '\t' << result.m_sceneJ << '\t' << result.m_frameIndexJ << '\t'
              << (result.m_initialRelocalisationQuality == Relocaliser::RELOCALISATION_GOOD ? "Good" : "Poor") << '\t'
#ifdef WITH_OPENCV
              << result.m_meanDepthDiff(0) << '\t' << result.m_targetValidFraction << '\t'
#endif
              << finalStatus << '\t' << goodStatus << '\n';
  }

  // Then, output the overall statistics.
  std::cout << "\nTried: " << m_results.size() << '\n';
  if(!m_results.empty())
  {
    float tried = static_cast<float>(m_results.size());
    std::cout << "Verified: " << verified << " (" << verified * 100 / tried << "%)\n";
    std::cout << "Rejected: " << rejected << " (" << rejected * 100 / tried << "%)\n";
    std::cout << "Failed: " << failed << " (" << failed * 100 / tried << "%)\n\n";

    std::cout << "Good: " << good << " (" << good * 100 / tried << "%)\n";
    std::cout << "Bad: " << bad << " (" << bad * 100 / tried << "%)\n";
    std::cout << "Unknown: " << unknown << " (" << unknown * 100 / tried << "%)\n\n";

    std::cout << "Expected Tries To First Verification: " << static_cast<int>(ceil(tried / verified)) << '\n';
    std::cout << "Actual Tries To First Verification: " << firstVerification << '\n';
  }

  // Finally, output the cluster statistics.
  CollaborativePoseOptimiser_CPtr poseOptimiser = m_context->get_collaborative_pose_optimiser();
  const std::vector<std::string> sceneIDs = m_context->get_scene_ids();
  for(size_t i = 0, sceneCount = sceneIDs.size(); i < sceneCount; ++i)
  {
    for(size_t j = 0; j < sceneCount; ++j)
    {
      if(j == i) continue;

      boost::optional<std::vector<CollaborativePoseOptimiser::SE3PoseCluster> > result = poseOptimiser->try_get_relative_transform_samples(sceneIDs[i], sceneIDs[j]);
      if(!result) continue;

      std::cout << "\nClusters " << i << " <- " << j << "\n\n";

      const std::vector<CollaborativePoseOptimiser::SE3PoseCluster>& clusters = *result;
      const size_t clusterCount = clusters.size();

      // Find the largest cluster.
      int largestClusterIndex = -1;
      size_t largestClusterSize = 0;
      for(size_t k = 0; k < clusterCount; ++k)
      {
        size_t clusterSize = clusters[k].size();
        if(clusterSize > largestClusterSize)
        {
          largestClusterIndex = static_cast<int>(k);
          largestClusterSize = clusterSize;
        }
      }

      boost::optional<SE3Pose> correctTransform = largestClusterIndex != -1 ? boost::optional<SE3Pose>(GeometryUtil::blend_poses(clusters[largestClusterIndex])) : boost::none;

      // Print out the individual clusters.
      for(size_t k = 0; k < clusterCount; ++k)
      {
        SE3Pose blendedTransform = GeometryUtil::blend_poses(clusters[k]);
        Vector3f t, r;
        blendedTransform.GetParams(t, r);

        bool correct = GeometryUtil::poses_are_similar(blendedTransform, *correctTransform, 20 * M_PI / 180, 0.10f);

        std::cout << (correct ? "Correct" : "Incorrect") << "; " << clusters[k].size() << "; " << t << "; " << r << '\n';
      }
    }
  }
}

void CollaborativeComponent::run_relocalisation()
{
  while(!m_stopRelocalisationThread)
  {
    // Wait for a relocalisation to be scheduled.
    {
      boost::unique_lock<boost::mutex> lock(m_mutex);
      while(!m_bestCandidate)
      {
        m_readyToRelocalise.wait(lock);

        // If the collaborative component is terminating, stop attempting relocalisations and let the thread terminate.
        if(m_stopRelocalisationThread) return;
      }
    }
	
    std::cout << "Attempting to relocalise frame " << m_bestCandidate->m_frameIndexJ << " of " << m_bestCandidate->m_sceneJ << " against " << m_bestCandidate->m_sceneI << "...";

    // Render synthetic images of the source scene from the relevant pose and copy them across to the GPU for use by the relocaliser.
    // The synthetic images have the size of the images in the target scene and are generated using the target scene's intrinsics.
    const SLAMState_CPtr slamStateI = m_context->get_slam_state(m_bestCandidate->m_sceneI);
    const SLAMState_CPtr slamStateJ = m_context->get_slam_state(m_bestCandidate->m_sceneJ);
    const View_CPtr viewI = slamStateI->get_view();

    ORFloatImage_Ptr depth(new ORFloatImage(slamStateI->get_depth_image_size(), true, true));
    ORUChar4Image_Ptr rgb(new ORUChar4Image(slamStateI->get_rgb_image_size(), true, true));

    VoxelRenderState_Ptr& renderStateD = m_depthRenderStates[m_bestCandidate->m_sceneI];
    m_visualisationGenerator->generate_depth_from_voxels(
      depth, slamStateJ->get_voxel_scene(), m_bestCandidate->m_localPoseJ, viewI->calib.intrinsics_d,
      renderStateD, DepthVisualiser::DT_ORTHOGRAPHIC
    );

    VoxelRenderState_Ptr& renderStateRGB = m_rgbRenderStates[m_bestCandidate->m_sceneI];
    m_visualisationGenerator->generate_voxel_visualisation(
      rgb, slamStateJ->get_voxel_scene(), m_bestCandidate->m_localPoseJ, viewI->calib.intrinsics_rgb,
      renderStateRGB, VisualisationGenerator::VT_SCENE_COLOUR, boost::none
    );

    depth->UpdateDeviceFromHost();
    rgb->UpdateDeviceFromHost();

#ifdef WITH_OPENCV
    // Make OpenCV copies of the synthetic images we're trying to relocalise (these may be needed later).
    cv::Mat3b cvSourceRGB = OpenCVUtil::make_rgb_image(rgb->GetData(MEMORYDEVICE_CPU), rgb->noDims.x, rgb->noDims.y);
    cv::Mat1b cvSourceDepth = OpenCVUtil::make_greyscale_image(depth->GetData(MEMORYDEVICE_CPU), depth->noDims.x, depth->noDims.y, OpenCVUtil::ROW_MAJOR, 100.0f);

  #if DEBUGGING
    // If we're debugging, show the synthetic images of the source scene to the user.
    cv::imshow("Source Depth", cvSourceDepth);
    cv::imshow("Source RGB", cvSourceRGB);
  #endif
#endif

    // Attempt to relocalise the synthetic images using the relocaliser for the target scene.
    Relocaliser_CPtr relocaliserI = m_context->get_relocaliser(m_bestCandidate->m_sceneI);
    std::vector<Relocaliser::Result> results = relocaliserI->relocalise(rgb.get(), depth.get(), m_bestCandidate->m_depthIntrinsicsI);
    boost::optional<Relocaliser::Result> result = results.empty() ? boost::none : boost::optional<Relocaliser::Result>(results[0]);

    // If the relocaliser returned a result, store the initial relocalisation quality for later examination.
    if(result) m_bestCandidate->m_initialRelocalisationQuality = result->quality;

    // If relocalisation succeeded, verify the result by thresholding the difference between the
    // source depth image and a rendered depth image of the target scene at the relevant pose.
    bool verified = false;
    if(result && (result->quality == Relocaliser::RELOCALISATION_GOOD || m_considerPoorRelocalisations))
    {
#ifdef WITH_OPENCV
      // Render synthetic images of the target scene from the relevant pose.
      m_visualisationGenerator->generate_depth_from_voxels(
        depth, slamStateI->get_voxel_scene(), result->pose.GetM(), viewI->calib.intrinsics_d,
        renderStateD, DepthVisualiser::DT_ORTHOGRAPHIC
      );

      m_visualisationGenerator->generate_voxel_visualisation(
        rgb, slamStateI->get_voxel_scene(), result->pose.GetM(), viewI->calib.intrinsics_rgb,
        renderStateRGB, VisualisationGenerator::VT_SCENE_COLOUR, boost::none
      );

      // Make OpenCV copies of the synthetic images of the target scene.
      cv::Mat3b cvTargetRGB = OpenCVUtil::make_rgb_image(rgb->GetData(MEMORYDEVICE_CPU), rgb->noDims.x, rgb->noDims.y);
      cv::Mat1b cvTargetDepth = OpenCVUtil::make_greyscale_image(depth->GetData(MEMORYDEVICE_CPU), depth->noDims.x, depth->noDims.y, OpenCVUtil::ROW_MAJOR, 100.0f);

    #if DEBUGGING
      // If we're debugging, show the synthetic images of the target scene to the user.
      cv::imshow("Target RGB", cvTargetRGB);
      cv::imshow("Target Depth", cvTargetDepth);
    #endif

      // Compute a binary mask showing which pixels are valid in both the source and target depth images.
      cv::Mat cvSourceMask;
      cv::inRange(cvSourceDepth, cv::Scalar(0,0,0), cv::Scalar(0,0,0), cvSourceMask);
      cv::bitwise_not(cvSourceMask, cvSourceMask);

      cv::Mat cvTargetMask;
      cv::inRange(cvTargetDepth, cv::Scalar(0,0,0), cv::Scalar(0,0,0), cvTargetMask);
      cv::bitwise_not(cvTargetMask, cvTargetMask);

      cv::Mat cvCombinedMask;
      cv::bitwise_and(cvSourceMask, cvTargetMask, cvCombinedMask);

      // Compute the difference between the source and target depth images, and mask it using the combined mask.
      cv::Mat cvDepthDiff, cvMaskedDepthDiff;
      cv::absdiff(cvSourceDepth, cvTargetDepth, cvDepthDiff);
      cvDepthDiff.copyTo(cvMaskedDepthDiff, cvCombinedMask);
    #if DEBUGGING
      cv::imshow("Masked Depth Difference", cvMaskedDepthDiff);
    #endif

      // Determine the average depth difference for valid pixels in the source and target depth images.
      m_bestCandidate->m_meanDepthDiff = cv::mean(cvMaskedDepthDiff);
    #if DEBUGGING
      std::cout << "\nMean Depth Difference: " << m_bestCandidate->m_meanDepthDiff << std::endl;
    #endif

      // Compute the fraction of the target depth image that is valid.
      m_bestCandidate->m_targetValidFraction = static_cast<float>(cv::countNonZero(cvTargetMask == 255)) / (cvTargetMask.size().width * cvTargetMask.size().height);
    #if DEBUGGING
      std::cout << "Valid Target Pixels: " << cv::countNonZero(cvTargetMask == 255) << std::endl;
    #endif

      // Decide whether or not to verify the relocalisation, based on the average depth difference and the fraction of the target depth image that is valid.
      verified = is_verified(*m_bestCandidate);
#else
      // If we didn't build with OpenCV, we can't do any verification, so just mark the relocalisation as verified and hope for the best.
      verified = true;
#endif
    }

    // If relocalisation succeeded and we successfully verified the result, add a sample of the
    // relative transform between the source and target scenes to the pose graph optimiser.
    if(verified)
    {
      // cjTwi^-1 * cjTwj = wiTcj * cjTwj = wiTwj
      m_bestCandidate->m_relativePose = ORUtils::SE3Pose(result->pose.GetInvM() * m_bestCandidate->m_localPoseJ.GetM());
      m_context->get_collaborative_pose_optimiser()->add_relative_transform_sample(m_bestCandidate->m_sceneI, m_bestCandidate->m_sceneJ, *m_bestCandidate->m_relativePose, m_mode);
      std::cout << "succeeded!" << std::endl;

#if defined(WITH_OPENCV) && DEBUGGING
      cv::waitKey(1);
#endif
    }
    else
    {
      std::cout << "failed :(" << std::endl;

#if defined(WITH_OPENCV) && DEBUGGING
      cv::waitKey(1);
#endif
    }

    // Record the results of the relocalisation we just tried if desired, and prepare for another relocalisation.
    {
      boost::unique_lock<boost::mutex> lock(m_mutex);
#if DEBUGGING
      m_results.push_back(*m_bestCandidate);
#endif
      m_bestCandidate.reset();
    }

    // In live mode, allow a bit of extra time for training before running the next relocalisation.
    // FIXME: This is a bit hacky - we might want to improve this in the future.
    if(m_mode == CM_LIVE) boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
  }
}

void CollaborativeComponent::score_candidates(std::list<CollaborativeRelocalisation>& candidates) const
{
  for(std::list<CollaborativeRelocalisation>::iterator it = candidates.begin(), iend = candidates.end(); it != iend; ++it)
  {
    CollaborativeRelocalisation& candidate = *it;

    // Boost candidates that may attach new confident nodes to the graph.
    CollaborativePoseOptimiser_CPtr poseOptimiser = m_context->get_collaborative_pose_optimiser();
    boost::optional<ORUtils::SE3Pose> globalPoseI = poseOptimiser->try_get_estimated_global_pose(candidate.m_sceneI);
    boost::optional<ORUtils::SE3Pose> globalPoseJ = poseOptimiser->try_get_estimated_global_pose(candidate.m_sceneJ);
    float newNodeBoost = (globalPoseI && !globalPoseJ) || (globalPoseJ && !globalPoseI) ? 1.0f : 0.0f;

    // Penalise candidates that will only add to an existing confident edge.
    boost::optional<CollaborativePoseOptimiser::SE3PoseCluster> largestCluster = poseOptimiser->try_get_largest_cluster(candidate.m_sceneI, candidate.m_sceneJ);
    int largestClusterSize = largestCluster ? static_cast<int>(largestCluster->size()) : 0;
    float confidencePenalty = 1.0f * std::max(largestClusterSize - CollaborativePoseOptimiser::confidence_threshold(), 0);

    // If we're in batch mode, penalise candidates that are too close to ones we've tried before.
    float homogeneityPenalty = 0.0f;

    if(m_mode == CM_BATCH)
    {
      std::map<std::pair<std::string,std::string>,std::set<int> >::const_iterator jt = m_triedFrameIndices.find(std::make_pair(candidate.m_sceneI, candidate.m_sceneJ));
      if(jt != m_triedFrameIndices.end())
      {
        const std::set<int>& triedFrameIndices = jt->second;
        for(std::set<int>::const_iterator kt = triedFrameIndices.begin(), kend = triedFrameIndices.end(); kt != kend; ++kt)
        {
          const ORUtils::SE3Pose& triedPose = m_trajectories.find(candidate.m_sceneJ)->second[*kt];
          if(GeometryUtil::poses_are_similar(candidate.m_localPoseJ, triedPose, 5 * M_PI / 180))
          {
            homogeneityPenalty = 5.0f;
            break;
          }
        }
      }
    }

    candidate.m_candidateScore = newNodeBoost - confidencePenalty - homogeneityPenalty;
  }
}

void CollaborativeComponent::try_schedule_relocalisation()
{
  {
    boost::unique_lock<boost::mutex> lock(m_mutex);

    // If an existing relocalisation attempt is in progress, early out.
    if(m_bestCandidate) return;

#if 1
    // Randomly generate a list of candidate relocalisations.
    const size_t desiredCandidateCount = 10;
    std::list<CollaborativeRelocalisation> candidates = generate_random_candidates(desiredCandidateCount);
#else
    // Generate the frames from the source scene in order, for evaluation purposes.
    std::list<CollaborativeRelocalisation> candidates = generate_sequential_candidate();
#endif
    if(candidates.empty()) return;

    // Score all of the candidates.
    score_candidates(candidates);

    // Sort the candidates in ascending order of score (this isn't strictly necessary, but makes debugging easier).
    candidates.sort(bind(&CollaborativeRelocalisation::m_candidateScore, _1) < bind(&CollaborativeRelocalisation::m_candidateScore, _2));

#if 0
    // Print out all of the candidates for debugging purposes.
    std::cout << "BEGIN CANDIDATES " << m_frameIndex << '\n';
    for(std::list<CollaborativeRelocalisation>::const_iterator it = candidates.begin(), iend = candidates.end(); it != iend; ++it)
    {
      const CollaborativeRelocalisation& candidate = *it;
      std::cout << candidate.m_sceneI << ' ' << candidate.m_sceneJ << ' ' << candidate.m_frameIndexJ << ' ' << candidate.m_candidateScore << '\n';
    }
    std::cout << "END CANDIDATES\n";
#endif

    // Schedule the best candidate for relocalisation.
    m_bestCandidate.reset(new CollaborativeRelocalisation(candidates.back()));

    // If we're in batch mode, record the index of the frame we're trying in case we want to avoid frames with similar poses later.
    if(m_mode == CM_BATCH)
    {
      std::set<int>& triedFrameIndices = m_triedFrameIndices[std::make_pair(m_bestCandidate->m_sceneI, m_bestCandidate->m_sceneJ)];
      triedFrameIndices.insert(m_bestCandidate->m_frameIndexJ);
    }
  }

  m_readyToRelocalise.notify_one();
}

bool CollaborativeComponent::update_trajectories()
{
  bool fusionMayStillRun = false;

  const std::vector<std::string> sceneIDs = m_context->get_scene_ids();
  for(size_t i = 0, sceneCount = sceneIDs.size(); i < sceneCount; ++i)
  {
    SLAMState_CPtr slamState = m_context->get_slam_state(sceneIDs[i]);
    if(!slamState || !slamState->get_view()) continue;

    SLAMState::InputStatus inputStatus = slamState->get_input_status();
    TrackingState_CPtr trackingState = slamState->get_tracking_state();
    if(inputStatus == SLAMState::IS_ACTIVE && trackingState->trackerResult == ITMTrackingState::TRACKING_GOOD)
    {
      m_trajectories[sceneIDs[i]].push_back(*trackingState->pose_d);
    }

    if(inputStatus != SLAMState::IS_TERMINATED) fusionMayStillRun = true;
  }

  return fusionMayStillRun;
}

}
