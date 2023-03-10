####################################
# CMakeLists.txt for modules/grove #
####################################

###########################
# Specify the target name #
###########################

SET(targetname grove)
cmake_policy(SET CMP0021 OLD)
################################
# Specify the libraries to use #
################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseALGLIB.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseBoost.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseCUDA.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseEigen.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseInfiniTAM.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenCV.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseOpenMP.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseScoreForests.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/UseTorch.cmake)

#############################
# Specify the project files #
#############################

##
SET(toplevel_sources src/CPUInstantiations.cpp)

IF(WITH_CUDA)
  SET(toplevel_sources ${toplevel_sources} src/CUDAInstantiations.cu)
ENDIF()

##
SET(clustering_headers include/grove/clustering/ExampleClustererFactory.h)
SET(clustering_templates include/grove/clustering/ExampleClustererFactory.tpp)

##
SET(clustering_cpu_headers include/grove/clustering/cpu/ExampleClusterer_CPU.h)
SET(clustering_cpu_templates include/grove/clustering/cpu/ExampleClusterer_CPU.tpp)

##
SET(clustering_cuda_headers include/grove/clustering/cuda/ExampleClusterer_CUDA.h)
SET(clustering_cuda_templates include/grove/clustering/cuda/ExampleClusterer_CUDA.tcu)

##
SET(clustering_interface_headers include/grove/clustering/interface/ExampleClusterer.h)
SET(clustering_interface_templates include/grove/clustering/interface/ExampleClusterer.tpp)

##
SET(clustering_shared_headers include/grove/clustering/shared/ExampleClusterer_Shared.h)

##
SET(features_sources src/features/FeatureCalculatorFactory.cpp)
SET(features_headers include/grove/features/FeatureCalculatorFactory.h)
SET(features_templates include/grove/features/FeatureCalculatorFactory.tpp)

##
SET(features_base_headers
include/grove/features/base/Descriptor.h
include/grove/features/base/RGBDPatchFeatureDifferenceType.h
)

##
SET(features_cpu_headers include/grove/features/cpu/RGBDPatchFeatureCalculator_CPU.h)
SET(features_cpu_templates include/grove/features/cpu/RGBDPatchFeatureCalculator_CPU.tpp)

##
SET(features_cuda_headers include/grove/features/cuda/RGBDPatchFeatureCalculator_CUDA.h)
SET(features_cuda_templates include/grove/features/cuda/RGBDPatchFeatureCalculator_CUDA.tcu)

##
SET(features_interface_headers include/grove/features/interface/RGBDPatchFeatureCalculator.h)
SET(features_interface_templates include/grove/features/interface/RGBDPatchFeatureCalculator.tpp)

##
SET(features_shared_headers include/grove/features/shared/RGBDPatchFeatureCalculator_Shared.h)

##
SET(forests_headers include/grove/forests/DecisionForestFactory.h)
SET(forests_templates include/grove/forests/DecisionForestFactory.tpp)

##
SET(forests_cpu_headers include/grove/forests/cpu/DecisionForest_CPU.h)
SET(forests_cpu_templates include/grove/forests/cpu/DecisionForest_CPU.tpp)

##
SET(forests_cuda_headers include/grove/forests/cuda/DecisionForest_CUDA.h)
SET(forests_cuda_templates include/grove/forests/cuda/DecisionForest_CUDA.tcu)

##
SET(forests_interface_headers include/grove/forests/interface/DecisionForest.h)
SET(forests_interface_templates include/grove/forests/interface/DecisionForest.tpp)

##
SET(forests_shared_headers include/grove/forests/shared/DecisionForest_Shared.h)

##
SET(keypoints_headers
include/grove/keypoints/Keypoint2D.h
include/grove/keypoints/Keypoint3DColour.h
)

##
SET(numbers_sources
src/numbers/CPURNG.cpp
)

SET(numbers_headers
include/grove/numbers/CPURNG.h
include/grove/numbers/CUDARNG.h
)

##
SET(ransac_sources src/ransac/PreemptiveRansacFactory.cpp)
SET(ransac_headers include/grove/ransac/PreemptiveRansacFactory.h)

##
SET(ransac_cpu_sources src/ransac/cpu/PreemptiveRansac_CPU.cpp)
SET(ransac_cpu_headers include/grove/ransac/cpu/PreemptiveRansac_CPU.h)

##
SET(ransac_cuda_sources src/ransac/cuda/PreemptiveRansac_CUDA.cu)
SET(ransac_cuda_headers include/grove/ransac/cuda/PreemptiveRansac_CUDA.h)

##
SET(ransac_interface_sources src/ransac/interface/PreemptiveRansac.cpp)
SET(ransac_interface_headers include/grove/ransac/interface/PreemptiveRansac.h)

##
SET(ransac_shared_headers
include/grove/ransac/shared/PoseCandidate.h
include/grove/ransac/shared/PreemptiveRansac_Shared.h
)

##
SET(relocalisation_sources src/relocalisation/ScoreRelocaliserFactory.cpp)
SET(relocalisation_headers include/grove/relocalisation/ScoreRelocaliserFactory.h)

##
SET(relocalisation_base_sources src/relocalisation/base/ScoreRelocaliserState.cpp)
SET(relocalisation_base_headers include/grove/relocalisation/base/ScoreRelocaliserState.h)

##
SET(relocalisation_cpu_sources
src/relocalisation/cpu/ScoreForestRelocaliser_CPU.cpp
src/relocalisation/cpu/ScoreGTRelocaliser_CPU.cpp
)

SET(relocalisation_cpu_headers
include/grove/relocalisation/cpu/ScoreForestRelocaliser_CPU.h
include/grove/relocalisation/cpu/ScoreGTRelocaliser_CPU.h
)

IF(WITH_TORCH)
  SET(relocalisation_cpu_sources ${relocalisation_cpu_sources} src/relocalisation/cpu/ScoreNetRelocaliser_CPU.cpp)
  SET(relocalisation_cpu_headers ${relocalisation_cpu_headers} include/grove/relocalisation/cpu/ScoreNetRelocaliser_CPU.h)
ENDIF()

##
SET(relocalisation_cuda_sources
src/relocalisation/cuda/ScoreForestRelocaliser_CUDA.cu
src/relocalisation/cuda/ScoreGTRelocaliser_CUDA.cu
)

SET(relocalisation_cuda_headers
include/grove/relocalisation/cuda/ScoreForestRelocaliser_CUDA.h
include/grove/relocalisation/cuda/ScoreGTRelocaliser_CUDA.h
)

IF(WITH_TORCH)
  SET(relocalisation_cuda_sources ${relocalisation_cuda_sources} src/relocalisation/cuda/ScoreNetRelocaliser_CUDA.cu)
  SET(relocalisation_cuda_headers ${relocalisation_cuda_headers} include/grove/relocalisation/cuda/ScoreNetRelocaliser_CUDA.h)
ENDIF()

##
SET(relocalisation_interface_sources
src/relocalisation/interface/ScoreForestRelocaliser.cpp
src/relocalisation/interface/ScoreGTRelocaliser.cpp
src/relocalisation/interface/ScoreRelocaliser.cpp
)

SET(relocalisation_interface_headers
include/grove/relocalisation/interface/ScoreForestRelocaliser.h
include/grove/relocalisation/interface/ScoreGTRelocaliser.h
include/grove/relocalisation/interface/ScoreRelocaliser.h
)

IF(WITH_TORCH)
  SET(relocalisation_interface_sources ${relocalisation_interface_sources} src/relocalisation/interface/ScoreNetRelocaliser.cpp)
  SET(relocalisation_interface_headers ${relocalisation_interface_headers} include/grove/relocalisation/interface/ScoreNetRelocaliser.h)
ENDIF()

##
SET(relocalisation_shared_headers
include/grove/relocalisation/shared/ScoreForestRelocaliser_Shared.h
include/grove/relocalisation/shared/ScoreGTRelocaliser_Shared.h
)

IF(WITH_TORCH)
  SET(relocalisation_shared_headers ${relocalisation_shared_headers} include/grove/relocalisation/shared/ScoreNetRelocaliser_Shared.h)
ENDIF()

##
SET(reservoirs_headers include/grove/reservoirs/ExampleReservoirsFactory.h)
SET(reservoirs_templates include/grove/reservoirs/ExampleReservoirsFactory.tpp)

##
SET(reservoirs_cpu_headers include/grove/reservoirs/cpu/ExampleReservoirs_CPU.h)
SET(reservoirs_cpu_templates include/grove/reservoirs/cpu/ExampleReservoirs_CPU.tpp)

##
SET(reservoirs_cuda_headers include/grove/reservoirs/cuda/ExampleReservoirs_CUDA.h)
SET(reservoirs_cuda_templates include/grove/reservoirs/cuda/ExampleReservoirs_CUDA.tcu)

##
SET(reservoirs_interface_headers include/grove/reservoirs/interface/ExampleReservoirs.h)
SET(reservoirs_interface_templates include/grove/reservoirs/interface/ExampleReservoirs.tpp)

##
SET(reservoirs_shared_headers include/grove/reservoirs/shared/ExampleReservoirs_Shared.h)

##
SET(scoreforests_headers
include/grove/scoreforests/Keypoint3DColourCluster.h
include/grove/scoreforests/ScorePrediction.h
)

##
SET(util_headers include/grove/util/Array.h)

#################################################################
# Collect the project files into sources, headers and templates #
#################################################################

SET(sources
${features_sources}
${numbers_sources}
${ransac_sources}
${ransac_cpu_sources}
${ransac_interface_sources}
${relocalisation_sources}
${relocalisation_base_sources}
${relocalisation_cpu_sources}
${relocalisation_interface_sources}
${toplevel_sources}
)

SET(headers
${clustering_headers}
${clustering_cpu_headers}
${clustering_interface_headers}
${clustering_shared_headers}
${features_headers}
${features_base_headers}
${features_cpu_headers}
${features_interface_headers}
${features_shared_headers}
${forests_headers}
${forests_cpu_headers}
${forests_interface_headers}
${forests_shared_headers}
${keypoints_headers}
${numbers_headers}
${ransac_headers}
${ransac_cpu_headers}
${ransac_interface_headers}
${ransac_shared_headers}
${relocalisation_headers}
${relocalisation_base_headers}
${relocalisation_cpu_headers}
${relocalisation_interface_headers}
${relocalisation_shared_headers}
${reservoirs_headers}
${reservoirs_cpu_headers}
${reservoirs_interface_headers}
${reservoirs_shared_headers}
${scoreforests_headers}
${util_headers}
)

SET(templates
${clustering_templates}
${clustering_cpu_templates}
${clustering_interface_templates}
${features_templates}
${features_cpu_templates}
${features_interface_templates}
${forests_templates}
${forests_cpu_templates}
${forests_interface_templates}
${reservoirs_templates}
${reservoirs_cpu_templates}
${reservoirs_interface_templates}
)

IF(WITH_CUDA)
  SET(sources ${sources}
    ${ransac_cuda_sources}
    ${relocalisation_cuda_sources}
  )

  SET(headers ${headers}
    ${clustering_cuda_headers}
    ${features_cuda_headers}
    ${forests_cuda_headers}
    ${ransac_cuda_headers}
    ${relocalisation_cuda_headers}
    ${reservoirs_cuda_headers}
  )

  SET(templates ${templates}
    ${clustering_cuda_templates}
    ${features_cuda_templates}
    ${forests_cuda_templates}
    ${reservoirs_cuda_templates}
  )
ENDIF()

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${toplevel_sources})
SOURCE_GROUP(clustering FILES ${clustering_headers} ${clustering_templates})
SOURCE_GROUP(clustering\\cpu FILES ${clustering_cpu_headers} ${clustering_cpu_templates})
SOURCE_GROUP(clustering\\cuda FILES ${clustering_cuda_headers} ${clustering_cuda_templates})
SOURCE_GROUP(clustering\\interface FILES ${clustering_interface_headers} ${clustering_interface_templates})
SOURCE_GROUP(clustering\\shared FILES ${clustering_shared_headers})
SOURCE_GROUP(features FILES ${features_sources} ${features_headers} ${features_templates})
SOURCE_GROUP(features\\base FILES ${features_base_headers})
SOURCE_GROUP(features\\cpu FILES ${features_cpu_headers} ${features_cpu_templates})
SOURCE_GROUP(features\\cuda FILES ${features_cuda_headers} ${features_cuda_templates})
SOURCE_GROUP(features\\interface FILES ${features_interface_headers} ${features_interface_templates})
SOURCE_GROUP(features\\shared FILES ${features_shared_headers})
SOURCE_GROUP(forests FILES ${forests_headers} ${forests_templates})
SOURCE_GROUP(forests\\cpu FILES ${forests_cpu_headers} ${forests_cpu_templates})
SOURCE_GROUP(forests\\cuda FILES ${forests_cuda_headers} ${forests_cuda_templates})
SOURCE_GROUP(forests\\interface FILES ${forests_interface_headers} ${forests_interface_templates})
SOURCE_GROUP(forests\\shared FILES ${forests_shared_headers})
SOURCE_GROUP(keypoints FILES ${keypoints_headers})
SOURCE_GROUP(numbers FILES ${numbers_sources} ${numbers_headers})
SOURCE_GROUP(ransac FILES ${ransac_sources} ${ransac_headers})
SOURCE_GROUP(ransac\\cpu FILES ${ransac_cpu_sources} ${ransac_cpu_headers})
SOURCE_GROUP(ransac\\cuda FILES ${ransac_cuda_sources} ${ransac_cuda_headers})
SOURCE_GROUP(ransac\\interface FILES ${ransac_interface_sources} ${ransac_interface_headers})
SOURCE_GROUP(ransac\\shared FILES ${ransac_shared_headers})
SOURCE_GROUP(relocalisation FILES ${relocalisation_sources} ${relocalisation_headers})
SOURCE_GROUP(relocalisation\\base FILES ${relocalisation_base_sources} ${relocalisation_base_headers})
SOURCE_GROUP(relocalisation\\cpu FILES ${relocalisation_cpu_sources} ${relocalisation_cpu_headers})
SOURCE_GROUP(relocalisation\\cuda FILES ${relocalisation_cuda_sources} ${relocalisation_cuda_headers})
SOURCE_GROUP(relocalisation\\interface FILES ${relocalisation_interface_sources} ${relocalisation_interface_headers})
SOURCE_GROUP(relocalisation\\shared FILES ${relocalisation_shared_headers})
SOURCE_GROUP(reservoirs FILES ${reservoirs_sources} ${reservoirs_headers} ${reservoirs_templates})
SOURCE_GROUP(reservoirs\\cpu FILES ${reservoirs_cpu_headers} ${reservoirs_cpu_templates})
SOURCE_GROUP(reservoirs\\cuda FILES ${reservoirs_cuda_headers} ${reservoirs_cuda_templates})
SOURCE_GROUP(reservoirs\\interface FILES ${reservoirs_interface_headers} ${reservoirs_interface_templates})
SOURCE_GROUP(reservoirs\\shared FILES ${reservoirs_shared_headers})
SOURCE_GROUP(scoreforests FILES ${scoreforests_headers})
SOURCE_GROUP(util FILES ${util_headers})

##########################################
# Specify additional include directories #
##########################################

INCLUDE_DIRECTORIES(include/grove)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/helpers)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/orx/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/modules/tvgutil/include)

##########################################
# Specify the target and where to put it #
##########################################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/SetCUDALibTarget.cmake)

#############################
# Specify things to install #
#############################

INSTALL(TARGETS ${targetname} DESTINATION lib)
