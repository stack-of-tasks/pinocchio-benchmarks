CONFIG_FILES(models.h models.jl)
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}")
#SET(METAPOD_MODELS_DIR "${CMAKE_INSTALL_PREFIX}/include/pinocchio-benchmarks")
INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}/models.jl" DESTINATION bin)

# generate models for metapod from urdf files
#ADD_CUSTOM_TARGET(mk_metapod_models_dir ALL COMMAND ${CMAKE_COMMAND} -E make_directory ${METAPOD_MODELS_DIR})
#FOREACH(model ${MODELS})
  #EXECUTE_PROCESS(COMMAND
    #"${CMAKE_INSTALL_PREFIX}/bin/metapodfromurdf" --name ${model} --directory "${METAPOD_MODELS_DIR}/metapod_${model}"
    #"${CMAKE_SOURCE_DIR}/models/${model}.urdf"
    #)
#ENDFOREACH(model ${MODELS})

# Tests

# RBDL
ADD_EXECUTABLE(rbdl-test rbdl-test)
PKG_CONFIG_USE_DEPENDENCY(rbdl-test rbdl)
TARGET_LINK_LIBRARIES(rbdl-test rbdl_urdfreader)
INSTALL(TARGETS rbdl-test RUNTIME DESTINATION bin)

# Pinocchio
ADD_EXECUTABLE(pinocchio-test pinocchio-test)
PKG_CONFIG_USE_DEPENDENCY(pinocchio-test pinocchio)
INSTALL(TARGETS pinocchio-test RUNTIME DESTINATION bin)

# KDL test
ADD_EXECUTABLE(kdl-test kdl-test)
PKG_CONFIG_USE_DEPENDENCY(kdl-test orocos-kdl)
PKG_CONFIG_USE_DEPENDENCY(kdl-test kdl_parser)
INSTALL(TARGETS kdl-test RUNTIME DESTINATION bin)

# metapod
#ADD_EXECUTABLE(metapod-test metapod-test)
#PKG_CONFIG_USE_DEPENDENCY(metapod-test metapod)
#INSTALL(TARGETS metapod-test RUNTIME DESTINATION bin)

# JuliaRobotics
INSTALL(FILES julia-test.jl DESTINATION bin
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

# Google Benchmarks

# RBDL
#ADD_EXECUTABLE(rbdl-benchmark rbdl-benchmark)
#PKG_CONFIG_USE_DEPENDENCY(rbdl-benchmark rbdl)
#PKG_CONFIG_USE_DEPENDENCY(rbdl-benchmark benchmark)
#TARGET_LINK_LIBRARIES(rbdl-benchmark rbdl_urdfreader pthread)
#INSTALL(TARGETS rbdl-benchmark RUNTIME DESTINATION bin)

## Pinocchio
#ADD_EXECUTABLE(pinocchio-benchmark pinocchio-benchmark)
#PKG_CONFIG_USE_DEPENDENCY(pinocchio-benchmark pinocchio)
#PKG_CONFIG_USE_DEPENDENCY(pinocchio-benchmark benchmark)
#TARGET_LINK_LIBRARIES(pinocchio-benchmark pthread)
#INSTALL(TARGETS pinocchio-benchmark RUNTIME DESTINATION bin)

# Benchmarks

# RBDL
ADD_EXECUTABLE(benchmarks-rbdl benchmarks-rbdl)
PKG_CONFIG_USE_DEPENDENCY(benchmarks-rbdl rbdl)
TARGET_LINK_LIBRARIES(benchmarks-rbdl rbdl_urdfreader)
INSTALL(TARGETS benchmarks-rbdl RUNTIME DESTINATION bin)

# Pinocchio
ADD_EXECUTABLE(benchmarks-pinocchio benchmarks-pinocchio)
PKG_CONFIG_USE_DEPENDENCY(benchmarks-pinocchio pinocchio)
INSTALL(TARGETS benchmarks-pinocchio RUNTIME DESTINATION bin)

# KDL
ADD_EXECUTABLE(benchmarks-kdl benchmarks-kdl)
PKG_CONFIG_USE_DEPENDENCY(benchmarks-kdl orocos-kdl)
PKG_CONFIG_USE_DEPENDENCY(benchmarks-kdl kdl_parser)
INSTALL(TARGETS benchmarks-kdl RUNTIME DESTINATION bin)

# JuliaRobotics
INSTALL(FILES benchmarks-julia.jl DESTINATION bin
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)


