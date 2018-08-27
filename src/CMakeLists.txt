CONFIG_FILES(models.h)
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}")

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
#ADD_EXECUTABLE(kdl-test kdl-test)
#PKG_CONFIG_USE_DEPENDENCY(kdl-test kdl_parser)
#INSTALL(TARGETS kdl-test RUNTIME DESTINATION bin)

# Google Benchmarks

# RBDL
ADD_EXECUTABLE(rbdl-benchmark rbdl-benchmark)
PKG_CONFIG_USE_DEPENDENCY(rbdl-benchmark rbdl)
PKG_CONFIG_USE_DEPENDENCY(rbdl-benchmark benchmark)
TARGET_LINK_LIBRARIES(rbdl-benchmark rbdl_urdfreader pthread)
INSTALL(TARGETS rbdl-benchmark RUNTIME DESTINATION bin)

# Pinocchio
ADD_EXECUTABLE(pinocchio-benchmark pinocchio-benchmark)
PKG_CONFIG_USE_DEPENDENCY(pinocchio-benchmark pinocchio)
PKG_CONFIG_USE_DEPENDENCY(pinocchio-benchmark benchmark)
TARGET_LINK_LIBRARIES(pinocchio-benchmark pthread)
INSTALL(TARGETS pinocchio-benchmark RUNTIME DESTINATION bin)

# StackTicToc Benchmarks

# RBDL
ADD_EXECUTABLE(benchmarks-rbdl benchmarks-rbdl)
PKG_CONFIG_USE_DEPENDENCY(benchmarks-rbdl rbdl)
TARGET_LINK_LIBRARIES(benchmarks-rbdl rbdl_urdfreader)
INSTALL(TARGETS benchmarks-rbdl RUNTIME DESTINATION bin)

# Pinocchio
ADD_EXECUTABLE(benchmarks-pinocchio benchmarks-pinocchio)
PKG_CONFIG_USE_DEPENDENCY(benchmarks-pinocchio pinocchio)
INSTALL(TARGETS benchmarks-pinocchio RUNTIME DESTINATION bin)
