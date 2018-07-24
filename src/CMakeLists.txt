# RBDL
ADD_EXECUTABLE(rbdl-bench rbdl-bench)
PKG_CONFIG_USE_DEPENDENCY(rbdl-bench rbdl)
TARGET_LINK_LIBRARIES(rbdl-bench rbdl_urdfreader)
INSTALL(TARGETS rbdl-bench RUNTIME DESTINATION bin)

# Pinocchio
ADD_EXECUTABLE(pinocchio-bench pinocchio-bench)
PKG_CONFIG_USE_DEPENDENCY(pinocchio-bench pinocchio)
INSTALL(TARGETS pinocchio-bench RUNTIME DESTINATION bin)

# RBDL & benchmark
ADD_EXECUTABLE(rbdl-benchmark rbdl-benchmark)
PKG_CONFIG_USE_DEPENDENCY(rbdl-benchmark rbdl)
PKG_CONFIG_USE_DEPENDENCY(rbdl-benchmark benchmark)
TARGET_LINK_LIBRARIES(rbdl-benchmark rbdl_urdfreader pthread)
INSTALL(TARGETS rbdl-benchmark RUNTIME DESTINATION bin)

# Pinocchio & benchmark
ADD_EXECUTABLE(pinocchio-benchmark pinocchio-benchmark)
PKG_CONFIG_USE_DEPENDENCY(pinocchio-benchmark pinocchio)
PKG_CONFIG_USE_DEPENDENCY(pinocchio-benchmark benchmark)
TARGET_LINK_LIBRARIES(pinocchio-benchmark pthread)
INSTALL(TARGETS pinocchio-benchmark RUNTIME DESTINATION bin)
