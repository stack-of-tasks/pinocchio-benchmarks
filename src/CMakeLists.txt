# RBDL
ADD_EXECUTABLE(rbdl-bench rbdl-bench)
PKG_CONFIG_USE_DEPENDENCY(rbdl-bench rbdl)
TARGET_LINK_LIBRARIES(rbdl-bench rbdl_urdfreader)
INSTALL(TARGETS rbdl-bench RUNTIME DESTINATION bin)

# Pinocchio
ADD_EXECUTABLE(pinocchio-bench pinocchio-bench)
PKG_CONFIG_USE_DEPENDENCY(pinocchio-bench pinocchio)
INSTALL(TARGETS pinocchio-bench RUNTIME DESTINATION bin)
