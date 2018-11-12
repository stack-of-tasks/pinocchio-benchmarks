CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

SET (CXX_DISABLE_WERROR ON)

INCLUDE(cmake/base.cmake)

SET(PROJECT_NAME pinocchio-benchmarks)
SET(PROJECT_DESCRIPTION "Benchmarks of the Pinocchio library, against RBDL & KDL")

SET(PROJECT_URL https://github.com/stack-of-tasks/${PROJECT_NAME})

SET(DOXYGEN_USE_MATHJAX YES)

SETUP_PROJECT()

ADD_REQUIRED_DEPENDENCY("eigen3")
ADD_REQUIRED_DEPENDENCY("pinocchio")
ADD_REQUIRED_DEPENDENCY("rbdl")
#ADD_REQUIRED_DEPENDENCY("orocos-kdl")
#ADD_REQUIRED_DEPENDENCY("kdl_parser")
ADD_REQUIRED_DEPENDENCY("urdfdom")
#ADD_REQUIRED_DEPENDENCY("benchmark")
#ADD_REQUIRED_DEPENDENCY("metapod")

SET(MODELS
  simple
  romeo
  nao
  poppy
  lwr
  atlas
  hyq
  talos
  tiago
  )

LIST(LENGTH MODELS MODELS_LEN)
STRING(REPLACE ";" "\", \"" MODELS_CXX "${MODELS}")
STRING(REPLACE ";" ".urdf;models/" MODEL_FILES "${MODELS}")
SET(MODEL_FILES "models/${MODEL_FILES}.urdf")

INSTALL(FILES ${MODEL_FILES}
  DESTINATION "share/pinocchio-benchmarks/models"
  )

ADD_SUBDIRECTORY("src")

SETUP_PROJECT_FINALIZE()
