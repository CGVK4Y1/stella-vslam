include(CMakeFindDependencyMacro)

find_dependency(Pangolin)

include("${CMAKE_CURRENT_LIST_DIR}/pangolin_viewerTargets.cmake")
