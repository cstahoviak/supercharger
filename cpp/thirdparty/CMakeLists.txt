cmake_minimum_required(VERSION 3.10.2)

include(FetchContent)

# TODO: Look into ccache for handling larger builds.

# Add pybind11
FetchContent_Declare(
  pybind11
  URL https://github.com/pybind/pybind11/archive/refs/tags/v2.13.6.tar.gz
)
FetchContent_MakeAvailable(pybind11)

# Add NLOpt
FetchContent_Declare(
  NLopt
  URL https://github.com/stevengj/nlopt/archive/refs/tags/v2.8.0.tar.gz
)
FetchContent_MakeAvailable(NLopt)

# Add googletest
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
)
FetchContent_MakeAvailable(googletest)

# TODO: Add glog
# FetchContent_Declare(
#   glog
#   URL https://github.com/google/glog/archive/refs/tags/v0.7.1.tar.gz
# )
# FetchContent_MakeAvailable(glog)