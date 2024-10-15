cmake_minimum_required(VERSION 3.10.2)

include(FetchContent)

# Add NLOpt
# TODO: Adding NLOpt like this creates an error with "#include <array>". I'm not
# sure why this is happening, so I'll have to build install NLOpt from source
# instead.
# include(FetchContent)
# FetchContent_Declare(
#   NLopt
#   URL https://github.com/stevengj/nlopt/archive/refs/tags/v2.8.0.tar.gz
# )
# FetchContent_MakeAvailable(NLopt)

# Add googletest
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
)
FetchContent_MakeAvailable(googletest)

# Add glog
FetchContent_Declare(
  glog
  URL https://github.com/google/glog/archive/refs/tags/v0.7.1.tar.gz
)
FetchContent_MakeAvailable(glog)