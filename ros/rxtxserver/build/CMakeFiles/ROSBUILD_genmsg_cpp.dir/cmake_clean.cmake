FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/rxtxserver/msg"
  "../src/rxtxserver/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/rxtxserver/AngularPos.h"
  "../msg_gen/cpp/include/rxtxserver/status.h"
  "../msg_gen/cpp/include/rxtxserver/distance.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
