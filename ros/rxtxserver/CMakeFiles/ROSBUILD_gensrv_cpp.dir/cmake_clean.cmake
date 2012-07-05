FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/rxtxserver/msg"
  "src/rxtxserver/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "srv_gen/cpp/include/rxtxserver/Read.h"
  "srv_gen/cpp/include/rxtxserver/Byte.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
