FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/rxtxserver/msg"
  "../src/rxtxserver/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/rxtxserver/msg/__init__.py"
  "../src/rxtxserver/msg/_AngularPos.py"
  "../src/rxtxserver/msg/_status.py"
  "../src/rxtxserver/msg/_distance.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
