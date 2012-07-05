FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/rxtxserver/msg"
  "../src/rxtxserver/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/rxtxserver/srv/__init__.py"
  "../src/rxtxserver/srv/_Read.py"
  "../src/rxtxserver/srv/_Byte.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
