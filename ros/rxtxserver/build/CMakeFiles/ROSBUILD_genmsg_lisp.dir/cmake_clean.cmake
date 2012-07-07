FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/rxtxserver/msg"
  "../src/rxtxserver/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/AngularPos.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_AngularPos.lisp"
  "../msg_gen/lisp/status.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_status.lisp"
  "../msg_gen/lisp/distance.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_distance.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
