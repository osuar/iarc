FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/rxtxserver/msg"
  "../src/rxtxserver/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/Read.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_Read.lisp"
  "../srv_gen/lisp/Byte.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_Byte.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
