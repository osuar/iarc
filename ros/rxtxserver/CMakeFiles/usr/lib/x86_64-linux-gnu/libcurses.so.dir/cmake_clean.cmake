FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/rxtxserver/msg"
  "src/rxtxserver/srv"
  "msg_gen"
  "srv_gen"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles//usr/lib/x86_64-linux-gnu/libcurses.so.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
