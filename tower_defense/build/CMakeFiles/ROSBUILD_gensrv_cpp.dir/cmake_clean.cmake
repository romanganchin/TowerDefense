file(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/tower_defense/msg"
  "../src/tower_defense/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/tower_defense/RandomConfigSrv.h"
  "../srv_gen/cpp/include/tower_defense/BuildRRTSrv.h"
  "../srv_gen/cpp/include/tower_defense/ExtendNodeSrv.h"
  "../srv_gen/cpp/include/tower_defense/RRTPlanSrv.h"
  "../srv_gen/cpp/include/tower_defense/CheckExtensionSrv.h"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
