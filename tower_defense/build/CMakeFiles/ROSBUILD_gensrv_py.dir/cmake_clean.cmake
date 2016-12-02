file(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/tower_defense/msg"
  "../src/tower_defense/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/tower_defense/srv/__init__.py"
  "../src/tower_defense/srv/_RandomConfigSrv.py"
  "../src/tower_defense/srv/_BuildRRTSrv.py"
  "../src/tower_defense/srv/_ExtendNodeSrv.py"
  "../src/tower_defense/srv/_RRTPlanSrv.py"
  "../src/tower_defense/srv/_CheckExtensionSrv.py"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
