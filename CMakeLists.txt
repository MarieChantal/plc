set(source_files
    model/plc-thread-dispatcher.cc
    model/plc-simulator-impl.cc
    model/plc-defs.cc
    model/plc-time.cc
    model/plc-value.cc
    model/plc-cable.cc
    model/plc-edge.cc
    model/plc-node.cc
    model/plc-graph.cc
    model/plc-channel.cc
    model/plc-backbone.cc
    model/plc-outlet.cc
    model/plc-interface.cc
    model/plc-noise.cc
    model/plc-dcmc-capacity.cc
    model/plc-link-performance-model.cc
    model/plc-interference.cc
    model/plc-header.cc
    model/plc-phy.cc
    model/plc-mac.cc
    model/plc-net-device.cc
    helper/plc-helper.cc
    helper/plc-spectrum-helper.cc
    helper/plc-device-helper.cc
    helper/plc-output-helper.cc
    helper/plc-trace-helper.cc
)

set(header_files
    model/plc-thread-dispatcher.h
    model/plc-simulator-impl.h
    model/plc.h
    model/plc-defs.h
    model/plc-time.h
    model/plc-value.h
    model/plc-cable.h
    model/plc-edge.h
    model/plc-node.h
    model/plc-graph.h
    model/plc-visitor.h
    model/plc-channel.h
    model/plc-backbone.h
    model/plc-outlet.h
    model/plc-interface.h
    model/plc-undirected-dfs.h
    model/plc-noise.h
    model/plc-dcmc-capacity.h
    model/plc-link-performance-model.h
    model/plc-interference.h
    model/plc-header.h
    model/plc-phy.h
    model/plc-mac.h
    model/plc-net-device.h
    helper/plc-spectrum-helper.h
    helper/plc-helper.h
    helper/plc-device-helper.h
    helper/plc-output-helper.h
    helper/plc-trace-helper.h
)

find_package(OpenMP REQUIRED)

build_lib(
  LIBNAME plc
  SOURCE_FILES ${source_files}
  HEADER_FILES ${header_files}
  LIBRARIES_TO_LINK
    ${libcore}
    ${libspectrum}
    ${libnetwork}
    ${OpenMP_CXX_LIBRARIES}
  TEST_SOURCES
    test/plc-test-suite.cc
)
