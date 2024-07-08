build_lib_example(
  NAME plc-mac-example
  SOURCE_FILES plc-mac-example.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libspectrum}
    ${libnetwork}
    ${libplc}
)

build_lib_example(
  NAME plc-netdevice-example
  SOURCE_FILES plc-netdevice-example.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libspectrum}
    ${libnetwork}
    ${libplc}
)

build_lib_example(
  NAME plc-phy-example
  SOURCE_FILES plc-phy-example.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libspectrum}
    ${libnetwork}
    ${libplc}
)

build_lib_example(
  NAME plc-psd-example
  SOURCE_FILES plc-psd-example.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libspectrum}
    ${libnetwork}
    ${libplc}
)

build_lib_example(
  NAME plc-test-ami
  SOURCE_FILES plc-test-ami.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libspectrum}
    ${libnetwork}
    ${libplc}
)

build_lib_example(
  NAME plc-test
  SOURCE_FILES plc-test.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libspectrum}
    ${libnetwork}
    ${libplc}
)

build_lib_example(
  NAME plc-2Devices-IPv4-SG-OnOff
  SOURCE_FILES plc-2Devices-IPv4-SG-OnOff.cc
  LIBRARIES_TO_LINK
    ${libcore}
    ${libspectrum}
    ${libnetwork}
    ${libplc}
    ${libflow-monitor}
    ${libapplications}
)