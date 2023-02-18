build_lib(
  LIBNAME thz
    SOURCE_FILES  helper/thz-mac-macro-helper.cc
                helper/traffic-generator-helper.cc
                helper/thz-energy-model-helper.cc
                helper/thz-mac-macro-client-helper.cc
                helper/thz-directional-antenna-helper.cc
                helper/thz-udp-client-server-helper.cc
                helper/thz-phy-nano-helper.cc
                helper/thz-mac-nano-helper.cc
                helper/thz-phy-macro-helper.cc
                helper/thz-helper.cc
                helper/thz-mac-macro-ap-helper.cc
                model/thz-spectrum-waveform.cc
                model/thz-phy-macro.cc
                model/thz-spectrum-propagation-loss.cc
                model/thz-phy-nano.cc
                model/thz-mac-nano.cc
                model/thz-udp-server.cc
                model/traffic-generator.cc
                model/thz-net-device.cc
                model/thz-dir-antenna.cc
                model/thz-channel.cc
                model/thz-udp-client.cc
                model/thz-mac-header.cc
                model/thz-spectrum-signal-parameters.cc
                model/thz-mac-macro.cc
                model/thz-udp-trace-client.cc
                model/thz-energy-model.cc
                model/thz-mac-macro-client.cc
                model/thz-mac-macro-ap.cc

  HEADER_FILES  helper/thz-phy-nano-helper.h
                helper/thz-mac-macro-ap-helper.h
                helper/thz-mac-macro-client-helper.h
                helper/thz-helper.h
                helper/thz-mac-macro-helper.h
                helper/thz-mac-nano-helper.h
                helper/traffic-generator-helper.h
                helper/thz-phy-macro-helper.h
                helper/thz-directional-antenna-helper.h
                helper/thz-energy-model-helper.h
                helper/thz-udp-client-server-helper.h
                model/thz-phy.h
                model/thz-mac-macro-client.h
                model/thz-net-device.h
                model/thz-phy-macro.h
                model/thz-energy-model.h
                model/thz-channel.h
                model/traffic-generator.h
                model/thz-udp-client.h
                model/thz-mac-macro.h
                model/thz-dir-antenna.h
                model/thz-mac-nano.h
                model/thz-udp-trace-client.h
                model/thz-udp-server.h
                model/thz-phy-nano.h
                model/thz-mac.h
                model/thz-mac-header.h
                model/thz-spectrum-signal-parameters.h
                model/thz-mac-macro-ap.h
                model/thz-spectrum-propagation-loss.h
                model/thz-spectrum-waveform.h

  LIBRARIES_TO_LINK ${libcore}
                ${libinternet}
                ${libmobility}
                ${libantenna}
                ${libpropagation}
                ${libspectrum}
                ${libnetwork}
                ${libapplications}

  TEST_SOURCES  test/test-thz-directional-antenna.cc
                test/test-thz-path-loss.cc
                test/test-thz-psd-macro.cc
                test/test-thz-psd-nano.cc
)

