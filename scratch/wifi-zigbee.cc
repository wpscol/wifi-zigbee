#include "ns3/applications-module.h"
#include "ns3/arp-cache.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/log.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/sixlowpan-helper.h"
#include "ns3/spectrum-module.h"
#include "ns3/wifi-module.h"
#include "ns3/zigbee-module.h"

#include <chrono>
#include <iomanip>

using namespace ns3;

void ReportWifiStats(Ptr<FlowMonitor> monitor, Ptr<Ipv4FlowClassifier> classifier, uint16_t wifiPort = 5000);
void ReportZigbeeStats(Ptr<FlowMonitor> monitor, Ptr<Ipv4FlowClassifier> classifier, uint16_t zigbeePort);

NS_LOG_COMPONENT_DEFINE("wifi-zigbee-coex");

int main(int argc, char* argv[]) {
  // Log configuration
  LogComponentEnable("wifi-zigbee-coex", LOG_LEVEL_INFO);

  // Simulation configuration
  double simulationTime = 30;
  double warmupTime = 1.5;

  // WiFi configuration
  std::string wifiStandard = "80211n";
  std::string wifiDataRate = "65Mbps";
  uint32_t wifiPacketSize = 1472;
  uint16_t wifiPort = 5000;

  // ZigBee configuration
  uint32_t nZigbee = 5;
  uint16_t zigbeePort = 9;
  std::string zigbeeDataRate = "2048kbps";
  uint32_t zigbeePacketSize = 100;

  // Get configuration
  CommandLine cmd;
  cmd.AddValue("wifiStandard", "Choose standard to use: 80211n | 80211ac | 80211ax ", wifiStandard);
  cmd.AddValue("wifiDataRate", "Data rate for Wifi devices", wifiDataRate);
  cmd.AddValue("wifiPacketSize", "Wifi packet size", wifiPacketSize);
  cmd.AddValue("nZigbee", "Number of ZigBee devices", nZigbee);
  cmd.AddValue("simulationTime", "How long simulation should run (s)", simulationTime);
  cmd.AddValue("warmupTime", "How many seconds to run as warmup (before measurements)", simulationTime);
  cmd.AddValue("zigbeeDataRate", "Data rate for Zigbee client", zigbeeDataRate);
  cmd.AddValue("zigbeePacketSize", "Packet size for Zigbee client", zigbeePacketSize);
  cmd.Parse(argc, argv);

  // Print configuration
  NS_LOG_INFO("wifi-zigbee-coex - configuration:");
  NS_LOG_INFO("> wifiStandard: " << wifiStandard);
  NS_LOG_INFO("> wifiDataRate: " << wifiDataRate);
  NS_LOG_INFO("> nZigbee: " << nZigbee);
  NS_LOG_INFO("> simulationTime: " << simulationTime);
  NS_LOG_INFO("> warmupTime: " << warmupTime);
  NS_LOG_INFO("> zigbeeDataRate: " << zigbeeDataRate);
  NS_LOG_INFO("> zigbeePacketSize: " << zigbeePacketSize);

  Ptr<SpectrumChannel> commonChannel = CreateObject<MultiModelSpectrumChannel>();
  commonChannel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
  commonChannel->AddPropagationLossModel(CreateObject<LogDistancePropagationLossModel>());

  // Single AP
  NodeContainer ap;
  ap.Create(1);

  // Two stations
  NodeContainer sta;
  sta.Create(2);

  // Zigbee devices
  NodeContainer zigbee;
  zigbee.Create(nZigbee);

  // Configure mobility
  MobilityHelper mob;
  mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mob.Install(ap);
  mob.Install(sta);
  mob.Install(zigbee);

  // Configure positions
  Ptr<ListPositionAllocator> apPos = CreateObject<ListPositionAllocator>();
  Ptr<ListPositionAllocator> staPos = CreateObject<ListPositionAllocator>();
  Ptr<ListPositionAllocator> zbPos = CreateObject<ListPositionAllocator>();

  // AP
  apPos->Add(Vector(0.0, 0.0, 1.5));

  // Stations
  staPos->Add(Vector(5.0, 0.0, 1.2));
  staPos->Add(Vector(-5.0, 0.0, 1.2));

  // ZigBee (on the line of the circle)
  for (uint32_t i = 0; i < nZigbee; ++i) {
    double angle = 2 * M_PI * i / nZigbee;
    double x = 10.0 * std::cos(angle);
    double y = 10.0 * std::sin(angle);
    zbPos->Add(Vector(x, y, 1.0));
  }

  mob.SetPositionAllocator(apPos);
  mob.Install(ap);

  mob.SetPositionAllocator(staPos);
  mob.Install(sta);

  mob.SetPositionAllocator(zbPos);
  mob.Install(zigbee);

  // Configure channel
  SpectrumWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel(commonChannel);
  // wifiPhy.Set("Frequency", UintegerValue(2412));

  // Set WiFi standard
  WifiHelper wifi;
  if (wifiStandard == "80211n") {
    wifi.SetStandard(WIFI_STANDARD_80211n);
  } else if (wifiStandard == "80211ac") {
    wifi.SetStandard(WIFI_STANDARD_80211ac);
  } else if (wifiStandard == "80211ax") {
    wifi.SetStandard(WIFI_STANDARD_80211ax);
  }

  // wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue("HtMcs7"), "ControlMode",
  //                              StringValue("HtMcs0"));

  wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");

  // Wifi configuration
  WifiMacHelper mac;
  Ssid ssid = Ssid("coex");

  mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
  NetDeviceContainer staDev = wifi.Install(wifiPhy, mac, sta);

  mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
  NetDeviceContainer apDev = wifi.Install(wifiPhy, mac, ap);

  // LR-WPAN configuration
  LrWpanHelper lrWpan;
  lrWpan.SetChannel(commonChannel);

  NetDeviceContainer zbDev = lrWpan.Install(zigbee);
  lrWpan.CreateAssociatedPan(zbDev, 0x1234);

  // Install Zigbee
  ZigbeeHelper zbHelper;
  zigbee::ZigbeeStackContainer zbStacks = zbHelper.Install(zbDev);

  // Setup coordinator
  Ptr<zigbee::ZigbeeStack> coord = zbStacks.Get(0);
  zigbee::NlmeNetworkFormationRequestParams form;
  form.m_scanDuration = 3;
  form.m_beaconOrder = 15;
  form.m_superFrameOrder = 15;

  const uint32_t ALL_CHANNELS = 0x07FFF800;
  form.m_scanChannelList.channelPageCount = 1;
  form.m_scanChannelList.channelsField.resize(1);
  form.m_scanChannelList.channelsField[0] = ALL_CHANNELS;

  Simulator::ScheduleWithContext(coord->GetNode()->GetId(), Seconds(0), &zigbee::ZigbeeNwk::NlmeNetworkFormationRequest,
                                 coord->GetNwk(), form);

  // Join Zigbee routers
  for (uint32_t i = 1; i < zbStacks.GetN(); ++i) {
    Ptr<zigbee::ZigbeeStack> st = zbStacks.Get(i);

    zigbee::CapabilityInformation ci;
    ci.SetDeviceType(zigbee::ROUTER);
    ci.SetAllocateAddrOn(true);

    zigbee::NlmeJoinRequestParams j;
    j.m_rejoinNetwork = zigbee::ASSOCIATION;
    j.m_capabilityInfo = ci.GetCapability();
    j.m_extendedPanId = 0;

    Simulator::Schedule(Seconds(0.5 * i), &zigbee::ZigbeeNwk::NlmeJoinRequest, st->GetNwk(), j);
  }

  // IP configuration
  InternetStackHelper inet;
  inet.Install(ap);
  inet.Install(sta);
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.0.0.0", "255.255.255.0");
  ipv4.Assign(NetDeviceContainer(apDev, staDev));

  // IP configuration Zigbee
  InternetStackHelper zigbeeInet;
  inet.Install(zigbee);

  // Install IPv4 Stack on 802.15.4
  SixLowPanHelper sixlowpan;
  NetDeviceContainer sixLowPanDevices = sixlowpan.Install(zbDev);

  // Assign IPv4 to ZigBee Zibhee
  Ipv4AddressHelper ipv4Zb;
  ipv4Zb.SetBase("10.255.255.0", "255.255.255.0");
  Ipv4InterfaceContainer zbIfaces = ipv4Zb.Assign(sixLowPanDevices);

  // Configure WiFi application
  PacketSinkHelper wifiSink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), wifiPort));
  ApplicationContainer wifiSinkApp = wifiSink.Install(ap.Get(0));
  wifiSinkApp.Start(Seconds(warmupTime));
  wifiSinkApp.Stop(Seconds(warmupTime + simulationTime));

  ApplicationContainer wifiTrafficApps;
  for (NodeContainer::Iterator it = sta.Begin(); it != sta.End(); ++it) {
    Address dst(InetSocketAddress(Ipv4Address("10.0.0.1"), wifiPort));
    OnOffHelper onoff("ns3::UdpSocketFactory", dst);
    onoff.SetConstantRate(DataRate(wifiDataRate), wifiPacketSize);

    ApplicationContainer app = onoff.Install(*it);
    app.Start(Seconds(warmupTime));
    app.Stop(Seconds(warmupTime + simulationTime));
    wifiTrafficApps.Add(app);
  }

  PacketSinkHelper zigbeeSinkHelper("ns3::UdpSocketFactory",
                                    ns3::Address(ns3::InetSocketAddress(ns3::Ipv4Address("10.255.255.1"), 9)));
  ApplicationContainer zigbeeSinkApp = zigbeeSinkHelper.Install(zigbee.Get(0));
  zigbeeSinkApp.Start(Seconds(warmupTime));
  zigbeeSinkApp.Stop(Seconds(warmupTime + simulationTime));

  Ipv4Address coordIp = zbIfaces.GetAddress(0);
  OnOffHelper zigbeeClientHelper("ns3::UdpSocketFactory",
                                 ns3::InetSocketAddress(InetSocketAddress(coordIp, zigbeePort)));
  // zigbeeClientHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
  // zigbeeClientHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
  zigbeeClientHelper.SetAttribute("DataRate", DataRateValue(DataRate(zigbeeDataRate)));
  zigbeeClientHelper.SetAttribute("PacketSize", UintegerValue(zigbeePacketSize));

  ApplicationContainer zigbeeClientApp = zigbeeClientHelper.Install(zigbee.Get(1)); // Drugi węzeł Zigbee jako klient
  zigbeeClientApp.Start(Seconds(warmupTime));
  zigbeeClientApp.Stop(Seconds(warmupTime + simulationTime));

  // FlowMonitor
  FlowMonitorHelper fm;
  Ptr<FlowMonitor> mon = fm.InstallAll();

  // Simulation start/finish
  Simulator::Stop(Seconds(warmupTime + simulationTime));

  NS_LOG_INFO("Starting simulation");
  auto t0 = std::chrono::steady_clock::now();
  Simulator::Run();
  auto t1 = std::chrono::steady_clock::now();

  auto finalTime = std::chrono::duration_cast<std::chrono::seconds>(t1 - t0).count();
  NS_LOG_INFO("Simulation finished in " << finalTime << " s");

  // Get results
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(fm.GetClassifier());

  // WiFi stats
  ReportWifiStats(mon, classifier, wifiPort);

  // Zigbee stats
  ReportZigbeeStats(mon, classifier, zigbeePort);

  Simulator::Destroy();
  return 0;
}

// Calculation function
void ReportWifiStats(Ptr<FlowMonitor> monitor, Ptr<Ipv4FlowClassifier> classifier, uint16_t wifiPort) {
  std::cout << "\n=== Wi-Fi Flow Summary ===\n"
            << "SrcAddr  → DstAddr    TxPkts   RxPkts   LostPkts   "
            << "Thru(Mb/s)    AvgDelay(ms)    Jitter(ms)    PDR(%)\n";

  monitor->CheckForLostPackets(); // Upewnij się, że statystyki są aktualne

  for (auto const& f : monitor->GetFlowStats()) {
    const FlowMonitor::FlowStats& st = f.second;
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(f.first);

    // Filtruj tylko UDP‐flowy na zadanym porcie WiFi
    if (t.protocol != 17 || t.destinationPort != wifiPort) {
      continue;
    }

    // Czas trwania (w sekundach)
    double duration = (st.timeLastRxPacket - st.timeFirstTxPacket).GetSeconds();
    if (duration == 0 && st.rxBytes > 0) {
      duration = (Simulator::Now() - st.timeFirstTxPacket).GetSeconds();
    }
    if (duration <= 0 && st.rxBytes == 0) {
      duration = -1.0; // brak ruchu
    }

    // Throughput w Mb/s
    double thrMbps = (duration > 0.0) ? (st.rxBytes * 8.0) / (duration * 1e6) : 0.0;

    // Średnie opóźnienie (ms)
    double avgDelay = (st.rxPackets > 0) ? st.delaySum.GetMilliSeconds() / st.rxPackets : 0.0;

    // Średni jitter (ms) (wymaga co najmniej dwóch odebranych pakietów)
    double avgJit = (st.rxPackets > 1) ? st.jitterSum.GetMilliSeconds() / (st.rxPackets - 1) : 0.0;

    // PDR (%) = 100 * rxPackets / txPackets
    double pdr = (st.txPackets > 0) ? (static_cast<double>(st.rxPackets) / st.txPackets) * 100.0 : 0.0;

    std::cout << std::fixed << std::setprecision(6) << t.sourceAddress << " → " << t.destinationAddress << "    "
              << st.txPackets << "    " << st.rxPackets << "    " << st.lostPackets << "    " << thrMbps << "    "
              << avgDelay << "    " << avgJit << "    " << pdr << "\n";
  }
}

void ReportZigbeeStats(Ptr<FlowMonitor> monitor, Ptr<Ipv4FlowClassifier> classifier, uint16_t zigbeePort) {
  std::cout << "\n=== ZigBee Flow Summary ===\n"
            << "SrcAddr  → DstAddr    TxPkts   RxPkts   LostPkts   "
            << "Thru(kb/s)    AvgDelay(ms)    Jitter(ms)    PDR(%)\n";

  monitor->CheckForLostPackets();

  for (auto const& f : monitor->GetFlowStats()) {
    const FlowMonitor::FlowStats& st = f.second;
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(f.first);

    // Filtrujemy tylko UDP flowy pod portem ZigBee (np. 9)
    if (t.protocol != 17 || t.destinationPort != zigbeePort) {
      continue;
    }

    // Czas trwania
    double duration = (st.timeLastRxPacket - st.timeFirstTxPacket).GetSeconds();
    if (duration == 0 && st.rxBytes > 0) {
      duration = (Simulator::Now() - st.timeFirstTxPacket).GetSeconds();
    }
    if (duration <= 0 && st.rxBytes == 0) {
      duration = -1.0;
    }

    // Throughput w kb/s (bo ZigBee dane są małe)
    double thrKbps = (duration > 0.0) ? (st.rxBytes * 8.0) / (duration * 1e3) : 0.0;

    double avgDelay = (st.rxPackets > 0) ? st.delaySum.GetMilliSeconds() / st.rxPackets : 0.0;

    double avgJit = (st.rxPackets > 1) ? st.jitterSum.GetMilliSeconds() / (st.rxPackets - 1) : 0.0;

    double pdr = (st.txPackets > 0) ? (static_cast<double>(st.rxPackets) / st.txPackets) * 100.0 : 0.0;

    std::cout << std::fixed << std::setprecision(6) << t.sourceAddress << " → " << t.destinationAddress << "    "
              << st.txPackets << "    " << st.rxPackets << "    " << st.lostPackets << "    " << thrKbps << "    "
              << avgDelay << "    " << avgJit << "    " << pdr << "\n";
  }
}