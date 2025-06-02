#include "ns3/applications-module.h"
#include "ns3/arp-cache.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
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
void PrintNodePositions(const std::string& tag, const NodeContainer& nodes);

NS_LOG_COMPONENT_DEFINE("wifi-zigbee-coex");

int main(int argc, char* argv[]) {
  // logging
  uint16_t logLevel = 3;

  // WiFi configuration
  std::string wifiStandard = "80211n";
  std::string wifiDataRate = "60Mbps";
  uint32_t wifiPacketSize = 1472;
  uint16_t wifiPort = 15;

  // ZigBee configuration
  double joinInterval = 0.5;
  double scanDuration = 3;
  double baseWarmupTime = scanDuration + 0.5;
  uint32_t nZigbee = 5;
  uint16_t zigbeePort = 9;
  std::string zigbeeDataRate = "64kbps";
  uint32_t zigbeePacketSize = 80;

  // Simulation configuration
  double simulationTime = 60;
  double warmupTime = joinInterval * nZigbee + baseWarmupTime + 1.0;

  // Get configura5.0n
  CommandLine cmd;
  cmd.AddValue("wifiStandard", "Choose standard to use: 80211n | 80211ac | 80211ax ", wifiStandard);
  cmd.AddValue("wifiDataRate", "Data rate for Wifi devices", wifiDataRate);
  cmd.AddValue("wifiPacketSize", "Wifi packet size", wifiPacketSize);
  cmd.AddValue("simulationTime", "How long simulation should run (s)", simulationTime);
  cmd.AddValue("warmupTime", "How many seconds to run as warmup (before measurements)", warmupTime);
  cmd.AddValue("nZigbee", "Number of ZigBee devices", nZigbee);
  cmd.AddValue("zigbeeDataRate", "Data rate for Zigbee client", zigbeeDataRate);
  cmd.AddValue("zigbeePacketSize", "Packet size for Zigbee client", zigbeePacketSize);
  cmd.AddValue("logLevel", "Declare logging level for simulation 0-4 <=> none,error,warn,info,debug", logLevel);
  cmd.Parse(argc, argv);

  // Log configuration
  switch (logLevel) {
  case 1:
    LogComponentEnable("wifi-zigbee-coex", LOG_LEVEL_ERROR);
    break;
  case 2:
    LogComponentEnable("wifi-zigbee-coex", LOG_LEVEL_WARN);
    break;
  case 3:
    LogComponentEnable("wifi-zigbee-coex", LOG_LEVEL_INFO);
    break;
  case 4:
    LogComponentEnable("wifi-zigbee-coex", LOG_LEVEL_DEBUG);
    break;
  default:
    break;
  }

  // Print configuration
  NS_LOG_INFO("wifi-zigbee-coex - configuration:");
  NS_LOG_INFO("> wifiStandard: " << wifiStandard);
  NS_LOG_INFO("> wifiDataRate: " << wifiDataRate);
  NS_LOG_INFO("> simulationTime: " << simulationTime);
  NS_LOG_INFO("> baseWarmupTime: " << baseWarmupTime);
  NS_LOG_INFO("> warmupTime: " << warmupTime);
  NS_LOG_INFO("> nZigbee: " << nZigbee);
  NS_LOG_INFO("> zigbeeDataRate: " << zigbeeDataRate);
  NS_LOG_INFO("> zigbeePacketSize: " << zigbeePacketSize);
  NS_LOG_INFO("> logLevel: " << logLevel);

  Ptr<SpectrumChannel> commonChannel = CreateObject<MultiModelSpectrumChannel>();
  commonChannel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
  commonChannel->AddPropagationLossModel(CreateObject<LogDistancePropagationLossModel>());
  {
    auto nak = CreateObject<NakagamiPropagationLossModel>();
    nak->SetAttribute("m0", DoubleValue(1.0));
    nak->SetAttribute("m1", DoubleValue(3.0));
    nak->SetAttribute("m2", DoubleValue(3.0));
    commonChannel->AddPropagationLossModel(nak);
  }
  NS_LOG_DEBUG("Successfuly configured common channel");

  // Single AP
  NodeContainer wifiAp;
  wifiAp.Create(1);

  // Two stations
  NodeContainer wifiSta;
  wifiSta.Create(2);

  // Zigbee devices
  NodeContainer zigbee;
  zigbee.Create(nZigbee);

  NS_LOG_DEBUG("Declared devices for the simulation");

  // Configure mobility
  MobilityHelper mob;
  mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mob.Install(wifiAp);
  mob.Install(wifiSta);
  mob.Install(zigbee);

  NS_LOG_DEBUG("Installed mobility model to the nodes");

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
  mob.Install(wifiAp);

  mob.SetPositionAllocator(staPos);
  mob.Install(wifiSta);

  mob.SetPositionAllocator(zbPos);
  mob.Install(zigbee);

  NS_LOG_DEBUG("Nodes located on the specified positions");

  PrintNodePositions("AP", wifiAp);
  PrintNodePositions("STA", wifiSta);
  PrintNodePositions("ZB", zigbee);

  // Configure channel
  SpectrumWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel(commonChannel);

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

  NS_LOG_DEBUG("Configured Wifi Phy");

  // Wifi configuration
  WifiMacHelper mac;
  Ssid ssid = Ssid("wifi-coex");

  mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
  NetDeviceContainer staDev = wifi.Install(wifiPhy, mac, wifiSta);

  mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
  NetDeviceContainer apDev = wifi.Install(wifiPhy, mac, wifiAp);

  NS_LOG_DEBUG("Configured WiFi stack");

  // LR-WPAN configuration
  LrWpanHelper lrWpan;
  lrWpan.SetChannel(commonChannel);

  NS_LOG_DEBUG("Configured LR-WPAN Phy");

  NetDeviceContainer zbDev = lrWpan.Install(zigbee);
  // Create one PAN
  lrWpan.CreateAssociatedPan(zbDev, 0x1234);

  NS_LOG_DEBUG("Configured PAN for zigbee connection");

  // Install Zigbee stack
  ZigbeeHelper zbHelper;
  zigbee::ZigbeeStackContainer zbStacks = zbHelper.Install(zbDev);

  NS_LOG_DEBUG("Installed zigbee stack");

  // Configure first device to be the coordinator
  if (nZigbee > 0) {
    Ptr<zigbee::ZigbeeStack> coordStack = zbStacks.Get(0);
    zigbee::NlmeNetworkFormationRequestParams formParams;
    formParams.m_scanDuration = scanDuration;
    formParams.m_beaconOrder = 15;
    formParams.m_superFrameOrder = 15;

    // Use "all 16 channels" on the 2.4 GHz
    const uint32_t ALL_CHANNELS = 0x07FFF800;
    formParams.m_scanChannelList.channelPageCount = 1;
    formParams.m_scanChannelList.channelsField.resize(1);
    formParams.m_scanChannelList.channelsField[0] = ALL_CHANNELS;

    Simulator::ScheduleWithContext(coordStack->GetNode()->GetId(), Seconds(baseWarmupTime),
                                   &zigbee::ZigbeeNwk::NlmeNetworkFormationRequest, coordStack->GetNwk(), formParams);

    NS_LOG_DEBUG("Configured coordinator Phy");
  }

  // Other zigbee nodes join as routers
  for (uint32_t i = 1; i < zbStacks.GetN(); i++) {
    Ptr<zigbee::ZigbeeStack> rtrStack = zbStacks.Get(i);
    zigbee::CapabilityInformation ci;
    ci.SetDeviceType(zigbee::ROUTER);
    ci.SetAllocateAddrOn(true);

    zigbee::NlmeJoinRequestParams joinParams;
    joinParams.m_rejoinNetwork = zigbee::ASSOCIATION;
    joinParams.m_capabilityInfo = ci.GetCapability();

    Simulator::ScheduleWithContext(rtrStack->GetNode()->GetId(), Seconds(baseWarmupTime + joinInterval * i),
                                   &zigbee::ZigbeeNwk::NlmeJoinRequest, rtrStack->GetNwk(), joinParams);

    NS_LOG_DEBUG("Configured Zigbee node as router, node: " << rtrStack->GetNode()->GetId());
  }

  // Assign a unique 16-bit "short" address to each device's MAC:
  for (uint32_t i = 0; i < zbDev.GetN(); i++) {
    Ptr<lrwpan::LrWpanNetDevice> lrwpanDev = DynamicCast<lrwpan::LrWpanNetDevice>(zbDev.Get(i));
    Ptr<lrwpan::LrWpanMac> mac = lrwpanDev->GetMac();
    Mac16Address shortAddr = Mac16Address::Allocate();
    mac->SetShortAddress(shortAddr);

    NS_LOG_DEBUG("Configured Zigbee node's MAC: " << shortAddr);
  }

  NS_LOG_DEBUG("Configured Zigbee stack");

  // IP configuration
  InternetStackHelper inet;
  inet.Install(wifiAp);
  inet.Install(wifiSta);
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.0.0.0", "255.255.255.0");
  ipv4.Assign(NetDeviceContainer(apDev, staDev));

  // IP configuration Zigbee
  InternetStackHelper zigbeeInet;
  zigbeeInet.Install(zigbee);
  Ipv4AddressHelper ipv4Zb;
  ipv4Zb.SetBase("172.16.0.0", "255.255.255.0");
  Ipv4InterfaceContainer zbIfaces = ipv4Zb.Assign(zbDev);

  // Wifi sink
  PacketSinkHelper wifiSink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), wifiPort));
  ApplicationContainer wifiSinkApp = wifiSink.Install(wifiAp.Get(0));
  wifiSinkApp.Start(Seconds(warmupTime));
  wifiSinkApp.Stop(Seconds(warmupTime + simulationTime));
  NS_LOG_DEBUG("Installed WiFi sink on access point, node: " << wifiAp.Get(0)->GetId());

  // Wifi traffic app
  ApplicationContainer wifiTrafficApps;
  OnOffHelper wifiTrafficApp("ns3::UdpSocketFactory", InetSocketAddress("10.0.0.1", wifiPort));
  wifiTrafficApp.SetAttribute("DataRate", DataRateValue(wifiDataRate));
  wifiTrafficApp.SetAttribute("PacketSize", UintegerValue(wifiPacketSize));
  for (uint32_t i = 0; i < wifiSta.GetN(); i++) {
    ApplicationContainer app = wifiTrafficApp.Install(wifiSta.Get(i));
    app.Start(Seconds(warmupTime));
    app.Stop(Seconds(warmupTime + simulationTime));
    wifiTrafficApps.Add(app);
    NS_LOG_DEBUG("Installed OnOff application on WiFi station, node: " << wifiSta.Get(i)->GetId());
  }

  // Zigbee sink
  if (nZigbee != 0) {
    PacketSinkHelper zbSink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), zigbeePort));
    ApplicationContainer zbSinkApp = zbSink.Install(zigbee.Get(0));
    zbSinkApp.Start(Seconds(warmupTime));
    zbSinkApp.Stop(Seconds(warmupTime + simulationTime));
    NS_LOG_DEBUG("Installed Zigbee PacketSink on coordinator, node: " << zigbee.Get(0)->GetId());
  }

  // Zigbee traffic app
  ApplicationContainer zigbeeTrafficApps;
  OnOffHelper zigbeeTrafficApp("ns3::UdpSocketFactory", InetSocketAddress("172.16.0.1", zigbeePort));
  zigbeeTrafficApp.SetAttribute("DataRate", DataRateValue(zigbeeDataRate));
  zigbeeTrafficApp.SetAttribute("PacketSize", UintegerValue(zigbeePacketSize));
  for (uint32_t i = 1; i < zigbee.GetN(); ++i) {
    ApplicationContainer app = zigbeeTrafficApp.Install(zigbee.Get(i));
    app.Start(Seconds(warmupTime));
    app.Stop(Seconds(warmupTime + simulationTime));
    zigbeeTrafficApps.Add(app);
    NS_LOG_DEBUG("Installed ZigBee OnOff application on node: " << zigbee.Get(i)->GetId());
  }

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

  monitor->CheckForLostPackets();

  for (auto const& f : monitor->GetFlowStats()) {
    const FlowMonitor::FlowStats& st = f.second;
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(f.first);

    if (t.protocol != 17 || t.destinationPort != wifiPort) {
      continue;
    }

    double duration = (st.timeLastRxPacket - st.timeFirstTxPacket).GetSeconds();
    if (duration == 0 && st.rxBytes > 0) {
      duration = (Simulator::Now() - st.timeFirstTxPacket).GetSeconds();
    }
    if (duration <= 0 && st.rxBytes == 0) {
      duration = -1.0;
    }

    double thrMbps = (duration > 0.0) ? (st.rxBytes * 8.0) / (duration * 1e6) : 0.0;
    double avgDelay = (st.rxPackets > 0) ? st.delaySum.GetMilliSeconds() / st.rxPackets : 0.0;
    double avgJit = (st.rxPackets > 1) ? st.jitterSum.GetMilliSeconds() / (st.rxPackets - 1) : 0.0;
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

    if (t.protocol != 17 || t.destinationPort != zigbeePort) {
      continue;
    }

    double duration = (st.timeLastRxPacket - st.timeFirstTxPacket).GetSeconds();
    if (duration == 0 && st.rxBytes > 0) {
      duration = (Simulator::Now() - st.timeFirstTxPacket).GetSeconds();
    }
    if (duration <= 0 && st.rxBytes == 0) {
      duration = -1.0;
    }

    double thrKbps = (duration > 0.0) ? (st.rxBytes * 8.0) / (duration * 1e3) : 0.0;
    double avgDelay = (st.rxPackets > 0) ? st.delaySum.GetMilliSeconds() / st.rxPackets : 0.0;
    double avgJit = (st.rxPackets > 1) ? st.jitterSum.GetMilliSeconds() / (st.rxPackets - 1) : 0.0;
    double pdr = (st.txPackets > 0) ? (static_cast<double>(st.rxPackets) / st.txPackets) * 100.0 : 0.0;

    std::cout << std::fixed << std::setprecision(6) << t.sourceAddress << " → " << t.destinationAddress << "    "
              << st.txPackets << "    " << st.rxPackets << "    " << st.lostPackets << "    " << thrKbps << "    "
              << avgDelay << "    " << avgJit << "    " << pdr << "\n";
  }
}

void PrintNodePositions(const std::string& tag, const NodeContainer& nodes) {
  for (uint32_t i = 0; i < nodes.GetN(); ++i) {
    Ptr<MobilityModel> mm = nodes.Get(i)->GetObject<MobilityModel>();
    Vector pos = mm->GetPosition();
    NS_LOG_INFO("[" << tag << "] Node " << nodes.Get(i)->GetId() << " position: x=" << pos.x << " y=" << pos.y
                    << " z=" << pos.z);
  }
}