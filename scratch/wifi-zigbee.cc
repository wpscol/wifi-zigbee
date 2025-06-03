#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/inet-socket-address.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/spectrum-module.h"
#include "ns3/wifi-module.h"
#include "ns3/zigbee-module.h"

#include <chrono>
#include <iomanip>

using namespace ns3;
using namespace ns3::lrwpan;
using namespace ns3::zigbee;

static void NwkDataIndication(Ptr<ZigbeeStack> stack, NldeDataIndicationParams params, Ptr<Packet> p);
static void NwkNetworkFormationConfirm(Ptr<ZigbeeStack> stack, NlmeNetworkFormationConfirmParams params);
static void NwkNetworkDiscoveryConfirm(Ptr<ZigbeeStack> stack, NlmeNetworkDiscoveryConfirmParams params);
static void NwkJoinConfirm(Ptr<ZigbeeStack> stack, NlmeJoinConfirmParams params);
static void NwkRouteDiscoveryConfirm(Ptr<ZigbeeStack> stack, NlmeRouteDiscoveryConfirmParams params);
static void SendData(Ptr<ZigbeeStack> stackSrc, Ptr<ZigbeeStack> stackDst);
static void TraceRoute(Mac16Address src, Mac16Address dst);
static void PrintNodePositions(const std::string& tag, const NodeContainer& nodes);

ZigbeeStackContainer zigbeeStacks;

NS_LOG_COMPONENT_DEFINE("wifi-zigbee-coex");

int main(int argc, char* argv[]) {
  LogComponentEnable("wifi-zigbee-coex", LOG_LEVEL_DEBUG);
  LogComponentEnable("ZigbeeNwk", LOG_LEVEL_DEBUG);

  RngSeedManager::SetSeed(1);
  RngSeedManager::SetRun(1);

  // WiFi configuration
  std::string wifiStandard = "80211n";
  // std::string wifiDataRate = "60Mbps";
  std::string wifiDataRate = "0";
  uint32_t wifiPacketSize = 1472;
  uint16_t wifiPort = 5000;

  // ZigBee configuration
  uint32_t nZigbee = 5;

  // Simulation configuration
  double simulationTime = 60;

  // Get configuration
  CommandLine cmd;
  cmd.AddValue("wifiDataRate", "Data rate for Wifi devices", wifiDataRate);
  cmd.AddValue("wifiPacketSize", "Wifi packet size", wifiPacketSize);
  cmd.AddValue("simulationTime", "How long simulation should run (s)", simulationTime);
  cmd.AddValue("nZigbee", "Number of ZigBee devices", nZigbee);
  cmd.Parse(argc, argv);

  // Print configuration
  NS_LOG_INFO("wifi-zigbee-coex - configuration:");
  NS_LOG_INFO("> wifiDataRate: " << wifiDataRate);
  NS_LOG_INFO("> simulationTime: " << simulationTime);
  NS_LOG_INFO("> nZigbee: " << nZigbee);

  // Single AP
  NodeContainer wifiApNodes;
  wifiApNodes.Create(1);

  // Two stations
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create(2);

  // Zigbee devices
  NodeContainer zigbeeNodes;
  zigbeeNodes.Create(nZigbee);

  NS_LOG_DEBUG("Declared devices for the simulation");

  // Configure mobility
  MobilityHelper mob;
  mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mob.Install(wifiApNodes);
  mob.Install(wifiStaNodes);
  mob.Install(zigbeeNodes);

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
  mob.Install(wifiApNodes);

  mob.SetPositionAllocator(staPos);
  mob.Install(wifiStaNodes);

  mob.SetPositionAllocator(zbPos);
  mob.Install(zigbeeNodes);
  NS_LOG_DEBUG("Nodes located on the specified positions");

  PrintNodePositions("AP", wifiApNodes);
  PrintNodePositions("STA", wifiStaNodes);
  PrintNodePositions("ZB", zigbeeNodes);

  // LR-WPAN configuration
  LrWpanHelper lrWpanHelper;
  NetDeviceContainer lrwpanDevices = lrWpanHelper.Install(zigbeeNodes);
  // Iterate each Zigbee device
  for (uint32_t i = 0; i < zigbeeNodes.GetN(); i++) {
    Ptr<LrWpanNetDevice> dev = lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>();
    // Coordinator
    if (i == 0) {
      dev->GetMac()->SetExtendedAddress("00:00:00:00:00:00:CA:FE");
    } else {
      // Set MAC for Zigbee node
      Mac64Address addr = Mac64Address::Allocate();
      dev->GetMac()->SetExtendedAddress(addr);
    }
  }
  NS_LOG_DEBUG("Configured MAC addressing on LR-WPAN stack");

  // Configure channel and loss models
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

  // Configure Zigbee channel
  for (uint32_t i = 0; i < zigbeeNodes.GetN(); i++) {
    Ptr<LrWpanNetDevice> dev = lrwpanDevices.Get(i)->GetObject<LrWpanNetDevice>();
    dev->SetChannel(commonChannel);
  }

  // Configure WiFi channel
  SpectrumWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel(commonChannel);

  // Set WiFi standard
  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211n);
  wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");

  // Wifi configuration
  WifiMacHelper mac;
  Ssid ssid = Ssid("wifi-coex");

  mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
  NetDeviceContainer staDev = wifi.Install(wifiPhy, mac, wifiStaNodes);

  mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
  NetDeviceContainer apDev = wifi.Install(wifiPhy, mac, wifiApNodes);

  NS_LOG_DEBUG("Configured WiFi stack");

  // Install Zigbee stack
  ZigbeeHelper zigbeeHelper;
  ZigbeeStackContainer zigbeeStackContainer = zigbeeHelper.Install(lrwpanDevices);
  for (uint32_t i = 0; i < zigbeeNodes.GetN(); i++) {
    Ptr<ZigbeeStack> stack = zigbeeStackContainer.Get(i)->GetObject<ZigbeeStack>();
    // Add to container for later analysis
    zigbeeStacks.Add(stack);
    // Assign streams
    stack->GetNwk()->AssignStreams(i * 10);

    // Coordinator
    if (i == 0) {
      stack->GetNwk()->SetNlmeNetworkFormationConfirmCallback(MakeBoundCallback(&NwkNetworkFormationConfirm, stack));
      stack->GetNwk()->SetNlmeRouteDiscoveryConfirmCallback(MakeBoundCallback(&NwkRouteDiscoveryConfirm, stack));
      stack->GetNwk()->SetNldeDataIndicationCallback(MakeBoundCallback(&NwkDataIndication, stack));
      continue;
    }

    // Other nodes
    stack->GetNwk()->SetNldeDataIndicationCallback(MakeBoundCallback(&NwkDataIndication, stack));
    stack->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(MakeBoundCallback(&NwkNetworkDiscoveryConfirm, stack));
    stack->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, stack));
  }
  NS_LOG_DEBUG("Installed zigbee stacks and callback functions");

  // Start the network
  for (uint32_t i = 0; i < zigbeeNodes.GetN(); i++) {
    Ptr<ZigbeeStack> stack = zigbeeStackContainer.Get(i)->GetObject<ZigbeeStack>();

    // Coordinator
    if (i == 0) {
      NlmeNetworkFormationRequestParams netFormParams;
      netFormParams.m_scanChannelList.channelPageCount = 1;
      netFormParams.m_scanChannelList.channelsField[0] = ALL_CHANNELS;
      netFormParams.m_scanDuration = 0;
      netFormParams.m_superFrameOrder = 15;
      netFormParams.m_beaconOrder = 15;

      Simulator::ScheduleWithContext(stack->GetNode()->GetId(), Seconds(1), &ZigbeeNwk::NlmeNetworkFormationRequest,
                                     stack->GetNwk(), netFormParams);
      continue;
    }

    // Nodes
    NlmeNetworkDiscoveryRequestParams netDiscParams;
    netDiscParams.m_scanChannelList.channelPageCount = 1;
    netDiscParams.m_scanChannelList.channelsField[0] = 0x00007800; // BitMap: Channels 11~14
    netDiscParams.m_scanDuration = 2;
    Simulator::ScheduleWithContext(stack->GetNode()->GetId(), Seconds(3 + i), &ZigbeeNwk::NlmeNetworkDiscoveryRequest,
                                   stack->GetNwk(), netDiscParams);
  }
  NS_LOG_DEBUG("Scheduled Zigbee network configuration");

  // WiFi IP configuration
  InternetStackHelper inet;
  inet.Install(wifiApNodes);
  inet.Install(wifiStaNodes);
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.0.0.0", "255.255.255.0");
  ipv4.Assign(NetDeviceContainer(apDev, staDev));

  // Wifi sink
  PacketSinkHelper wifiSink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), wifiPort));
  ApplicationContainer wifiSinkApp = wifiSink.Install(wifiApNodes.Get(0));
  // wifiSinkApp.Start(Seconds(warmupTime));
  // wifiSinkApp.Stop(Seconds(warmupTime + simulationTime));
  NS_LOG_DEBUG("Installed WiFi sink on access point, node: " << wifiApNodes.Get(0)->GetId());

  // Wifi traffic app
  ApplicationContainer wifiTrafficApps;
  OnOffHelper wifiTrafficApp("ns3::UdpSocketFactory", InetSocketAddress("10.0.0.1", wifiPort));
  wifiTrafficApp.SetAttribute("DataRate", DataRateValue(wifiDataRate));
  wifiTrafficApp.SetAttribute("PacketSize", UintegerValue(wifiPacketSize));
  for (uint32_t i = 0; i < wifiStaNodes.GetN(); i++) {
    ApplicationContainer app = wifiTrafficApp.Install(wifiStaNodes.Get(i));
    // app.Start(Seconds(warmupTime));
    // app.Stop(Seconds(warmupTime + simulationTime));
    wifiTrafficApps.Add(app);
    NS_LOG_DEBUG("Installed OnOff application on WiFi station, node: " << wifiStaNodes.Get(i)->GetId());
  }

  Simulator::Schedule(Seconds(8), &SendData, zigbeeStacks.Get(0), zigbeeStacks.Get(nZigbee - 1));

  // WiFi flow monitor
  FlowMonitorHelper fm4;
  Ptr<FlowMonitor> monitor4 = fm4.Install(NodeContainer(wifiStaNodes, wifiApNodes));

  // Simulation start/finish
  Simulator::Stop(Seconds(120));

  NS_LOG_INFO("Starting simulation");
  auto t0 = std::chrono::steady_clock::now();
  Simulator::Run();
  auto t1 = std::chrono::steady_clock::now();
  auto finalTime = std::chrono::duration_cast<std::chrono::seconds>(t1 - t0).count();
  NS_LOG_INFO("Simulation finished in " << finalTime << " s");

  Simulator::Destroy();
  return 0;
}

static void PrintNodePositions(const std::string& tag, const NodeContainer& nodes) {
  for (uint32_t i = 0; i < nodes.GetN(); ++i) {
    Ptr<MobilityModel> mm = nodes.Get(i)->GetObject<MobilityModel>();
    Vector pos = mm->GetPosition();
    NS_LOG_INFO("[" << tag << "] Node " << nodes.Get(i)->GetId() << " position: x=" << pos.x << " y=" << pos.y
                    << " z=" << pos.z);
  }
}

static void TraceRoute(Mac16Address src, Mac16Address dst) {
  std::cout << "\n";
  std::cout << "Traceroute to destination [" << dst << "] (Time: " << Simulator::Now().As(Time::S) << "):\n";
  Mac16Address target = src;
  uint32_t count = 1;
  while (target != Mac16Address("FF:FF") && target != dst) {
    Ptr<ZigbeeStack> zstack;

    for (auto i = zigbeeStacks.Begin(); i != zigbeeStacks.End(); i++) {
      zstack = *i;
      if (zstack->GetNwk()->GetNetworkAddress() == target) {
        break;
      }
    }

    bool neighbor = false;
    target = zstack->GetNwk()->FindRoute(dst, neighbor);
    if (target == Mac16Address("FF:FF")) {
      std::cout << count << ". Node " << zstack->GetNode()->GetId() << " [" << zstack->GetNwk()->GetNetworkAddress()
                << " | " << zstack->GetNwk()->GetIeeeAddress() << "]: "
                << " Destination Unreachable\n";
    } else {
      std::cout << count << ". Node " << zstack->GetNode()->GetId() << " [" << zstack->GetNwk()->GetNetworkAddress()
                << " | " << zstack->GetNwk()->GetIeeeAddress() << "]: "
                << "NextHop [" << target << "] ";
      if (neighbor) {
        std::cout << "(*Neighbor)\n";
      } else {
        std::cout << "\n";
      }
      count++;
    }
  }
  std::cout << "\n";
}

static void CreateManyToOneRoutes(Ptr<ZigbeeStack> zigbeeStackConcentrator, Ptr<ZigbeeStack> zigbeeStackSrc) {
  // Generate all the routes to the concentrator device
  NlmeRouteDiscoveryRequestParams routeDiscParams;
  routeDiscParams.m_dstAddrMode = NO_ADDRESS;
  Simulator::ScheduleNow(&ZigbeeNwk::NlmeRouteDiscoveryRequest, zigbeeStackConcentrator->GetNwk(), routeDiscParams);

  // Give a few seconds to allow the creation of the route and
  // then print the route trace and tables from the source
  Simulator::Schedule(Seconds(3), &TraceRoute, zigbeeStackSrc->GetNwk()->GetNetworkAddress(),
                      zigbeeStackConcentrator->GetNwk()->GetNetworkAddress());

  // Print the content of the source device tables (Neighbor, Discovery, Routing)
  Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>(&std::cout);
  Simulator::Schedule(Seconds(4), &ZigbeeNwk::PrintNeighborTable, zigbeeStackSrc->GetNwk(), stream);

  Simulator::Schedule(Seconds(4), &ZigbeeNwk::PrintRoutingTable, zigbeeStackSrc->GetNwk(), stream);

  Simulator::Schedule(Seconds(4), &ZigbeeNwk::PrintRouteDiscoveryTable, zigbeeStackSrc->GetNwk(), stream);
}

static void NwkDataIndication(Ptr<ZigbeeStack> stack, NldeDataIndicationParams params, Ptr<Packet> p) {
  std::cout << "Received packet of size " << p->GetSize() << "\n";
}

static void NwkNetworkFormationConfirm(Ptr<ZigbeeStack> stack, NlmeNetworkFormationConfirmParams params) {
  std::cout << "NlmeNetworkFormationConfirmStatus = " << params.m_status << "\n";
}

static void NwkNetworkDiscoveryConfirm(Ptr<ZigbeeStack> stack, NlmeNetworkDiscoveryConfirmParams params) {
  // See Zigbee Specification r22.1.0, 3.6.1.4.1
  // This method implements a simplistic version of the method implemented
  // in a zigbee APL layer. In this layer a candidate Extended PAN Id must
  // be selected and a NLME-JOIN.request must be issued.

  if (params.m_status == NwkStatus::SUCCESS) {
    std::cout << "    Network discovery confirm Received. Networks found "
              << "(" << params.m_netDescList.size() << ")\n";

    for (const auto& netDescriptor : params.m_netDescList) {
      std::cout << "      ExtPanID: 0x" << std::hex << netDescriptor.m_extPanId << std::dec << "\n"
                << "      CH:  " << static_cast<uint32_t>(netDescriptor.m_logCh) << std::hex << "\n"
                << "      Pan Id: 0x" << netDescriptor.m_panId << std::hex << "\n"
                << "      stackprofile: " << std::dec << static_cast<uint32_t>(netDescriptor.m_stackProfile) << "\n"
                << "      ----------------\n ";
    }

    NlmeJoinRequestParams joinParams;

    zigbee::CapabilityInformation capaInfo;
    capaInfo.SetDeviceType(ROUTER);
    capaInfo.SetAllocateAddrOn(true);

    joinParams.m_rejoinNetwork = zigbee::JoiningMethod::ASSOCIATION;
    joinParams.m_capabilityInfo = capaInfo.GetCapability();
    joinParams.m_extendedPanId = params.m_netDescList[0].m_extPanId;

    Simulator::ScheduleNow(&ZigbeeNwk::NlmeJoinRequest, stack->GetNwk(), joinParams);
  } else {
    std::cout << " WARNING: Unable to discover networks | status: " << params.m_status << "\n";
  }
}

static void NwkJoinConfirm(Ptr<ZigbeeStack> stack, NlmeJoinConfirmParams params) {
  if (params.m_status == NwkStatus::SUCCESS) {
    std::cout << Simulator::Now().As(Time::S) << " The device joined the network SUCCESSFULLY with short address ["
              << std::hex << params.m_networkAddress << "] on the Extended PAN Id: " << std::hex
              << params.m_extendedPanId << "\n"
              << std::dec;

    // 3 - After dev  is associated, it should be started as a router
    //     (i.e. it becomes able to accept request from other devices to join the network)
    NlmeStartRouterRequestParams startRouterParams;
    Simulator::ScheduleNow(&ZigbeeNwk::NlmeStartRouterRequest, stack->GetNwk(), startRouterParams);
  } else {
    std::cout << Simulator::Now().As(Time::S) << " The device FAILED to join the network with status "
              << params.m_status << "\n";
  }
}

static void NwkRouteDiscoveryConfirm(Ptr<ZigbeeStack> stack, NlmeRouteDiscoveryConfirmParams params) {
  std::cout << "NlmeRouteDiscoveryConfirmStatus = " << params.m_status << "\n";
}