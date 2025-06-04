/*
 * Copyright (c) 2024 Tokushima University, Japan
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Authors:
 *
 *  Alberto Gallegos Ramonet <alramonet@is.tokushima-u.ac.jp>
 */

/**
 * Mesh routing example with data transmission using a simple topology.
 *
 * This example shows the NWK layer procedure to perform a route request.
 * Prior the route discovery and data transmission, an association-based join is performed.
 * The procedure requires a sequence of primitive calls on a specific order in the indicated
 * devices.
 *
 *
 *  Network Extended PAN id: 0X000000000000CA:FE (based on the PAN coordinator address)
 *
 *  Devices Addresses:
 *
 *  [Coordinator] ZC  (dev0 | Node 0): [00:00:00:00:00:00:CA:FE]  [00:00]
 *  [Router 1]    ZR1 (dev1 | Node 1): [00:00:00:00:00:00:00:01]  [short addr assigned by ZC]
 *  [Router 2]    ZR2 (dev2 | Node 2): [00:00:00:00:00:00:00:02]  [short addr assigned by ZR1]
 *  [Router 3]    ZR3 (dev3 | Node 3): [00:00:00:00:00:00:00:03]  [short addr assigned by ZR2]
 *  [Router 4]    ZR4 (dev4 | Node 4): [00:00:00:00:00:00:00:04]  [short addr assigned by ZR1]
 *
 *  Topology:
 *
 *  ZC--------ZR1------------ZR2----------ZR3
 *             |
 *             |
 *            ZR4
 */

#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/simulator.h"
#include "ns3/single-model-spectrum-channel.h"
#include "ns3/spectrum-module.h"
#include "ns3/wifi-module.h"
#include "ns3/zigbee-module.h"

#include <iostream>

using namespace ns3;
using namespace ns3::lrwpan;
using namespace ns3::zigbee;

NS_LOG_COMPONENT_DEFINE("ZigbeeRouting");

ZigbeeStackContainer zigbeeStacks;

// Calculate QoS dla ZigBee
static const uint32_t g_totalDevices = 5;
static const uint16_t c_zigbeeBufferSize = 64;
static uint32_t g_joinedCount = 0;
static bool g_networkReady = false;

struct QoSInfo {
  uint32_t sentPackets = 0;
  uint32_t recvPackets = 0;
  double sumDelays = 0.0;
  double sumLqi = 0.0;
};
static std::map<uint32_t, QoSInfo> qosMap;
static std::map<uint32_t, std::map<uint32_t, std::set<uint32_t>>> receivedTracker;
static uint32_t g_seqNo = 0;

static void NwkNetworkFormationConfirm(Ptr<ZigbeeStack> stack, NlmeNetworkFormationConfirmParams params) {
  NS_LOG_INFO("NlmeNetworkFormationConfirmStatus = " << params.m_status << "\n");
}

static void NwkNetworkDiscoveryConfirm(Ptr<ZigbeeStack> stack, NlmeNetworkDiscoveryConfirmParams params) {
  // See Zigbee Specification r22.1.0, 3.6.1.4.1
  // This method implements a simplistic version of the method implemented
  // in a zigbee APL layer. In this layer a candidate Extended PAN Id must
  // be selected and a NLME-JOIN.request must be issued.

  if (params.m_status == NwkStatus::SUCCESS) {
    NS_LOG_INFO(" Network discovery confirm Received. Networks found (" << params.m_netDescList.size() << "):\n");

    for (const auto& netDescriptor : params.m_netDescList) {
      NS_LOG_INFO(" ExtPanID: 0x" << std::hex << netDescriptor.m_extPanId << "\n"
                                  << std::dec << " CH:  " << static_cast<uint32_t>(netDescriptor.m_logCh) << "\n"
                                  << std::hex << " Pan ID: 0x" << netDescriptor.m_panId << "\n"
                                  << " Stack profile: " << std::dec
                                  << static_cast<uint32_t>(netDescriptor.m_stackProfile) << "\n"
                                  << "--------------------");
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
    NS_ABORT_MSG("Unable to discover networks | status: " << params.m_status);
  }
}

static void NwkJoinConfirm(Ptr<ZigbeeStack> stack, NlmeJoinConfirmParams params) {
  if (params.m_status == NwkStatus::SUCCESS) {
    NS_LOG_INFO(Simulator::Now().As(Time::S)
                << " Node " << stack->GetNode()->GetId() << " | "
                << " The device joined the network SUCCESSFULLY with short address " << std::hex
                << params.m_networkAddress << " on the Extended PAN Id: " << std::hex << params.m_extendedPanId << "\n"
                << std::dec);

    // Iterate joined devices
    ++g_joinedCount;
    if (g_joinedCount == (g_totalDevices - 1)) {
      g_networkReady = true;
      NS_LOG_INFO(Simulator::Now().As(Time::S) << " | All Zigbee nodes joined the network" << std::endl);
    }

    // 3 - After dev 1 is associated, it should be started as a router
    //     (i.e. it becomes able to accept request from other devices to join the network)
    NlmeStartRouterRequestParams startRouterParams;
    Simulator::ScheduleNow(&ZigbeeNwk::NlmeStartRouterRequest, stack->GetNwk(), startRouterParams);
  } else {
    NS_LOG_ERROR(" The device FAILED to join the network with status " << params.m_status << "\n");
  }
}

static void NwkRouteDiscoveryConfirm(Ptr<ZigbeeStack> stack, NlmeRouteDiscoveryConfirmParams params) {
  NS_LOG_INFO("NlmeRouteDiscoveryConfirmStatus = " << params.m_status << "\n");
}

static void SendDataPeriod(Ptr<ZigbeeStack> stackSrc, Ptr<ZigbeeStack> stackDst, double interval) {
  if (!g_networkReady) {
    return;
  }

  if (c_zigbeeBufferSize < 16) {
    NS_ABORT_MSG("c_zigbeeBufferSize must be >= 16");
  }

  uint32_t srcNodeId = stackSrc->GetNode()->GetId();
  uint32_t destNodeId = stackDst->GetNode()->GetId();

  uint8_t buf[c_zigbeeBufferSize];
  double nowSeconds = Simulator::Now().GetSeconds();
  memcpy(buf + 0, &srcNodeId, 4);
  memcpy(buf + 4, &g_seqNo, 4);
  memcpy(buf + 8, &nowSeconds, 8);

  g_seqNo++;

  qosMap[destNodeId].sentPackets += 1;

  Ptr<Packet> p = Create<Packet>(buf, c_zigbeeBufferSize);
  NldeDataRequestParams dataReqParams;
  dataReqParams.m_dstAddrMode = UCST_BCST;
  dataReqParams.m_dstAddr = stackDst->GetNwk()->GetNetworkAddress();
  dataReqParams.m_nsduHandle = 1;
  dataReqParams.m_nsduLength = p->GetSize();
  dataReqParams.m_discoverRoute = ENABLE_ROUTE_DISCOVERY;

  Simulator::ScheduleNow(&ZigbeeNwk::NldeDataRequest, stackSrc->GetNwk(), dataReqParams, p);

  NS_LOG_DEBUG(Simulator::Now().GetSeconds()
               << "s Node" << srcNodeId << " sent packet seq=" << (g_seqNo - 1) << " size=" << p->GetSize() << " bytes"
               << " to " << dataReqParams.m_dstAddr << " totalSent =" << qosMap[srcNodeId].sentPackets);

  // Every interval
  Simulator::Schedule(Seconds(interval), &SendDataPeriod, stackSrc, stackDst, interval);
}

static void NwkDataIndication(Ptr<ZigbeeStack> stack, NldeDataIndicationParams params, Ptr<Packet> p) {
  if (p->GetSize() < 16) {
    NS_LOG_WARN("NwkDataIndication: packet too small (" << p->GetSize() << " bytes)");
    return;
  }

  uint8_t header[16];
  p->CopyData(header, 16);

  uint32_t srcNodeId;
  uint32_t seqNo;
  double sendTime;
  memcpy(&srcNodeId, header + 0, 4);
  memcpy(&seqNo, header + 4, 4);
  memcpy(&sendTime, header + 8, 8);

  uint32_t destNodeId = stack->GetNode()->GetId();

  // Duplicate check
  auto& srcMap = receivedTracker[destNodeId];
  auto& seqSet = srcMap[srcNodeId];
  if (seqSet.count(seqNo) > 0) {
    NS_LOG_WARN("Duplicate packet at Node" << destNodeId << " from Node" << srcNodeId << " [seq=" << seqNo
                                           << "] ignored");
    return;
  }
  seqSet.insert(seqNo);

  double recvTime = Simulator::Now().GetSeconds();
  double delay = recvTime - sendTime;

  double lqi = params.m_linkQuality; // 0..255

  auto& info = qosMap[destNodeId];
  info.recvPackets += 1;
  info.sumDelays += delay;
  info.sumLqi += lqi;

  NS_LOG_DEBUG(Simulator::Now().GetSeconds()
               << "s Node" << stack->GetNode()->GetId() << " <- Node" << srcNodeId << " [seq=" << seqNo << "]"
               << "  delay=" << std::fixed << std::setprecision(3) << delay << "s"
               << "  LQI=" << lqi << "  totalRecv=" << info.recvPackets);
}

static void PrintWifiFlowStats(FlowMonitorHelper& flowHelper, Ptr<FlowMonitor> flowMonitor) {
  // 1) Account for any lost packets
  flowMonitor->CheckForLostPackets();

  // 2) Get the classifier to look up IP‐addresses for each flow ID
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());

  // 3) Retrieve flow statistics map
  std::map<FlowId, FlowMonitor::FlowStats> stats = flowMonitor->GetFlowStats();

  // 4) Print header, now including PDR
  NS_LOG_UNCOND("=== WiFi FlowMonitor Statistics at " << Simulator::Now().GetSeconds() << "s ===");
  NS_LOG_UNCOND(
      "FlowID | Source Addr       | Dest Addr         | TxPkts | RxPkts | PDR   | LostPkts | Throughput(Kbps)");
  NS_LOG_UNCOND("-----------------------------------------------------------------------------------------------");

  // 5) Loop over each flow and print metrics
  for (auto& flow : stats) {
    FlowId flowId = flow.first;
    const FlowMonitor::FlowStats& fs = flow.second;

    // Find the corresponding 5‐tuple (source/dest IP, ports, protocol)
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);

    uint64_t txPackets = fs.txPackets;
    uint64_t rxPackets = fs.rxPackets;
    uint64_t lostPackets = fs.lostPackets;

    // Compute PDR (avoid division by zero)
    double pdr = 0.0;
    if (txPackets > 0) {
      pdr = double(rxPackets) / double(txPackets);
    }

    // Compute throughput = (rxBytes * 8) / duration (in Kbps)
    double throughput = 0.0;
    double duration = fs.timeLastRxPacket.GetSeconds() - fs.timeFirstTxPacket.GetSeconds();
    if (duration > 0.0) {
      throughput = (fs.rxBytes * 8.0) / (1000.0 * duration);
    }

    NS_LOG_UNCOND(std::setw(6) << flowId << " | " << std::setw(17) << t.sourceAddress << " | " << std::setw(17)
                               << t.destinationAddress << " | " << std::setw(6) << txPackets << " | " << std::setw(6)
                               << rxPackets << " | " << std::fixed << std::setprecision(2) << std::setw(5) << pdr
                               << " | " << std::setw(8) << lostPackets << " | " << std::fixed << std::setprecision(2)
                               << std::setw(14) << throughput);
  }
  NS_LOG_UNCOND("-----------------------------------------------------------------------------------------------");
}

static void PrintZigbeeQoS() {
  double now = Simulator::Now().GetSeconds();

  NS_LOG_UNCOND("=== ZigBee QoS SUMMARY at " << now << "s ===");
  NS_LOG_UNCOND("NodeId | SentPkts | RecvPkts |  PDR   | AvgDelay(s) | AvgLQI");
  NS_LOG_UNCOND("-------------------------------------------------------------");

  for (auto& kv : qosMap) {
    uint32_t nid = kv.first;
    const QoSInfo& info = kv.second;
    uint32_t sent = info.sentPackets;
    uint32_t recv = info.recvPackets;

    double pdr = 0.0;
    if (sent > 0) {
      pdr = double(recv) / double(sent);
    }

    double avgDelay = 0.0;
    double avgLqi = 0.0;
    if (recv > 0) {
      avgDelay = info.sumDelays / double(recv);
      avgLqi = info.sumLqi / double(recv);
    }

    NS_LOG_UNCOND(std::setw(6) << nid << " | " << std::setw(8) << sent << " | " << std::setw(8) << recv << " | "
                               << std::fixed << std::setprecision(2) << std::setw(5) << pdr << " | " << std::fixed
                               << std::setprecision(3) << std::setw(11) << avgDelay << " | " << std::fixed
                               << std::setprecision(1) << std::setw(6) << avgLqi);
  }
}

int main(int argc, char* argv[]) {
  LogComponentEnableAll(LogLevel(LOG_PREFIX_TIME | LOG_PREFIX_FUNC | LOG_PREFIX_NODE));
  // Enable logs for further details
  // LogComponentEnable("ZigbeeNwk", LOG_LEVEL_DEBUG);

  // Simulation settings
  std::string wifiDataRate = "160Mbps";
  uint32_t wifiChannelWidth = 40;
  uint32_t wifiPacketSize = 1472;
  uint16_t wifiPort = 5000;
  double heartbeatInterval = 0.5;
  double simulationTime = 60;
  uint32_t rngRun = 1;
  uint32_t seed = 1;
  uint32_t logLevel = 3;

  CommandLine cmd;
  cmd.AddValue("logLevel", "0=ERROR, 1=WARN, 2=INFO, 3=DEBUG, 4=LOGIC", logLevel);
  cmd.AddValue("wifiDataRate", "DataRate for WiFi (e.g. \"160Mbps\")", wifiDataRate);
  cmd.AddValue("wifiChannelWidth", "WiFi channel width (MHz)", wifiChannelWidth);
  cmd.AddValue("wifiPacketSize", "Size of each heartbeat packet (bytes)", wifiPacketSize);
  cmd.AddValue("heartbeatInterval", "Interval between heartbeats (s)", heartbeatInterval);
  cmd.AddValue("simulationTime", "Total simulation time (seconds)", simulationTime);
  cmd.AddValue("rngRun", "RNG run number (for SetRun)", rngRun);
  cmd.AddValue("seed", "RNG seed (for SetSeed)", seed);
  cmd.Parse(argc, argv);

  NS_LOG_UNCOND("\n============================================================");
  NS_LOG_UNCOND(" Simulation parameters:");
  NS_LOG_UNCOND("   wifiDataRate      = " << wifiDataRate);
  NS_LOG_UNCOND("   wifiChannelWidth  = " << wifiChannelWidth);
  NS_LOG_UNCOND("   wifiPacketSize    = " << wifiPacketSize);
  NS_LOG_UNCOND("   heartbeatInterval = " << heartbeatInterval);
  NS_LOG_UNCOND("   simulationTime    = " << simulationTime);
  NS_LOG_UNCOND("   rngRun            = " << rngRun);
  NS_LOG_UNCOND("   seed              = " << seed);
  NS_LOG_UNCOND("   logLevel          = " << logLevel);
  NS_LOG_UNCOND("============================================================");

  LogLevel ns3LogLevel = LOG_LEVEL_ERROR;
  switch (logLevel) {
  case 0:
    ns3LogLevel = LOG_LEVEL_ERROR;
    break;
  case 1:
    ns3LogLevel = LOG_LEVEL_WARN;
    break;
  case 2:
    ns3LogLevel = LOG_LEVEL_INFO;
    break;
  case 3:
    ns3LogLevel = LOG_LEVEL_DEBUG;
    break;
  case 4:
    ns3LogLevel = LOG_LEVEL_LOGIC;
    break;
  default:
    std::cerr << "Invalid logLevel “" << logLevel << "”. Using INFO (3) by default.\n";
    ns3LogLevel = LOG_LEVEL_INFO;
    break;
  }
  LogComponentEnable("ZigbeeNwk", ns3LogLevel);
  LogComponentEnable("ZigbeeRouting", ns3LogLevel);
  // LogComponentEnable("WifiPhy", ns3LogLevel);
  // LogComponentEnable("WifiMac", ns3LogLevel);
  // LogComponentEnable("ConstantRateWifiManager", ns3LogLevel);
  // LogComponentEnable("UdpServer", ns3LogLevel);
  // LogComponentEnable("UdpSocketImpl", ns3LogLevel);

  RngSeedManager::SetSeed(seed);
  RngSeedManager::SetRun(rngRun);

  NodeContainer wifiApNodes;
  wifiApNodes.Create(1);

  NodeContainer wifiStaNodes;
  wifiStaNodes.Create(3);

  NodeContainer zigbeeNodes;
  zigbeeNodes.Create(5);

  //// Configure MAC
  LrWpanHelper lrWpanHelper;
  NetDeviceContainer lrwpanDevices = lrWpanHelper.Install(zigbeeNodes);
  Ptr<LrWpanNetDevice> dev0 = lrwpanDevices.Get(0)->GetObject<LrWpanNetDevice>();
  Ptr<LrWpanNetDevice> dev1 = lrwpanDevices.Get(1)->GetObject<LrWpanNetDevice>();
  Ptr<LrWpanNetDevice> dev2 = lrwpanDevices.Get(2)->GetObject<LrWpanNetDevice>();
  Ptr<LrWpanNetDevice> dev3 = lrwpanDevices.Get(3)->GetObject<LrWpanNetDevice>();
  Ptr<LrWpanNetDevice> dev4 = lrwpanDevices.Get(4)->GetObject<LrWpanNetDevice>();

  // Device must ALWAYS have IEEE Address (Extended address) assigned.
  // Network address (short address) are assigned by the the JOIN mechanism
  dev0->GetMac()->SetExtendedAddress("00:00:00:00:00:00:CA:FE");
  dev1->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:01");
  dev2->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:02");
  dev3->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:03");
  dev4->GetMac()->SetExtendedAddress("00:00:00:00:00:00:00:04");

  // Configure channel and loss models
  Ptr<SpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel>();
  channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
  channel->AddPropagationLossModel(CreateObject<LogDistancePropagationLossModel>());
  {
    auto nak = CreateObject<NakagamiPropagationLossModel>();
    nak->SetAttribute("m0", DoubleValue(1.0));
    nak->SetAttribute("m1", DoubleValue(3.0));
    nak->SetAttribute("m2", DoubleValue(3.0));
    channel->AddPropagationLossModel(nak);
  }

  dev0->SetChannel(channel);
  dev1->SetChannel(channel);
  dev2->SetChannel(channel);
  dev3->SetChannel(channel);
  dev4->SetChannel(channel);

  // Configure WiFi
  SpectrumWifiPhyHelper wifiPhyHelper;
  wifiPhyHelper.SetChannel(channel);
  wifiPhyHelper.Set("ChannelSettings", StringValue("{6," + std::to_string(wifiChannelWidth) + ", BAND_2_4GHZ, 0}"));

  WifiHelper wifiHelper;
  wifiHelper.SetStandard(WIFI_STANDARD_80211n);
  wifiHelper.SetRemoteStationManager("ns3::MinstrelHtWifiManager");

  WifiMacHelper wifiMacHelper;
  Ssid ssid = Ssid("wifi-coex");

  wifiMacHelper.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
  NetDeviceContainer staDev = wifiHelper.Install(wifiPhyHelper, wifiMacHelper, wifiStaNodes);

  wifiMacHelper.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
  NetDeviceContainer apDev = wifiHelper.Install(wifiPhyHelper, wifiMacHelper, wifiApNodes);

  //// Configure NWK

  ZigbeeHelper zigbee;
  ZigbeeStackContainer zigbeeStackContainer = zigbee.Install(lrwpanDevices);

  Ptr<ZigbeeStack> zstack0 = zigbeeStackContainer.Get(0)->GetObject<ZigbeeStack>();
  Ptr<ZigbeeStack> zstack1 = zigbeeStackContainer.Get(1)->GetObject<ZigbeeStack>();
  Ptr<ZigbeeStack> zstack2 = zigbeeStackContainer.Get(2)->GetObject<ZigbeeStack>();
  Ptr<ZigbeeStack> zstack3 = zigbeeStackContainer.Get(3)->GetObject<ZigbeeStack>();
  Ptr<ZigbeeStack> zstack4 = zigbeeStackContainer.Get(4)->GetObject<ZigbeeStack>();

  // Add the stacks to a container to later on print routes.
  zigbeeStacks.Add(zstack0);
  zigbeeStacks.Add(zstack1);
  zigbeeStacks.Add(zstack2);
  zigbeeStacks.Add(zstack3);
  zigbeeStacks.Add(zstack4);

  // Assign streams to the zigbee stacks to obtain
  // reprodusable results from random events occurring inside the stack.
  zstack0->GetNwk()->AssignStreams(0);
  zstack1->GetNwk()->AssignStreams(10);
  zstack2->GetNwk()->AssignStreams(20);
  zstack3->GetNwk()->AssignStreams(30);
  zstack4->GetNwk()->AssignStreams(40);

  // Zigbee nodes position
  Ptr<ConstantPositionMobilityModel> dev0Mobility = CreateObject<ConstantPositionMobilityModel>();
  dev0Mobility->SetPosition(Vector(0, 0, 0));
  dev0->GetPhy()->SetMobility(dev0Mobility);

  Ptr<ConstantPositionMobilityModel> dev1Mobility = CreateObject<ConstantPositionMobilityModel>();
  dev1Mobility->SetPosition(Vector(10, 0, 0));
  dev1->GetPhy()->SetMobility(dev1Mobility);

  Ptr<ConstantPositionMobilityModel> dev2Mobility = CreateObject<ConstantPositionMobilityModel>();
  dev2Mobility->SetPosition(Vector(20, 0, 0));
  dev2->GetPhy()->SetMobility(dev2Mobility);

  Ptr<ConstantPositionMobilityModel> dev3Mobility = CreateObject<ConstantPositionMobilityModel>();
  dev3Mobility->SetPosition(Vector(30, 0, 0));
  dev3->GetPhy()->SetMobility(dev3Mobility);

  Ptr<ConstantPositionMobilityModel> dev4Mobility = CreateObject<ConstantPositionMobilityModel>();
  dev4Mobility->SetPosition(Vector(10, 10, 0));
  dev4->GetPhy()->SetMobility(dev4Mobility);

  Ptr<ConstantPositionMobilityModel> apMob = CreateObject<ConstantPositionMobilityModel>();
  apMob->SetPosition(Vector(15.0, 0.0, 0));
  wifiApNodes.Get(0)->AggregateObject(apMob);
  Ptr<WifiNetDevice> apWifiDev = apDev.Get(0)->GetObject<WifiNetDevice>();
  apWifiDev->GetPhy()->SetMobility(apMob);

  Ptr<ConstantPositionMobilityModel> sta0mob = CreateObject<ConstantPositionMobilityModel>();
  sta0mob->SetPosition(Vector(0.0, 10.0, 0));
  wifiStaNodes.Get(0)->AggregateObject(sta0mob);
  Ptr<WifiNetDevice> sta0dev = staDev.Get(0)->GetObject<WifiNetDevice>();
  sta0dev->GetPhy()->SetMobility(sta0mob);

  Ptr<ConstantPositionMobilityModel> sta1mob = CreateObject<ConstantPositionMobilityModel>();
  sta1mob->SetPosition(Vector(-5.0, 0.0, 0));
  wifiStaNodes.Get(1)->AggregateObject(sta1mob);
  Ptr<WifiNetDevice> sta1dev = staDev.Get(1)->GetObject<WifiNetDevice>();
  sta1dev->GetPhy()->SetMobility(sta1mob);

  Ptr<ConstantPositionMobilityModel> sta2mob = CreateObject<ConstantPositionMobilityModel>();
  sta2mob->SetPosition(Vector(15, 5.0, 0));
  wifiStaNodes.Get(2)->AggregateObject(sta2mob);
  Ptr<WifiNetDevice> sta2dev = staDev.Get(2)->GetObject<WifiNetDevice>();
  sta2dev->GetPhy()->SetMobility(sta2mob);

  // NWK callbacks hooks
  // These hooks are usually directly connected to the APS layer
  // In this case, there is no APS layer, therefore, we connect the event outputs
  // of all devices directly to our static functions in this example.

  zstack0->GetNwk()->SetNlmeNetworkFormationConfirmCallback(MakeBoundCallback(&NwkNetworkFormationConfirm, zstack0));
  zstack0->GetNwk()->SetNlmeRouteDiscoveryConfirmCallback(MakeBoundCallback(&NwkRouteDiscoveryConfirm, zstack0));

  zstack0->GetNwk()->SetNldeDataIndicationCallback(MakeBoundCallback(&NwkDataIndication, zstack0));
  zstack1->GetNwk()->SetNldeDataIndicationCallback(MakeBoundCallback(&NwkDataIndication, zstack1));
  zstack2->GetNwk()->SetNldeDataIndicationCallback(MakeBoundCallback(&NwkDataIndication, zstack2));
  zstack3->GetNwk()->SetNldeDataIndicationCallback(MakeBoundCallback(&NwkDataIndication, zstack3));
  zstack4->GetNwk()->SetNldeDataIndicationCallback(MakeBoundCallback(&NwkDataIndication, zstack4));

  zstack1->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack1));
  zstack2->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack2));
  zstack3->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack3));
  zstack4->GetNwk()->SetNlmeNetworkDiscoveryConfirmCallback(MakeBoundCallback(&NwkNetworkDiscoveryConfirm, zstack4));

  zstack1->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack1));
  zstack2->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack2));
  zstack3->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack3));
  zstack4->GetNwk()->SetNlmeJoinConfirmCallback(MakeBoundCallback(&NwkJoinConfirm, zstack4));

  // 1 - Initiate the Zigbee coordinator, start the network
  // ALL_CHANNELS = 0x07FFF800 (Channels 11~26)
  NlmeNetworkFormationRequestParams netFormParams;
  netFormParams.m_scanChannelList.channelPageCount = 1;
  netFormParams.m_scanChannelList.channelsField[0] = ALL_CHANNELS;
  netFormParams.m_scanDuration = 0;
  netFormParams.m_superFrameOrder = 15;
  netFormParams.m_beaconOrder = 15;

  Simulator::ScheduleWithContext(zstack0->GetNode()->GetId(), Seconds(1), &ZigbeeNwk::NlmeNetworkFormationRequest,
                                 zstack0->GetNwk(), netFormParams);

  // 2- Schedule devices sequentially find and join the network.
  //    After this procedure, each device make a NLME-START-ROUTER.request to become a router

  NlmeNetworkDiscoveryRequestParams netDiscParams;
  netDiscParams.m_scanChannelList.channelPageCount = 1;
  netDiscParams.m_scanChannelList.channelsField[0] = 0x00007800; // BitMap: Channels 11~14
  netDiscParams.m_scanDuration = 2;
  Simulator::ScheduleWithContext(zstack1->GetNode()->GetId(), Seconds(3), &ZigbeeNwk::NlmeNetworkDiscoveryRequest,
                                 zstack1->GetNwk(), netDiscParams);

  NlmeNetworkDiscoveryRequestParams netDiscParams2;
  netDiscParams2.m_scanChannelList.channelPageCount = 1;
  netDiscParams2.m_scanChannelList.channelsField[0] = 0x00007800; // BitMap: Channels 11~14
  netDiscParams2.m_scanDuration = 2;
  Simulator::ScheduleWithContext(zstack2->GetNode()->GetId(), Seconds(4), &ZigbeeNwk::NlmeNetworkDiscoveryRequest,
                                 zstack2->GetNwk(), netDiscParams2);

  NlmeNetworkDiscoveryRequestParams netDiscParams3;
  netDiscParams2.m_scanChannelList.channelPageCount = 1;
  netDiscParams2.m_scanChannelList.channelsField[0] = 0x00007800; // BitMap: Channels 11~14
  netDiscParams2.m_scanDuration = 2;
  Simulator::ScheduleWithContext(zstack3->GetNode()->GetId(), Seconds(5), &ZigbeeNwk::NlmeNetworkDiscoveryRequest,
                                 zstack3->GetNwk(), netDiscParams3);

  NlmeNetworkDiscoveryRequestParams netDiscParams4;
  netDiscParams4.m_scanChannelList.channelPageCount = 1;
  netDiscParams4.m_scanChannelList.channelsField[0] = 0x00007800; // BitMap: Channels 11~14
  netDiscParams4.m_scanDuration = 2;
  Simulator::ScheduleWithContext(zstack4->GetNode()->GetId(), Seconds(6), &ZigbeeNwk::NlmeNetworkDiscoveryRequest,
                                 zstack4->GetNwk(), netDiscParams4);

  // Install WiFi Stack
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
  wifiSinkApp.Start(Seconds(0));
  wifiSinkApp.Stop(Seconds(simulationTime));

  // Wifi traffic app
  ApplicationContainer wifiTrafficApps;
  OnOffHelper wifiTrafficApp("ns3::UdpSocketFactory", InetSocketAddress("10.0.0.1", wifiPort));
  wifiTrafficApp.SetAttribute("DataRate", DataRateValue(wifiDataRate));
  wifiTrafficApp.SetAttribute("PacketSize", UintegerValue(wifiPacketSize));
  for (uint32_t i = 0; i < wifiStaNodes.GetN(); i++) {
    ApplicationContainer app = wifiTrafficApp.Install(wifiStaNodes.Get(i));
    app.Start(Seconds(16));
    app.Stop(Seconds(16 + simulationTime));
    wifiTrafficApps.Add(app);
  }

  Simulator::Schedule(Seconds(16), &SendDataPeriod, zstack0, zstack1, heartbeatInterval);
  Simulator::Schedule(Seconds(16.2), &SendDataPeriod, zstack0, zstack2, heartbeatInterval);
  Simulator::Schedule(Seconds(16.4), &SendDataPeriod, zstack0, zstack3, heartbeatInterval);
  Simulator::Schedule(Seconds(16.6), &SendDataPeriod, zstack0, zstack4, heartbeatInterval);

  Simulator::Stop(Seconds(simulationTime));

  FlowMonitorHelper flowHelper;
  Ptr<FlowMonitor> flowMonitor = flowHelper.InstallAll();

  Simulator::Run();

  PrintWifiFlowStats(flowHelper, flowMonitor);
  PrintZigbeeQoS();

  Simulator::Destroy();
  return 0;
}