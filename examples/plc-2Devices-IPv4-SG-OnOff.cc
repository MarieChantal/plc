/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012 University of British Columbia, Vancouver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Alexander Schloegl <alexander.schloegl@gmx.de>
 */

#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>
#include <string>

#include <ns3/on-off-helper.h>
#include <ns3/onoff-application.h>
#include <ns3/applications-module.h>
#include <ns3/core-module.h>
#include <ns3/nstime.h>
#include <ns3/node.h>
#include <ns3/network-module.h>
#include <ns3/simulator.h>
#include <ns3/internet-stack-helper.h>
#include <ns3/internet-module.h>
#include <ns3/packet-sink-helper.h>
#include <ns3/udp-client-server-helper.h>
#include <ns3/traffic-control-helper.h>
#include <ns3/ipv4-address-helper.h>
#include <ns3/output-stream-wrapper.h>
#include <ns3/plc.h>
#include <ns3/flow-monitor-module.h>
#include <ns3/flow-monitor-helper.h>


using namespace ns3;


/* Type definitions */
struct PLC_node_App {
  Ptr<Application> srcApp;
  Ptr<PacketSink> packetSink;
  uint64_t totalRx = 0;
  double throughput = 0;
  Time startTime;
  Time stopTime;
};

typedef std::map<Ptr<PLC_Node>, PLC_node_App> PLC_node_App_List;
typedef PLC_node_App_List::iterator PLC_node_App_List_I;
typedef PLC_node_App_List::const_iterator PLC_node_App_List_CI;

/** Application variables **/
uint32_t ackNb = 0;
uint32_t pkNb = 0;
uint32_t rcvNb = 0;
uint64_t delayMean = 0;
uint64_t increment =0;
Ptr<ns3::Socket> sink;
uint64_t totalRx = 0;
double avgthroughput = 0;
double throughput = 0;
Ptr<PacketSink> sink1;
double LastRcv = 0;
double FirstRcv = 0;

PLC_node_App_List plc_node_App_List;    /* List of PLC devices. */

Ptr<PacketSink> sinkF;                         /* Pointer to the packet sink application */
uint64_t lastTotalRx = 0;                     /* The value of the last total received bytes */

void
CalculateThroughput ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */
  double cur = (sinkF->GetTotalRx() - lastTotalRx) * (double) 8/1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;
  lastTotalRx = sinkF->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput);
}

void
ShowTime (void)
{

	std::cout << Simulator::Now ().GetSeconds () << " Rx: " << sink1->GetTotalRx () << std::endl; //TO ADD
	if (totalRx != sink1->GetTotalRx () ){
		totalRx = sink1->GetTotalRx ();
		LastRcv= Simulator::Now ().GetSeconds ();
	}
	else{
		if (LastRcv!= 0 && FirstRcv == 0){
			FirstRcv= Simulator::Now ().GetSeconds ();
		}
	}
	Simulator::Schedule (MilliSeconds (100), &ShowTime);
}

////////////////

void
SinkRx (Ptr<const Packet> p, const Address &ad)
{
  std::cout << "/////////////////////////////RCV: " << *p << std::endl;
}

void
sendPacket (Ptr<ns3::Socket> socket, Ptr<Packet> packet)
{
	socket->Send(packet);
}

void
ReceivedACK(void)
{
	ackNb++;
	NS_LOG_UNCOND(Simulator::Now() << ": ACK received!");
}
void
ReceivedData (Ptr<Packet> p, Mac48Address addr1 , Mac48Address addr2)
{
	    increment++;
		NS_LOG_UNCOND(Simulator::Now().GetInteger () << ": Received Data! from " << addr1 << " to: " << addr2 << " Delay: " << (Simulator::Now().GetInteger () - (increment * 1000000000))/1000000.0 << "ms");
		rcvNb++;
		delayMean = delayMean + (Simulator::Now().GetInteger () - (increment * 1000000000));
}

int main (int argc, char *argv[])
{
	uint32_t nodeUserNumber = 1;//55
	uint32_t nodeDistance = 10;//in km
	std::string dataRateUDP = "20.0kbps";

	// Enable logging
	//LogComponentEnableAll(LOG_PREFIX_TIME);
	//LogComponentEnable("PLC_Mac", LOG_LEVEL_FUNCTION);
	//LogComponentEnable ("OnOffApplication", LOG_LEVEL_FUNCTION);
	//LogComponentEnable ("PacketSink", LOG_LEVEL_FUNCTION);
	//LogComponentEnable("PLC_Channel", LOG_LEVEL_FUNCTION);
	//LogComponentEnable("PLC_SimulatorImpl", LOG_LEVEL_FUNCTION);
	//LogComponentEnable("FlowMonitor", LOG_LEVEL_FUNCTION);
	//LogComponentEnable("PLC_Phy", LOG_LEVEL_LOGIC);
	//LogComponentEnable("PLC_LinkPerformanceModel", LOG_LEVEL_FUNCTION);
	//LogComponentEnable("PLC_ErrorModel", LOG_LEVEL_FUNCTION);
	//LogComponentEnable("UdpServer", LOG_LEVEL_FUNCTION);

	//LogComponentEnable("PLC_Node", LOG_LEVEL_FUNCTION);
	//LogComponentEnable("PLC_Interference", LOG_LEVEL_FUNCTION);
	//LogComponentEnable("PLC_Interface", LOG_LEVEL_FUNCTION);
	//LogComponentEnable("PLC_Cable", LOG_LEVEL_FUNCTION);
	//LogComponentEnable("PLC_BackboneBranch", LOG_LEVEL_FUNCTION);

	// Enable packet printing
	//Packet::EnablePrinting();

	/* Command line argument parser setup. */
	CommandLine cmd;

	cmd.AddValue ("nodeNb", "Number of nodes", nodeUserNumber);
	cmd.AddValue ("dist", "Distance between 2 nodes", nodeDistance);
	cmd.AddValue ("rate", "Datarate", dataRateUDP);
	cmd.Parse (argc, argv);

	// Initialize physical environment
	PLC_Time::SetTimeModel(50, MicroSeconds(735)); // 60Hz main frequency, symbol length 735us (G3) (defines time granularity)
	// Define spectrum model
    PLC_SpectrumModelHelper smHelper;
    Ptr<const SpectrumModel> sm;
    //sm = smHelper.GetSpectrumModel(0e6,30e6, 1024);//0,10e6, 100
    sm = smHelper.GetG3SpectrumModel();
	// Define transmit power spectral density
    Ptr<SpectrumValue> txPsd = Create<SpectrumValue> (sm);
    (*txPsd) = 1e-8; // -50dBm/Hz 1e-8

    // Create cable type
	//Ptr<PLC_Cable> cable = CreateObject<PLC_NAYY150SE_Cable> (sm);

	Ptr<PLC_Cable> mainCable = CreateObject<PLC_MV_Overhead_Cable> (sm);//PLC_MV_Overhead_Cable
	Ptr<PLC_Cable> houseCable = CreateObject<PLC_NAYY150SE_Cable> (sm);//PLC_NAYY150SE_Cable
	Ptr<PLC_Cable> houseCable2 = CreateObject<PLC_AL3x95XLPE_Cable> (sm);//PLC_NAYY150SE_Cable

	// Create nodes
	Ptr<PLC_Node> n1 = CreateObject<PLC_Node> ();
	Ptr<PLC_Node> n2 = CreateObject<PLC_Node> ();

	//std::vector<  Ptr<PLC_Node>  > plc_nodes_f;
	//plc_nodes_f.resize(uint32_t (nodeUserNumber),0);

	/*for  (uint32_t n = 0; n < nodeUserNumber; n++)
			{
				plc_nodes_f[n] = CreateObject<PLC_Node> ();
			}*/

	n1->SetPosition(0.0	, 0.0,0.0); //Concentrator

	//plc_nodes_f[0]->SetPosition(	0.0	,	5000.0	,0.0);
	n2->SetPosition( 0.0, nodeDistance * 1000, 0.0);
	n1->SetName("Node1");
	n2->SetName("Node2");
	/*for  (uint32_t n = 0; n < nodeUserNumber; n++)
				{
					//std::cout << " ///// Name: " <<  " Node"+std::to_string(n+2) << std::endl;
					plc_nodes_f[n]->SetName("Node"+std::to_string(n+2));
				}*/

	PLC_NodeList nodes;
	nodes.push_back(n1);
	nodes.push_back(n2);

	/*for  (uint32_t n = 0; n < nodeUserNumber; n++)
					{
						nodes.push_back(plc_nodes_f[n]);
					}*/

	//Link distribution nodes to nodes

	CreateObject<PLC_Line> (houseCable, n1, n2);

	// Setup channel
	PLC_ChannelHelper channelHelper(sm);
	channelHelper.Install(nodes);
	//channelHelper.Install(dist_nodes);
	Ptr<PLC_Channel> channel = channelHelper.GetChannel();

	// Create outlets

	Ptr<PLC_Outlet> o1 = CreateObject<PLC_Outlet> (n1, Create<PLC_ConstImpedance> (sm, PLC_Value(50,0)));
	Ptr<PLC_Outlet> o2 = CreateObject<PLC_Outlet> (n2, Create<PLC_ConstImpedance> (sm, PLC_Value(50,0)));


	// Set background noise
    Ptr<SpectrumValue> noiseFloor = CreateWorstCaseBgNoise(sm)->GetNoisePsd();//CreateWorstCaseBgNoise
	//Ptr<SpectrumValue> noiseFloor = Create<SpectrumValue> (sm);
	//(*noiseFloor) = 0.000000001;


    //Create net devices

    n1->SetImpedance(Create<PLC_ConstImpedance> (sm, PLC_Value(250,0)));
    n2->SetImpedance(Create<PLC_ConstImpedance> (sm, PLC_Value(250,0)));
    //plc_nodes_f[0]->SetImpedance(Create<PLC_ConstImpedance> (sm, PLC_Value(250,0)));
    PLC_NetDeviceHelper deviceHelper(sm, txPsd, nodes);
    deviceHelper.DefinePhyType(PLC_InformationRatePhy::GetTypeId());//PLC_InformationRatePhy
    deviceHelper.DefineMacType(PLC_ArqMac::GetTypeId());
    deviceHelper.SetNoiseFloor(noiseFloor);
    deviceHelper.SetTxImpedance(Create<PLC_ConstImpedance> (sm, PLC_Value(40,0)));
    deviceHelper.SetRxImpedance(Create<PLC_ConstImpedance> (sm, PLC_Value(100,0)));

    // Set modulation and coding scheme
    deviceHelper.SetHeaderModulationAndCodingScheme(ModulationAndCodingScheme(QPSK_1_2,0));//BPSK_1_4
    deviceHelper.SetPayloadModulationAndCodingScheme(ModulationAndCodingScheme(QPSK_1_2,0));//BPSK_1_2

    deviceHelper.Setup();

    PLC_NetdeviceMap devMap = deviceHelper.GetNetdeviceMap();

	// Get ns-3 nodes
    NodeContainer n = deviceHelper.GetNS3Nodes();

    //Install ipv4 stack on nodes
    InternetStackHelper inetHelper;
    inetHelper.SetIpv4StackInstall(1);
    inetHelper.Install(n);

	// Get net devices
    NetDeviceContainer d = deviceHelper.GetNetDevices();



    //NetDeviceContainer plcDevices = GetNetDevices ();

      Ptr<NetDevice> netDevice;
      for (NetDeviceContainer::Iterator i = d.Begin (); i != d.End (); ++i)
        {
          netDevice = (*i);
          channel->AddDevice (netDevice);
        }


    // Calculate channels
       	  	channel->InitTransmissionChannels();
       	  	channel->CalcTransmissionChannels();

       	  	std::cout << "Channel Calculated" << std::endl;


    // Assign IP addresses
    Ipv4AddressHelper ipv4AddrHelper;
    ipv4AddrHelper.SetBase ("192.168.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifCntr;
    ifCntr = ipv4AddrHelper.Assign (d);
    Address serverAddress;//ADD
    serverAddress = Address (ifCntr.GetAddress (1));//ADD

    uint16_t dst_id = 0;
      //auto devMap = devHelper.GetNetdeviceMap();

      Ptr<Node> destinationNode = devMap[nodes.at (dst_id)->GetName ()]->GetObject<PLC_NetDevice> ()->GetNode ();

      NodeContainer c;
      uint16_t id = 1;

      Ptr<Node> sourceNode = devMap[nodes.at (id)->GetName ()]->GetObject<PLC_NetDevice> ()->GetNode ();
      c.Add (sourceNode);

    /* Install Simple UDP Server on Concentrator */
      PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), 9000));
      ApplicationContainer sinkApp = sinkHelper.Install (destinationNode);
      sink1 = StaticCast<PacketSink> (sinkApp.Get (0));
      sinkApp.Start (Seconds (0.0));
      sinkApp.Stop (Seconds (20.0));


      /* Install UDP Transmitter on the Smart meters 1 */
      ApplicationContainer srcApp;
      OnOffHelper src ("ns3::UdpSocketFactory", InetSocketAddress (ifCntr.GetAddress (0), 9000));
      src.SetAttribute ("MaxBytes", UintegerValue (0));//0
      src.SetAttribute ("PacketSize", UintegerValue (1024));
      src.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1e6]"));
      src.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
      src.SetAttribute ("DataRate", DataRateValue (DataRate (dataRateUDP)));//125max

      std::cout << "install First Smart meter starting app at: 2.0s" << std::endl;
      srcApp = src.Install (c);
      //std::cout << "install First Smart meter starting app at: " << x << std::endl;


      srcApp.Start (Seconds (1.0));//x
      srcApp.Stop (Seconds (20.0));//24

    //End ipv4

    /* Install FlowMonitor on all nodes */
        FlowMonitorHelper flowmon;
        Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
        std::cout << "FlowMon installed" << std::endl;


	//Config::ConnectWithoutContext ("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx", MakeCallback (&SinkRx));
	Simulator::Schedule (Seconds (0.1), &ShowTime);

	//time_t start, end;
    //time (&start);
    Simulator::Stop (Seconds (50.0));
    //std::cout << "Time STOP: " << time_to_send << "s" << std::endl;
    // Initialize simulation
    Simulator::Run();

	// Cleanup simulation
	Simulator::Destroy();
	//time (&end);
	//double dif = difftime (end, start);
	//NS_LOG_UNCOND ("Simulation Execution Time: " << dif << "s");
	std::cout << "  1st pkt rcv at:  " << FirstRcv <<"  Last pkt rcv at:  " << LastRcv << std::endl;

	uint32_t packetlossinflows = 0;
	/* Print per flow statistics */

	monitor->CheckForLostPackets ();
	monitor->SerializeToXmlFile("plc.flowmon", true, true);
	          monitor->CheckForLostPackets ();
	          Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
	          FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
	          for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
	            {
	              Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
	              std::cout << "Flow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")" << std::endl;
	              std::cout << "  Tx Packets: " << i->second.txPackets << std::endl;
	              std::cout << "  Tx Bytes:   " << i->second.txBytes << std::endl;
	             // std::cout << "  TxOffered:  " << i->second.txBytes *8.0 / ((dif-1)*1e6) << "Mbps (WRONG)" <<  std::endl;
	              std::cout << "  Rx Packets: " << i->second.rxPackets << std::endl;
	             // std::cout << "  Rx Bytes:   " << i->second.rxBytes *8.0 / ((dif-1)*1e6) << "Mbps (WRONG)" << std::endl;
	              std::cout << "  Throughput: " << i->second.rxBytes*8.0 / (((LastRcv) - (2.0))*1e3) << "kbps ()" <<  std::endl;
	              std::cout << "  Sum Jitter:  " << i->second.jitterSum << std::endl;

	              if (i->second.rxPackets != 0)
	                    {
	                        std::cout << "  AverageDelay: " << i->second.delaySum/i->second.rxPackets << std::endl;
	                    }
	              else
	                    {
	                        std::cout << "  AverageDelay: " << 0 << std::endl;
	                    }
	              std::cout << "  Packet Loss:   " << 100*((i->second.txPackets-i->second.rxPackets)*1.0)/i->second.txPackets << "%" << std::endl;
	              if (i->second.txPackets-i->second.rxPackets != 0)
	              	  	  	  {
	            	  	  	  	  	  packetlossinflows++;
	              	                    }
	            }


	          std::cout << "Number of flows with packet loss: " << packetlossinflows << std::endl;

	          for (PLC_node_App_List_I it = plc_node_App_List.begin (); it != plc_node_App_List.end (); it++)
	          			{
	        	  std::cout << "Stop: " << it->second.stopTime << std::endl;

	        	  std::cout <<	"Start: " <<  it->second.startTime << std::endl;


	          			}
	return EXIT_SUCCESS;
}

