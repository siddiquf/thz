// 28—Works great for basic exhaustive scheme for config # 28

/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2021 Northeastern University (https://unlab.tech/)
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
 * Author: Qing Xia <qingxia@buffalo.edu>
 *         Zahed Hossain <zahedhos@buffalo.edu>
 *         Josep Miquel Jornet <jmjornet@buffalo.edu>
 *         Daniel Morales <danimoralesbrotons@gmail.com>
 *
 *  Modified by: Farhan Siddiqui <farhansi@gmail.com>
             Bikash Muzamdar <bikashmazumdar2000@gmail.com>
 */

#include "ns3/attribute.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/log.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/node.h"
#include "ns3/core-module.h"
#include "thz-mac-header.h"
#include "thz-mac-macro-client.h"
#include "thz-phy-macro.h"
#include "thz-dir-antenna.h"
#include "thz-net-device.h"
#include <vector>
#include <iostream>
#include <iterator>
#include <string>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <stdio.h>
#include <time.h>
#include <sstream>
#include <math.h> //added

NS_LOG_COMPONENT_DEFINE ("THzMacMacroClient");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (THzMacMacroClient);

THzMacMacroClient::THzMacMacroClient ()
  : THzMac (),
  m_phy (0),
  m_state (AP_DISCOVERY),
  m_ctsTimeoutEvent (),
  m_ackTimeoutEvent (),
  m_sendDataEvent (),
  m_LinkTimeoutEvent(),
  m_pktData (0)

{
  m_ctsReceived = 0;
  m_backoffSeq = 0;
  m_nav = Simulator::Now ();
  m_localNav = Simulator::Now ();
  m_ite = 0;
  m_throughputAll = 0;
  m_state = AP_DISCOVERY; //initial state to enable AP discovery
  m_discard = 0;
  m_send = 0;
  m_rxIniAngle = 0;
  m_sector = -1;
  m_result.clear ();
  m_rtsAnswered = true;
  m_angle = 0;
  rounds=0; 
  dir = 1; 
  m_tLink = Seconds(0.001);
  ctaCount=0;
  time_cta_rcvd_2 = Seconds(0);
  link_loss_time = Seconds(0);
  time_rediscovery = Seconds(0);
  m_discovery_count = 0;
  Simulator::ScheduleNow(&THzMacMacroClient::InitVariables, this);
}

THzMacMacroClient::~THzMacMacroClient ()
{
  Clear ();
}

void
THzMacMacroClient::Clear ()
{
  m_pktTx = 0;
  m_pktData = 0;
  m_pktQueue.clear ();
  m_seqList.clear ();
  m_pktRec = 0;
  m_throughput = 0;
  m_throughputAll = 0;
}

TypeId
THzMacMacroClient::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::THzMacMacroClient")
    .SetParent<THzMac> ()
    .AddConstructor<THzMacMacroClient> ()
    .AddAttribute ("HandshakeWays",
                   "Number of control packets interchanged as handshake",
                   UintegerValue (3),
                   MakeUintegerAccessor (&THzMacMacroClient::m_ways),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("BoSlots",
                   "Slots for Start Backoff",
                   UintegerValue (5),
                   MakeUintegerAccessor (&THzMacMacroClient::m_boSlots),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("SlotTime",
                   "Time slot duration for MAC backoff",
                   TimeValue (NanoSeconds (2)),
                   MakeTimeAccessor (&THzMacMacroClient::m_slotTime),
                   MakeTimeChecker ())
    .AddAttribute ("SlotTime3way",
                   "Time slot duration for MAC backoff for 3-way",
                   TimeValue (NanoSeconds (2)),
                   MakeTimeAccessor (&THzMacMacroClient::m_slotTime_3way),
                   MakeTimeChecker ())
    .AddAttribute ("SifsTime",
                   "Short Inter-frame Space",
                   TimeValue (PicoSeconds (0)),
                   MakeTimeAccessor (&THzMacMacroClient::m_sifs),
                   MakeTimeChecker ())
    .AddAttribute ("DifsTime",
                   "DFS Inter-frame Space",
                   TimeValue (PicoSeconds (0)),
                   MakeTimeAccessor (&THzMacMacroClient::m_difs),
                   MakeTimeChecker ())
    .AddAttribute ("QueueLimit",
                   "Maximum packets to queue at MAC",
                   UintegerValue (10000),
                   MakeUintegerAccessor (&THzMacMacroClient::m_queueLimit),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("RtsRetryLimit",
                   "Maximum Limit for RTS Retransmission",
                   UintegerValue (7),
                   MakeUintegerAccessor (&THzMacMacroClient::m_rtsRetryLimit),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("DataRetryLimit",
                   "Maximum Limit for Data Retransmission",
                   UintegerValue (5),
                   MakeUintegerAccessor (&THzMacMacroClient::m_dataRetryLimit),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("OutputFile",
                   "name of the output file",
                   StringValue ("result_macro-central.txt"),
                   MakeStringAccessor (&THzMacMacroClient::outputFile),
                   MakeStringChecker ())
    .AddAttribute ("PacketSize",
                   "Minimum packet size",
                   UintegerValue (15000),
                   MakeUintegerAccessor (&THzMacMacroClient::m_MinEnquePacketSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("DataRate",
                   "name of the output file",
                   DoubleValue (148.01e9),
                   MakeDoubleAccessor (&THzMacMacroClient::m_dataRate),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Radius",
                   "max supported radius",
                   DoubleValue (7.0),
                   MakeDoubleAccessor (&THzMacMacroClient::m_radius),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("PropDelay",
                   "name of the output file",
                   TimeValue (PicoSeconds(3333)),
                   MakeTimeAccessor (&THzMacMacroClient::m_tProp),
                   MakeTimeChecker ())
    .AddTraceSource ("CtsTimeout",
                     "Trace Hookup for CTS Timeout",
                     MakeTraceSourceAccessor (&THzMacMacroClient::m_traceCtsTimeout),
                     "ns3::THzMac::TimeTracedCallback")
    .AddTraceSource ("AckTimeout",
                     "Trace Hookup for ACK Timeout",
                     MakeTraceSourceAccessor (&THzMacMacroClient::m_traceAckTimeout),
                     "ns3::THzMac::TimeTracedCallback")
    .AddTraceSource ("SendDataDone",
                     "Trace Hookup for sending a data",
                     MakeTraceSourceAccessor (&THzMacMacroClient::m_traceSendDataDone),
                     "ns3::THzMac::SendDataDoneTracedCallback")
    .AddTraceSource ("Enqueue",
                     "Trace Hookup for enqueue a data",
                     MakeTraceSourceAccessor (&THzMacMacroClient::m_traceEnqueue),
                     "ns3::THzMac::TimeTracedCallback")
    .AddTraceSource ("Throughput",
                     "Trace Hookup for Throughput",
                     MakeTraceSourceAccessor (&THzMacMacroClient::m_traceThroughput),
                     "ns3::THzMac::ThroughputTracedCallback")
  ;
  return tid;
}

void
THzMacMacroClient::InitVariables (void) {

  m_tData = Seconds ((m_MinEnquePacketSize + 53) * 8 / m_dataRate);  // 53 bytes of overhead (48 B MAC + 5 B PHY)

  m_backoffActive = false;
  m_thzAD = m_device -> GetDirAntenna ();
  m_beamwidth = m_thzAD -> GetBeamwidth ();

  Ptr<UniformRandomVariable> unifRandom = CreateObject<UniformRandomVariable> ();
  unifRandom->SetAttribute ("Min", DoubleValue (0));
  unifRandom->SetAttribute ("Max", DoubleValue (360));
  int i_angle;
  int i_beamwidth;
 
  do {
    m_angle = unifRandom->GetInteger (0,360);
    i_angle = (int) m_angle;
    i_beamwidth = (int) m_beamwidth;
  } while ((i_angle % i_beamwidth) != 0);

  
  NS_LOG_UNCOND (" - Node " << m_device -> GetNode () -> GetId() << " initial angle " << m_angle << " client beamwidth " << m_beamwidth);
  m_thzAD->TuneTxOrientation (m_angle);
  
  m_thzAD -> SetBeamwidth (m_beamwidth);  // to set m_exponent

  m_nodeId = m_device -> GetNode () -> GetId ();
  m_thzAD->SetAttribute ("TuneRxTxMode", DoubleValue (0)); // set as transmitter
  m_thzAD->SetAttribute ("TxInitialAngle", DoubleValue (m_angle)); //must be same as  m_thzAD->TuneTxOrientation (m_angle);?
  m_clientMobility = m_device-> GetNode ()->GetObject<MobilityModel> ();

   Vector current_position = m_clientMobility->GetPosition ();
   NS_LOG_UNCOND (Simulator::Now() << " - Node " << m_nodeId << " init. X: " << current_position.x << " Y: " << current_position.y);


  /** Set antenna rotation parameters **/

     m_tSector = GetCtrlDuration (THZ_PKT_TYPE_CTA) + m_tProp + GetSifs() + GetMaxBackoff() +                                                                                                                         GetCtrlDuration (THZ_PKT_TYPE_RTS) + m_tProp + GetSifs() + GetMaxBackoff() +                                                                                                                         GetCtrlDuration (THZ_PKT_TYPE_CTS) + m_tProp + GetSifs() + GetMaxBackoff() +                                                                                                                                                    m_tData + m_tProp + GetSifs() +  GetMaxBackoff() +                                                                                                                       GetCtrlDuration (THZ_PKT_TYPE_ACK) + m_tProp + GetSifs() + GetMaxBackoff() ;


  m_nSector = 360 / m_beamwidth;
  m_tMaxCircle = m_nSector * m_tSector;
  /* Setting the antenna turning speed double of AP **/
  m_turningSpeed = (((double) 1 / (double) m_tMaxCircle.GetNanoSeconds()) * 1e9);
  m_thzAD -> SetTxTurningSpeed(m_turningSpeed);
  m_state = AP_DISCOVERY;
  NS_LOG_UNCOND ("Client : tSector: " << m_tSector << " tCircle: " << m_tMaxCircle << " turning speed " << m_turningSpeed << " m_nSector " << m_nSector);

  NS_LOG_UNCOND ("at node " << m_nodeId  << " now " << Simulator::Now () << " state: " << StateToString (m_state));

  PositionsRecord();
   NS_LOG_UNCOND ("Initial computed m_angle " << m_angle << " sector time: " << m_tSector << " m_beamwidth " << m_beamwidth);

  // Turn TX antenna
   Simulator::ScheduleNow(&THzMacMacroClient::TurnTxAntenna, this);
 
}
  
void
THzMacMacroClient::TurnTxAntenna (void) {

  NS_LOG_UNCOND("THzMacMacroClient::TurnTxAntenna " << Simulator::Now());
    
  if (m_state != AP_DISCOVERY ) {
    NS_LOG_UNCOND("Node " << m_nodeId  << " no longer in discovery mode - current state is  " << StateToString(m_state));
    return;
  }

  NS_LOG_UNCOND("Node " << m_nodeId << " m_tSector is " << m_tSector << " rounds = " << rounds);
  
  if (m_state == AP_DISCOVERY) {

    NS_LOG_UNCOND ("****Turn Tx Antenna at node " << m_nodeId << " now " << Simulator::Now () << " state: " << StateToString (m_state) << "****");
    NS_LOG_UNCOND ("Turn Tx Antenna angle " << m_angle << " sector:" << m_tSector << " m_beamwidth " << m_beamwidth);
   
    
    if ( rounds <= 10 && dir == 1) {
      m_angle = m_angle + m_beamwidth;
    }
    else if (rounds <= 10 && dir == -1) {
      m_angle = m_angle - m_beamwidth;
    }
    
   
    if(rounds > 10 && dir == 1) {
      m_angle = m_angle  + (m_beamwidth/2);
    }
    else if (rounds > 10 && dir == -1) {
      m_angle = m_angle - (m_beamwidth/2);
    }
    
    
    if (m_angle  >= 360) {
        rounds++;
         dir = -1;
     } else if (m_angle <= 0) {
        rounds++;
        dir = 1;
     }
      
  
    while (m_angle <= -360)
    {
      NS_LOG_UNCOND("in < -360");
      m_angle += 360;
      
    }

    while (m_angle > 360)
    {
      NS_LOG_UNCOND("in > 360");
      m_angle -= 360;
    }

    if (m_sectorTimeoutEvent.IsRunning()) {
      m_sectorTimeoutEvent.Cancel();
    }
    NS_LOG_UNCOND (Simulator::Now() << " - TX - ---------- turning to sector at " << m_angle  << " ---------- at node " << m_nodeId );
    m_thzAD->TuneTxOrientation (m_angle);  // turn to next sector
    NS_LOG_UNCOND ("Node " << m_nodeId  << " m_angle is " << m_angle);
   
    m_sectorTimeoutEvent = Simulator::Schedule (m_tSector ,&THzMacMacroClient::SectorTimeout, this);
  }
}



void
THzMacMacroClient::SectorTimeout ()
{
  NS_LOG_UNCOND (" THzMacMacroClient::SectorTimeout (): at node " << m_nodeId  << " now " << Simulator::Now () << " state: " << StateToString (m_state));
  if (m_state == AP_DISCOVERY )
  {
      NS_LOG_UNCOND(Simulator::Now() << " - CLIENT - ---------- SECTOR TIMEOUT. No CTA received from AP. Turning to next sector ");
      TurnTxAntenna();     
  }
  else {
    
      NS_LOG_UNCOND("Sim time now: " << Simulator::Now() << " Node " << m_nodeId << " no longer in discovery mode - now facing AP at angle  " << m_angle );
  }
  
}

void
  THzMacMacroClient::LocationChange (void) {

     m_discovery_count++;

     NS_LOG_UNCOND("locationchange Radius = " << m_radius);

     Vector current_position = m_clientMobility->GetPosition ();
     Vector position = m_clientMobility->GetPosition ();
     position.x = -position.x;
     position.y = -position.y;
     m_clientMobility->SetPosition (position);
     m_angle=0; rounds=0;
     Vector new_position = m_clientMobility->GetPosition ();
     if (new_position != current_position) {
       std::cout << "Location of node " << m_nodeId << " has changed to " << position.x << " " << position.y << "@ time" << Simulator::Now() << std::endl;
       NS_LOG_UNCOND (Simulator::Now() << " - current position of node " << m_nodeId << " init. X: " << current_position.x << " Y: " << current_position.y);
       NS_LOG_UNCOND (Simulator::Now() << " - new position of node " << m_nodeId << " init. X: " << new_position.x << " Y: " << new_position.y);
     }
}

void
THzMacMacroClient::LinkTimeout () {
  NS_LOG_UNCOND (" THzMacMacroClient::LinkTimeout at node " << m_nodeId << "at time " << Simulator::Now() << " m_tlink is " << m_tLink);
  link_loss_time =  Simulator::Now();
  m_state=AP_DISCOVERY;
  dir=1;
  Simulator::ScheduleNow(&THzMacMacroClient::TurnTxAntenna, this);
  }


bool
THzMacMacroClient::Enqueue (Ptr<Packet> packet, Mac48Address dest)
{
  
  NS_LOG_DEBUG("THzMacMacroClient::Enqueue");
  if (packet->GetSize () < m_MinEnquePacketSize)
    {
      m_pktQueue.push_front (packet);
      m_pktQueue.pop_front ();
    }
  else
    {
      THzMacHeader header = THzMacHeader (m_address, dest, THZ_PKT_TYPE_DATA);
      m_sequence++;
      header.SetSequence (m_sequence);
      packet->AddHeader (header);
      m_pktQueue.push_back (packet);

      Rec rec;
      rec.RecSize = packet->GetSize ();
      rec.RecTime = Simulator::Now ();
      rec.RecSeq = m_sequence;
      rec.RecRetry = 0;
      rec.Recpacket = packet;
      rec.BackoffLife = 0;
      m_rec.push_back (rec);
      NS_LOG_UNCOND (Simulator::Now() << " - " << m_nodeId << " - ***!!!*** Packet enqueued with size " << packet->GetSize() << ". Queue: " << m_pktQueue.size());
      StateRecord(m_pktQueue.size()-1);
    }
  return false;
}


void
THzMacMacroClient::Dequeue ()
{
  NS_LOG_FUNCTION (m_pktQueue.size ());
  m_pktQueue.remove (m_pktData);
}

void
THzMacMacroClient::ReceivePacket (Ptr<THzPhy> phy, Ptr<Packet> packet)
{
  THzMacHeader header;
  packet->PeekHeader (header);
  NS_LOG_DEBUG ("at node " << m_nodeId << " from " << header.GetSource () << " now " << Simulator::Now () << " state: " << StateToString (m_state));
  NS_LOG_DEBUG (Simulator::Now() << " - TX - ---------- received message at sector  " << m_angle + m_beamwidth << " ----------");
  ChannelBecomesBusy ();
  switch (m_state)
    {
    case WAIT_TX:
    case RX:
    case BACKOFF:
    case IDLE:
      m_state = RX;
      break;
    case WAIT_ACK:
    case TX:
    case COLL:
      break;
    case AP_DISCOVERY: 
      // m_state = RX;  
      break;
    }
    //NS_LOG_DEBUG (" now " << Simulator::Now () << " state: " << StateToString (m_state));

}

void
THzMacMacroClient::ReceivePacketDone (Ptr<THzPhy> phy, Ptr<Packet> packet, bool success, double rxPower)
{

  NS_LOG_UNCOND("THzMacMacroClient::ReceivePacketDone  - m_angle is " << m_angle << " rxpower is " << rxPower);
  THzMacHeader header;
  packet->PeekHeader (header);
  NS_LOG_DEBUG (Simulator::Now() << " - TX - ---------- received message at sector  " << m_angle + m_beamwidth << " ----------");

  // important: change m_state in the specific ReceiveXXX function.
  if(m_state == AP_DISCOVERY || m_state == RX || (m_state == WAIT_ACK && header.GetType() == THZ_PKT_TYPE_ACK))    // This solves the problem of receiving CTA from next sector before receiving ACK (if DATA tx failed)
  {
    if (!success)
      {
        NS_LOG_UNCOND ("The packet is not encoded correctly. Drop it now at client!");
        return;
      }
    switch (header.GetType ())
      {
      case THZ_PKT_TYPE_RTS:
      case THZ_PKT_TYPE_DATA:
        NS_LOG_DEBUG (Simulator::Now() << " - ERROR: Can only receive CTS or ACK");
        break;
      case THZ_PKT_TYPE_CTA:
       
        NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " - Receive CTA");
        if (m_ways == 1)
          {
            ReceiveCta1 (packet);
          }
        if (m_ways == 3)
          {
            ReceiveCta3 (packet);
          }
        break;
      case THZ_PKT_TYPE_CTS:
        NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " - Receive CTS");
        ReceiveCts (packet);
        break;
      case THZ_PKT_TYPE_ACK:
        NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " - Receive ACK");
        ReceiveAck (packet);
        break;
      default:
        break;
      }
    return;
  }
  NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " - !!! WARNING: Packet received in state " << m_state << ", shouldn't have been received");
}

void
THzMacMacroClient::ReceiveCta3 (Ptr<Packet> packet)
{
  
  Time max;
  NS_LOG_UNCOND("Node " << m_nodeId << " received cta3 -- ctaCount is " << ctaCount);

  if (m_sectorTimeoutEvent.IsRunning()) {
     NS_LOG_UNCOND("THzMacMacroClient::ReceiveCta3 sector timer running... canceling it.. " << m_nodeId);
     m_sectorTimeoutEvent.Cancel();
  }
  
  time_cta_rcvd_2 = Simulator::Now();   //collect timestamp of cta message for computing rediscovery time

  if(ctaCount <= 4) {
    cta_timestamp[ctaCount]=Simulator::Now();
    NS_LOG_UNCOND("Node " << m_nodeId << "cta count is " << ctaCount);
  }
  else {
    NS_LOG_UNCOND("Node " << m_nodeId << "cta count > 4 " << ctaCount);
    for(int i = 1; i <=4 ; i++)
    {
        cta_timestamp[i-1] = cta_timestamp[i];
    }
    
    cta_timestamp[4]=Simulator::Now();
    
    for(int i = 0; i < 4 ; i++)
    {
        cta_time[i] = cta_timestamp[i+1] - cta_timestamp[i];
	NS_LOG_UNCOND("cta time  " << i << "  is " << cta_time[i]);
    }
    max = cta_time[0];

    for (int i = 0; i < 4; i++)
    {
        if (cta_time[i] > max)
        {
            max = cta_time[i];
        }
    }
    NS_LOG_UNCOND("max cta time recorded recently is " << max);
  }

  ctaCount++;

  if(m_state == AP_DISCOVERY ) {
   if (link_loss_time != PicoSeconds(0)) {                      // check if link was rediscovered- in this case link_loss_time will not be 0 but the time at which link was lost
    NS_LOG_UNCOND("link loss time != 0 for node " << m_nodeId);
    time_rediscovery =  time_cta_rcvd_2 - link_loss_time;       //time_cta_rcvd_2 indicates the time at which link was rediscovered indicated by receipt of cta
    Simulator::ScheduleNow (&THzMacMacroClient::ReDiscoveryTimeRecord,this,time_rediscovery);
    NS_LOG_UNCOND("Node " << m_nodeId << " REDISCOVERED AP at angle " << m_angle << " at time " << Simulator::Now() << " after " << rounds << " rounds");
    NS_LOG_UNCOND("writing to redis rec for node " << m_nodeId);
   }
  }
  
  if (ctaCount >= 5 ) {
     
     if(new_position == current_position) {
       NS_LOG_UNCOND("Position not changed for node " << m_nodeId << " max is " << max );
       if (max > Seconds(0.001)) {
	   m_tLink = max ;         //update link loss detection timer to include recent timing history of cta receipts
	   NS_LOG_UNCOND("m_tlink is now max");
       }
       else {
	  NS_LOG_UNCOND("m_tlink is no longer max");
	  m_tLink = Seconds(0.001) ; 
       }
       NS_LOG_UNCOND("m_tLink value is " << m_tLink << " at time " << Simulator::Now() << " at node " << m_nodeId << "................");
      }
      else{
	current_position = new_position; 
	m_tLink = Seconds(0.001) ;
      }
   }

   if(m_LinkTimeoutEvent.IsRunning()) {   //cancel timer for link loss detection if running
      m_LinkTimeoutEvent.Cancel();
      NS_LOG_UNCOND("Link timeout event canceled for node " << m_nodeId);
   }
   
  
  if(m_state == AP_DISCOVERY ) {
    m_state = IDLE;
    if (link_loss_time == Seconds(0)) {
      Simulator::ScheduleNow (&THzMacMacroClient::DiscoveryTimeRecord,this);
      NS_LOG_UNCOND("Node " << m_nodeId << " DISCOVERED AP at angle " << m_angle << " at time " << Simulator::Now() << " after " << rounds << " full 360 degree rotations");
      NS_LOG_DEBUG("m_nodeId % 2: " << m_nodeId % 2);
      NS_LOG_DEBUG("Number of times node  " << m_nodeId << " discovered AP is = " << m_discovery_count);
       if (m_nodeId % 2 == 0 &&  m_discovery_count == 0) {   // 50% of existing nodes will switch their locations
	 Simulator::Schedule(Seconds(0.001),&THzMacMacroClient::LocationChange, this);
      }
    }
  }

 link_loss_time=Seconds(0);


  m_LinkTimeoutEvent = Simulator::Schedule (m_tLink ,&THzMacMacroClient::LinkTimeout, this); //restart timer for link loss detection
  NS_LOG_UNCOND("Link timeout event restarted for node " << m_nodeId);
   
  THzMacHeader ctaHeader;
  packet->RemoveHeader (ctaHeader);
  NS_LOG_UNCOND (Simulator::Now() << " - " << m_nodeId << " - CTA3 received " << ctaHeader.GetFlags());

  // DUMMY CTA: Mandatory answer Dummy RTS
  if (ctaHeader.GetFlags() == 1)
    {
      Ptr<Packet> rts = Create<Packet> (0);
      THzMacHeader header = THzMacHeader (m_address, ctaHeader.GetSource(), THZ_PKT_TYPE_RTS);
      header.SetFlags(1); // indicate Dummy RTS
      rts->AddHeader(header);
      Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
      uint32_t cw = uv->GetInteger (1, m_boSlots);
      Time t_backoffStart = GetSlotTime() * cw;

      NS_LOG_UNCOND (Simulator::Now() << " - " << m_nodeId << " - DUMMY RTS will be sent in " << t_backoffStart);
      Simulator::Schedule(t_backoffStart, &THzMacMacroClient::SendPacket, this, rts, 0, 0);
      return;
    }

  // Feedback CTA: record which is the assigned sector
  if(ctaHeader.GetFlags() == 2 && ctaHeader.GetDestination() == m_address)
    {
      m_sector = ctaHeader.GetSector();
      NS_LOG_UNCOND (Simulator::Now() << " - " << m_nodeId << " - Feedback CTA Received. Assigned sector " << m_sector);
      return;
    }

  // If queue empty, Do nothing
  if (m_pktQueue.size () == 0)
    {
      NS_LOG_UNCOND (Simulator::Now() << " - " << m_nodeId << " - CTA3 Received. Queue is empty, do nothing"); //corrected to cta from cts
      return;
    }

  // Detect RTS collision
  if (m_ways == 3 && !m_rtsAnswered)  // 3-way, for the case when RTS collides and AP skips the sector, AP will send CTA instead of the expected CTS
    {
      NS_LOG_UNCOND (Simulator::Now() << " - " << m_nodeId << " - RTS Unanswered. No CTS has been received");
      CtsTimeout(m_lastSeq);
      m_rtsAnswered = true; // So it doesn't trigger CTS Timeout again at next sector
    }

  // If in BO, decrease life
  if (m_backoffActive)
    {
      DecreaseBackoff();
      return;
    }

    m_ctsReceived = 0;  // for 3-way, need to set cts received to 0. Maybe makes more sense to do this after ACK is received

  // Check sector
  if (m_sector > -1 && ctaHeader.GetSector() != m_sector)
    {
      NS_LOG_UNCOND (Simulator::Now() << " - " << m_nodeId << " - CTA3 received. Not my sector (" << m_sector << "). Do nothing");
      return;
    }

  // Random Start Backoff
  Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
  uint32_t cw = uv->GetInteger (1, m_boSlots);
  Time t_backoffStart = GetSlotTime() * cw;

  // Send RTS
  std::list<Rec>::iterator it = m_rec.begin();
  m_pktData = it -> Recpacket;
  THzMacHeader dataHeader;
  m_pktData->PeekHeader (dataHeader);
  m_state = WAIT_TX;
  NS_LOG_UNCOND (Simulator::Now() << " - " << m_nodeId << " - CTA3 received. Sending RTS after " << t_backoffStart << " of BO. At node " << m_nodeId);
  Simulator::Schedule(t_backoffStart, &THzMacMacroClient::SendRts, this, m_pktData, it -> RecRetry);
  m_lastSeq = dataHeader.GetSequence();
  m_rtsAnswered = false;
}


void
THzMacMacroClient::ReceiveCta1 (Ptr<Packet> packet)
{
  m_state = IDLE;

  THzMacHeader ctaHeader;
  packet->RemoveHeader (ctaHeader);
  NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " - CTA received " << ctaHeader.GetFlags());

  // If queue empty, Do nothing
  if (m_pktQueue.size () == 0)
    {
      NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " - CTA Received. Queue is empty, do nothing");
      return;
    }

  // If in BO, decrease life
  if (m_backoffActive)
    {
      DecreaseBackoff();
      return;
    }

  // Fairness
  m_clientMobility = m_device-> GetNode ()->GetObject<MobilityModel> ();
  Vector pos = m_clientMobility->GetPosition ();
  double d = std::sqrt (pow (pos.x,(double)2) + pow (pos.y,(double)2) + pow (pos.z,(double)2));
  Time t_fairness = 2 * m_tProp - PicoSeconds (6666 * d);

  // Random Start Backoff
  Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
  uint32_t cw = uv->GetInteger (1, m_boSlots);
  Time t_backoffStart = GetSlotTime() * cw;

  // Send DATA
  std::list<Rec>::iterator it = m_rec.begin();
  m_pktData = it -> Recpacket;
  THzMacHeader dataHeader;
  m_pktData->PeekHeader (dataHeader);
  m_state = WAIT_TX;
  NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " - CTA received. Sending DATA after " << t_fairness + t_backoffStart << " of BO + Fairness.");
  m_timeCTSrx = Simulator::Now();
  m_sendDataEvent = Simulator::Schedule (t_fairness + t_backoffStart, &THzMacMacroClient::SendData, this, m_pktData, 0);
}


void
THzMacMacroClient::ReceiveCts (Ptr<Packet> packet)
{
  m_state = IDLE;

  THzMacHeader ctsHeader;
  packet->RemoveHeader (ctsHeader);
  NS_LOG_UNCOND (Simulator::Now() << " - " << m_nodeId << " - CTS received in response to rts " << ctsHeader.GetFlags() << "at node " << m_nodeId);

  m_ctsReceived++;
  if (ctsHeader.GetDestination() == m_address)  // Access granted to send the DATA requested
    {
      std::list<Rec>::iterator it = m_rec.begin();
      for (; it != m_rec.end(); ++it)
        {
          if (it->RecSeq == ctsHeader.GetSequence())
            {
              m_rtsAnswered = true;
              Time waitToSend = ctsHeader.GetDuration();  // Set Wait time indicated by AP
              m_timeCTSrx = Simulator::Now();

              int mcs = 0;
              if (ctsHeader.GetFlags() >= 10 && ctsHeader.GetFlags() <= 14)
                {
                  mcs = ctsHeader.GetFlags(); // Set MCS as indicated by AP
                }
              Simulator::Schedule(waitToSend, &THzMacMacroClient::SendData, this, it->Recpacket, mcs);
              NS_LOG_UNCOND (Simulator::Now() << " - " << m_nodeId << " - CTS RECEIVED destined to me. MCS " << ctsHeader.GetFlags() << ". Sending packet " << ctsHeader.GetSequence() << " after " << waitToSend << " using mcs " << mcs);
            }
        }
    }
  else
    {
      NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " - CTS RECEIVED not destined to me. CTS count: " << m_ctsReceived);
    }
}

void
THzMacMacroClient::DecreaseBackoff ()
{
  std::list<Rec>::iterator it = m_rec.begin();    // use of an iterator is not necessary. But might be useful for future implementations (e.g., multiple DATA packets in a window, some of them not ACK)
  for (; it != m_rec.end(); ++it)
    {
      if (it -> RecSeq == m_backoffSeq)
        {
          it -> BackoffLife--;
          NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " - Decrease Backoff life to: " << it -> BackoffLife);

          if (it -> BackoffLife == 0)
            {
              m_backoffActive = false;
            }
          return;
        }
    }
  NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " - ERROR. Packet with BO active not found. This point should never be reached");
}

void
THzMacMacroClient::SendRts (Ptr<Packet> data, uint16_t retry)
{
  THzMacHeader dataHeader;
  data->PeekHeader (dataHeader);
  Ptr<Packet> rts = Create<Packet> (0);
  THzMacHeader header = THzMacHeader (m_address, dataHeader.GetDestination(), THZ_PKT_TYPE_RTS);
  header.SetSequence (dataHeader.GetSequence());
  header.SetRetry (retry);
  header.SetFlags (0);
  rts->AddHeader (header);
  NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " - RTS sent");
  SendPacket (rts, 0, 0);
}

void
THzMacMacroClient::ChannelBecomesBusy (void)
{
  if(m_sendDataEvent.IsRunning())
    {
      m_sendDataEvent.Cancel();
    }
}

void
THzMacMacroClient::SendData (Ptr<Packet> packet, int mcs)
{
  NS_LOG_UNCOND("THzMacMacroClient::SendData");
  m_state = WAIT_TX;
  m_pktData = packet;
  NS_LOG_UNCOND (Simulator::Now() << " - SEND DATA at node: " << m_nodeId << " now: " << Simulator::Now () << " QueueSize " << m_pktQueue.size ());
  THzMacHeader header;
  m_pktData->PeekHeader (header);
  if (header.GetDestination () == GetBroadcast ())  // Broadcast
    {
      NS_LOG_DEBUG (Simulator::Now() << " - Broadcast of data not supported");
    }
  if (header.GetDestination () != GetBroadcast ())  // Unicast
    {
      if (SendPacket (m_pktData, 1, mcs))
        {
          Time ackTimeout;
          if (m_ways == 3)  // 3-way
            {
              ackTimeout = (m_tData + GetMaxBackoff() + GetCtrlDuration (THZ_PKT_TYPE_CTS) + GetCtrlDuration (THZ_PKT_TYPE_ACK)) * m_ctsReceived + m_tProp + GetSifs () + m_tProp + NanoSeconds(10);
              ackTimeout = ackTimeout - (Simulator::Now() - m_timeCTSrx);
            }
          else // 1-way
            {
              ackTimeout = m_tData + m_tProp + GetSifs () + GetCtrlDuration (THZ_PKT_TYPE_ACK) + m_tProp + NanoSeconds(1);
            }
          AckTimeouts at;
          at.sequence = header.GetSequence ();
          at.m_ackTimeoutEvent = Simulator::Schedule (ackTimeout, &THzMacMacroClient::AckTimeout, this, header.GetSequence ());
          m_ackTimeouts.push_back (at);
          NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " scheduling ack timeout at: " << Simulator::Now () + ackTimeout << ". ackTimeout: " << ackTimeout);
        }
      else
        {
          m_state = IDLE;
        }
    }
}

bool
THzMacMacroClient::SendPacket (Ptr<Packet> packet, bool rate, uint16_t mcs)
{
  NS_LOG_UNCOND ("THzMacMacroClient::SendPacket state " << m_state << " now " << Simulator::Now ());

  if (m_state == IDLE || m_state == WAIT_TX)
    {
      if (m_phy->SendPacket (packet, rate, mcs))
        {
          m_state = TX;
          m_pktTx = packet;
          return true;
        }
      else
        {
          m_state = IDLE;
        }
    }
  return false;
}

void
THzMacMacroClient::SendPacketDone (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION ("at node " << m_nodeId << " state " << StateToString (m_state));
  NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " - SendPacketDone");

  if (m_state != TX || m_pktTx != packet)
    {
      NS_LOG_DEBUG ("ERROR. Something is wrong!");
      return;
    }
  m_state = IDLE;
  THzMacHeader header;
  packet->PeekHeader (header);
  switch (header.GetType ())
    {
    case THZ_PKT_TYPE_RTS:
    case THZ_PKT_TYPE_CTS:
    case THZ_PKT_TYPE_ACK:
      break;

    case THZ_PKT_TYPE_DATA:
      NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " - DATA Tx finished. Seq: " << header.GetSequence());
      m_state = WAIT_ACK;
      if (header.GetDestination () == GetBroadcast ())
        {
          NS_LOG_DEBUG (Simulator::Now() << " - ERROR. Broadcast not supported");
          return;
        }
      break;

    default:
      break;
    }
}

void
THzMacMacroClient::ReceiveAck (Ptr<Packet> packet)
{
  NS_LOG_UNCOND("THzMacMacroClient::ReceiveAck at node " << m_nodeId);
  NS_LOG_FUNCTION ("at node " << m_nodeId);
  THzMacHeader header;
  packet->RemoveHeader (header);

  if (header.GetDestination () == m_address)
    {
      m_state = IDLE;
      NS_LOG_UNCOND (Simulator::Now() << " - " << m_nodeId << " - ~~ ACK RECEIVED.");
      std::list<AckTimeouts>::iterator it = m_ackTimeouts.begin ();
      for (; it != m_ackTimeouts.end (); ++it)
        {
          if (it->sequence == header.GetSequence ())
            {
              it->m_ackTimeoutEvent.Cancel ();
              Simulator::Schedule (NanoSeconds (0.001), &THzMacMacroClient::SendDataDone, this, true, header.GetSequence ());
              it = m_ackTimeouts.erase (it);
              return;
            }
        }
    }
  else
  {
    NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " - ACK was not for me");
  }
}

void
THzMacMacroClient::SendDataDone (bool success, uint16_t sequence)
{
  NS_LOG_FUNCTION ("at node " << m_nodeId);
  std::list<Rec>::iterator it = m_rec.begin ();
  for (; it != m_rec.end (); it++)
    {
      if (it->RecSeq == sequence)
        {
          Result result;
          result.nodeid = m_nodeId;
          m_result.clear ();

          if (success)
            {
              NS_LOG_FUNCTION ("Success to transmit packet at node: " << m_nodeId);
              if (m_pktQueue.size () == 0)
                {
                  NS_LOG_DEBUG ("node: " << m_nodeId << " senddatadone check queue empty");
                  return;
                }
              m_pktQueue.remove (it->Recpacket);
              m_send++;
              m_tend = Simulator::Now ();
              m_tstart = it->RecTime;
              m_timeRec = (m_tend - m_tstart);
              result.Psize = (it->RecSize - 53); //byte
              result.delay = m_timeRec;
              result.success = true;
              result.discard = false;
              m_result.push_front (result);
              Simulator::ScheduleNow (&THzMacMacroClient::ResultsRecord,this);
              m_throughput = (it->RecSize - 53) * 8 / m_timeRec.GetSeconds ();
              m_throughputAll += m_throughput;
              m_ite += 1;
              m_throughputavg = m_throughputAll / (m_ite);
              m_traceThroughput (m_throughputavg);
              NS_LOG_UNCOND (m_nodeId << " - *** Successfully Sent Packet number " << m_send << " from node " << m_nodeId << " Discard " << m_discard << " Total send " << (m_send + m_discard) << " #queue " << m_pktQueue.size () << ". S [bps]= " << m_throughputavg);
              NS_LOG_UNCOND ("  throughput : " << m_throughput << " of node " << m_nodeId);
              NS_LOG_UNCOND ("  average throughput : " << m_throughputavg << " of node " << m_nodeId);
            }
          else
            {
              NS_LOG_FUNCTION ("Fail to transmit packet at node: " << m_nodeId);
              m_discard++;
              result.Psize = (it->RecSize - 53); //byte
              result.delay = Seconds (0);
              result.success = false;
              result.discard = true;
              m_result.push_front (result);
              Simulator::ScheduleNow (&THzMacMacroClient::ResultsRecord,this);
              NS_LOG_DEBUG (m_nodeId << " - !!!!! Discard Packet number " << m_discard << " from node " << m_nodeId << " Total send " << (m_send + m_discard) << " #queue " << m_pktQueue.size ());

            }
          NS_LOG_DEBUG ("NODE: " << m_nodeId << " SEND DATA DONE: m_sequence = " << sequence);
          it = m_rec.erase (it);
        }
    }
}

// -------------------------- Timeout ----------------------------------

void
THzMacMacroClient::CtsTimeout (uint16_t sequence)
{
  m_state = IDLE;
  std::list<Rec>::iterator it = m_rec.begin ();
  for (; it != m_rec.end (); ++it)
    {
      if (it->RecSeq == sequence)
        {
          it->RecRetry = it->RecRetry + 1;
          NS_LOG_DEBUG ("NODE: " << m_nodeId << " CTS T/O: m_sequence = " << sequence << " RETRY = " << it->RecRetry);
          if (it->RecRetry >= m_rtsRetryLimit)
            {
              m_pktQueue.remove (it->Recpacket);
              Simulator::Schedule (NanoSeconds (0.001), &THzMacMacroClient::SendDataDone, this, false, sequence); // Discard
            }
          else
            {
              m_backoffActive = true;
              m_backoffSeq = it -> RecSeq;
              Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
              it->BackoffLife = uv->GetInteger (1, pow (double(2.0), double(it->RecRetry))); // Set Backoff life
              NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " - CTS Timeout. Number of tries: " << it->RecRetry << " BO life: " << it->BackoffLife);

            }
          CollisionsRecord(it->RecRetry);
          m_ctsReceived = 0;
          return;
        }
    }
}


void
THzMacMacroClient::AckTimeout (uint16_t sequence)
{
  NS_LOG_UNCOND("Ack timeout at client");
  m_state = IDLE;
  std::list<AckTimeouts>::iterator ait = m_ackTimeouts.begin ();
  for (; ait != m_ackTimeouts.end (); )
    {
      if (ait->sequence == sequence)
        {
          ait = m_ackTimeouts.erase (ait);
          break;
        }
      else
        {
          ++ait;
        }
    }
  NS_LOG_DEBUG ("!!! ACK timeout !!!");
  if(m_ways == 3)
    {
      NS_LOG_UNCOND (Simulator::Now() << " - *** ERROR *** ACK should always be received... (no DATA collisions in ADAPT-3)");
    }

  std::list<Rec>::iterator it = m_rec.begin ();
  for (; it != m_rec.end (); ++it)
    {
      if (it->RecSeq == sequence)
        {
          it->RecRetry = it->RecRetry + 1;
          NS_LOG_DEBUG ("NODE: " << m_nodeId << " ACK T/O: m_sequence = " << sequence << " RETRY = " << it->RecRetry);
          if (it->RecRetry >= m_dataRetryLimit)
            {
              m_pktQueue.remove (it->Recpacket);
              Simulator::Schedule (NanoSeconds (0.001), &THzMacMacroClient::SendDataDone, this, false, sequence); // Discard
            }
          else
            {
              NS_LOG_DEBUG ("at node " << m_nodeId << " ack timeout at:" << Simulator::Now () << " #queue " << m_pktQueue.size ());
              m_backoffActive = true;
              m_backoffSeq = it -> RecSeq;
              Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
              it->BackoffLife = uv->GetInteger (1, pow (double(2.0), double(it->RecRetry)));  // Set Backoff life. Minimum 1, if min is set to 0, GetInteger() doesn't work
              NS_LOG_DEBUG (Simulator::Now() << " - " << m_nodeId << " - ------ ACK TIMEOUT. Backoff Life: " << it->BackoffLife);
            }
            CollisionsRecord(it->RecRetry);
            return;
        }
    }
}

// ------------------------ Set Functions -----------------------------
void
THzMacMacroClient::AttachPhy (Ptr<THzPhy> phy)
{
  m_phy = phy;
}
void
THzMacMacroClient::SetDevice (Ptr<THzNetDevice> dev)
{
  NS_LOG_UNCOND("THzMacMacroClient::SetDevice");
  m_device = dev;
}

void
THzMacMacroClient::SetAddress (Mac48Address addr)
{
  NS_LOG_FUNCTION (addr);
  m_address = addr;
  // to help each node have different random seed
  uint8_t tmp[6];
  m_address.CopyTo (tmp);
}
void
THzMacMacroClient::SetForwardUpCb (Callback<void, Ptr<Packet>, Mac48Address, Mac48Address> cb)
{
  m_forwardUpCb = cb;
}

void
THzMacMacroClient::SetSlotTime (Time duration)
{
  m_slotTime = duration;
}
// ------------------------ Get Functions -----------------------------
Time
THzMacMacroClient::GetSlotTime (void)
{
  if(m_ways == 3) { // 3-way
    return m_slotTime_3way;
  }
  return m_slotTime;
}
Time
THzMacMacroClient::GetSifs (void) const
{
  return m_sifs;
}
Time
THzMacMacroClient::GetDifs (void) const
{
  return m_difs;
}

Mac48Address
THzMacMacroClient::GetAddress () const
{
  return this->m_address;
}

Mac48Address
THzMacMacroClient::GetBroadcast (void) const
{
  return Mac48Address::GetBroadcast ();
}
Time
THzMacMacroClient::GetCtrlDuration (uint16_t type)
{
  THzMacHeader header = THzMacHeader (m_address, m_address, type);
  return m_phy->CalTxDuration (header.GetSize (), 0, 0);
}
Time
THzMacMacroClient::GetDataDuration (Ptr<Packet> p)
{
  return m_phy->CalTxDuration (0, p->GetSize (), 0);
}

std::string
THzMacMacroClient::StateToString (State state)
{
  switch (state)
    {
    case IDLE:
      return "IDLE";
    case BACKOFF:
      return "BACKOFF";
    case WAIT_TX:
      return "WAIT_TX";
    case TX:
      return "TX";
    case WAIT_ACK:
      return "WAIT_ACK";
    case RX:
      return "RX";
    case COLL:
      return "COLL";
    case AP_DISCOVERY:
      return "AP_DISCOVERY";
    
    default:
      return "??";
    }
}


// --------------------------- ETC -------------------------------------
//DONE no changes
bool
THzMacMacroClient::IsNewSequence (Mac48Address addr, uint16_t seq)
{
  std::list<std::pair<Mac48Address, uint16_t> >::iterator it = m_seqList.begin ();
  for (; it != m_seqList.end (); ++it)
    {
      if (it->first == addr)
        {
          if (it->second == 65536 && seq < it->second)
            {
              it->second = seq;
              return true;
            }
          else if (seq > it->second)
            {
              it->second = seq;
              return true;
            }
          else
            {
              return false;
            }
        }
    }
  std::pair<Mac48Address, uint16_t> newEntry;
  newEntry.first = addr;
  newEntry.second = seq;
  m_seqList.push_back (newEntry);
  return true;
}


Time
THzMacMacroClient::GetMaxBackoff()
{
  return GetSlotTime() * m_boSlots;
}


/***************Record functions below**************/


void
THzMacMacroClient::ResultsRecord ()
{
  /*----------------------------------------------------------------------------------------
   * enable the result printing in a .txt file by uncommenting the content in this function
   *----------------------------------------------------------------------------------------*/

   std::stringstream txtname;
   txtname << "scratch/" << outputFile;
   std::string filename = txtname.str ();

   std::ofstream resultfile;
   resultfile.open (filename.c_str (), std::ios::app);
   std::list<Result>::iterator it = m_result.begin ();
   resultfile << it->nodeid << "\t" << it->Psize << "\t" << it->delay.GetNanoSeconds() << "\t" << it->success << "\t" << it->discard << std::endl;
   resultfile.close ();
   return;

}

void
THzMacMacroClient::CollisionsRecord (uint16_t retry)
{
  /*----------------------------------------------------------------------------------------
   * enable the collision printing in a .txt file by uncommenting the content in this function
   *----------------------------------------------------------------------------------------*/
/*
   std::stringstream txtname;
   txtname << "scratch/collisions_" << outputFile;
   std::string filename = txtname.str ();

   std::ofstream resultfile;
   resultfile.open (filename.c_str (), std::ios::app);
   resultfile << m_nodeId << "\t" << retry << std::endl;
   resultfile.close ();
   return;
*/
}

void
THzMacMacroClient::StateRecord (uint16_t state)
{
  /*----------------------------------------------------------------------------------------
   * enable the state printing in a .txt file by uncommenting the content in this function
   *----------------------------------------------------------------------------------------*/
  /*
   std::stringstream txtname;
   txtname << "scratch/state_" << outputFile;
   std::string filename = txtname.str ();

   std::ofstream resultfile;
   resultfile.open (filename.c_str (), std::ios::app);
   resultfile << m_nodeId << "\t" << state << std::endl;
   resultfile.close ();
   return;
*/
   }


void THzMacMacroClient::PositionsRecord ()
{
  /*----------------------------------------------------------------------------------------
   * enable the node position printing in a .txt file by uncommenting the content in this function
   *----------------------------------------------------------------------------------------*/

  /*
  RngSeedManager seed;
  int seed_num = seed.GetSeed ();

  std::stringstream txtname;
  txtname << "scratch/position_seed" << seed_num << ".txt";
  std::string filename = txtname.str ();

  m_clientMobility = m_device-> GetNode ()->GetObject<MobilityModel> ();
  Vector pos = m_clientMobility->GetPosition ();

  std::ofstream resultfile;
  resultfile.open (filename.c_str (), std::ios::app);
  resultfile << m_nodeId << "\t" << pos.x << "\t" << pos.y << "\t" << m_angle << std::endl;
  resultfile.close ();
  return;
  */
}

void
THzMacMacroClient::DiscoveryTimeRecord ()
{
  /*----------------------------------------------------------------------------------------
   * enable the result printing in a .txt file by uncommenting the content in this function
   *----------------------------------------------------------------------------------------*/

  NS_LOG_UNCOND("In DiscoveryTimeRecord node " <<  m_nodeId);
   std::stringstream txtname;
   txtname << "scratch/DiscoveryTime_" << outputFile;
   std::string filename = txtname.str ();

   std::ofstream resultfile;
   resultfile.open (filename.c_str (), std::ios::app);
   resultfile << m_nodeId << "\t" <<  rounds << "\t" << m_angle << "\t" <<  Simulator::Now().GetNanoSeconds() <<  std::endl;
   resultfile.close ();
   rounds=0;
   return;

}


void
THzMacMacroClient::ReDiscoveryTimeRecord (Time reDiscoveryTime)
{
  /*----------------------------------------------------------------------------------------
   * enable the result printing in a .txt file by uncommenting the content in this function
   *----------------------------------------------------------------------------------------*/

  NS_LOG_UNCOND("In ReDiscoveryTimeRecord node " <<  m_nodeId);
   std::stringstream txtname;
   txtname << "scratch/ReDiscoveryTime_" << outputFile;
   std::string filename = txtname.str ();

   std::ofstream resultfile;
   resultfile.open (filename.c_str (), std::ios::app);
   resultfile << m_nodeId << "\t" << reDiscoveryTime.GetNanoSeconds()  <<  std::endl;
   resultfile.close ();
   
   return;

}

} /* namespace ns3 */
