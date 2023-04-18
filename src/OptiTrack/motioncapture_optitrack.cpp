/***
 * TAKEN FROM https://github.com/IMRCLab/libmotioncapture
 * Aug 26, 2021
 */

#include <Core/util.h>
#include "motioncapture_optitrack.h"

#ifdef RAI_OPTITRACK

#include <boost/asio.hpp>

using boost::asio::ip::udp;

namespace libmotioncapture {

  constexpr const char* MULTICAST_ADDRESS = "239.255.42.99";
  constexpr int PORT_COMMAND = 1510;
  constexpr int PORT_DATA = 1511;
  constexpr int MAX_PACKETSIZE = 100000;  // max size of packet (actual packet size is dynamic)
  constexpr int MAX_NAMELENGTH = 256;

  constexpr int NAT_CONNECT           = 0;
  constexpr int NAT_SERVERINFO        = 1;
  constexpr int NAT_REQUEST_MODELDEF  = 4;
  constexpr int NAT_MODELDEF          = 5;

  class MotionCaptureOptitrackImpl{
  public:
    MotionCaptureOptitrackImpl()
      : version()
      , versionMajor(0)
      , versionMinor(0)
      , io_service()
      , socket(io_service)
      , sender_endpoint()
      , data(20000)
    {
    }
    // void getObjectByRigidbody(
    //   const RigidBody& rb,
    //   Object& result) const
    //   {
    //     std::stringstream sstr;
    //     sstr << rb.id();
    //     const std::string name_number = sstr.str();
    //     std::string name_cf = "cf";
    //     const std::string name = name_cf + name_number;

    //     auto const translation = rb.location();
    //     auto const quaternion = rb.orientation();

    //     if(rb.trackingValid()) {
    //         Eigen::Vector3f position(
    //           -translation.y,     
    //           translation.x,
    //           translation.z);

    //         Eigen::Quaternionf rotation(
    //           quaternion.qw,
    //           -quaternion.qy,
    //           quaternion.qx,
    //           quaternion.qz
    //           );

    //         result = Object(name, position, rotation);

    //     } else {
    //         result = Object(name);
    //     }
    //   } 

    void parseModelDef(const char* data)
    {
      const char *ptr = data;
      int major = versionMajor;
//      int minor = versionMinor;

      // First 2 Bytes is message ID
      int MessageID = 0;
      memcpy(&MessageID, ptr, 2); ptr += 2;
      // printf("Message ID : %d\n", MessageID);

      // Second 2 Bytes is the size of the packet
      int nBytes = 0;
      memcpy(&nBytes, ptr, 2); ptr += 2;
      // printf("Byte count : %d\n", nBytes);

      if(MessageID == NAT_MODELDEF) // Data Descriptions
      {
        // number of datasets
        int nDatasets = 0; memcpy(&nDatasets, ptr, 4); ptr += 4;
        // printf("Dataset Count : %d\n", nDatasets);

        for(int i=0; i < nDatasets; i++)
        {
          // printf("Dataset %d\n", i);

          int type = 0; memcpy(&type, ptr, 4); ptr += 4;
          // printf("Type : %d\n", i, type);

          if(type == 0)   // markerset
          {
            ptr += strlen(ptr) + 1; // name

            // marker data
            int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
            // printf("Marker Count : %d\n", nMarkers);

            for(int j=0; j < nMarkers; j++)
            {
              ptr += strlen(ptr) + 1;
            }
          }
          else if(type ==1)   // rigid body
          {
            char szName[MAX_NAMELENGTH];
            if(major >= 2)
            {
              // name
              strcpy(szName, ptr);
              ptr += strlen(ptr) + 1;
              // printf("Name: %s\n", szName);
            }

            int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
            // printf("ID : %d\n", ID);

            rigidBodyDefinitions[ID].name = szName;
            rigidBodyDefinitions[ID].ID = ID;
         
            memcpy(&rigidBodyDefinitions[ID].parentID, ptr, 4); ptr +=4;
            memcpy(&rigidBodyDefinitions[ID].xoffset, ptr, 4); ptr +=4;
            memcpy(&rigidBodyDefinitions[ID].yoffset, ptr, 4); ptr +=4;
            memcpy(&rigidBodyDefinitions[ID].zoffset, ptr, 4); ptr +=4;

            // Per-marker data (NatNet 3.0 and later)
            if ( major >= 3 )
            {
              int nMarkers = 0; memcpy( &nMarkers, ptr, 4 ); ptr += 4;

              // Marker positions
              nBytes = nMarkers * 3 * sizeof( float );
              ptr += nBytes;

              // Marker required active labels
              nBytes = nMarkers * sizeof( int );
              ptr += nBytes;
            }
          }
          else if(type ==2)   // skeleton
          {
            // char szName[MAX_NAMELENGTH];
            // strcpy(szName, ptr);
            ptr += strlen(ptr) + 1;
            // printf("Name: %s\n", szName);

            // int ID = 0; memcpy(&ID, ptr, 4);
            ptr +=4;
            // printf("ID : %d\n", ID);

            int nRigidBodies = 0; memcpy(&nRigidBodies, ptr, 4); ptr +=4;
            // printf("RigidBody (Bone) Count : %d\n", nRigidBodies);

            for(int i=0; i< nRigidBodies; i++)
            {
                if(major >= 2)
                {
                    // RB name
                    // char szName[MAX_NAMELENGTH];
                    // strcpy(szName, ptr);
                    ptr += strlen(ptr) + 1;
                    // printf("Rigid Body Name: %s\n", szName);
                }

                // int ID = 0; memcpy(&ID, ptr, 4);
                ptr +=4;
                // printf("RigidBody ID : %d\n", ID);

                // int parentID = 0; memcpy(&parentID, ptr, 4);
                ptr +=4;
                // printf("Parent ID : %d\n", parentID);

                // float xoffset = 0; memcpy(&xoffset, ptr, 4);
                ptr +=4;
                // printf("X Offset : %3.2f\n", xoffset);

                // float yoffset = 0; memcpy(&yoffset, ptr, 4);
                ptr +=4;
                // printf("Y Offset : %3.2f\n", yoffset);

                // float zoffset = 0; memcpy(&zoffset, ptr, 4);
                ptr +=4;
                // printf("Z Offset : %3.2f\n", zoffset);
            }
          }
        }   // next dataset

       // printf("End Packet\n-------------\n");

      }
    }

  public:
    // NatNetClient client;
    std::string version;
    int versionMajor;
    int versionMinor;
    // Point3f axisMultiplier;
    // Point3f axisOrder;

    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket;
    boost::asio::ip::udp::endpoint sender_endpoint;
    std::vector<char> data;

    struct rigidBody {
      int ID;
      float x;
      float y;
      float z;
      float qx;
      float qy;
      float qz;
      float qw;
      float fError; // mean marker error
      bool bTrackingValid;
    };
    std::vector<rigidBody> rigidBodies;

    struct marker {
      float x;
      float y;
      float z;
    };
    std::vector<marker> markers;

    struct rigidBodyDefinition {
      std::string name;
      int ID;
      int parentID;
      float xoffset;
      float yoffset;
      float zoffset;
    };
    std::map<int, rigidBodyDefinition> rigidBodyDefinitions;
  };

  MotionCaptureOptitrack::MotionCaptureOptitrack(
    const std::string& hostname)
  {
    pImpl = new MotionCaptureOptitrackImpl;

    // Connect to command port to query version
    boost::asio::io_service io_service_cmd;
    udp::socket socket_cmd(io_service_cmd, udp::endpoint(udp::v4(), 0));
    udp::resolver resolver_cmd(io_service_cmd);
    udp::endpoint endpoint_cmd = *resolver_cmd.resolve({udp::v4(), hostname, std::to_string(PORT_COMMAND)});

    typedef struct
    {
      unsigned short iMessage;                // message ID (e.g. NAT_FRAMEOFDATA)
      unsigned short nDataBytes;              // Num bytes in payload
    } sRequest;

    typedef struct
    {
      unsigned short iMessage;
      unsigned short nDataBytes;
      char szName[256];
      unsigned char Version[4];
      unsigned char NatNetVersion[4];
    } sResponse;

    sRequest connectCmd = {NAT_CONNECT, 0};
    socket_cmd.send_to(boost::asio::buffer(&connectCmd, sizeof(connectCmd)), endpoint_cmd);

    sResponse response;
    udp::endpoint sender_endpoint;
    size_t reply_length = socket_cmd.receive_from(
        boost::asio::buffer(&response, sizeof(response)), sender_endpoint);

    if (response.iMessage != NAT_SERVERINFO) {
      throw std::runtime_error("Could not query NatNet version!");
    }

    std::ostringstream stringStream;
    stringStream << (int)response.NatNetVersion[0] << "."
                 << (int)response.NatNetVersion[1] << "."
                 << (int)response.NatNetVersion[2] << "."
                 << (int)response.NatNetVersion[3];
    pImpl->version = stringStream.str();

    pImpl->versionMajor = response.NatNetVersion[0];
    pImpl->versionMinor = response.NatNetVersion[1];

    // query model def
    sRequest modelDefCmd = {NAT_REQUEST_MODELDEF, 0};
    socket_cmd.send_to(boost::asio::buffer(&modelDefCmd, sizeof(modelDefCmd)), endpoint_cmd);
    std::vector<char> modelDef(20000);
    reply_length = socket_cmd.receive_from(
        boost::asio::buffer(modelDef.data(), modelDef.size()), sender_endpoint);
    modelDef.resize(reply_length);
    pImpl->parseModelDef(modelDef.data());

    // connect to data port to receive mocap data
    auto listen_address = boost::asio::ip::address::from_string("0.0.0.0");
    auto multicast_address = boost::asio::ip::address::from_string(MULTICAST_ADDRESS);

    // Create the socket so that multiple may be bound to the same address.
    boost::asio::ip::udp::endpoint listen_endpoint(
        listen_address, PORT_DATA);
    pImpl->socket.open(listen_endpoint.protocol());
    pImpl->socket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    pImpl->socket.bind(listen_endpoint);

    // Join the multicast group.
    pImpl->socket.set_option(
        boost::asio::ip::multicast::join_group(multicast_address));
  }

  const std::string & MotionCaptureOptitrack::version() const
  {
    return pImpl->version;
  }

  void MotionCaptureOptitrack::waitForNextFrame()
  {
    // use a loop to get latest data
    do {
      pImpl->data.resize(20000);
      size_t length = pImpl->socket.receive_from(boost::asio::buffer(pImpl->data.data(), pImpl->data.size()), pImpl->sender_endpoint);
      pImpl->data.resize(length);
    } while (pImpl->socket.available() > 0);

    if (pImpl->data.size() > 4) {
      char *ptr = pImpl->data.data();
      int major = pImpl->versionMajor;
      int minor = pImpl->versionMinor;

      // First 2 Bytes is message ID
      int MessageID = 0;
      memcpy(&MessageID, ptr, 2); ptr += 2;
      // printf("Message ID : %d\n", MessageID);

      // Second 2 Bytes is the size of the packet
      int nBytes = 0;
      memcpy(&nBytes, ptr, 2); ptr += 2;
      // printf("Byte count : %d\n", nBytes);

      if(MessageID == 7)      // FRAME OF MOCAP DATA packet
      {
        // Next 4 Bytes is the frame number
        int frameNumber = 0; memcpy(&frameNumber, ptr, 4); ptr += 4;
        // printf("Frame # : %d\n", frameNumber);
      
        // Next 4 Bytes is the number of data sets (markersets, rigidbodies, etc)
        int nMarkerSets = 0; memcpy(&nMarkerSets, ptr, 4); ptr += 4;
        // printf("Marker Set Count : %d\n", nMarkerSets);

        // Loop through number of marker sets and get name and data
        for (int i=0; i < nMarkerSets; i++)
        {
          ptr += strlen(ptr) + 1;
          int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
          ptr += nMarkers * 12;
        }

        // Loop through unlabeled markers
        // OtherMarker list is Deprecated
        int nOtherMarkers = 0; memcpy(&nOtherMarkers, ptr, 4); ptr += 4;
        // printf("Unidentified Marker Count : %d\n", nOtherMarkers);
        ptr += nOtherMarkers * 12;

        // Loop through rigidbodies
        int nRigidBodies = 0; memcpy(&nRigidBodies, ptr, 4); ptr += 4;
        pImpl->rigidBodies.resize(nRigidBodies);
        // printf("Rigid Body Count : %d\n", nRigidBodies);
        for (int j=0; j < nRigidBodies; j++)
        {
          // Rigid body position and orientation 
          memcpy(&pImpl->rigidBodies[j].ID, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].x, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].y, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].z, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].qx, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].qy, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].qz, ptr, 4); ptr += 4;
          memcpy(&pImpl->rigidBodies[j].qw, ptr, 4); ptr += 4;

          // NatNet version 2.0 and later
          if(major >= 2)
          {
            // Mean marker error
            memcpy(&pImpl->rigidBodies[j].fError, ptr, 4); ptr += 4;
          }

          // NatNet version 2.6 and later
          if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) ) 
          {
            // params
            short params = 0; memcpy(&params, ptr, 2); ptr += 2;
            pImpl->rigidBodies[j].bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
          }
        } // Go to next rigid body

        // Skeletons (NatNet version 2.1 and later)
        // (we do not support skeletons)
        if( ((major == 2)&&(minor>0)) || (major>2))
        {
          int nSkeletons = 0; memcpy(&nSkeletons, ptr, 4); ptr += 4;
          // printf("Skeleton Count : %d\n", nSkeletons);

          // Loop through skeletons
          for (int j=0; j < nSkeletons; j++)
          {
            // skeleton id
            // int skeletonID = 0;
            // memcpy(&skeletonID, ptr, 4);
            ptr += 4;

            // Number of rigid bodies (bones) in skeleton
            int nRigidBodies = 0;
            memcpy(&nRigidBodies, ptr, 4); ptr += 4;
            // printf("Rigid Body Count : %d\n", nRigidBodies);

            // Loop through rigid bodies (bones) in skeleton
            for (int j=0; j < nRigidBodies; j++)
            {
              // Rigid body position and orientation
              ptr += 8*4;

              // Mean marker error (NatNet version 2.0 and later)
              if(major >= 2)
              {
                ptr += 4;
              }

              // Tracking flags (NatNet version 2.6 and later)
              if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) ) 
              {
                ptr += 2;
              }
            } // next rigid body
          } // next skeleton
        }
        
        // labeled markers (NatNet version 2.3 and later)
        // labeled markers - this includes all markers: Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
        if( ((major == 2)&&(minor>=3)) || (major>2))
        {
          int nLabeledMarkers = 0;
          memcpy(&nLabeledMarkers, ptr, 4); ptr += 4;
          pImpl->markers.resize(nLabeledMarkers);
          // printf("Labeled Marker Count : %d\n", nLabeledMarkers);

          // Loop through labeled markers
          for (int j=0; j < nLabeledMarkers; j++)
          {
            // id
            // Marker ID Scheme:
            // Active Markers:
            //   ID = ActiveID, correlates to RB ActiveLabels list
            // Passive Markers: 
            //   If Asset with Legacy Labels
            //      AssetID   (Hi Word)
            //      MemberID  (Lo Word)
            //   Else
            //      PointCloud ID
            // int ID = 0; memcpy(&ID, ptr, 4);
            ptr += 4;
            // int modelID, markerID;
            // DecodeMarkerID(ID, &modelID, &markerID);

            memcpy(&pImpl->markers[j].x, ptr, 4); ptr += 4;
            memcpy(&pImpl->markers[j].y, ptr, 4); ptr += 4;
            memcpy(&pImpl->markers[j].z, ptr, 4); ptr += 4;
            // size
            //float size = 0.0f; memcpy(&size, ptr, 4);
            ptr += 4;

            // NatNet version 2.6 and later
            if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) ) 
            {
              // marker params
              // short params = 0; memcpy(&params, ptr, 2);
              ptr += 2;
              // bool bOccluded = (params & 0x01) != 0;     // marker was not visible (occluded) in this frame
              // bool bPCSolved = (params & 0x02) != 0;     // position provided by point cloud solve
              // bool bModelSolved = (params & 0x04) != 0;  // position provided by model solve
              // if ((major >= 3) || (major == 0))
              // {
              //   bool bHasModel = (params & 0x08) != 0;     // marker has an associated asset in the data stream
              //   bool bUnlabeled = (params & 0x10) != 0;    // marker is 'unlabeled', but has a point cloud ID
              //   bool bActiveMarker = (params & 0x20) != 0; // marker is an actively labeled LED marker
              // }
            }

            // NatNet version 3.0 and later
            // float residual = 0.0f;
            if ((major >= 3) || (major == 0))
            {
              // Marker residual
              // memcpy(&residual, ptr, 4);
              ptr += 4;
            }
          }
        }

        // Force Plate data (NatNet version 2.9 and later)
        if (((major == 2) && (minor >= 9)) || (major > 2))
        {
          int nForcePlates;
          memcpy(&nForcePlates, ptr, 4); ptr += 4;
          for (int iForcePlate = 0; iForcePlate < nForcePlates; iForcePlate++)
          {
            // ID
            // int ID = 0; memcpy(&ID, ptr, 4);
            ptr += 4;
            // printf("Force Plate : %d\n", ID);

            // Channel Count
            int nChannels = 0; memcpy(&nChannels, ptr, 4); ptr += 4;

            // Channel Data
            for (int i = 0; i < nChannels; i++)
            {
              // printf(" Channel %d : ", i);
              int nFrames = 0; memcpy(&nFrames, ptr, 4); ptr += 4;
              for (int j = 0; j < nFrames; j++)
              {
                  // float val = 0.0f;  memcpy(&val, ptr, 4);
                  ptr += 4;
                  // printf("%3.2f   ", val);
              }
              // printf("\n");
            }
          }
        }

        // Device data (NatNet version 3.0 and later)
        if (((major == 2) && (minor >= 11)) || (major > 2))
        {
          int nDevices;
          memcpy(&nDevices, ptr, 4); ptr += 4;
          for (int iDevice = 0; iDevice < nDevices; iDevice++)
          {
            // ID
            // int ID = 0; memcpy(&ID, ptr, 4);
            ptr += 4;
            // printf("Device : %d\n", ID);

            // Channel Count
            int nChannels = 0; memcpy(&nChannels, ptr, 4); ptr += 4;

            // Channel Data
            for (int i = 0; i < nChannels; i++)
            {
              // printf(" Channel %d : ", i);
              int nFrames = 0; memcpy(&nFrames, ptr, 4); ptr += 4;
              for (int j = 0; j < nFrames; j++)
              {
                  // float val = 0.0f;  memcpy(&val, ptr, 4); 
                  ptr += 4;
                  // printf("%3.2f   ", val);
              }
              // printf("\n");
            }
          }
        }
    
        // software latency (removed in version 3.0)
        if ( major < 3 )
        {
          // float softwareLatency = 0.0f; memcpy(&softwareLatency, ptr, 4);
          ptr += 4;
          // printf("software latency : %3.3f\n", softwareLatency);
        }

        // timecode
        // unsigned int timecode = 0;  memcpy(&timecode, ptr, 4);
        ptr += 4;
        // unsigned int timecodeSub = 0; memcpy(&timecodeSub, ptr, 4);
        ptr += 4;
        // char szTimecode[128] = "";
        // TimecodeStringify(timecode, timecodeSub, szTimecode, 128);

        // timestamp
        // double timestamp = 0.0f;

        // NatNet version 2.7 and later - increased from single to double precision
        if( ((major == 2)&&(minor>=7)) || (major>2))
        {
          // memcpy(&timestamp, ptr, 8);
          ptr += 8;
        }
        else
        {
          // float fTemp = 0.0f;
          // memcpy(&fTemp, ptr, 4);
          ptr += 4;
          // timestamp = (double)fTemp;
        }
        // printf("Timestamp : %3.3f\n", timestamp);

        // high res timestamps (version 3.0 and later)
        if ( (major >= 3) || (major == 0) )
        {
          // uint64_t cameraMidExposureTimestamp = 0;
          // memcpy( &cameraMidExposureTimestamp, ptr, 8 );
          ptr += 8;
          // printf( "Mid-exposure timestamp : %" PRIu64"\n", cameraMidExposureTimestamp );

          // uint64_t cameraDataReceivedTimestamp = 0;
          // memcpy( &cameraDataReceivedTimestamp, ptr, 8 );
          ptr += 8;
          // printf( "Camera data received timestamp : %" PRIu64"\n", cameraDataReceivedTimestamp );

          // uint64_t transmitTimestamp = 0;
          // memcpy( &transmitTimestamp, ptr, 8 );
          ptr += 8;
          // printf( "Transmit timestamp : %" PRIu64"\n", transmitTimestamp );
        }

        // frame params
        short params = 0;  memcpy(&params, ptr, 2);
        ptr += 2;
        // bool bIsRecording = (params & 0x01) != 0;                  // 0x01 Motive is recording
//        bool bTrackedModelsChanged = (params & 0x02) != 0;         // 0x02 Actively tracked model list has changed

        // end of data tag
        // int eod = 0; memcpy(&eod, ptr, 4); ptr += 4;
        // printf("End Packet\n-------------\n");
      }
      else
      {
          printf("Unrecognized Packet Type.\n");
      }
    }

  }

  const std::map<std::string, RigidBody>& MotionCaptureOptitrack::rigidBodies() const
  {
    // TODO: avoid copies here...
    rigidBodies_.clear();
    for (const auto& rb : pImpl->rigidBodies) {
      if (rb.bTrackingValid) {
        const auto& def = pImpl->rigidBodyDefinitions[rb.ID];

        Eigen::Vector3f position(
          rb.x + def.xoffset,
          rb.y + def.yoffset,
          rb.z + def.zoffset);

        Eigen::Quaternionf rotation(
          rb.qw, // w
          rb.qx, // x
          rb.qy, // y
          rb.qz  // z
          );
        rigidBodies_[def.name] = RigidBody(def.name, position, rotation);
      }
    }
    return rigidBodies_;
  }

  const PointCloud& MotionCaptureOptitrack::pointCloud() const
  {
    // TODO: avoid copies here...
    pointcloud_.resize(pImpl->markers.size(), Eigen::NoChange);
    for (size_t r = 0; r < pImpl->markers.size(); ++r) {
      const auto& marker = pImpl->markers[r];
      pointcloud_.row(r) << marker.x, marker.y, marker.z;
    }
    return pointcloud_;
  }

  MotionCaptureOptitrack::~MotionCaptureOptitrack()
  {
    delete pImpl;
  }

}

#else


namespace libmotioncapture {
  MotionCaptureOptitrack::MotionCaptureOptitrack(const std::string& hostname){ NICO }

    MotionCaptureOptitrack::~MotionCaptureOptitrack(){ NICO }
    const std::string& MotionCaptureOptitrack::version() const { NICO }
    void MotionCaptureOptitrack::waitForNextFrame(){ NICO }
    const std::map<std::string, RigidBody>& MotionCaptureOptitrack::rigidBodies() const{ NICO }
    const PointCloud& MotionCaptureOptitrack::pointCloud() const { NICO }
}

#endif //RAI_OPTITRACK
