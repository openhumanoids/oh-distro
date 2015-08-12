/* Maintainers: Claudia Perez D'Arpino, Eric Bakan (Based on previous MIT driver)
 * Driver for using vicon with LCM. Tested in the Holodeck.
 * Requirements: ViconDataStreamSDK
 *
 */

#include <sys/time.h>
#include <stdio.h>
#include <iostream>
#include <ViconDataStreamSDK_CPP/Client.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/vicon_drc.hpp>

using namespace ViconDataStreamSDK::CPP;


// this is a local version of bot_timestamp_now 
// inserted here to avoid dependency for now, mfallon, jan 2013:
int64_t _timestamp_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void printLCM(viconstructs::vicon_t& vicon)
{
    std::cout << "Vicon: " << vicon.utime << " | "<< vicon.nummodels <<  " Models" << std::endl;
    int32_t i, j;
    for(i = 0; i < vicon.nummodels; i++)
    {
        viconstructs::model_t& model = vicon.models[i];
        std::cout << "Model " << i+1 << ": " << model.name << std::endl;

        std::cout << model.nummarkers << " Markers:" << std::endl;
        for(j = 0; j < model.nummarkers; j++)
        {
            viconstructs::marker_t& marker = model.markers[j];
            std::cout << "Marker: " << marker.name << std::endl;
            std::cout << "x: " << marker.xyz[0]<< " y: " << marker.xyz[1] << " z: " << marker.xyz[2] << " o: " << marker.o<< std::endl << std::endl;
        }

        std::cout <<  model.numsegments << " Segments:\n" << std::endl;
        for(j = 0; j < model.numsegments; j++)
        {
            viconstructs::segment_t& segment = model.segments[j];
            std::cout << "Segment: " << segment.name << std::endl;
            std::cout << "A: x: " << segment.A[0]<< " y: " << segment.A[1]<< " z: " << segment.A[2]<< std::endl;
            std::cout << "T: x: " << segment.T[0]<< " y: " << segment.T[1]<< " z: " << segment.T[2]<< std::endl;
            std::cout << "ba: x: " << segment.ba[0]<< " y: " << segment.ba[1]<< " z: " << segment.ba[2]<< std::endl;
            std::cout << "bt: x: " << segment.bt[0]<< " y: " << segment.bt[1]<< " z: " << segment.bt[2]<< std::endl;
            std::cout << std::endl;
        }
    }
}

viconstructs::model_t& findModel(std::vector<viconstructs::model_t>& models, std::string modelname)
{
    for(int i = 0; i < models.size(); i++)
    {
        if (models[i].name == modelname)
        {
            return models[i];
        }
    }
    viconstructs::model_t model;
    model.name = modelname;
    model.nummarkers = 0;
    model.numsegments = 0;
    models.push_back(model);
    return models.back();
}

viconstructs::marker_t& findMarker(std::vector<viconstructs::marker_t>& markers, std::string markername)
{
    for(int i = 0; i < markers.size(); i++)
    {
        if (markers[i].name == markername)
        {
            return markers[i];
        }
    }
    viconstructs::marker_t marker;
    marker.name = markername;
    markers.push_back(marker);
    return markers.back();
}

viconstructs::segment_t& findSegment(std::vector<viconstructs::segment_t>& segments, std::string segmentname)
{
    for(int i = 0; i < segments.size(); i++)
    {
        if (segments[i].name == segmentname)
        {
            return segments[i];
        }
    }
    viconstructs::segment_t segment;
    segment.name = segmentname;
    segments.push_back(segment);
    return segments.back();
}

viconstructs::xyz_t getXYZ(double vals[3])
{
    viconstructs::xyz_t xyz;
    xyz.x = vals[0];
    xyz.y = vals[1];
    xyz.z = vals[2];
    return xyz;
}

int main(int argc, char** argv)
{

    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " <vicon tracker host:port ex. 192.168.10.1:801>" << std::endl;
        return 1;
    }

    lcm::LCM lcm;
    if (!lcm.good())
    {
        return 1;
    }

    std::string HostName = argv[1];
    std::cout << "Using Vicon host: " << HostName << std::endl;
    std::cout << "Connecting to Vicon host..." << std::endl;
    Client vicon;
    while (!vicon.IsConnected().Connected)
    {
        vicon.Connect( HostName );
    }

    std::cout << "Connected to Vicon host." << std::endl;

    // Enable some different data types
    vicon.EnableSegmentData();
    vicon.EnableMarkerData();
    vicon.EnableUnlabeledMarkerData();
    vicon.EnableDeviceData();
    vicon.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );

    // Set the global up axis
    vicon.SetAxisMapping( Direction::Forward, Direction::Left, Direction::Up ); // Z-up

    std::cout << "Vicon Initialized" << std::endl;

    viconstructs::vicon_t system;

    int counter = 0;
    while (1)
    {
        // Get a frame
        while( vicon.GetFrame().Result != Result::Success ) { }

        system.nummodels = vicon.GetSubjectCount().SubjectCount;

        // Uh oh, models disappeared - realloc the system
        if (system.nummodels < system.models.size())
        {
            system.models.clear();
            system.models.reserve(system.nummodels);
        }

        if (system.nummodels == 0)
        {
            std::cout << "no models detected" << std::endl;
            continue;
        }

        // Generate the models

        for (int32_t i = 0; i < system.nummodels; i++)
        {
            viconstructs::model_t& model = findModel(system.models, vicon.GetSubjectName(i).SubjectName);

            model.nummarkers = vicon.GetMarkerCount(model.name).MarkerCount;
            if (model.nummarkers < model.markers.size())
            {
                model.markers.clear();
                model.markers.reserve(model.nummarkers);
            }

            for(int32_t j = 0; j < model.nummarkers; j++)
            {
                viconstructs::marker_t& marker = findMarker(model.markers, vicon.GetMarkerName(model.name, j).MarkerName);
                Output_GetMarkerGlobalTranslation translation = vicon.GetMarkerGlobalTranslation(model.name, marker.name);
                marker.o = translation.Occluded;
                memcpy(marker.xyz, translation.Translation, 3*sizeof(double));
            }

            model.numsegments = vicon.GetSegmentCount(model.name).SegmentCount;
            if (model.numsegments < model.segments.size())
            {
                model.segments.clear();
                model.segments.reserve(model.numsegments);
            }

            for (int32_t j = 0; j < model.numsegments; j++)
            {
                viconstructs::segment_t& segment = findSegment(model.segments, vicon.GetSegmentName(model.name, j).SegmentName);
                Output_GetSegmentGlobalRotationEulerXYZ A = vicon.GetSegmentGlobalRotationEulerXYZ(model.name, segment.name);
                Output_GetSegmentGlobalTranslation T = vicon.GetSegmentGlobalTranslation(model.name, segment.name);
                Output_GetSegmentLocalRotationEulerXYZ ba = vicon.GetSegmentLocalRotationEulerXYZ(model.name, segment.name);
                Output_GetSegmentLocalTranslation bt = vicon.GetSegmentLocalTranslation(model.name, segment.name);
                memcpy(segment.A, A.Rotation, 3*sizeof(double));
                memcpy(segment.T, T.Translation, 3*sizeof(double));
                memcpy(segment.ba, ba.Rotation, 3*sizeof(double));
                memcpy(segment.bt, bt.Translation, 3*sizeof(double));
            }

        }

        system.utime = _timestamp_now();
        lcm.publish("drc_vicon", &system);


        if (++counter % 200 == 0)
        {
            printLCM(system);
            std::cout << counter <<" @ " << system.utime << "\n";
        }
    }

    vicon.Disconnect();
    return 0;
}
