#include <ViconDataStreamSDK_CPP/Client.h>
#include <lcmtypes/vicon_drc.hpp>
#include <lcm/lcm-cpp.hpp>
#include <stdio.h>
#include <iostream>

using namespace ViconDataStreamSDK::CPP;

void printLCM(viconstructs::vicon_t& vicon)
{
    std::cout << "Vicon: " << vicon.nummodels <<  " Models" << std::endl;
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
            std::cout << "x: " << marker.xyz.x << " y: " << marker.xyz.y << " z: " << marker.xyz.z << " o: " << marker.xyz.x << std::endl << std::endl;
        }

        std::cout <<  model.numsegments << " Segments:\n" << std::endl;
        for(j = 0; j < model.numsegments; j++)
        {
            viconstructs::segment_t& segment = model.segments[j];
            std::cout << "Segment: " << segment.name << std::endl;
            std::cout << "A: x: " << segment.A.x << " y: " << segment.A.x << " z: " << segment.A.x << std::endl;
            std::cout << "T: x: " << segment.T.x << " y: " << segment.T.x << " z: " << segment.T.x << std::endl;
            std::cout << "ba: x: " << segment.ba.x << " y: " << segment.ba.x << " z: " << segment.ba.x << std::endl;
            std::cout << "bt: x: " << segment.bt.x << " y: " << segment.bt.x << " z: " << segment.bt.x << std::endl;
            std::cout << std::endl;
        }
        std::cout << std::endl << std::endl;
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
    lcm::LCM lcm;
    if (!lcm.good())
    {
        return 1;
    }

    std::string HostName = "192.168.20.99:801";
    Client vicon;
    while( !vicon.IsConnected().Connected )
    {
        vicon.Connect( HostName );
    }

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
    // Get a frame
    while( vicon.GetFrame().Result != Result::Success ) { }
    int nummodels = vicon.GetSubjectCount().SubjectCount;

    // Uh oh, models disappeared - realloc the system
    if (nummodels < system.nummodels)
    {
        system.nummodels = nummodels;
        system.models.empty();
        system.models.reserve(nummodels);
    }

    // Generate the models
    for(int32_t i = 0; i < nummodels; i++)
    {
        viconstructs::model_t& model = findModel(system.models, vicon.GetSubjectName(i).SubjectName);

        
        model.nummarkers = vicon.GetMarkerCount(model.name).MarkerCount;
        if (model.nummarkers < model.markers.size())
        {
            model.markers.empty();
            model.markers.reserve(model.nummarkers);
        }

        for(int32_t j = 0; j < model.nummarkers; j++)
        {
            viconstructs::marker_t& marker = findMarker(model.markers, vicon.GetMarkerName(model.name, j).MarkerName);
            Output_GetMarkerGlobalTranslation translation = vicon.GetMarkerGlobalTranslation(model.name, marker.name);
            marker.o = translation.Occluded;
            marker.xyz = getXYZ(translation.Translation);
        }

        model.numsegments = vicon.GetSegmentCount(model.name).SegmentCount;
        if (model.numsegments < model.segments.size())
        {
            model.segments.empty();
            model.segments.reserve(model.numsegments);
        }

        for(int32_t j = 0; j < model.numsegments; j++)
        {
            viconstructs::segment_t& segment = findSegment(model.segments, vicon.GetSegmentName(model.name, j).SegmentName);
            Output_GetSegmentGlobalRotationEulerXYZ A = vicon.GetSegmentGlobalRotationEulerXYZ(model.name, segment.name);
            Output_GetSegmentGlobalTranslation T = vicon.GetSegmentGlobalTranslation(model.name, segment.name);
            Output_GetSegmentLocalRotationEulerXYZ ba = vicon.GetSegmentLocalRotationEulerXYZ(model.name, segment.name);
            Output_GetSegmentLocalTranslation bt = vicon.GetSegmentLocalTranslation(model.name, segment.name);
            segment.A = getXYZ(A.Rotation);
            segment.T = getXYZ(T.Translation);
            segment.ba = getXYZ(ba.Rotation);
            segment.bt = getXYZ(bt.Translation);
        }
    }
    //printLCM(system);
    lcm.publish("drc_vicon", &system);
    vicon.Disconnect();
    return 0;
}
