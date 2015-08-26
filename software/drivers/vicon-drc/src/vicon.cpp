#include <ViconDataStreamSDK_CPP/Client.h>
#include <lcmtypes/vicon_drc.hpp>
#include <lcm/lcm-cpp.hpp>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <unistd.h>

// Should eventually get rid of this :P
using namespace ViconDataStreamSDK::CPP;

/**
  * Print out the members of a viconstructs::vicon_t LCM message
  * @param vicon the message
  */
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
            std::cout << "x: " << marker.xyz[0] << " y: " << marker.xyz[1] << " z: " << marker.xyz[2] << " o: " << marker.o << std::endl << std::endl;
        }

        std::cout <<  model.numsegments << " Segments:\n" << std::endl;
        for(j = 0; j < model.numsegments; j++)
        {
            viconstructs::segment_t& segment = model.segments[j];
            std::cout << "Segment: " << segment.name << std::endl;
            std::cout << "A: x: " << segment.A[0] << " y: " << segment.A[1] << " z: " << segment.A[2] << std::endl;
            std::cout << "T: x: " << segment.T[0] << " y: " << segment.T[1] << " z: " << segment.T[2] << std::endl;
            std::cout << "ba: x: " << segment.ba[0] << " y: " << segment.ba[1] << " z: " << segment.ba[2] << std::endl;
            std::cout << "bt: x: " << segment.bt[0] << " y: " << segment.bt[1] << " z: " << segment.bt[2] << std::endl;
            std::cout << std::endl;
        }
        std::cout << std::endl << std::endl;
    }
}

/**
  * A set of functions which finds and returns an object with a given name from a std::vector
  * or, if it doesn't exist yet, appends a new object with the name to the vector and returns it
  * I feel like there should be a better way to do this via polymorphism, templates, or C++11
  * but copy/paste+regex is easy enough
  */
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

/**
  * Converts a double[3] to a viconstructs::xyz_t
  * @param vals the xyz values to place
  * @return the new struct
  */
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
    // Broadcast lcm channel
    lcm::LCM lcm;
    if (!lcm.good())
    {
        return 1;
    }

    // Setup vars
    // Connect to the client
    std::string HostName = "192.168.20.99:801";
    // Number of times to repeat
    int numtimes = 1;
    int udelay = 0;

    // Parse command line flags
    while (--argc > 0)
    {
        std::string sw(*(++argv));
        if (sw == "-h")
        {
            std::cout << "Available flags (default val):" << std::endl;
            std::cout << "-h                 Display help" << std::endl;
            std::cout << "-host hostname     Set hostname (" << HostName << ")" << std::endl;
            std::cout << "-n numtimes        Number of times to repeat (" << numtimes << ")" << std::endl;
            std::cout << "-d delay           Delay between loop in microseconds (" << udelay << ")" << std::endl;
            return 0;
        }
        if (--argc > 0)
        {
            std::string arg(*(++argv));
            if (sw == "-host")
            {
                HostName = arg;
            }
            else
            {
                std::stringstream ss(arg);

                if (sw == "-n")
                {
                    ss >> numtimes;
                }
                else if (sw == "-d")
                {
                    ss >> udelay;
                }
                else
                {
                    std::cout << "Error: unknown flag " << sw << std::endl;
                    return 1;
                }

                if (!ss)
                {
                    std::cout << "Error parsing " << arg << " as int" << std::endl;
                    return 1;
                }
            }
        }
        else
        {
            std::cout << "Error: No arg passed to flag " << sw << std::endl;
            return 1;
        }

    }
    std::cout << "Parameters:" << std::endl;
    std::cout << "Hostname: " << HostName << std::endl;
    std::cout << "numtimes: " << numtimes << std::endl;
    std::cout << "udelay: " << udelay << std::endl;
    std::cout << std::endl;

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

    // Switch to repeat infinite number of times vs 

    while (numtimes < 0 || numtimes--) // repeat if a negative number is passed
    {
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
            // Get the model
            viconstructs::model_t& model = findModel(system.models, vicon.GetSubjectName(i).SubjectName);
            
            // Get markers
            model.nummarkers = vicon.GetMarkerCount(model.name).MarkerCount;
            if (model.nummarkers < model.markers.size())
            {
                model.markers.empty();
                model.markers.reserve(model.nummarkers);
            }

            // Get marker data
            for(int32_t j = 0; j < model.nummarkers; j++)
            {
                viconstructs::marker_t& marker = findMarker(model.markers, vicon.GetMarkerName(model.name, j).MarkerName);
                Output_GetMarkerGlobalTranslation translation = vicon.GetMarkerGlobalTranslation(model.name, marker.name);
                marker.o = translation.Occluded;
                memcpy(marker.xyz, translation.Translation, 3*sizeof(double));
            }

            // Get segments
            model.numsegments = vicon.GetSegmentCount(model.name).SegmentCount;
            if (model.numsegments < model.segments.size())
            {
                model.segments.empty();
                model.segments.reserve(model.numsegments);
            }

            // Get segment data
            for(int32_t j = 0; j < model.numsegments; j++)
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

                //segment.A = getXYZ(A.Rotation);
                //segment.T = getXYZ(T.Translation);
                //segment.ba = getXYZ(ba.Rotation);
                //segment.bt = getXYZ(bt.Translation);
            }
        }

        //printLCM(system);
        std::cout << "Message Sent" << std::endl;

        lcm.publish("drc_vicon", &system);
        if (udelay)
        {
            usleep(udelay);
        }
    }
    vicon.Disconnect();
    return 0;
}
