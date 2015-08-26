#ifndef __STRUCTS_H_
#define __STRUCTS_H_

#include "lcmtypes/vicon.h"

/**
  * Vicon-specific data structures and algorithms
  */

/**
  * Represents a linked list node which contains a mapping of vicon channel to
  * pointer where the data on the channel should go
  */
typedef struct channelmap_t {
    unsigned int channel;
    double* datapos;
    struct channelmap_t* next;
} channelmap_t;

/**
  * Contains three channel numbers for a tuple of (x,y,z) values
  */
typedef struct {
    unsigned int x;
    unsigned int y;
    unsigned int z;
} xyz_descriptor_t;

/**
  * Linked list node which contains a marker's name and the channel numbers of
  * its x,y,z,o values
  */
typedef struct marker_descriptor_t {
    char* name;
    xyz_descriptor_t xyz;
    unsigned int o;
    struct marker_descriptor_t* next;
} marker_descriptor_t;

/**
  * Linked list node which contains the segment name and the channels of all of
  * all its data members
  */
typedef struct segment_descriptor_t {
    char* name;
    xyz_descriptor_t A;
    xyz_descriptor_t T;
    xyz_descriptor_t ba;
    xyz_descriptor_t bt;
    xyz_descriptor_t r;
    xyz_descriptor_t t;
    struct segment_descriptor_t* next;
} segment_descriptor_t;

/**
  * Linked list node which contains the model name and linked lists of its
  * markers and segments
  */
typedef struct model_descriptor_t {
    char* name;
    marker_descriptor_t* markers;
    segment_descriptor_t* segments;
    struct model_descriptor_t* next;
} model_descriptor_t;

/**
  * Free a linked list of channelmap_t
  */
void free_channelmap(channelmap_t* channelmap);

/**
  * Free a linked list of model_descriptor_t
  * Frees each node's name, markers, and segments as well
  */
void free_models(model_descriptor_t* models);

/**
  * Free a linked list of marker_descriptor_t
  * Frees each node's name as well
  */
void free_markers(marker_descriptor_t* markers);

/**
  * Free a linked list of segment_descriptor_t
  * Frees each node's name as well
  */
void free_segments(segment_descriptor_t* segments);

/**
  * Given a new channel name/number, insert it into the existing linked list of
  * models
  * Assumes that models is initialized, though it can have null/0 values for
  * its data members
  * @param channelName the full channel name given by the vicon
  * @param channelNum the channel number
  * @param models the linkedlist of models describing the system as parsed so far
  */
int addChannel(char* channelName, unsigned int channelNum, model_descriptor_t* models);

/**
  * Pushes up to 3 channelmap_t's to the end of channelmaps which map the
  * channels in xyzd to the data positions in xyz
  * @param xyz the LCM data structure to be modified when vicon broadcasts new data
  * @param xyzd the channel numbers for a tuple of (x,y,z) data values
  * @param channelmaps the linkedlist to append to
  * @return the new tail of the linked list
  */
channelmap_t* mapVec(viconstructs_xyz_t* xyz, xyz_descriptor_t* xyzd, channelmap_t* channelmaps);

/**
  * Given a linked list of model descriptors and an initialized channelmaps
  * node, creates the LCM viconstructs_vicon_t* structure and the channelmap_t
  * for easy updating
  * @param models the linked list of all models in the system
  * @param channelmaps the initialized (but possibly null/0-valued) linked list
  * of channel mapping
  * @return the complete LCM viconstructs_vicon_t system descriptor structure
  */
viconstructs_vicon_t* genLCM(model_descriptor_t* models, channelmap_t* channelmaps);

/**
  * Prints out the current model_descriptor_t linked list
  * Useful for debugging
  * @param channelmaps the linked list to traverse
  */
void printModelDescriptors(model_descriptor_t* models);

/**
  * Prints out the current viconstructs_vicon_t linked list
  * Useful for debugging
  * @param channelmaps the linked list to traverse
  */
void printLCM(viconstructs_vicon_t* vicon);

/**
  * Prints out the current channelmap_t linked list
  * Useful for debugging
  * @param channelmaps the linked list to traverse
  */
void printChannelMaps(channelmap_t* channelmaps);

#endif // __STRUCTS_H_
