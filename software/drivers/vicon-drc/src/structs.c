#include "structs.h"

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// Define an enum for data type checking
typedef enum dataype
{
    UNKNOWN,
    MARKER,
    SEGMENT,
    NUMDATATYPES
} datatype;

void free_channelmap(channelmap_t* channelmap)
{
    if (channelmap)
    {
        if (channelmap->next)
        {
            free_channelmap(channelmap->next);
        }
        free(channelmap);
    }
}

void free_models(model_descriptor_t* models)
{
    if (models)
    {
        if (models->name)
        {
            free(models->name);
        }
        // Null checks built-into function call
        free_markers(models->markers);
        free_segments(models->segments);
        free_models(models->next);
        free(models);
    }
}

void free_markers(marker_descriptor_t* markers)
{
    if (markers)
    {
        if (markers->name)
        {
            free(markers->name);
        }
        free_markers(markers->next);
        free(markers);
    }
}

void free_segments(segment_descriptor_t* segments)
{
    if (segments)
    {
        if (segments->name)
        {
            free(segments->name);
        }
        free_segments(segments->next);
        free(segments);
    }
}

// Determine whether a channel datum describes marker, segment, or unknown channel
datatype getDataType(char* datum)
{
    if (datum[0] == 'P')
    {
        return MARKER;
    }
    if (datum[0] == 'A' ||
        datum[0] == 'T' ||
        datum[0] == 'r' ||
        datum[0] == 't' ||
        (datum[0] == 'b' && (datum[1] == 'a' || datum[1] == 't')))
    {
        return SEGMENT;
    }
    return UNKNOWN;
}


int addChannel(char* channelName, unsigned int channelNum, model_descriptor_t* models)
{
    // Null check
    if (!models || !channelName)
    {
        return 0;
    }

    // Get the datum
    size_t len = strlen(channelName);
    size_t datumlen=0; // offset because segment channels can have a longer datum length (e.g. 'X-ba' vs 'X-A')
    char* datum;
    if (channelName[len-5] == '<')
    {
        datum=channelName+len-4;
    }
    else if (channelName[len-6] == '<')
    {
        datumlen=1;
        datum=channelName+len-5;
    }
    else
    {
        // Neither a segment nor a marker
        return 0;
    }

    // Parse based on data type
    datatype type = getDataType(datum);
    if (type == MARKER || type == SEGMENT)
    {
        // Get the separator char pos
        char* cp = strrchr(channelName, ':');
        if (!cp)
        {
            return 0;
        }

        // Get the model and item names
        unsigned int modellen=(cp-channelName) / sizeof(char);
        size_t itemnamelen = len-modellen-7-datumlen;
        char* itemname = malloc((itemnamelen+1) * sizeof(char));
        char* modelname = malloc((modellen+1) * sizeof(char));
        strncpy(itemname,cp+1,itemnamelen);
        itemname[itemnamelen]=0;
        strncpy(modelname,channelName,modellen);
        modelname[modellen]=0;

        // Find / generate new model
        model_descriptor_t* model = models;
        while (model)
        {
            // First run through, no data initialized
            if (!model->name)
            {
                model->name = modelname;
                modelname = NULL;
                break;
            }
            // Found right model
            if (strcmp(model->name,modelname) == 0)
            {
                break;
            }
            // At end of the line, need to create new model
            // NOTE: at the beginning of the next loop, the new model
            // will be at the head of the search and will trigger the
            // !model->name case above, exiting properly
            if (!model->next)
            {
                model_descriptor_t* newmodel = malloc(sizeof(model_descriptor_t));
                newmodel->name = NULL;
                newmodel->markers = NULL;
                newmodel->segments = NULL;
                newmodel->next = NULL;
                model->next = newmodel;
            }
            model = model->next;
        }

        // Have the right model, add the new item

        if (type == MARKER)
        {
            // Find the proper marker
            marker_descriptor_t* marker = model->markers;

            // No existing markers
            if (!marker)
            {
                marker = malloc(sizeof(marker_descriptor_t));
                marker->name = NULL;
                marker->next = NULL;
                model->markers = marker;
            }
            // Existing markers
            else
            {
                // Same as above
                while (marker)
                {
                    if (!marker->name)
                    {
                        marker->name = itemname;
                        itemname = NULL;
                        break;
                    }
                    if (strcmp(marker->name, itemname) == 0)
                    {
                        break;
                    }
                    if (!marker->next)
                    {
                        marker_descriptor_t* newmarker = malloc(sizeof(marker_descriptor_t));
                        newmarker->name = NULL;
                        newmarker->next = NULL;
                        marker->next = newmarker;
                    }
                    marker = marker->next;
                }
            }
            
            // Assign the channelnum to the right pos
            switch(datum[2])
            {
                case 'X':
                    marker->xyz.x = channelNum;
                    break;
                case 'Y':
                    marker->xyz.y = channelNum;
                    break;
                case 'Z':
                    marker->xyz.z = channelNum;
                    break;
                case 'O':
                    marker->o = channelNum;
                    break;
                default:
                    break;
            }

        }
        // Segment
        else
        {
            // Find the proper segment
            segment_descriptor_t* segment = model->segments;

            // No existing segments
            if (!segment)
            {
                segment = malloc(sizeof(segment_descriptor_t));
                segment->name = NULL;
                segment->next = NULL;
                model->segments = segment;
            }
            // Existing segments
            else
            {
                // Same as above
                while (segment)
                {
                    if (!segment->name)
                    {
                        segment->name = itemname;
                        itemname = NULL;
                        break;
                    }
                    if (strcmp(segment->name, itemname) == 0)
                    {
                        break;
                    }
                    if (!segment->next)
                    {
                        segment_descriptor_t* newsegment = malloc(sizeof(segment_descriptor_t));
                        newsegment->name = NULL;
                        newsegment->next = NULL;
                        segment->next = newsegment;
                    }
                    segment = segment->next;
                }
            }

            // Assign the channelnum to the right pos
            switch(datum[0])
            {
                case 'A':
                    switch(datum[2])
                    {
                        case 'X':
                            segment->A.x = channelNum;
                            break;
                        case 'Y':
                            segment->A.y = channelNum;
                            break;
                        case 'Z':
                            segment->A.z = channelNum;
                            break;
                        default:
                            break;
                    }
                    break;
                case 'T':
                    switch(datum[2])
                    {
                        case 'X':
                            segment->T.x = channelNum;
                            break;
                        case 'Y':
                            segment->T.y = channelNum;
                            break;
                        case 'Z':
                            segment->T.z = channelNum;
                            break;
                        default:
                            break;
                    }
                    break;
                case 'r':
                    switch(datum[2])
                    {
                        case 'X':
                            segment->r.x = channelNum;
                            break;
                        case 'Y':
                            segment->r.y = channelNum;
                            break;
                        case 'Z':
                            segment->r.z = channelNum;
                            break;
                        default:
                            break;
                    }
                    break;
                case 't':
                    switch(datum[2])
                    {
                        case 'X':
                            segment->t.x = channelNum;
                            break;
                        case 'Y':
                            segment->t.y = channelNum;
                            break;
                        case 'Z':
                            segment->t.z = channelNum;
                            break;
                        default:
                            break;
                    }
                    break;
                case 'b':
                    switch(datum[1])
                    {
                        case 'a':
                            switch(datum[3])
                            {
                                case 'X':
                                    segment->ba.x = channelNum;
                                    break;
                                case 'Y':
                                    segment->ba.y = channelNum;
                                    break;
                                case 'Z':
                                    segment->ba.z = channelNum;
                                    break;
                                default:
                                    break;
                            }
                            break;
                        case 't':
                            switch(datum[3])
                            {
                                case 'X':
                                    segment->bt.x = channelNum;
                                    break;
                                case 'Y':
                                    segment->bt.y = channelNum;
                                    break;
                                case 'Z':
                                    segment->bt.z = channelNum;
                                    break;
                                default:
                                    break;
                            }
                            break;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }

        }

        // Clean up unused char*'s
        if (itemname)
        {
            free(itemname);
        }
        if (modelname)
        {
            free(modelname);
        }
        return 1;
    }

    return 0;
}

channelmap_t* mapVec(viconstructs_xyz_t* xyz, xyz_descriptor_t* xyzd, channelmap_t* channelmaps)
{
    channelmap_t* c = channelmaps; // Save head
    if (!xyz || !xyzd || !channelmaps || !(xyzd->x || xyzd->y || xyzd->z))
    {
        return channelmaps;
    }

    channelmap_t *x = NULL, *y = NULL, *z = NULL;

    // Gen channelmaps
    if (xyzd->x)
    {
        x = malloc(sizeof(channelmap_t));
        x->channel = xyzd->x;
        x->datapos = &(xyz->x);
    }

    if (xyzd->y)
    {
        y = malloc(sizeof(channelmap_t));
        y->channel = xyzd->y;
        y->datapos = &(xyz->y);
    }

    if (xyzd->z)
    {
        z = malloc(sizeof(channelmap_t));
        z->channel = xyzd->z;
        z->datapos = &(xyz->z);
    }

    // Not established yet, init
    if (!channelmaps->datapos)
    {
        if (x)
        {
            channelmaps->channel = x->channel;
            channelmaps->datapos = x->datapos;
            free(x);
            x = NULL;
        }
        else if (y)
        {
            channelmaps->channel = y->channel;
            channelmaps->datapos = y->datapos;
            free(y);
            y = NULL;
        }
        else if (z)
        {
            channelmaps->channel = z->channel;
            channelmaps->datapos = z->datapos;
            free(z);
            z = NULL;
        }
        else
        {
            return channelmaps;
        }
    }

    // Push the vars
    if (x)
    {
        channelmaps->next = x;
        channelmaps = x;
    }

    if (y)
    {
        channelmaps->next = y;
        channelmaps = y;
    }

    if (z)
    {
        channelmaps->next = z;
        channelmaps = z;
    }

    channelmaps->next = NULL;

    return channelmaps;

}


viconstructs_vicon_t* genLCM(model_descriptor_t* models, channelmap_t* channelmaps)
{
    if (!models || !channelmaps)
    {
        return NULL;
    }

    viconstructs_xyz_t zerovec = {0.0, 0.0, 0.0};

    viconstructs_vicon_t* vicon = malloc(sizeof(viconstructs_vicon_t));
    vicon->nummodels = 0;
    vicon->models = NULL;

    // 2-pass: once for counts, then again for data

    // Models
    model_descriptor_t* model;
    for(model = models; model; model = model->next)
    {
        vicon->nummodels++;
    }
    printf("vicon has %d models\n",vicon->nummodels);
    vicon->models = malloc(vicon->nummodels * sizeof(viconstructs_model_t));

    int32_t modelnum = 0;
    for(model = models; model; model = model->next)
    {
        // Init the drc model
        viconstructs_model_t* drcmodel = vicon->models+modelnum;
        drcmodel->name = strdup(model->name);
        drcmodel->nummarkers = 0;
        drcmodel->numsegments = 0;
        drcmodel->markers = NULL;
        drcmodel->segments = NULL;

        int32_t itemnum = 0;

        // Markers
        marker_descriptor_t* marker;
        for(marker = model->markers; marker; marker = marker->next)
        {
            drcmodel->nummarkers++;
        }

        printf("model %d has %d markers\n",modelnum,drcmodel->nummarkers);

        drcmodel->markers = malloc(drcmodel->nummarkers * sizeof(viconstructs_marker_t));

        for(marker = model->markers; marker; marker = marker->next)
        {
            // Init the drc marker
            viconstructs_marker_t* drcmarker = drcmodel->markers+itemnum;
            drcmarker->name = strdup(marker->name);
            drcmarker->xyz = zerovec;
            drcmarker->o = 0;

            // Add the channelmaps
            // Add XYZ
            channelmap_t* c = channelmaps;
            channelmaps = mapVec(&(drcmarker->xyz), &(marker->xyz), channelmaps);
            // Add O
            if (channelmaps && marker->o)
            {
                if(!channelmaps->datapos)
                {
                    channelmaps->channel = marker->o;
                    channelmaps->datapos = &(drcmarker->o);
                    channelmaps->next = NULL;
                }
                else
                {
                    channelmap_t* o = malloc(sizeof(channelmap_t));
                    o->channel = marker->o;
                    o->datapos = &(drcmarker->o);
                    o->next = NULL;
                    channelmaps->next = o;
                    channelmaps = o;
                }
            }
            int incr = 0;
            while (c)
            {
                incr++;
                c = c->next;
            }
            printf("just added %d marker mappings\n",incr);

            itemnum++;
        }
        
        // Segments
        itemnum = 0;
        segment_descriptor_t* segment;
        for(segment = model->segments; segment; segment = segment->next)
        {
            drcmodel->numsegments++;
        }

        printf("model %d has %d segments\n",modelnum,drcmodel->numsegments);

        drcmodel->segments = malloc(drcmodel->numsegments * sizeof(viconstructs_segment_t));

        for(segment = model->segments; segment; segment = segment->next)
        {
            // Init the drc segment
            viconstructs_segment_t* drcsegment = drcmodel->segments+itemnum;
            drcsegment->name = strdup(segment->name);
            drcsegment->A = zerovec;
            drcsegment->T = zerovec;
            drcsegment->ba = zerovec;
            drcsegment->bt = zerovec;
            drcsegment->r = zerovec;
            drcsegment->t = zerovec;

            // Add the channelmaps
            channelmap_t* c = channelmaps;
            channelmaps = mapVec(&(drcsegment->A), &(segment->A), channelmaps);
            channelmaps = mapVec(&(drcsegment->T), &(segment->T), channelmaps);
            channelmaps = mapVec(&(drcsegment->ba), &(segment->ba), channelmaps);
            channelmaps = mapVec(&(drcsegment->bt), &(segment->bt), channelmaps);
            channelmaps = mapVec(&(drcsegment->r), &(segment->r), channelmaps);
            channelmaps = mapVec(&(drcsegment->t), &(segment->t), channelmaps);
            int incr = 0;
            while (c)
            {
                incr++;
                c = c->next;
            }
            incr--;
            printf("just added %d segment mappings\n",incr);

            itemnum++;
        }

        modelnum++;
    }
    return vicon;
}

void printModelDescriptors(model_descriptor_t* models)
{
    while (models)
    {
        if (models->name)
        {
            printf("Model: %s\n", models->name);
        }
        if (models->markers)
        {
            marker_descriptor_t* marker = models->markers;
            while (marker)
            {
                printf("Marker: %s\n", marker->name);
                printf("x: %d y: %d z: %d o: %d\n", marker->xyz.x, marker->xyz.y, marker->xyz.z, marker->o);
                printf("\n");
                marker = marker->next;
            }
        }
        if (models->segments)
        {
            segment_descriptor_t* segment = models->segments;
            while (segment)
            {
                printf("Segment: %s\n", segment->name);
                printf("A: x: %d y: %d z: %d\n", segment->A.x, segment->A.y, segment->A.z);
                printf("T: x: %d y: %d z: %d\n", segment->T.x, segment->T.y, segment->T.z);
                printf("ba: x: %d y: %d z: %d\n", segment->ba.x, segment->ba.y, segment->ba.z);
                printf("bt: x: %d y: %d z: %d\n", segment->bt.x, segment->bt.y, segment->bt.z);
                printf("r: x: %d y: %d z: %d\n", segment->r.x, segment->r.y, segment->r.z);
                printf("t: x: %d y: %d z: %d\n", segment->t.x, segment->t.y, segment->t.z);
                printf("\n");
                segment = segment->next;
            }
        }
        models = models->next;
    }
}

void printLCM(viconstructs_vicon_t* vicon)
{
    if (!vicon)
    {
        return;
    }
    printf("Vicon: %" PRId32 " Models\n", vicon->nummodels);
    int32_t i, j;
    for(i = 0; i < vicon->nummodels; i++)
    {
        viconstructs_model_t* model = vicon->models+i;
        printf("Model %" PRId32 ": %s\n", i+1, model->name);

        printf("%" PRId32 "  Markers:\n", model->nummarkers);
        for(j = 0; j < model->nummarkers; j++)
        {
            viconstructs_marker_t* marker = model->markers+j;
            printf("Marker: %s\n", marker->name);
            printf("x: %f y: %f z: %f o: %f\n\n", marker->xyz.x, marker->xyz.y, marker->xyz.z, marker->o);
        }

        printf("\n%" PRId32 " Segments:\n", model->numsegments);
        for(j = 0; j < model->numsegments; j++)
        {
            viconstructs_segment_t* segment = model->segments+j;
            printf("Segment: %s\n", segment->name);
            printf("A: x: %f y: %f z: %f\n", segment->A.x, segment->A.y, segment->A.z);
            printf("T: x: %f y: %f z: %f\n", segment->T.x, segment->T.y, segment->T.z);
            printf("ba: x: %f y: %f z: %f\n", segment->ba.x, segment->ba.y, segment->ba.z);
            printf("bt: x: %f y: %f z: %f\n", segment->bt.x, segment->bt.y, segment->bt.z);
            printf("r: x: %f y: %f z: %f\n", segment->r.x, segment->r.y, segment->r.z);
            printf("t: x: %f y: %f z: %f\n", segment->t.x, segment->t.y, segment->t.z);
            printf("\n");
        }
        printf("\n\n");
    }
}

void printChannelMaps(channelmap_t* channelmaps)
{
    while (channelmaps && channelmaps->channel && channelmaps->datapos)
    {
        printf("Channel %d: %p\n", channelmaps->channel, channelmaps->datapos);
        channelmaps = channelmaps->next;
    }
}
