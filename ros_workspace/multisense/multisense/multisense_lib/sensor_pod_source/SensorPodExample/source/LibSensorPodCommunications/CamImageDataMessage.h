/**
 * @file LibSensorPodCommunications/CamImageDataMessage.h
 *
 * This message contains image data, but no stereo data.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-05-02, dlr@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_CamImageDataMessage
#define LibSensorPodCommunications_CamImageDataMessage

#include "AbstractSerializedMessage.h"
#include "malloc.h"

class CamImageDataMessage : AbstractSerializedMessage
{
private:

	//
	// Indicates if image data was allocated in the deserialize
	// routine, so we can clean it up.
	//

	bool allocatedLocally;

public:
     enum {MSG_ID = SP_CAM_IMAGE_DATA};

    //
    // Metadata about this camera image.
    //

    float framesPerSecond;
    float gain;

    uint32_t exposureTime;

    uint32_t frameCount;

    crl::TimeStamp timeStamp;
    
    // Microradians.
    uint32_t angle;

    uint16_t width;
    uint16_t height;

    //
    // Image data.
    //

    uint8_t *leftImage;
    uint8_t *rightImage;

    /*
     * Constructor. Does nothing but initialize locals.
     */
    CamImageDataMessage()
      : allocatedLocally(false),
        framesPerSecond(0),
        gain(0.0),
        exposureTime(0),
        frameCount(0),
        timeStamp(0.0),
        angle(0),
        width(0),
        height(0),
        leftImage(0),
        rightImage(0)
    {
    }


    /*
     * This constructor pre-allocates storage for images.
     */
    CamImageDataMessage(uint32_t frameCountArg,
                        crl::TimeStamp const& timeStampArg,
                        uint16_t widthArg,
                        uint16_t heightArg)
      : allocatedLocally(true),
        framesPerSecond(0),
        gain(0.0),
        exposureTime(0),
        frameCount(frameCountArg),
        timeStamp(timeStampArg),
        angle(0),
        width(widthArg),
        height(heightArg),
        leftImage((uint8_t*)malloc(width * height * sizeof(uint8_t))),
        rightImage((uint8_t*)malloc(width * height * sizeof(uint8_t)))
    {
        if(!leftImage || !rightImage) {
            throw std::bad_alloc();
        }
    }

 
    /*
     * Destructor. Cleans up allocated images (if they were allocated).
     */
    ~CamImageDataMessage()
    {
    	if(allocatedLocally)
    	{
    		free(leftImage);
    		free(rightImage);
    	}
    }

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(SP_CAM_IMAGE_DATA);

        //
        // Write metadata.
        //

        message << framesPerSecond;
        message << gain;
        message << exposureTime;

        message << frameCount;

        message << timeStamp;
        
        message << angle;

        message << width;
        message << height;

        //
        // Write the image blobs here.
        //

        message.write((const unsigned char*) leftImage, sizeof(uint8_t) * width * height);
        message.write((const unsigned char*) rightImage, sizeof(uint8_t) * width * height);
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(SP_CAM_IMAGE_DATA);

        //
        // Read metadata.
        //

        message >> framesPerSecond;
		message >> gain;
		message >> exposureTime;

        message >> frameCount;

        message >> timeStamp;
        
        message >> angle;

        message >> width;
        message >> height;

        //
        // Allocate room for these images.
        //

        allocatedLocally = true;

        leftImage = (uint8_t*) malloc(sizeof(uint8_t) * width * height);
        message.read((unsigned char*) leftImage, sizeof(uint8_t) * width * height);

        rightImage = (uint8_t*) malloc(sizeof(uint8_t) * width * height);
        message.read((unsigned char*) rightImage, sizeof(uint8_t) * width * height);
    }
};

#endif
