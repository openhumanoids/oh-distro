/**
 * @file LibSensorPodCommunications/CamDataMessage.h
 *
 * This message contains raw LIDAR data.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-04-14, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_CamDataMessage
#define LibSensorPodCommunications_CamDataMessage

#include "AbstractSerializedMessage.h"
#include "malloc.h"

class CamDataMessage : AbstractSerializedMessage
{
private:
	//
	// Indicates if image data was allocated in the deserialize
	// routine, so we can clean it up.
	//

	bool allocatedLocally;

public:
    enum {MSG_ID = SP_CAM_DATA};

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
    // Raw picture data. These are points to make
    // transmitting to the network API a little
    // faster.
    //

    uint8_t *grayScaleImage;
    uint16_t *disparityImage;

    /*
     * Constructor. Does nothing but initialize locals.
     */
    CamDataMessage()
      : allocatedLocally(false),
        framesPerSecond(0),
        gain(0.0),
        exposureTime(0),
        frameCount(0),
        timeStamp(0.0),
        angle(0),
        width(0),
        height(0),
        grayScaleImage(0),
        disparityImage(0)
    {
      // Empty.
    }
  

    /*
     * Constructor that preallocates storage.
     */
    CamDataMessage(uint32_t frameCountArg,
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
        grayScaleImage((uint8_t*)malloc(width * height * sizeof(uint8_t))),
        disparityImage((uint16_t*)malloc(width * height * sizeof(uint16_t)))
    {
      // Empty.
    }
  
  
    /*
     * Destructor. Cleans up allocated images (if they were allocated).
     */
    ~CamDataMessage()
    {
    	if(allocatedLocally)
    	{
    		free(grayScaleImage);
    		free(disparityImage);
    	}
    }

    //
    // Serialization routine.
    //

    virtual void serialize(SensorPodMessageBuffer& message)
    {
        message.setType(SP_CAM_DATA);

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
        // Write the raw blobs here.
        //

        message.write((const unsigned char*) grayScaleImage, sizeof(uint8_t) * width * height);
        message.write((const unsigned char*) disparityImage, sizeof(uint16_t) * width * height);
    }

    //
    // Deserialization routine.
    //

    virtual void deserialize(SensorPodMessageBuffer& message)
    {
        message.confirmType(SP_CAM_DATA);

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

        grayScaleImage = (uint8_t*) malloc(sizeof(uint8_t) * width * height);
        message.read((unsigned char*) grayScaleImage, sizeof(uint8_t) * width * height);

        disparityImage = (uint16_t*) malloc(sizeof(uint16_t) * width * height);
		message.read((unsigned char*) disparityImage, sizeof(uint16_t) * width * height);
    }
};

#endif
