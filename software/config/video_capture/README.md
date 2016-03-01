# Video Capture

To enable video capture on a machine add the following to your ``~/.bashrc`` and also set the location where to save the logs:

```
export OH_VIDEO_CAPTURE=1
export OH_VIDEO_CAPTURE_LOCATION=~/logs/video-logs
```

## Setup

1. Download the Blackmagic driver from [here](https://software.blackmagicdesign.com/DesktopVideo/v10.6/Blackmagic_Desktop_Video_Linux_10.6.tar.gz?__token__=exp=1456833843~acl=/DesktopVideo/v10.6/Blackmagic_Desktop_Video_Linux_10.6.tar.gz*~hmac=07eefa1b88dbe92a5123e7c2eae67918318eb461aec73516f42d6a4fe8fae166)
2. Install the driver in the archive: ``sudo dpkg -i desktopvideo_*.deb``
3. Run ``sudo modprobe blackmagic`` to load the driver
4. Compile the DecklinkCapture utility by: ``cd $DRC_BASE/software/drivers/decklink && make``
