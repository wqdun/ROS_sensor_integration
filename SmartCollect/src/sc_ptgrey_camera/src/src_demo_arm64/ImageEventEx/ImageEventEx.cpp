//=============================================================================
// Copyright � 2017 FLIR Integrated Imaging Solutions, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: ImageEventEx.cpp 347612 2017-12-01 20:44:28Z wgallego $
//=============================================================================

//=============================================================================
//
// ImageEventEx
//   This example illustrates how users can implement partial image event
//   notification.  Partial image event notification is a mechanism that
//   provides the user with access to image data as it arrives in the PC's
//   memory - before the entire image is available.
//
//   This functionality is achieved by having the user associate a series of
//   events at various locations throughout the image.  The events are then
//   signaled as the matching portion of the image arrives.  This allows the
//   user to start processing the data immediately without having to wait for
//   image transmission to complete.
//
//   This type of capability is particularly useful in applications requiring
//   extremely low latency.  One example is in applications involving moving
//   the camera, stopping only to take pictures.  In this case, setting the
//   earliest event possible is a good method for indicating the end of
//   integration and that it is safe to move the camera without disrupting
//   affecting capture.
//
//   This functionality is also available with Custom Image mode however there
//   are some additional things to watch out for when using this mode.
//   Event notifications must be set on packet boundaries so you have to compute
//   the total image size including padding when deciding where to set event
//   sizes.  There will be at most one padded packet transmitted, so simply
//   taking the ceiling of the computed image size divided by the packet size:
//
//   numOfPackets = ceiling(( rows * cols * bytesPerPixel)/bytesPerPacket)
//
//   will give you the number of packets transmitter per image.
//
//   If the camera has already been started with the chosen bytes per packet,
//   this value can be queried from the format 7 registers. See the entry for
//   PACKET_PER_FRAME_INQ (0x048) in the PGR IEEE-1394 Digital Camera Register
//   Reference.
//
//   NOTE:
//   Depending on your operating system, in order for this example to work on
//   your PC, you may have to install a hotfix from Microsoft.
//
//   Please read the following knowledge base article on our website for more
//   information.
//
//   http://www.ptgrey.com/support/kb/details.asp?id=153
//
//=============================================================================

//=============================================================================
// Project Includes
//=============================================================================
#include "stdafx.h"

#include "FlyCapture2.h"
#include <iostream>
#include <math.h>
#include <sstream>

using namespace FlyCapture2;
using namespace std;
//=============================================================================
// Macro Definitions
//=============================================================================
#define _EVENTS 3
#define _BUFFERS 12

#define _COLS 640
#define _ROWS 480
#define _BYTES_PER_PIXEL 1
#define _IMAGE_SIZE _COLS *_ROWS *_BYTES_PER_PIXEL

// Only used for standard
#define _FRAMERATE FRAMERATE_30
#define _VIDEOMODE VIDEOMODE_640x480Y8

// Only used for custom image
#define _MODE MODE_0
#define _PIXEL_FORMAT PIXEL_FORMAT_MONO8

// Define this to use custom image
#define _USE_CUSTOM

//=============================================================================
// Linux implementation of _kbhit()
//=============================================================================
#if !defined(_WIN32) && !defined(_WIN64)
int _kbhit()
{
    static const int STDIN = 0;
    static bool initialized = false;

    if (!initialized)
    {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
#endif

//=============================================================================
// Function Definitions
//=============================================================================

void PrintBuildInfo()
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion(&fc2Version);

    ostringstream version;
    version << "FlyCapture2 library version: " << fc2Version.major << "."
            << fc2Version.minor << "." << fc2Version.type << "."
            << fc2Version.build;
    cout << version.str() << endl;

    ostringstream timeStamp;
    timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
    cout << timeStamp.str() << endl << endl;
}

void PrintError(Error error) { error.PrintErrorTrace(); }

int RunGrabLoop(Camera *pCamera)
{
    Error error;

    // Start capture
    error = pCamera->StartCapture();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    int imageCount = 0;
    while (_kbhit() == 0)
    {
        Image image;

        // Iterate through all the events
        for (int part = 0; part < _EVENTS; part++)
        {
            error = pCamera->WaitForBufferEvent(&image, part);
            if (error != PGRERROR_OK)
            {
                PrintError(error);
                return -1;
            }
            cout << "Received image event notification " << part
                 << " for image " << imageCount << endl;
        }

        imageCount++;
    }

    // Stop camera
    error = pCamera->StopCapture();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    // Disconnect camera
    error = pCamera->Disconnect();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    return 0;
}

int RunCustom()
{
    Format7Info fmt7Info;
    bool isSupported = false;
    Error error;

    Camera camera;
    error = camera.Connect();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    CameraInfo camInfo;
    error = camera.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    if (camInfo.interfaceType != INTERFACE_IEEE1394)
    {
        cout << "Only 1394 cameras are supported in this example." << endl;
        return -1;
    }

    // Check the video mode and framerate
    fmt7Info.mode = _MODE;
    error = camera.GetFormat7Info(&fmt7Info, &isSupported);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    if (isSupported == false)
    {
        // Format 7 mode is not valid for this camera
        cout << "The format 7 mode is not valid for this camera" << endl;
        return -1;
    }

    if ((_PIXEL_FORMAT & fmt7Info.pixelFormatBitField) == 0)
    {
        // Pixel format not supported!
        cout << "Pixel format is not supported" << endl;
        return -1;
    }

    if (_COLS > fmt7Info.maxWidth || _ROWS > fmt7Info.maxHeight)
    {
        // ROI is out of bounds!
        cout << "ROI is out of bounds!" << endl;
        return -1;
    }

    Format7ImageSettings fmt7ImageSettings;
    fmt7ImageSettings.mode = _MODE;
    fmt7ImageSettings.offsetX = 0;
    fmt7ImageSettings.offsetY = 0;
    fmt7ImageSettings.width = _COLS;
    fmt7ImageSettings.height = _ROWS;
    fmt7ImageSettings.pixelFormat = _PIXEL_FORMAT;

    bool isValid;
    Format7PacketInfo fmt7PacketInfo;

    // Validate the settings to make sure that they are valid
    error = camera.ValidateFormat7Settings(
        &fmt7ImageSettings, &isValid, &fmt7PacketInfo);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    if (!isValid)
    {
        // Settings are not valid
        cout << "Format7 settings are not valid" << endl;
        return -1;
    }

    // Set the settings to the camera
    error = camera.SetFormat7Configuration(
        &fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    // Set up the image event notifications.
    FC2Config config;
    config.numImageNotifications = _EVENTS;

    error = camera.SetConfiguration(&config);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    // Set user buffers

    // 1. Calculate the user buffers sizes on packet boundary
    unsigned int bytesPerPacket = fmt7PacketInfo.recommendedBytesPerPacket;
    unsigned int numOfPackets = static_cast<unsigned int>(
        ceil((_IMAGE_SIZE) / static_cast<double>(bytesPerPacket)));
    unsigned int imageSize = numOfPackets * bytesPerPacket;

    // 2. Allocate the memory for the user buffers
    unsigned char *pBuffers = new unsigned char[imageSize * _BUFFERS];

    // 3. Set the new allocated memory as the user buffers that the camera will
    // fill
    error = camera.SetUserBuffers(pBuffers, imageSize, _BUFFERS);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    int retVal = RunGrabLoop(&camera);
    if (retVal != 0)
    {
        delete[] pBuffers;
        return -1;
    }

    delete[] pBuffers;

    return 0;
}

int RunStandard()
{
    FC2Config config;
    bool isSupported = false;
    Error error;

    Camera camera;
    error = camera.Connect();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    // check the video mode and framerate
    error = camera.GetVideoModeAndFrameRateInfo(
        _VIDEOMODE, _FRAMERATE, &isSupported);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    if (isSupported == false)
    {
        // Videomode and framerate are not valid for this camera
        cout << "The video mode and frame rate is not supported on this camera"
             << endl;
        return -1;
    }

    // Set the VideoMode and FrameRate. This function is only used
    // for firewire and usb2 cameras
    error = camera.SetVideoModeAndFrameRate(_VIDEOMODE, _FRAMERATE);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    // Set up the image event notifications.
    config.numImageNotifications = _EVENTS;

    error = camera.SetConfiguration(&config);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    int retVal = RunGrabLoop(&camera);
    if (retVal != 0)
    {
        return -1;
    }

    return 0;
}

int main(int argc, char *argv[])
{
    PrintBuildInfo();

    int ch;

    cout << "*******************************************************" << endl;
    cout << "* WARNING - ensure you have Windows XP Service Pack 2 *" << endl;
    cout << "* or Microsoft Hotfix 94672 (Win2000) or Hotfix 94674 *" << endl;
    cout << "* (WinXP) installed on your PC to avoid a potential   *" << endl;
    cout << "* system crash. See the following for more info:      *" << endl;
    cout << "*                                                     *" << endl;
    cout << "* http://ptgrey.com/support/kb/index.asp?a=4&q=153    *" << endl;
    cout << "*******************************************************" << endl;

    cout << endl;
    cout << "Hit any key to continue, or the 'X' key to exit..." << endl;

    ch = cin.get();
    if (ch == 'X' || ch == 'x')
    {
        return 0;
    }

#if defined(_USE_CUSTOM)
    return RunCustom();
#else
    return RunStandard();
#endif
}
