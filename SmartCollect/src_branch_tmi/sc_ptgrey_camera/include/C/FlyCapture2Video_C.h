//=============================================================================
// Copyright (c) 2001-2018 FLIR Systems, Inc. All Rights Reserved.
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

#ifndef FLIR_FLYCAPTURE2VIDEO_C_H
#define FLIR_FLYCAPTURE2VIDEO_C_H

//=============================================================================
// Global C header file for FlyCapture2 Video.
//
// This file defines the C API for FlyCapture2 Video
//=============================================================================

#include "FlyCapture2Platform_C.h"
#include "FlyCapture2Defs_C.h"
#include "FlyCapture2VideoDefs_C.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
    * @defgroup CVideoRecorder Video Recording Operation
    *
    * @brief The video recording operation provides the functionality for the user to record
    * images to an video file.
    */
    /*@{*/

    /**
    * Create a Video context.
    *
    * @param pVideoContext A video context.
    *
    * @return A fc2Error indicating the success or failure of the function.
    */
    FLYCAPTURE2_C_API fc2Error
        fc2VideoCreate(
            fc2VideoContext* pVideoContext);

    /**
    * Open an AVI file in preparation for writing Images to disk.
    * The size of AVI files is limited to 2GB. The filenames are
    * automatically generated using the filename specified.
    *
    * @param VideoContext The video context to use.
    * @param pFileName The filename of the AVI file.
    * @param pOption Options to apply to the AVI file.
    *
    * @see SetMaximumFileSize()
    * @see fc2Close()
    * @see fc2AVIOption
    *
    * @return A fc2Error indicating the success or failure of the function.
    */
    FLYCAPTURE2_C_API fc2Error
        fc2VideoAVIOpen(
            fc2VideoContext VideoContext,
            const char*  pFileName,
            fc2AVIOption* pOption);

    /**
    * Open an MJPEG file in preparation for writing Images to disk.
    * The size of AVI files is limited to 2GB. The filenames are
    * automatically generated using the filename specified.
    *
    * @param VideoContext The AVI context to use.
    * @param pFileName The filename of the AVI file.
    * @param pOption Options to apply to the AVI file.
    *
    * @see fc2Close()
    * @see fc2MJPGOption
    *
    * @return A fc2Error indicating the success or failure of the function.
    */
    FLYCAPTURE2_C_API fc2Error
        fc2VideoMJPGOpen(
            fc2VideoContext VideoContext,
            const char*  pFileName,
            fc2MJPGOption* pOption);

    /**
    * Open an H.264 video file in preparation for writing Images to disk.
    * If the file extension is not specified, MP4 will be used as the default
    * container. Consult ffmpeg documentation for a list of supported containers.
    *
    * @param pFileName The filename of the video file.
    * @param pOption H.264 options to apply to the video file.
    *
    * @see fc2Close()
    * @see fc2H264Option
    *
    * @return A fc2Error indicating the success or failure of the function.
    */
    FLYCAPTURE2_C_API fc2Error
        fc2VideoH264Open(
            fc2VideoContext VideoContext,
            const char*  pFileName,
            fc2H264Option* pOption);

    /**
    * Append an image to the video file.
    *
    * @param VideoContext The video context to use.
    * @param pImage The image to append.
    *
    * @return A fc2Error indicating the success or failure of the function.
    */
    FLYCAPTURE2_C_API fc2Error
        fc2VideoAppend(
            fc2VideoContext VideoContext,
            fc2Image* pImage);

    /**
    * Set the maximum file size (in megabytes) of a AVI/MP4 file. A new AVI/MP4 file
    * is created automatically when file size limit is reached. Setting
    * a maximum size of 0 indicates no limit on file size.
    *
    * @param VideoContext The video context to use.
    * @param size The maximum video file size in MB.
    *
    * @return A fc2Error indicating the success or failure of the function.
    */
    FLYCAPTURE2_C_API fc2Error
        fc2VideoSetMaximumSize(
            fc2VideoContext VideoContext,
            unsigned int size);

    /**
    * Close the video file.
    *
    * @param VideoContext The video context to use.
    *
    * @return A fc2Error indicating the success or failure of the function.
    */
    FLYCAPTURE2_C_API fc2Error
        fc2VideoClose(
            fc2VideoContext VideoContext);

    /**
    * Destroy a video context.
    *
    * @param VideoContext A video context.
    *
    * @return A fc2Error indicating the success or failure of the function.
    */
    FLYCAPTURE2_C_API fc2Error
        fc2VideoDestroy(
            fc2VideoContext VideoContext);

    /*@}*/

#ifdef __cplusplus
}
#endif

/*@}*/

#endif /* FLIR_FLYCAPTURE2VIDEO_C_H */