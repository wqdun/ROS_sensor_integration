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

#ifndef FLIR_FLYCAPTURE2VIDEO_H
#define FLIR_FLYCAPTURE2VIDEO_H

#include "FlyCapture2Platform.h"
#include "FlyCapture2VideoDefs.h"

namespace FlyCapture2
{
    class Error;
    class Image;

    /**
     * The FlyCapture2Video class provides the functionality for the user to record
     * images to an AVI file.
     */
    class FLYCAPTURE2_API FlyCapture2Video
    {
    public:

        /**
         * Default constructor.
         */
        FlyCapture2Video();

        /**
         * Default destructor.
         */
        virtual ~FlyCapture2Video();

        /**
         * Open an AVI file in preparation for writing Images to disk.
         * The size of AVI files is limited to 2GB. The filenames are
         * automatically generated using the filename specified.
         *
         * @param pFileName The filename of the AVI file.
         * @param pOption Options to apply to the AVI file.
         *
         * @see SetMaximumFileSize()
         * @see Close()
         *
         * @return An Error indicating the success or failure of the function.
         */
        virtual Error Open(
            const char* pFileName,
            AVIOption*  pOption);

        /**
         * Open an MJPEG AVI file in preparation for writing Images to disk.
         * The size of AVI files is limited to 2GB. The filenames are
         * automatically generated using the filename specified.
         *
         * @param pFileName The filename of the AVI file.
         * @param pOption MJPEG options to apply to the AVI file.
         *
         * @see SetMaximumFileSize()
         * @see Close()
         * @see MJPGOption
         *
         * @return An Error indicating the success or failure of the function.
         */
        virtual Error Open(
            const char* pFileName,
            MJPGOption*  pOption);


        /**
         * Open an H.264 video file in preparation for writing Images to disk.
         * If the file extension is not specified, MP4 will be used as the default
         * container. Consult ffmpeg documentation for a list of supported containers.
         *
         * @param pFileName The filename of the video file.
         * @param pOption H.264 options to apply to the video file.
         *
         * @see Close()
         * @see H264Option
         *
         * @return An Error indicating the success or failure of the function.
         */
        virtual Error Open(
            const char* pFileName,
            H264Option*  pOption);


        /**
         * Append an image to the AVI/MP4 file.
         *
         * @param pImage The image to append.
         *
         * @return An Error indicating the success or failure of the function.
         */
        virtual Error Append(Image* pImage);

        /**
         * Close the AVI/MP4 file.
         *
         * @see Open()
         *
         * @return An Error indicating the success or failure of the function.
         */
        virtual Error Close();

        /**
        * Set the maximum file size (in megabytes) of a AVI/MP4 file. A new AVI/MP4 file
        * is created automatically when file size limit is reached. Setting
        * a maximum size of 0 indicates no limit on file size.
        *
        * @param size The maximum AVI file size in MB.
        *
        * @see Append()
        *
        */
        virtual void SetMaximumFileSize(unsigned int size);

    private:

        FlyCapture2Video(const FlyCapture2Video&);
        FlyCapture2Video& operator=(const FlyCapture2Video&);

        struct FlyCapture2VideoData; // Forward declaration

        FlyCapture2VideoData* m_pFlyCapture2VideoData;
    };
}

#endif // FLIR_FLYCAPTURE2VIDEO_H
