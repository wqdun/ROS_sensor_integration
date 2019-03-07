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

#ifndef FLIR_FLYCAPTURE2VIDEODEFS_H
#define FLIR_FLYCAPTURE2VIDEODEFS_H

#include <memory.h>

namespace FlyCapture2
{
    /**
     * @defgroup VideoSaveStructures Video saving structures.
     *
     * These structures define various parameters used for saving videos.
     */

    /*@{*/

    /** Options for saving MJPG files. */
    struct MJPGOption
    {
        /** Frame rate of the stream */
        float frameRate;

        /** Image quality (1-100) */
        unsigned int quality;

        unsigned int reserved[256];

        MJPGOption()
        {
            frameRate = 15.0;
            quality = 75;
            memset(reserved, 0, sizeof (reserved));
        }
    };

    /** Options for saving H264 files. */
    struct H264Option
    {
        /** Frame rate of the stream */
        float frameRate;

        /** Width of source image */
        unsigned int width;

        /** Height of source image */
        unsigned int height;

        /** Bitrate to encode at */
        unsigned int bitrate;

        /** Reserved for future use */
        unsigned int reserved[256];

        H264Option()
        {
            frameRate = 15.0;
            width = 0;
            height = 0;
            bitrate = 1000000;
            memset(reserved, 0, sizeof (reserved));
        }
    };

    /** Options for saving AVI files. */
    struct AVIOption
    {
        /** Frame rate of the stream */
        float frameRate;

        /** Reserved for future use */
        unsigned int reserved[256];

        AVIOption()
        {
            frameRate = 15.0;
            memset(reserved, 0, sizeof (reserved));
        }
    };

    /*@}*/
}

#endif // FLIR_FLYCAPTURE2VIDEODEFS_H