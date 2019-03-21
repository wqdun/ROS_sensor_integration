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

#ifndef FLIR_FLYCAPTURE2VIDEODEFS_C_H
#define FLIR_FLYCAPTURE2VIDEODEFS_C_H

//=============================================================================
// Definitions header file for FlyCapture2 Video C API.
//
// Holds enumerations, typedefs and structures that are used across the
// FlyCapture2 Video C API wrapper.
//
// Please see FlyCapture2Defs.h or the API documentation for full details
// of the various enumerations and structures.
//=============================================================================

#ifdef __cplusplus
extern "C"
{
#endif

    /**
    * @defgroup CTypeDefs TypeDefs
    */

    /*@{*/

    /**
    * A context referring to the video recorder object.
    */
    typedef void* fc2VideoContext;

    /*@}*/

    /**
    * @defgroup CStructures Structures
    */

    /*@{*/

    /**
    * @defgroup CVideoSaveStructures Video saving structures.
    *
    * These structures define various parameters used for saving videos.
    */

    /*@{*/

    /** Options for saving MJPG files. */
    typedef struct _fc2MJPGOption
    {
        /** Frame rate of the stream */
        float frameRate;
        /** Image quality (1-100) */
        unsigned int quality;
        unsigned int reserved[256];

    } fc2MJPGOption;

    /** Options for saving H264 files. */
    typedef struct _fc2H264Option
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

    } fc2H264Option;

    /** Options for saving AVI files. */
    typedef struct _fc2AVIOption
    {
        /** Frame rate of the stream */
        float frameRate;
        /** Reserved for future use */
        unsigned int reserved[256];
    } fc2AVIOption;

    /*@}*/

    /*@}*/

#ifdef __cplusplus
};
#endif

#endif // FLIR_FLYCAPTURE2VIDEODEFS_C_H