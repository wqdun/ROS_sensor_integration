//=============================================================================
// Copyright © 2008 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with PGR.
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: ImageDrawingArea.h,v 1.15 2009-08-26 18:30:35 soowei Exp $
//=============================================================================

#ifndef PGR_FC2_IMAGEDRAWINGAREA_H
#define PGR_FC2_IMAGEDRAWINGAREA_H

#include "FrameRateCounter.h"

/**
 * This class inherits from Gtk::DrawingArea and provides the ability to
 * draw RGB/RGBU images to the screen.
 */
class ImageDrawingArea : public Gtk::DrawingArea
{
	public:
		/** Constructor. */
		ImageDrawingArea(BaseObjectType* cobject, const Glib::RefPtr<Gnome::Glade::Xml>& refGlade);

		/** Destructor. */
		virtual ~ImageDrawingArea();

		/**
		 * Set the pixbuf to draw on to the DrawingArea.
		 *
		 * @param refPixBuf The pixbuf containing the image data to be drawn.
		 */
		void SetPixBuf(Glib::RefPtr<Gdk::Pixbuf> refPixBuf);

		/**
		 * Set the stretch parameter of the DrawingArea.
		 *
		 * @param stretch Whether to stretch the image to fit the DrawingArea.
		 */
		void SetStretchToFit( bool stretch );

		/**
		 * Set the crosshair parameter of the DrawingArea.
		 *
		 * @param show Whether to show a crosshair.
		 */
		void SetShowCrosshair( bool show );

		/**
		 * Set the crosshair color.
		 *
		 * @param color The crosshair to set.
		 */
		void SetCrosshairColor( Gdk::Color color );

		/**
		 * Get the crosshair color.
		 *
		 * @return The crosshair color.
		 */
		Gdk::Color GetCrosshairColor();

		/**
		 * Get the current position of the mouse cursor over the DrawingArea.
		 * The value is given in image coordinates.
		 *
		 * @param pXPos X position of the cursor.
		 * @param pYPos Y position of the cursor.
		 */
		void GetMouseCoordinates( int* pXPos, int* pYPos );

		/**
		 * Get the displayed frame rate.
		 *
		 * @return Displayed frame rate.
		 */
		double GetDisplayedFrameRate();

		/** Reset the frame rate. */
		void ResetFrameRate();

	protected:
		/**
		 * Implementation of on_expose_event() to draw the image
		 * to the drawing area.
		 *
		 * @param event The expose event.
		 */
		virtual bool on_expose_event(GdkEventExpose* event);

	private:
		int m_xPos;
		int m_yPos;

		unsigned int m_imageWidth;
		unsigned int m_imageHeight;

		bool m_showCrosshair;
		bool m_stretchToFit;

		bool m_leftMBHeld;

		FrameRateCounter m_displayedFrameRate;

		Gdk::Color m_colorCrosshair;

		// Fleur cursor (for moving)
		Gdk::Cursor m_moveCursor;

		double m_startX;
		double m_startY;

		double m_adjStartX;
		double m_adjStartY;

		Glib::RefPtr<Gdk::Pixbuf> m_pixBuf;
		Glib::RefPtr<Gdk::Pixbuf> m_scaledPixBuf;

		Glib::Mutex m_pixBufMutex;

		static double SnapValue( double coord );

		bool on_button_press_event(GdkEventButton* event);
		bool on_button_release_event( GdkEventButton* event );
		bool on_motion_notify_event (GdkEventMotion* event);

		void SetImageSize( unsigned int width, unsigned int height );

		void ShowCrosshair();
};

#endif // #ifndef PGR_FC2_IMAGEDRAWINGAREA_H
