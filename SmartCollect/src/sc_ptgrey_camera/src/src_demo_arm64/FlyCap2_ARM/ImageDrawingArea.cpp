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
// $Id: ImageDrawingArea.cpp,v 1.24 2009-08-26 18:30:35 soowei Exp $
//=============================================================================

#include "Precompiled.h"
#include "ImageDrawingArea.h"

	ImageDrawingArea::ImageDrawingArea(BaseObjectType* cobject, const Glib::RefPtr<Gnome::Glade::Xml>& /*refGlade*/)
: Gtk::DrawingArea(cobject),
	m_displayedFrameRate(60),
	m_moveCursor( Gdk::FLEUR )
{
	m_imageWidth = 0;
	m_imageHeight = 0;

	m_xPos = -1;
	m_yPos = -1;

	m_colorCrosshair.set_rgb_p( 1, 0, 0 );

	m_showCrosshair = false;
	m_stretchToFit = false;
	m_leftMBHeld = false;

	Gdk::EventMask mask =
		Gdk::BUTTON_PRESS_MASK |
		Gdk::BUTTON_RELEASE_MASK |
		Gdk::BUTTON1_MOTION_MASK |
		Gdk::POINTER_MOTION_MASK;
	set_events( mask );
}

ImageDrawingArea::~ImageDrawingArea()
{
}

bool ImageDrawingArea::on_button_press_event(GdkEventButton* event)
{
	switch ( event->button )
	{
		case 1:
			{
				m_leftMBHeld = true;
				m_startX = event->x;
				m_startY = event->y;
				get_window()->set_cursor( m_moveCursor );

				Gtk::Viewport* pViewport = (Gtk::Viewport*)get_parent();
				Gtk::Adjustment* pHAdjustment = pViewport->get_hadjustment();
				Gtk::Adjustment* pVAdjustment = pViewport->get_vadjustment();

				m_adjStartX = pHAdjustment->get_value();
				m_adjStartY = pVAdjustment->get_value();
			}
			break;
		default:
			break;
	}

	return true;
}

bool ImageDrawingArea::on_button_release_event( GdkEventButton* event )
{
	switch ( event->button )
	{
		case 1:
			m_leftMBHeld = false;
			get_window()->set_cursor();
			break;
		default:
			break;
	}

	return true;
}

bool ImageDrawingArea::on_motion_notify_event( GdkEventMotion* event )
{
	if ( m_leftMBHeld == true )
	{
		// Get the parent viewport
		Gtk::Viewport* pViewport = (Gtk::Viewport*)get_parent();

		// Do some calculations and apply the horizontal adjustment
		Gtk::Adjustment* pHAdjustment = pViewport->get_hadjustment();
		const double hMin = pHAdjustment->get_lower();
		const double hMax = pHAdjustment->get_upper();
		const double hPageSize = pHAdjustment->get_page_size();
		const double hRange = (hMax - hMin - hPageSize);
		const double hDiff = m_startX - event->x;
		double hVal = pHAdjustment->get_value() + hDiff;
		hVal = std::max( hVal, hMin );
		hVal = std::min( hVal, hMin + hRange );
		pHAdjustment->set_value( hVal );

		// Do some calculations and apply the vertical adjustment
		Gtk::Adjustment* pVAdjustment = pViewport->get_vadjustment();
		const double vMin = pVAdjustment->get_lower();
		const double vMax = pVAdjustment->get_upper();
		const double vPageSize = pVAdjustment->get_page_size();
		const double vRange = (vMax - vMin - vPageSize);
		const double vDiff = m_startY - event->y;
		double vVal = pVAdjustment->get_value() + vDiff;
		vVal = std::max( vVal, vMin );
		vVal = std::min( vVal, vMin + vRange );
		pVAdjustment->set_value( vVal );
	}
	else
	{
		const unsigned int absCurrXPos = static_cast<unsigned int>(event->x);
		const unsigned int absCurrYPos = static_cast<unsigned int>(event->y);

		// Get the drawing area size
		int screenWidth;
		int screenHeight;
		get_window()->get_size( screenWidth, screenHeight );

		unsigned int offsetX = 0;
		unsigned int offsetY = 0;

		if ( screenWidth < static_cast<int>(m_imageWidth) )
		{
			offsetX = 0;
		}
		else
		{
			offsetX = (screenWidth - m_imageWidth) / 2;
		}

		if ( screenHeight < static_cast<int>(m_imageHeight) )
		{
			offsetY = 0;
		}
		else
		{
			offsetY = (screenHeight - m_imageHeight) / 2;
		}

		if ( m_stretchToFit == true )
		{
			const float xScale = m_imageWidth / static_cast<float>(screenWidth);
			const float yScale = m_imageHeight / static_cast<float>(screenHeight);

			m_xPos = static_cast<int>(absCurrXPos * xScale);
			m_yPos = static_cast<int>(absCurrYPos * yScale);
		}
		else
		{
			if ( absCurrXPos > offsetX &&
					absCurrXPos - offsetX >= 0 &&
					absCurrXPos < offsetX + m_imageWidth &&
					absCurrYPos > offsetY &&
					absCurrYPos - offsetX >= 0 &&
					absCurrYPos < offsetY + m_imageHeight )
			{
				m_xPos = absCurrXPos - offsetX;
				m_yPos = absCurrYPos - offsetY;
			}
			else
			{
				m_xPos = -1;
				m_yPos = -1;
			}
		}
	}

	return true;
}

void ImageDrawingArea::SetImageSize( unsigned int width, unsigned int height )
{
	m_imageWidth = width;
	m_imageHeight = height;
}

void ImageDrawingArea::SetStretchToFit( bool stretch )
{
	m_stretchToFit = stretch;
}

void ImageDrawingArea::SetShowCrosshair( bool show )
{
	m_showCrosshair = show;
}

void ImageDrawingArea::SetCrosshairColor( Gdk::Color color )
{
	m_colorCrosshair = color;
}

Gdk::Color ImageDrawingArea::GetCrosshairColor()
{
	return m_colorCrosshair;
}

void ImageDrawingArea::GetMouseCoordinates( int* pXPos, int* pYPos )
{
	*pXPos = m_xPos;
	*pYPos = m_yPos;
}

bool ImageDrawingArea::on_expose_event( GdkEventExpose* /*event*/ )
{
	if ( m_pixBuf == NULL )
	{
		return true;
	}

	Glib::Mutex::Lock lock(m_pixBufMutex);

	int width = m_pixBuf->get_width();
	int height = m_pixBuf->get_height();

	if ( m_stretchToFit == true )
	{
		SetImageSize( width, height );

		set_size_request( 0, 0 );

		int screenWidth = 0;
		int screenHeight = 0;
		get_window()->get_size( screenWidth, screenHeight );

		Gdk::InterpType interpType;
		if ( screenWidth < width || screenHeight < height )
		{
			interpType = Gdk::INTERP_BILINEAR;
		}
		else
		{
			interpType = Gdk::INTERP_NEAREST;
		}

		m_scaledPixBuf = m_pixBuf->scale_simple( screenWidth, screenHeight, interpType );

		get_window()->draw_pixbuf(
				get_style()->get_black_gc(),
				m_scaledPixBuf,
				0,
				0,
				0,
				0,
				screenWidth,
				screenHeight,
				Gdk::RGB_DITHER_NONE,
				0,
				0 );
	}
	else
	{
		SetImageSize( width, height );

		set_size_request( width, height );

		int screenWidth = 0;
		int screenHeight = 0;
		get_window()->get_size( screenWidth, screenHeight );

		int offsetX = 0;
		int offsetY = 0;

		screenWidth < width ? offsetX = 0 : offsetX = (screenWidth - width) / 2;
		screenHeight < height ? offsetY = 0 : offsetY = (screenHeight - height) / 2;

		get_window()->draw_pixbuf(
				get_style()->get_black_gc(),
				m_pixBuf,
				0,
				0,
				offsetX,
				offsetY,
				width,
				height,
				Gdk::RGB_DITHER_NONE,
				0,
				0 );
	}

	lock.release();

	if ( m_showCrosshair )
	{
		ShowCrosshair();
	}

	m_displayedFrameRate.NewFrame();

	return true;
}

void ImageDrawingArea::SetPixBuf( Glib::RefPtr<Gdk::Pixbuf> refPixBuf )
{
	Glib::Mutex::Lock lock(m_pixBufMutex);

	m_pixBuf = refPixBuf;
}

void ImageDrawingArea::ShowCrosshair()
{
	Cairo::RefPtr<Cairo::Context> refCairo = get_window()->create_cairo_context();

	refCairo->save();

	// Get image dimensions
	int width = m_pixBuf->get_width();
	int height = m_pixBuf->get_height();

	// Get widget dimensions
	int screenWidth = 0;
	int screenHeight = 0;
	get_window()->get_size( screenWidth, screenHeight );

	// Set draw color
	refCairo->set_source_rgb(
			m_colorCrosshair.get_red_p(),
			m_colorCrosshair.get_green_p(),
			m_colorCrosshair.get_blue_p() );

	// Set line iWidth
	refCairo->set_line_width(2.0);

	double centerX = SnapValue( screenWidth / 2 );
	double centerY = SnapValue( screenHeight / 2 );

	// Move to the center
	refCairo->move_to( centerX, centerY );

	// Figure out the radius
	double radius;
	radius = SnapValue( std::max( width, height ) / 50 );

	// Draw horizontal line
	refCairo->rel_move_to( -radius, 0 );
	refCairo->rel_line_to( radius * 2, 0 );

	// Move back to the center
	refCairo->move_to( centerX, centerY );

	// Draw vertical line
	refCairo->rel_move_to( 0, -radius );
	refCairo->rel_line_to( 0, radius * 2 );

	refCairo->stroke();

	refCairo->restore();
}

double ImageDrawingArea::GetDisplayedFrameRate()
{
	return m_displayedFrameRate.GetFrameRate();
}

void ImageDrawingArea::ResetFrameRate()
{
	m_displayedFrameRate.Reset();
}

double ImageDrawingArea::SnapValue( double coord )
{
	return floor(coord) + 0.5;
}
