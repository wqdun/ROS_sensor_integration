//=============================================================================
// Copyright � 2008 Point Grey Research, Inc. All Rights Reserved.
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
// $Id: FlycapWindow.cpp,v 1.124 2009-09-08 22:53:20 soowei Exp $
//=============================================================================

#include "Precompiled.h"
#include "FlycapWindow.h"
#include "SaveImageFileChooserDialog.h"
#include "HelpLauncher.h"
#include <string.h>

int FlycapWindow::m_activeWindows = 0;

FlycapWindow::FlycapWindow() :
	m_processedFrameRate(60)
{
	m_run = false;
	m_pNewImageEvent = NULL;
	m_pBusArrivalEvent = NULL;
	m_pBusRemovalEvent = NULL;

	m_prevPanePos = 225;

	m_pInformationPane = NULL;
	m_pArea = NULL;
	m_pHistogramWindow = NULL;

	m_numEmitted = 0;
}

FlycapWindow::~FlycapWindow()
{
	if ( m_pInformationPane != NULL )
	{
		delete m_pInformationPane;
		m_pInformationPane = NULL;
	}

	if ( m_pHistogramWindow != NULL )
	{
		delete m_pHistogramWindow;
		m_pHistogramWindow = NULL;
	}

	if ( m_pWindow != NULL )
	{
		delete m_pWindow;
		m_pWindow = NULL;
	}
}


	bool
FlycapWindow::Initialize()
{
	// Load Glade file

	const char* k_flycap2Glade = "FlyCap2.glade";

#ifdef GLIBMM_EXCEPTIONS_ENABLED
	try
	{
		m_refXml = Gnome::Glade::Xml::create(k_flycap2Glade);
	}
	catch(const Gnome::Glade::XmlError& ex)
	{
		char szSecondary[512];
		sprintf(
				szSecondary,
				"Error: %s. Make sure that the file is present.",
				ex.what().c_str() );

		Gtk::MessageDialog dialog( "Error loading Glade file", false, Gtk::MESSAGE_ERROR );
		dialog.set_secondary_text( szSecondary );
		dialog.run();

		return false;
	}
#else
	std::auto_ptr<Gnome::Glade::XmlError> error;
	m_refXml = Gnome::Glade::Xml::create(k_flycap2Glade, "", "", error);
	if(error.get())
	{
		char szSecondary[512];
		sprintf(
				szSecondary,
				"Error: %s. Make sure that the file is present.",
				ex.what().c_str() );

		Gtk::MessageDialog dialog( "Error loading Glade file", false, Gtk::MESSAGE_ERROR );
		dialog.set_secondary_text( szSecondary );
		dialog.run();

		return false;
	}
#endif

	m_refXml->get_widget( "window", m_pWindow );
	if ( m_pWindow == NULL )
	{
		return false;
	}

	GetWidgets();
	AttachSignals();

	LoadPGRLogo();
	LoadFlyCap2Icon();

	m_pWindow->set_default_icon( m_iconPixBuf );
	m_pWindow->set_default_size( 1024, 768 );
	m_pScrolledWindow->set_policy( Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC );

	m_pInformationPane->Initialize();

	UpdateColorProcessingMenu();

	m_pMenuDrawImage->set_active( true );
	m_pMenuShowToolbar->set_active( true );
	m_pMenuShowInfoPane->set_active( true );

	return true;
}

	void
FlycapWindow::GetWidgets()
{
	// Menu bar
	m_refXml->get_widget("menubar", m_pMenubar);

	// Tool bar
	m_refXml->get_widget("toolbar", m_pToolbar);

	// File menu
	m_refXml->get_widget("menu_new_camera", m_pMenuNewCamera);
	m_refXml->get_widget("menu_start", m_pMenuStart);
	m_refXml->get_widget("menu_stop", m_pMenuStop);
	m_refXml->get_widget("menu_save_as", m_pMenuSaveAs);
	m_refXml->get_widget("menu_quit", m_pMenuQuit);

	// View menu
	m_refXml->get_widget("menu_draw_image", m_pMenuDrawImage);
	m_refXml->get_widget("menu_draw_crosshair", m_pMenuDrawCrosshair);
	m_refXml->get_widget("menu_change_crosshair_color", m_pMenuChangeCrosshairColor);
	m_refXml->get_widget("menu_show_toolbar", m_pMenuShowToolbar);
	m_refXml->get_widget("menu_show_info_pane", m_pMenuShowInfoPane);
	m_refXml->get_widget("menu_stretch_image", m_pMenuStretchImage);
	m_refXml->get_widget("menu_fullscreen", m_pMenuFullscreen);

	// Color processing menu
	m_refXml->get_widget("menu_cpa_none", m_pMenuCPA_None);
	m_refXml->get_widget("menu_cpa_nnf", m_pMenuCPA_NNF);
	m_refXml->get_widget("menu_cpa_hq_linear", m_pMenuCPA_HQ_Linear);
	m_refXml->get_widget("menu_cpa_edge_sensing", m_pMenuCPA_Edge_Sensing);
	m_refXml->get_widget("menu_cpa_rigorous", m_pMenuCPA_Rigorous);

	// Help menu
	m_refXml->get_widget("menu_help", m_pMenuHelp);
	m_refXml->get_widget("menu_about", m_pMenuAbout);

	// Tool bar buttons
	m_refXml->get_widget("toolbutton_item_new_camera", m_pNewCameraButton);
	m_refXml->get_widget("toolbar_item_start", m_pStartButton);
	m_refXml->get_widget("toolbar_item_stop", m_pStopButton);
	m_refXml->get_widget("toolbar_item_cam_ctl", m_pCamCtlButton);
	m_refXml->get_widget("toolbar_item_histogram", m_pHistogramButton);

	// The scrolled window that holds the drawing area
	m_refXml->get_widget("scrolledwindow1", m_pScrolledWindow);

	// Status bar
	m_refXml->get_widget("statusbarRGB", m_pStatusBarRGB);

	// Pane that holds the scrolled window and info pane
	m_refXml->get_widget_derived("hpaned", m_pInformationPane);

	// Custom drawing area
	m_refXml->get_widget_derived("disp_image", m_pArea);

	// Histogram window
	m_refXml->get_widget_derived("window_histogram", m_pHistogramWindow);
}

	void
FlycapWindow::AttachSignals()
{
	m_pWindow->signal_delete_event().connect(sigc::mem_fun( *this, &FlycapWindow::OnDestroy ));

	m_pMenuNewCamera->signal_activate().connect(sigc::mem_fun( *this, &FlycapWindow::OnToolbarNewCamera ));
	m_pMenuStart->signal_activate().connect(sigc::mem_fun( *this, &FlycapWindow::OnToolbarStart ));
	m_pMenuStop->signal_activate().connect(sigc::mem_fun( *this, &FlycapWindow::OnToolbarStop ));
	m_pMenuSaveAs->signal_activate().connect(sigc::mem_fun(*this, &FlycapWindow::OnMenuSaveAs ));
	m_pMenuQuit->signal_activate().connect(sigc::mem_fun( *this, &FlycapWindow::OnMenuQuit ));

	m_pMenuDrawCrosshair->signal_toggled().connect(sigc::mem_fun( *this, &FlycapWindow::OnMenuDrawCrosshair ));
	m_pMenuChangeCrosshairColor->signal_activate().connect(sigc::mem_fun( *this, &FlycapWindow::OnMenuChangeCrosshairColor ));
	m_pMenuShowToolbar->signal_toggled().connect(sigc::mem_fun( *this, &FlycapWindow::OnMenuShowToolbar ));
	m_pMenuShowInfoPane->signal_toggled().connect(sigc::mem_fun( *this, &FlycapWindow::OnMenuShowInfoPane ));
	m_pMenuStretchImage->signal_toggled().connect(sigc::mem_fun( *this, &FlycapWindow::OnMenuStretchImage ));
	m_pMenuFullscreen->signal_toggled().connect(sigc::mem_fun( *this, &FlycapWindow::OnMenuFullscreen ));

	m_pMenuCPA_None->signal_toggled().connect(
			sigc::bind<ColorProcessingAlgorithm, Gtk::RadioMenuItem*>(
				sigc::mem_fun(*this, &FlycapWindow::OnMenuCPAClicked),
				NO_COLOR_PROCESSING,
				m_pMenuCPA_None ) );

	m_pMenuCPA_NNF->signal_toggled().connect(
			sigc::bind<ColorProcessingAlgorithm, Gtk::RadioMenuItem*>(
				sigc::mem_fun(*this, &FlycapWindow::OnMenuCPAClicked),
				NEAREST_NEIGHBOR,
				m_pMenuCPA_NNF ) );

	m_pMenuCPA_HQ_Linear->signal_toggled().connect(
			sigc::bind<ColorProcessingAlgorithm, Gtk::RadioMenuItem*>(
				sigc::mem_fun(*this, &FlycapWindow::OnMenuCPAClicked),
				HQ_LINEAR,
				m_pMenuCPA_HQ_Linear ) );

	m_pMenuCPA_Edge_Sensing->signal_toggled().connect(
			sigc::bind<ColorProcessingAlgorithm, Gtk::RadioMenuItem*>(
				sigc::mem_fun(*this, &FlycapWindow::OnMenuCPAClicked),
				EDGE_SENSING,
				m_pMenuCPA_Edge_Sensing ) );

	m_pMenuCPA_Rigorous->signal_toggled().connect(
			sigc::bind<ColorProcessingAlgorithm, Gtk::RadioMenuItem*>(
				sigc::mem_fun(*this, &FlycapWindow::OnMenuCPAClicked),
				RIGOROUS,
				m_pMenuCPA_Rigorous ) );

	m_pMenuHelp->signal_activate().connect(sigc::mem_fun(*this, &FlycapWindow::OnMenuHelp));
	m_pMenuAbout->signal_activate().connect(sigc::mem_fun( *this, &FlycapWindow::OnMenuAbout));

	m_pNewCameraButton->signal_clicked().connect(sigc::mem_fun(*this, &FlycapWindow::OnToolbarNewCamera));
	m_pStartButton->signal_clicked().connect(sigc::mem_fun(*this, &FlycapWindow::OnToolbarStart));
	m_pStopButton->signal_clicked().connect(sigc::mem_fun(*this, &FlycapWindow::OnToolbarStop));
	m_pCamCtlButton->signal_clicked().connect(sigc::mem_fun(*this, &FlycapWindow::OnToolbarCameraControl));
	m_pHistogramButton->signal_clicked().connect(sigc::mem_fun(*this, &FlycapWindow::OnToolbarHistogram));

	m_pNewImageEvent = new Glib::Dispatcher();
	m_pNewImageEvent->connect(sigc::mem_fun( *this, &FlycapWindow::OnImageCaptured ));

	m_pBusArrivalEvent = new Glib::Dispatcher();
	m_pBusArrivalEvent->connect(sigc::mem_fun( *this, &FlycapWindow::OnBusArrivalHandler ));

	m_pBusRemovalEvent = new Glib::Dispatcher();
	m_pBusRemovalEvent->connect(sigc::mem_fun( *this, &FlycapWindow::OnBusRemovalHandler ));
}

	bool
FlycapWindow::Cleanup()
{
	if( m_pNewImageEvent != NULL )
	{
		delete m_pNewImageEvent;
		m_pNewImageEvent = NULL;
	}

	if ( m_pBusArrivalEvent != NULL )
	{
		delete m_pBusArrivalEvent;
		m_pBusArrivalEvent = NULL;
	}

	if ( m_pBusRemovalEvent != NULL )
	{
		delete m_pBusRemovalEvent;
		m_pBusRemovalEvent = NULL;
	}

	return true;
}

	bool
FlycapWindow::OnDestroy( GdkEventAny* /*event*/ )
{
	// The destroy signal is emitted when the "X" button is clicked

	// Stop the camera and the grab thread
	Stop();

	if ( --m_activeWindows <= 0 )
	{
		// If there are no more windows left open, quit the main thread
		Gtk::Main::quit();
	}
	else
	{
		// Hide the window
		m_pWindow->hide();
	}

	return true;
}

	void
FlycapWindow::OnMenuSaveAs()
{
	TriggerMode currTrigger;
	m_camera.GetTriggerMode( &currTrigger );
	if ( currTrigger.onOff == true )
	{
		ShowErrorMessageDialog(
				"Image saving disabled when in trigger mode",
				"Images cannot be saved when the camera is in trigger mode.");
		return;
	}

	Glib::Mutex::Lock saveLock(m_rawImageMutex);

	// Make a local copy of the image
	Image tempImage;
	tempImage.DeepCopy( &m_rawImage );

	saveLock.release();

	char serialNumber[64];
	sprintf( serialNumber, "%u", m_camInfo.serialNumber );

	std::string defaultFileName( serialNumber );

	SaveImageFileChooserDialog saveDialog( m_pWindow, defaultFileName );

	std::string filename;
	ImageFileFormat fileFormat = FROM_FILE_EXT;
	saveDialog.Run( filename, fileFormat );
	if ( filename.length() == 0 )
	{
		return;
	}

	Error rawError;
	if ( fileFormat == RAW )
	{
		rawError = tempImage.Save( filename.c_str(), RAW );
		if ( rawError != PGRERROR_OK )
		{
			ShowErrorMessageDialog( "Failed to save image", rawError );
		}
	}
	else if ( fileFormat == PGM )
	{
		PixelFormat tempPixelFormat = tempImage.GetPixelFormat();
		if ( tempPixelFormat == PIXEL_FORMAT_MONO8 ||
				tempPixelFormat == PIXEL_FORMAT_MONO12 ||
				tempPixelFormat == PIXEL_FORMAT_MONO16 )
		{
			Error error = tempImage.Save( filename.c_str(), fileFormat );
			if ( error != PGRERROR_OK )
			{
				ShowErrorMessageDialog( "Failed to convert image", error );
			}
		}
		else
		{
			ShowErrorMessageDialog( "Invalid file format", "Non mono images cannot be saved as PGM." );
		}
	}
	else
	{
		Error conversionError;
		Image convertedImage;
		conversionError = tempImage.Convert( &convertedImage );
		if ( conversionError != PGRERROR_OK )
		{
			ShowErrorMessageDialog( "Failed to convert image", conversionError );
		}

		Error convertedError;
		convertedError = convertedImage.Save( filename.c_str(), fileFormat );
		if ( convertedError != PGRERROR_OK )
		{
			ShowErrorMessageDialog( "Failed to save image", convertedError );
		}
	}
}

	void
FlycapWindow::OnMenuQuit()
{
	Stop();

	Gtk::Main::quit();
	return;
}

	void
FlycapWindow::OnToolbarNewCamera()
{
	bool retVal;

	retVal = Stop();

	m_camCtlDlg.Hide();
	m_camCtlDlg.Disconnect();

	m_camera.Disconnect();

	// Disable the toolbar
	m_pToolbar->set_sensitive( false );

	// Display the camera selection dialog
	CameraSelectionDlg camSlnDlg;
	PGRGuid arGuid[64];
	unsigned int size = 64;

	bool ok;
	camSlnDlg.ShowModal( &ok, arGuid, &size );

	// Enable the toolbar
	m_pToolbar->set_sensitive( true );

	if ( ok != true )
	{
		return;
	}

	if ( size < 1 )
	{
		// Inform user that they can only choose 1 camera
		Gtk::MessageDialog dialog( "No cameras selected", false, Gtk::MESSAGE_ERROR );
		dialog.set_secondary_text( "There were no cameras selected." );
		dialog.run();

		return;
	}
	else if ( size > 1 )
	{
		// Inform user that they can only choose 1 camera
		Gtk::MessageDialog dialog( "Unable to start more than 1 camera", false, Gtk::MESSAGE_ERROR );
		dialog.set_secondary_text( "Unable to start more than 1 camera in this mode." );
		dialog.run();

		return;
	}

	retVal = Start( arGuid[0] );
}

	void
FlycapWindow::OnToolbarStart()
{
	if ( m_camera.IsConnected() != true )
	{
		return;
	}

	Error error;
	error = m_camera.StartCapture();
	if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
	{
		ShowErrorMessageDialog( "Bandwidth exceeded", error );
		return;
	}
	else if ( error != PGRERROR_OK )
	{
		ShowErrorMessageDialog( "Failed to start image capture", error, true );
		return;
	}

	m_pStartButton->set_sensitive( false );
	m_pStopButton->set_sensitive( true );

	m_pMenuStart->set_sensitive( false );
	m_pMenuStop->set_sensitive( true );

	RegisterCallbacks();

	LaunchGrabThread();
}

	void
FlycapWindow::OnToolbarStop()
{
	Stop();
}

	void
FlycapWindow::OnToolbarCameraControl()
{
	m_camCtlDlg.IsVisible() ? m_camCtlDlg.Hide() : m_camCtlDlg.Show();
}

	void
FlycapWindow::OnToolbarHistogram()
{
	m_pHistogramWindow->is_visible() ? m_pHistogramWindow->hide() : m_pHistogramWindow->show();
}

	bool
FlycapWindow::Start( PGRGuid guid )
{
	Error error;

	error = m_camera.Connect( &guid );
	if ( error != PGRERROR_OK )
	{
		ShowErrorMessageDialog( "Failed to connect to camera", error );
		return false;
	}

	// Force the camera to PGR's Y16 endianness
	ForcePGRY16Mode();

	// Connect the camera control dialog to the selected camera
	m_camCtlDlg.Connect( &m_camera );

	// Get the camera info and print it out
	error = m_camera.GetCameraInfo( &m_camInfo );
	if ( error != PGRERROR_OK )
	{
		ShowErrorMessageDialog( "Failed to get camera info from camera", error );
		return false;
	}

	char title[512];
	sprintf(
			title,
			"FlyCap2 - %s %s (%u)",
			m_camInfo.vendorName,
			m_camInfo.modelName,
			m_camInfo.serialNumber );
	m_pWindow->set_title( title );

	error = m_camera.StartCapture();
	if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
	{
		ShowErrorMessageDialog( "Bandwidth exceeded", error );
		return false;
	}
	else if ( error != PGRERROR_OK )
	{
		ShowErrorMessageDialog( "Failed to start image capture", error, true );
		return false;
	}

	// Reset frame rate counters
	m_processedFrameRate.Reset();
	m_pArea->ResetFrameRate();

	RegisterCallbacks();

	SetRunStatus( true );

	LaunchGrabThread();

	// Set the scrolled window to center the image
	CenterImageInWindow();

	m_pStartButton->set_sensitive( false );
	m_pStopButton->set_sensitive( true );

	m_pMenuStart->set_sensitive( false );
	m_pMenuStop->set_sensitive( true );

	return true;
}


	bool
FlycapWindow::Stop()
{
	if( GetRunStatus() != true )
	{
		return false;
	}

	// Stop the image capture
	Error error;
	error = m_camera.StopCapture();
	if ( error != PGRERROR_OK )
	{
		// This may fail when the camera was removed, so don't show
		// an error message
	}

	KillGrabThread();

	UnregisterCallbacks();

	// Stop stretching and fullscreen
	m_pMenuStretchImage->set_active( false );
	m_pMenuFullscreen->set_active( false );

	// Load the PGR logo file
	LoadPGRLogo();

	// Assign the new pix buf to the drawing area and redraw it
	m_pArea->SetPixBuf( m_displayPixBuf );
	m_pArea->queue_draw();

	// Hide the camera control dialog
	m_camCtlDlg.Hide();

	// Hide the histogram window
	m_pHistogramWindow->hide();

	// Update the status bar
	UpdateStatusBar();

	m_pStartButton->set_sensitive( true );
	m_pStopButton->set_sensitive( false );

	m_pMenuStart->set_sensitive( true );
	m_pMenuStop->set_sensitive( false );

	return true;
}

	void
FlycapWindow::RegisterCallbacks()
{
	Error error;

	// Register arrival callbacks
	error = m_busMgr.RegisterCallback( &FlycapWindow::OnBusArrival, ARRIVAL, this, &m_cbArrivalHandle );
	if ( error != PGRERROR_OK )
	{
		ShowErrorMessageDialog( "Failed to register callback", error );
	}

	// Register removal callbacks
	error = m_busMgr.RegisterCallback( &FlycapWindow::OnBusRemoval, REMOVAL, this, &m_cbRemovalHandle );
	if ( error != PGRERROR_OK )
	{
		ShowErrorMessageDialog( "Failed to register callback", error );
	}
}

	void
FlycapWindow::UnregisterCallbacks()
{
	Error error;

	// Unregister arrival callback
	error = m_busMgr.UnregisterCallback( m_cbArrivalHandle );
	if ( error != PGRERROR_OK )
	{
		ShowErrorMessageDialog( "Failed to unregister callback", error );
	}

	// Unregister removal callback
	error = m_busMgr.UnregisterCallback( m_cbRemovalHandle );
	if ( error != PGRERROR_OK )
	{
		ShowErrorMessageDialog( "Failed to unregister callback", error );
	}
}

void FlycapWindow::CenterImageInWindow()
{
	Gtk::Adjustment* hAdjustment = m_pScrolledWindow->get_hadjustment();
	Gtk::Adjustment* vAdjustment = m_pScrolledWindow->get_vadjustment();

	double hMin = hAdjustment->get_lower();
	double hMax = hAdjustment->get_upper();
	double hMid = (hMax - hMin) / 2.0;
	hAdjustment->set_value( hMid );

	double vMin = vAdjustment->get_lower();
	double vMax = vAdjustment->get_upper();
	double vMid = (vMax - vMin) / 2.0;
	vAdjustment->set_value( vMid );

	m_pScrolledWindow->set_hadjustment( hAdjustment );
	m_pScrolledWindow->set_vadjustment( vAdjustment );
}

	void
FlycapWindow::LaunchGrabThread()
{
	SetRunStatus( true );

	m_pGrabLoop = Glib::Thread::create(
			sigc::bind<FlycapWindow*>(
				sigc::mem_fun(*this, &FlycapWindow::GrabLoop), this ),
			true );
}

	void
FlycapWindow::KillGrabThread()
{
	// Kill the grab thread
	SetRunStatus( false );
	m_pGrabLoop->join();
}

	void
FlycapWindow::SetRunStatus( bool runStatus )
{
	Glib::Mutex::Lock saveLock(m_runMutex);
	m_run = runStatus;
}

	bool
FlycapWindow::GetRunStatus()
{
	Glib::Mutex::Lock saveLock(m_runMutex);
	return m_run;
}

void FlycapWindow::ConvertMonoToOutput(Image* pIn, Image* pOut)
{
	/*pOut = new Image(
	  pIn->GetRows(),
	  pIn->GetCols(),
	  sk_outputPixelFormat );*/

	unsigned char* pDataIn = pIn->GetData();
	unsigned char* pDataOut = pOut->GetData();
	for( int i = 0; i < pIn->GetRows()*pIn->GetCols(); i++ )
	{
		pDataOut[(i*4)] =  pDataIn[i];
		pDataOut[(i*4)+1] =  pDataIn[i];
		pDataOut[(i*4)+2] =  pDataIn[i];
		pDataOut[(i*4)+3] = 255;
	}
	return;
}

	void
FlycapWindow::GrabLoop( FlycapWindow* pWindow )
{
	if( pWindow == NULL )
	{
		return;
	}

	while( GetRunStatus() == true )
	{
		{
			Glib::Mutex::Lock saveLock(pWindow->m_rawImageMutex);

			// Get the image
			Error error = pWindow->m_camera.RetrieveBuffer( &m_rawImage );
			if ( error != PGRERROR_OK )
			{
				PrintGrabLoopError( error );
				continue;
			}
		}

		// A new image was received
		pWindow->m_processedFrameRate.NewFrame();

		// Get the image dimensions
		PixelFormat pixelFormat;
		BayerTileFormat bayerFormat;
		unsigned int rows, cols, stride;
		m_rawImage.GetDimensions( &rows, &cols, &stride, &pixelFormat, &bayerFormat );

		// Try to lock the window's pixbuf.
		Glib::Mutex::Lock pixBufLock(pWindow->m_pixBufMutex, Glib::NOT_LOCK );
		if ( pixBufLock.try_acquire() == true )
		{
			// If we have a new image size, update the pixbufs.
			if( pWindow->m_renderPixBuf->get_width() != static_cast<int>(cols) ||
					pWindow->m_renderPixBuf->get_height() != static_cast<int>(rows) )
			{
				pWindow->m_renderPixBuf = Gdk::Pixbuf::create( Gdk::COLORSPACE_RGB, true, 8, cols, rows );
				pWindow->m_displayPixBuf = Gdk::Pixbuf::create( Gdk::COLORSPACE_RGB, true, 8, cols, rows );
			}

			Image tempImage(
					rows,
					cols,
					pWindow->m_renderPixBuf->get_rowstride(),
					(unsigned char*)pWindow->m_renderPixBuf->get_pixels(),
					pWindow->m_renderPixBuf->get_rowstride() * rows,
					sk_outputPixelFormat,
					bayerFormat );

			PixelFormat pixelFormat2;
			BayerTileFormat bayerFormat2;
			unsigned int rows2, cols2, stride2;

			tempImage.GetDimensions( &rows2, &cols2, &stride2, &pixelFormat2, &bayerFormat2 );

			// Convert the image to RGBU
			Error error = m_rawImage.Convert( sk_outputPixelFormat, &tempImage );
			if ( error != PGRERROR_OK )
			{
				PrintGrabLoopError( error );
				pixBufLock.release();
				continue;
			}

			//ConvertMonoToOutput(&m_rawImage,&tempImage);

			m_convertedImage = tempImage;

			Glib::RefPtr<Gdk::Pixbuf> swap = pWindow->m_displayPixBuf;
			pWindow->m_displayPixBuf = pWindow->m_renderPixBuf;
			pWindow->m_renderPixBuf = swap;

			pixBufLock.release();

			// Calculate image statistics
			PixelFormat rawPixelFormat = m_rawImage.GetPixelFormat();

			if ( rawPixelFormat == PIXEL_FORMAT_MONO8 ||
					rawPixelFormat == PIXEL_FORMAT_MONO16 )
			{
				m_pHistogramWindow->SetImageForStatistics( m_rawImage );
			}
			else
			{
				m_pHistogramWindow->SetImageForStatistics( m_convertedImage );
			}

			/*if ( error != PGRERROR_OK )
			  {
			  PrintGrabLoopError( error );
			  } */

			Glib::Mutex::Lock emitLock( m_emitMutex );

			pWindow->m_pNewImageEvent->emit();
			m_numEmitted++;

			emitLock.release();
		}
	}
}

	void
FlycapWindow::PrintGrabLoopError( Error fc2Error )
{
	time_t rawtime;
	struct tm * timeinfo;
	time( &rawtime );
	timeinfo = localtime( &rawtime );

	char currTimeStr[128];
	sprintf( currTimeStr, "%s", asctime( timeinfo ) );
	currTimeStr[ strlen(currTimeStr) - 1 ] = '\0';

	char errorMsg[1024];
	sprintf(
			errorMsg,
			"%s: Grab loop had an error: %s\n",
			currTimeStr,
			fc2Error.GetDescription() );

	std::cout << errorMsg;
}

	void
FlycapWindow::OnImageCaptured()
{
	Glib::Mutex::Lock emitLock( m_emitMutex );

	if ( m_numEmitted == 0 )
	{
		return;
	}
	else
	{
		m_numEmitted = 0;
	}

	emitLock.release();

	Glib::Mutex::Lock pixBufLock(m_pixBufMutex, Glib::NOT_LOCK );
	if ( pixBufLock.try_acquire() == true )
	{
		// Set the pixbuf that will be used by the drawing area
		m_pArea->SetPixBuf( m_displayPixBuf );

		// Redraw the image
		if ( m_pMenuDrawImage->get_active() == true )
		{
			m_pArea->queue_draw();
		}

		UpdateInformationPane();
		UpdateStatusBar();
		UpdateHistogramWindow();
	}
}

	bool
FlycapWindow::Run( PGRGuid guid )
{
	bool retVal;

	retVal = Initialize();
	if ( retVal != true )
	{
		return false;
	}

	retVal = Start( guid );
	if ( retVal != true )
	{
		return false;
	}

	m_pWindow->show();

	m_activeWindows++;

	return true;
}

	void
FlycapWindow::SetTimestamping( bool onOff )
{
	Error error;
	EmbeddedImageInfo info;

	// Get configuration
	error = m_camera.GetEmbeddedImageInfo( &info );
	if ( error != PGRERROR_OK )
	{
		ShowErrorMessageDialog( "Failed to get embedded image info", error );
		return;
	}

	// Set timestamping to on
	if ( onOff == true )
	{
		info.timestamp.onOff = true;
	}
	else
	{
		info.timestamp.onOff = false;
	}

	// Set configuration
	error = m_camera.SetEmbeddedImageInfo( &info );
	if ( error != PGRERROR_OK )
	{
		ShowErrorMessageDialog( "Failed to set embededded image info", error );
		return;
	}
}

	void
FlycapWindow::ForcePGRY16Mode()
{
	Error error;
	const unsigned int k_imageDataFmtReg = 0x1048;
	unsigned int value = 0;
	error = m_camera.ReadRegister( k_imageDataFmtReg, &value );
	if ( error != PGRERROR_OK )
	{
		// Error
	}

	value &= ~(0x1 << 0);

	error = m_camera.WriteRegister( k_imageDataFmtReg, value );
	if ( error != PGRERROR_OK )
	{
		// Error
	}
}

	void
FlycapWindow::UpdateColorProcessingMenu()
{
	ColorProcessingAlgorithm cpa = Image::GetDefaultColorProcessing();

	switch (cpa)
	{
		case NO_COLOR_PROCESSING:
			m_pMenuCPA_None->set_active( true );
			break;

		case NEAREST_NEIGHBOR:
			m_pMenuCPA_NNF->set_active( true );
			break;

		case HQ_LINEAR:
			m_pMenuCPA_HQ_Linear->set_active( true );
			break;

		case EDGE_SENSING:
			m_pMenuCPA_Edge_Sensing->set_active( true );
			break;

		case RIGOROUS:
			m_pMenuCPA_Rigorous->set_active( true );
			break;

		case DEFAULT:
		default:
			break;
	}
}

	void
FlycapWindow::UpdateInformationPane()
{
	Error error;
	InformationPane::InformationPaneStruct infoStruct = {0};

	// Set up the frame rate data
	Property prop;
	prop.type = FRAME_RATE;
	error = m_camera.GetProperty( &prop );
	if ( error == PGRERROR_OK )
	{
		infoStruct.fps.processedFrameRate = m_processedFrameRate.GetFrameRate();
		infoStruct.fps.displayedFrameRate = m_pArea->GetDisplayedFrameRate();
		infoStruct.fps.requestedFrameRate = (error == PGRERROR_OK) ? prop.absValue : 0.0;
	}

	// Set up the timestamp data
	if ( m_camInfo.interfaceType == INTERFACE_IEEE1394 )
	{
		infoStruct.timestamp = m_rawImage.GetTimeStamp();
	}

	// Set up the image info data
	m_rawImage.GetDimensions(
			&infoStruct.imageInfo.height,
			&infoStruct.imageInfo.width,
			&infoStruct.imageInfo.stride,
			&infoStruct.imageInfo.pixFmt );

	// Set up the embedded image info data
	const unsigned int k_frameInfoReg = 0x12F8;
	unsigned int frameInfoRegVal = 0;
	error = m_camera.ReadRegister( k_frameInfoReg, &frameInfoRegVal );
	if ( error == PGRERROR_OK && (frameInfoRegVal >> 31) != 0 )
	{
		const int k_numEmbeddedInfo = 10;

		ImageMetadata metadata = m_rawImage.GetMetadata();
		unsigned int* pEmbeddedInfo = infoStruct.embeddedInfo.arEmbeddedInfo;

		for (int i=0; i < k_numEmbeddedInfo; i++)
		{
			switch (i)
			{
				case 0: pEmbeddedInfo[i] = metadata.embeddedTimeStamp; break;
				case 1: pEmbeddedInfo[i] = metadata.embeddedGain; break;
				case 2: pEmbeddedInfo[i] = metadata.embeddedShutter; break;
				case 3: pEmbeddedInfo[i] = metadata.embeddedBrightness; break;
				case 4: pEmbeddedInfo[i] = metadata.embeddedExposure; break;
				case 5: pEmbeddedInfo[i] = metadata.embeddedWhiteBalance; break;
				case 6: pEmbeddedInfo[i] = metadata.embeddedFrameCounter; break;
				case 7: pEmbeddedInfo[i] = metadata.embeddedStrobePattern; break;
				case 8: pEmbeddedInfo[i] = metadata.embeddedGPIOPinState; break;
				case 9: pEmbeddedInfo[i] = metadata.embeddedROIPosition; break;
			}
		}
	}

	m_pInformationPane->UpdateInformationPane( infoStruct );
}

	void
FlycapWindow::UpdateStatusBar()
{
	char info[512];

	if( GetRunStatus() == true )
	{
		// Get the current mouse position
		int xPos = 0;
		int yPos = 0;
		m_pArea->GetMouseCoordinates( &xPos, &yPos );

		// Set up the frame rate data
		Property prop;
		prop.type = FRAME_RATE;
		Error error = m_camera.GetProperty( &prop );

		unsigned int redVal = 0;
		unsigned int greenVal = 0;
		unsigned int blueVal = 0;

		Glib::Mutex::Lock pixBufLock(m_pixBufMutex, Glib::NOT_LOCK );
		if ( pixBufLock.try_acquire() == true )
		{
			if ( xPos >= 0 && yPos >= 0 )
			{
				const unsigned int bpp = m_convertedImage.GetBitsPerPixel() / 8;
				unsigned int index = (yPos * m_convertedImage.GetStride()) + (xPos * bpp);

				redVal = *m_convertedImage[index++];
				greenVal = *m_convertedImage[index++];
				blueVal = *m_convertedImage[index];
			}

			pixBufLock.release();
		}

		sprintf(
				info,
				"Frame Rate (Proc/Disp/Req): %3.2fHz / %3.2fHz / %3.2fHz | Cursor: (%4u, %4u) | RGB: (%3d %3d %3d)",
				m_processedFrameRate.GetFrameRate(),
				m_pArea->GetDisplayedFrameRate(),
				(error == PGRERROR_OK) ? prop.absValue : 0.0,
				xPos >= 0 ? xPos : 0,
				yPos >= 0 ? yPos : 0,
				redVal,
				greenVal,
				blueVal );
	}
	else
	{
		sprintf( info, "Camera not started" );
	}

	m_pStatusBarRGB->pop();
	m_pStatusBarRGB->push( info );
}

void FlycapWindow::UpdateHistogramWindow()
{
	if ( m_pHistogramWindow->is_visible() == true )
	{
		m_pHistogramWindow->Update();
	}
}

void FlycapWindow::CenterScrolledWindow()
{
	// Set the scrolled window to center the image
	Gtk::Adjustment* hAdjustment = m_pScrolledWindow->get_hadjustment();
	Gtk::Adjustment* vAdjustment = m_pScrolledWindow->get_vadjustment();

	const double hMin = hAdjustment->get_lower();
	const double hMax = hAdjustment->get_upper();
	const double hPageSize = hAdjustment->get_page_size();
	const double hRange = (hMax - hMin - hPageSize);
	const double hMid = (hMin + hRange) / 2.0;
	hAdjustment->set_value( hMid );

	const double vMin = vAdjustment->get_lower();
	const double vMax = vAdjustment->get_upper();
	const double vPageSize = vAdjustment->get_page_size();
	const double vRange = (vMax - vMin - vPageSize);
	const double vMid = (vMin + vRange) / 2.0;
	vAdjustment->set_value( vMid );

	m_pScrolledWindow->set_hadjustment( hAdjustment );
	m_pScrolledWindow->set_vadjustment( vAdjustment );
}

	void
FlycapWindow::LoadPGRLogo()
{
	// Attempt to load logo file
	m_displayPixBuf = Gdk::Pixbuf::create_from_inline( sizeof(PGRLogo), PGRLogo, false );
	m_renderPixBuf = Gdk::Pixbuf::create_from_inline( sizeof(PGRLogo), PGRLogo, false );
}

	void
FlycapWindow::LoadFlyCap2Icon()
{
	m_iconPixBuf = Gdk::Pixbuf::create_from_inline( sizeof(PGRIcon), PGRIcon, false );
}

	void
FlycapWindow::OnMenuDrawCrosshair()
{
	bool show = m_pMenuDrawCrosshair->get_active();
	m_pArea->SetShowCrosshair( show );
}

	void
FlycapWindow::OnMenuChangeCrosshairColor()
{
	Gtk::ColorSelectionDialog colorSlnDlg;

	Gdk::Color currColor = m_pArea->GetCrosshairColor();
	colorSlnDlg.get_colorsel()->set_current_color( currColor );

	int response = colorSlnDlg.run();

	switch( response )
	{
		case Gtk::RESPONSE_OK:
			{
				Gdk::Color newColor = colorSlnDlg.get_colorsel()->get_current_color();
				m_pArea->SetCrosshairColor( newColor );
			}
			break;
		case Gtk::RESPONSE_CANCEL:
		case Gtk::RESPONSE_NONE:
		default:
			break;
	}
}

	void
FlycapWindow::OnMenuShowToolbar()
{
	(m_pMenuShowToolbar->get_active() == true) ? m_pToolbar->show() : m_pToolbar->hide();
}

	void
FlycapWindow::OnMenuShowInfoPane()
{
	bool show = m_pMenuShowInfoPane->get_active();

	if ( show == true )
	{
		m_pInformationPane->set_position( m_prevPanePos );
	}
	else
	{
		// Get the current pane location
		m_prevPanePos = m_pInformationPane->get_position();
		m_pInformationPane->set_position( 0 );
	}
}

	void
FlycapWindow::OnMenuStretchImage()
{
	m_pArea->SetStretchToFit( m_pMenuStretchImage->get_active() );
}

void FlycapWindow::OnMenuFullscreen()
{
	bool fullscreen = m_pMenuFullscreen->get_active();

	if ( fullscreen == true )
	{
		m_pWindow->fullscreen();
		m_pToolbar->hide();
		m_pStatusBarRGB->hide();
		m_prevPanePos = m_pInformationPane->get_position();
		m_pInformationPane->set_position(0);
		m_pMenuStretchImage->set_active(true);
	}
	else
	{
		m_pWindow->unfullscreen();
		m_pToolbar->show();
		m_pStatusBarRGB->show();
		m_pInformationPane->set_position( m_prevPanePos );
		m_pMenuStretchImage->set_active(false);
	}
}

	void
FlycapWindow::OnMenuCPAClicked( ColorProcessingAlgorithm cpa, Gtk::RadioMenuItem* pMenuItem )
{
	if( pMenuItem->get_active() != true )
	{
		return;
	}

	Image::SetDefaultColorProcessing( cpa );
}

	void
FlycapWindow::OnBusArrival( void* pParam, unsigned int serial )
{
	static_cast<FlycapWindow*>(pParam)->m_pBusArrivalEvent->emit();
}

	void
FlycapWindow::OnBusArrivalHandler()
{
	// Nothing to be done here
}

	void
FlycapWindow::OnBusRemoval( void* pParam,  unsigned int serial )
{
	static_cast<FlycapWindow*>(pParam)->m_pBusRemovalEvent->emit();
}

	void
FlycapWindow::OnBusRemovalHandler()
{
	Stop();
}

	void
FlycapWindow::OnMenuHelp()
{
	LaunchHelp();
}

	void
FlycapWindow::OnMenuAbout()
{
	Gtk::AboutDialog aboutDlg;

	char timeStamp[512];
	sprintf( timeStamp, "%s %s", __DATE__, __TIME__ );

	Glib::ustring comments( "Image acquisition and camera control application for FlyCapture 2.\nBuilt: " );
	comments += timeStamp;

	aboutDlg.set_program_name( "FlyCap2" );
	aboutDlg.set_comments( comments );
	aboutDlg.set_copyright( "Copyright 2009 Point Grey Research, Inc. All Rights Reserved." );

	FC2Version fc2Version;
	Utilities::GetLibraryVersion( &fc2Version );
	char version[128];
	sprintf( version, "%d.%d.%d.%d", fc2Version.major, fc2Version.minor, fc2Version.type, fc2Version.build );

	aboutDlg.set_version( version );

	Glib::ustring ustrLicense;
	ustrLicense.append(
			"The FlyCapture Software Development Kit (the \"Software\") is owned and copyrighted by Point Grey Research, Inc.  All rights are reserved.\n"
			"The Original Purchaser is granted a license to use the Software subject to the following restrictions and limitations.\n"
			"1.	The license is to the Original Purchaser only, and is nontransferable unless you have received written permission of Point Grey Research, Inc.\n"
			"2.	The Original Purchaser may use the Software only with Point Grey Research, Inc. cameras owned by the Original Purchaser, including but not limited to, Flea, Flea2, Firefly2, Firefly MV, Dragonfly, Dragonfly2, Dragonfly Express or Scorpion Camera Modules.\n"
			"3.	The Original Purchaser may make back-up copies of the Software for his or her own use only, subject to the use limitations of this license.\n"
			"4.	Subject to s.5 below, the Original Purchaser may not engage in, nor permit third parties to engage in, any of the following:\n"
			"a)	Providing or disclosing the Software to third parties.\n"
			"b)	Making alterations or copies of any kind of the Software (except as specifically permitted in s.3 above).\n"
			"c)	Attempting to un-assemble, de-compile or reverse engineer the Software in any way.\n"
			"Granting sublicenses, leases or other rights in the Software to others.\n"
			"5.	Original Purchasers who are Original Equipment Manufacturers may make Derivative Products with the Software. Derivative Products are new software products developed, in whole or in part, using the Software and other Point Grey Research, Inc. products.\n"
			"Point Grey Research, Inc. hereby grants a license to Original Equipment Manufacturers to incorporate and distribute the libraries found in the Software with the Derivative Products.\n"
			"The components of any Derivative Product that contain the Software libraries may only be used with Point Grey Research, Inc. products, or images derived from such products.\n"
			"5.1	By the distribution of the Software libraries with Derivative Products, Original Purchasers agree to:\n"
			"a)	not permit further redistribution of the Software libraries by end-user customers;\n"
			"b)	include a valid copyright notice on any Derivative Product; and\n"
			"c)	indemnify, hold harmless, and defend Point Grey Research, Inc. from and against any claims or lawsuits, including attorney's fees, that arise or result from the use or distribution of any Derivative Product.\n"
			"6.	Point Grey Research, Inc. reserves the right to terminate this license if there are any violations of its terms or if there is a default committed by the Original Purchaser.\n"
			"Upon termination, for any reason, all copies of the Software must be immediately returned to Point Grey Research, Inc. and the Original Purchaser shall be liable to Point Grey Research, Inc. for any and all damages suffered as a result of the violation or default.");

	aboutDlg.set_wrap_license( true );
	aboutDlg.set_license( ustrLicense );
	aboutDlg.set_logo( m_iconPixBuf );

	aboutDlg.run();
}

	int
FlycapWindow::ShowErrorMessageDialog( Glib::ustring mainTxt, Glib::ustring secondaryTxt )
{
	Gtk::MessageDialog dialog( mainTxt, false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK );
	dialog.set_secondary_text( secondaryTxt );

	return dialog.run();
}

	int
FlycapWindow::ShowErrorMessageDialog( Glib::ustring mainTxt, Error error, bool detailed )
{
	if ( detailed == true )
	{
		char tempStr[1024];
		sprintf(
				tempStr,
				"Source: %s(%u) Built: %s - %s\n",
				error.GetFilename(),
				error.GetLine(),
				error.GetBuildDate(),
				error.GetDescription() );

		Glib::ustring errorTxt(tempStr);

		Error cause = error.GetCause();
		while( cause.GetType() != PGRERROR_UNDEFINED )
		{
			sprintf(
					tempStr,
					"+-> From: %s(%d) Built: %s - %s\n",
					error.GetFilename(),
					error.GetLine(),
					error.GetBuildDate(),
					error.GetDescription() );

			errorTxt.append( tempStr );

			cause = cause.GetCause();
		}

		return ShowErrorMessageDialog( mainTxt, errorTxt );
	}
	else
	{
		return ShowErrorMessageDialog( mainTxt, error.GetDescription() );
	}
}
