/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup vision_components
 * @file
 * Reads camera images using OpenCV's HighGUI library.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#include <string>
#include <list>
#include <iostream>
#include <log4cpp/Category.hh>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utUtil/OS.h>
#include <utVision/Image.h>

#include <opencv/highgui.h>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.HighguiFrameGrabber" ) );

using namespace Ubitrack;
using namespace Ubitrack::Vision;
using namespace Ubitrack::Measurement;

namespace Ubitrack { namespace Drivers {

/**
 * @ingroup vision_components
 * Pushes images from a camera using the HighGUI library.
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * \c Output push port of type Measurement
Measurement::ImageMeasurement.
 *
 * @par Configuration
 * \verbatim <Configuration index="0"/> \endverbatim
 * \c index can be skipped if the default camera is sufficient
 */
class HighguiFrameGrabber
	: public Dataflow::Component
{
public:

	/** constructor */
	HighguiFrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

	/** destructor, waits until thread stops */
	~HighguiFrameGrabber();

	/** Component start method. starts the thread */
	virtual void start();

	/** Component stop method, stops thread */
	virtual void stop();

protected:
	// thread main loop
	void ThreadProc();

	// camera index
	int m_cameraIndex;
	
	//image counter
	int m_counter;
	
	// divisor
	int m_divisor;

	/**
	* New property values	
	*/
	int m_width;
	int m_height;
	int m_imageFormat;
	int m_exposure;
	int m_flash;
	int m_focus;
	int m_whiteBalance;
	int m_antibanding;

	// the thread
	boost::scoped_ptr< boost::thread > m_Thread;

	// stop the thread?
	volatile bool m_bStop;

	// the ports
	Dataflow::PushSupplier< ImageMeasurement > m_outPort;
	Dataflow::PushSupplier< ImageMeasurement > m_colorPort;
};


void HighguiFrameGrabber::stop()
{
	if ( m_running )
	{
		m_running = false;
		m_bStop = true;
		LOG4CPP_INFO( logger, "Trying to stop highgui frame grabber");
		if ( m_Thread )
		{
			m_Thread->join();
			LOG4CPP_INFO( logger, "Trying to stop highgui frame grabber, thread joined");
		}
	}
}

void HighguiFrameGrabber::start()
{
	if ( !m_running )
	{
		m_running = true;
		m_bStop = false;
		m_Thread.reset( new boost::thread( boost::bind ( &HighguiFrameGrabber::ThreadProc, this ) ) );
	}
}



HighguiFrameGrabber::HighguiFrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_cameraIndex( 0 )
	, m_counter( 0 )
	, m_divisor( 1 )
	, m_width(320)
	, m_height(240)
	, m_imageFormat(0)
	, m_exposure(0)
	, m_flash(1)
	, m_focus(2)
	, m_whiteBalance(0)
	, m_antibanding(2)
	, m_bStop( true )
	, m_outPort( "Output", *this )
	, m_colorPort( "ColorOutput", *this )

{
	subgraph->m_DataflowAttributes.getAttributeData( "highguiCameraIndex", m_cameraIndex );
	
	subgraph->m_DataflowAttributes.getAttributeData( "divisor", m_divisor );

	subgraph->m_DataflowAttributes.getAttributeData( "imageWidth", m_width );
	subgraph->m_DataflowAttributes.getAttributeData( "imageHeight", m_height );

	subgraph->m_DataflowAttributes.getAttributeData( "imageFormat", m_imageFormat );
#ifdef ANDROID
	
	
	subgraph->m_DataflowAttributes.getAttributeData( "androidExposure", m_exposure );
	subgraph->m_DataflowAttributes.getAttributeData( "androidFlash", m_flash );
	subgraph->m_DataflowAttributes.getAttributeData( "androidFocus", m_focus );
	subgraph->m_DataflowAttributes.getAttributeData( "androidWhiteBalance", m_whiteBalance );
	subgraph->m_DataflowAttributes.getAttributeData( "androidAntibanding", m_antibanding );
#endif

	stop();
}


HighguiFrameGrabber::~HighguiFrameGrabber()
{
	stop();
}


void HighguiFrameGrabber::ThreadProc()
{
	LOG4CPP_INFO( logger, "Trying to open camera #" << m_cameraIndex );
	
	CvCapture* cap = cvCaptureFromCAM( m_cameraIndex );
	if ( !cap )
	{
		LOG4CPP_ERROR( logger, "HighguiFrameGrabber: unable to open camera" );
		return;
	}

	// Properties might be set like this...
// 	cvSetCaptureProperty( cap, CV_CAP_PROP_BRIGHTNESS, 0.3 );
// 	cvSetCaptureProperty( cap, CV_CAP_PROP_FRAME_WIDTH, 640 );
// 	cvSetCaptureProperty( cap, CV_CAP_PROP_FRAME_HEIGHT, 480 );
// 	cvSetCaptureProperty( cap, CV_CAP_PROP_CONTRAST, 0.8 );
// 	cvSetCaptureProperty( cap, CV_CAP_PROP_GAIN, 0.1 );


	cvSetCaptureProperty( cap, CV_CAP_PROP_FRAME_WIDTH, m_width );
	cvSetCaptureProperty( cap, CV_CAP_PROP_FRAME_HEIGHT, m_height );
#ifdef ANDROID
	
	cvSetCaptureProperty( cap, CV_CAP_PROP_EXPOSURE, m_exposure);
	cvSetCaptureProperty( cap, CV_CAP_PROP_ANDROID_FLASH_MODE, m_flash );
	cvSetCaptureProperty( cap, CV_CAP_PROP_ANDROID_FOCUS_MODE, m_focus );
	cvSetCaptureProperty( cap, CV_CAP_PROP_ANDROID_WHITE_BALANCE, m_whiteBalance );
	cvSetCaptureProperty( cap, CV_CAP_PROP_ANDROID_ANTIBANDING, m_antibanding );
	
#else
	
#endif



	//Grabbing a frame in order to get the cam properties
	cvGrabFrame( cap );
	IplImage* pIpl = cvRetrieveFrame( cap , m_imageFormat);

	// reading the frame size from the frame
	LOG4CPP_INFO(logger, "Frame width: " << pIpl->width << "pixel");
	LOG4CPP_INFO(logger, "Frame height: " << pIpl->height  << "pixel");
	LOG4CPP_INFO(logger, "Frame channels: " << pIpl->nChannels);
	LOG4CPP_INFO(logger, "Frame depth: " << pIpl->depth );
	LOG4CPP_INFO(logger, "Frame ChannelSequence: " << pIpl->channelSeq[0]<< pIpl->channelSeq[1]<< pIpl->channelSeq[2]<< pIpl->channelSeq[3] );	

	while ( !m_bStop )
	{
		cvGrabFrame( cap );
		Measurement::Timestamp time( Measurement::now() - 10000000L );
		LOG4CPP_DEBUG( logger, "time = " << time / 1000000 );
		
#ifdef ANDROID
		IplImage* pIpl = cvRetrieveFrame( cap, m_imageFormat );		
#else
		IplImage* pIpl = cvRetrieveFrame( cap );
#endif
		
		// choose only certain frames and drop the others
		if( (  m_counter++ % m_divisor ) != 0 )
		{
			if ( !pIpl )
				cvReleaseImage( &pIpl );
			continue;
		}
		
		
		if ( !pIpl )
		{
			LOG4CPP_WARN( logger, "HighguiFrameGrabber: Error in cvRetrieveFrame, hold on a second" );
			Util::sleep( 1000 );
		}
		else
		{
			if (m_colorPort.isConnected()) {
				// convert to color
#ifdef ANDROID
				boost::shared_ptr< Image > pImage( new Image( pIpl->width, pIpl->height, pIpl->nChannels ) );
				//cvConvertImage( pIpl, *pImage );
				cvCopy( pIpl, *pImage );	
				pImage->origin = pIpl->origin;							
#else
				boost::shared_ptr< Image > pImage( new Image( pIpl->width, pIpl->height, 3 ) );
				cvConvertImage( pIpl, *pImage );
				pImage->origin = pIpl->origin;
				pImage->channelSeq[0]='B';
				pImage->channelSeq[1]='G';
				pImage->channelSeq[2]='R';
#endif
				m_colorPort.send( ImageMeasurement( time, pImage ) );
				
			}
			if (m_outPort.isConnected()) {
				// convert to greyscale
				boost::shared_ptr< Image > pImage( new Image( pIpl->width, pIpl->height, 1 ) );
				cvConvertImage( pIpl, *pImage );
				pImage->origin = pIpl->origin;
				m_outPort.send( ImageMeasurement( time, pImage ) );
			}
		}
	}

	cvReleaseCapture( &cap );
}

} } // namespace Ubitrack::Driver

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::HighguiFrameGrabber > ( "HighguiFrameGrabber" );
}
