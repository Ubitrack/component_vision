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
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.AVIFrameGrabber" ) );

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
 * \verbatim <Configuration filename="0"/> \endverbatim
 */
class AVIFrameGrabber
	: public Dataflow::Component
{
public:

	/** constructor */
	AVIFrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

	/** destructor, waits until thread stops */
	~AVIFrameGrabber();

	/** Component start method. starts the thread */
	virtual void start();

	/** Component stop method, stops thread */
	virtual void stop();

protected:
	// thread main loop
	void ThreadProc();

	// camera filename
	std::string m_cameraFilename;

	// the thread
	boost::scoped_ptr< boost::thread > m_Thread;

	// stop the thread?
	volatile bool m_bStop;

	// the ports
	Dataflow::PushSupplier< ImageMeasurement > m_outPort;
	Dataflow::PushSupplier< ImageMeasurement > m_outPortGS;
};


void AVIFrameGrabber::stop()
{
	if ( m_running )
	{
		if ( m_Thread )
		{
			m_bStop = true;
			m_Thread->join();
		}
		m_running = false;
	}
}

void AVIFrameGrabber::start()
{
	if ( !m_running )
	{
		m_bStop = false;
		m_Thread.reset( new boost::thread( boost::bind ( &AVIFrameGrabber::ThreadProc, this ) ) );
		m_running = true;
	}
}



AVIFrameGrabber::AVIFrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_cameraFilename( "" )
	, m_bStop( true )
	, m_outPort( "ColorOutput", *this )
	, m_outPortGS( "Output", *this )
{
	subgraph->m_DataflowAttributes.getAttributeData( "AVIFilename", m_cameraFilename );

	stop();
}


AVIFrameGrabber::~AVIFrameGrabber()
{
	stop();
}


void AVIFrameGrabber::ThreadProc()
{
	LOG4CPP_INFO( logger, "Trying to open Video: " << m_cameraFilename );
	CvCapture* cap = cvCaptureFromAVI( m_cameraFilename.c_str() );
	if ( !cap )
	{
		LOG4CPP_ERROR( logger, "AVIFrameGrabber: unable to open video!" );
		return;
	}
	LOG4CPP_INFO( logger, "success" );
	//cvSetCaptureProperty(cap,CV_CAP_PROP_POS_FRAMES,750);
	Measurement::Timestamp faketime( Measurement::now() );

	Util::sleep( 200 );

	while ( !m_bStop )
	{
		cvGrabFrame( cap );
		Measurement::Timestamp time( Measurement::now() );
		LOG4CPP_DEBUG( logger, "time = " << time / 1000000 );
		IplImage* pIpl = cvRetrieveFrame( cap );
		
		if ( !pIpl )
		{
			LOG4CPP_WARN( logger, "AVIFrameGrabber: Error in cvRetrieveFrame, reseting video" );
			cvSetCaptureProperty( cap,CV_CAP_PROP_POS_FRAMES, 1 );
			Util::sleep( 100 );
		}
		else
		{
			if(m_outPort.isConnected())
			{
				boost::shared_ptr< Image > pImage( new Image(pIpl, false ));
				m_outPort.send( ImageMeasurement( time, pImage ) );
			}
			if(m_outPortGS.isConnected())
			{
				boost::shared_ptr< Image > pImageGS( new Image( pIpl->width, pIpl->height, 1 ) );
				cvConvertImage( pIpl, *pImageGS );
				m_outPortGS.send( ImageMeasurement( time, pImageGS ) );
			}


#ifndef _DEBUG
			Util::sleep( 30 ); // ~60.0 fps
#else
			Util::sleep( 150 ); // 5 fps
#endif
//			faketime += 300532145;
		}
	}

	cvReleaseCapture( &cap );
}

} } // namespace Ubitrack::Driver

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::AVIFrameGrabber > ( "AVIFrameGrabber" );
}
