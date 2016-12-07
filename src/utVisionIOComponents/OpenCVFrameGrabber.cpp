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
 * Acquires stereo images from cameras using opencv.
 *
 * @author Ulrich Eck <ulrich.eck@magicvisionlab.com>
 */

#include <string>
#include <list>
#include <iostream>
#include <algorithm>
#include <log4cpp/Category.hh>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tuple/tuple.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utUtil/OS.h>
#include <utUtil/TracingProvider.h>
#include <utVision/Image.h>
#include <utVision/Undistortion.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio/videoio.hpp"

#include <utVision/OpenCLManager.h>





// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.device_camera_opencv.OpenCVFrameGrabber" ) );

using namespace Ubitrack;
using namespace Ubitrack::Vision;


namespace Ubitrack { namespace Drivers {



/**
 * @ingroup vision_components
 * Pushes images from one camera using libdc1394.
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * \c ColorOutput push port of type Ubitrack::Vision::Measurement::ImageMeasurement.
 * \c GreyOutput  push port of type Ubitrack::Vision::Measurement::ImageMeasurement.
 */
class OpenCVFrameGrabber
	: public Dataflow::Component
{
public:

	/** constructor */
	OpenCVFrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

	/** destructor, waits until thread stops */
	~OpenCVFrameGrabber();

	/** Component start method. starts the thread */
	virtual void start();

	/** Component stop method, stops thread */
	virtual void stop();

	/** handler method for incoming pull requests */

	Measurement::Matrix3x3 getIntrinsic( Measurement::Timestamp t )
	{
		if (m_undistorter) {
			return Measurement::Matrix3x3( t, m_undistorter->getMatrix() );
		} else {
			UBITRACK_THROW( "No undistortion configured for OpenCVFrameGrabber" );
		}
	}

protected:
	// thread main loop
	void ThreadProc();

	// helper functions
	int  camera_setup();
	void camera_cleanup();

	// the thread
	boost::scoped_ptr< boost::thread > m_Thread;

	// stop the thread?
	volatile bool m_bStop;

	// the ports
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_colorPort;
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_greyPort;
	Dataflow::PullSupplier< Measurement::Matrix3x3 > m_intrinsicsPort;

	/** undistorter */
	boost::shared_ptr<Vision::Undistortion> m_undistorter;

	// camera data structures
	cv::VideoCapture m_videocapture;

	unsigned int m_width, m_height;
	bool m_autoGPUUpload;

	unsigned int m_camid;
	unsigned int m_cam_domain;
	unsigned int m_divisor;
	unsigned int m_divisor_count;

};


void OpenCVFrameGrabber::stop()
{
 	LOG4CPP_TRACE( logger, "Stopping thread..." );

	if ( m_running )
	{
		LOG4CPP_TRACE( logger, "Thread was running" );

		if ( m_Thread )
		{
			m_bStop = true;
			m_Thread->join();
		}
		m_running = false;
	}
}


void OpenCVFrameGrabber::start()
{
 	LOG4CPP_TRACE( logger, "Starting thread..." );

	if ( !m_running )
	{
		m_bStop = false;
		m_Thread.reset( new boost::thread( boost::bind ( &OpenCVFrameGrabber::ThreadProc, this ) ) );
		m_running = true;
	}
}


OpenCVFrameGrabber::OpenCVFrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_videocapture(0)
	, m_bStop( true )
	, m_width( 640 )
	, m_height( 480 )
	, m_camid( 0 )
	, m_cam_domain(cv::CAP_ANY)
	, m_divisor( 1 )
	, m_divisor_count( 0 )
	, m_colorPort( "ColorOutput", *this )
	, m_greyPort( "Output", *this )
	, m_intrinsicsPort( "Intrinsics", *this, boost::bind( &OpenCVFrameGrabber::getIntrinsic, this, _1 ) )
	, m_autoGPUUpload(false)

{
	subgraph->m_DataflowAttributes.getAttributeData( "Divisor", m_divisor );
	subgraph->m_DataflowAttributes.getAttributeData( "CamId", m_camid );
	subgraph->m_DataflowAttributes.getAttributeData( "CamDomain", m_cam_domain );
	
	// (daniel) added for compatibility with trackman pattern, but it's inconsistent, as resolution attribute on 'Camera' node does same thing
	subgraph->m_DataflowAttributes.getAttributeData( "SizeX", m_width ); 
	subgraph->m_DataflowAttributes.getAttributeData( "SizeY", m_height );
	
	// read parameters ### works only for a single camera, but how are the cameras named in a multi-camera setup? And who uses such a thing anyway?
	// Why not use multiple instance of a single-camera framegrabber pattern?
	Graph::UTQLSubgraph::NodePtr camNode = subgraph->getNode( "Camera" );
	if ( !camNode )
		UBITRACK_THROW( "No 'Camera' node" );
	if ( camNode->hasAttribute( "resolution" ) ) 
	{
		try 
		{
			std::string res = camNode->getAttribute( "resolution" ).getText();
			std::istringstream resStream( res );
			resStream >> m_width;
			resStream >> m_height;
		}
		catch( ... ) 
		{
			UBITRACK_THROW( "Invalid value for attribute 'resolution'" );
		}
	}

 	LOG4CPP_INFO( logger, "Set camera resolution: " << m_width << "x" << m_height );

	if (subgraph->m_DataflowAttributes.hasAttribute("cameraModelFile")){
		std::string cameraModelFile = subgraph->m_DataflowAttributes.getAttributeString("cameraModelFile");
		m_undistorter.reset(new Vision::Undistortion(cameraModelFile));
	}
	else {
		std::string intrinsicFile = subgraph->m_DataflowAttributes.getAttributeString("intrinsicMatrixFile");
		std::string distortionFile = subgraph->m_DataflowAttributes.getAttributeString("distortionFile");


		m_undistorter.reset(new Vision::Undistortion(intrinsicFile, distortionFile));
	}

	Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
	if (oclManager.isEnabled()) {
		if (subgraph->m_DataflowAttributes.hasAttribute("uploadImageOnGPU")){
			m_autoGPUUpload = subgraph->m_DataflowAttributes.getAttributeString("uploadImageOnGPU") == "true";
			LOG4CPP_INFO(logger, "Upload to GPU enabled? " << m_autoGPUUpload);
		}
		if (m_autoGPUUpload){
			oclManager.activate();
			LOG4CPP_INFO(logger, "Require OpenCLManager");
		}
	}

	stop();
}


OpenCVFrameGrabber::~OpenCVFrameGrabber()
{
 	stop();
}


void OpenCVFrameGrabber::camera_cleanup()
{
// 	if( m_videocapture.isOpened() )
//    {
//		LOG4CPP_TRACE( logger, "release videocapture device." );
//		m_videocapture.release();
//    }
	LOG4CPP_TRACE( logger, "cleanup finished." );
}


int OpenCVFrameGrabber::camera_setup()
{
 	LOG4CPP_INFO( logger, "Setting up camera #" << m_camid << " domain: " << m_cam_domain );

 	if( m_videocapture.isOpened() )
    {
    	m_videocapture.release();
    }

	m_videocapture.open(m_camid + m_cam_domain);
    if (!m_videocapture.isOpened()) {
        LOG4CPP_WARN(logger, "Unable to open Camera: " << m_camid );
        return 0;
    }

   	m_videocapture.set(cv::CAP_PROP_FRAME_WIDTH, m_width);
	m_videocapture.set(cv::CAP_PROP_FRAME_HEIGHT, m_height);
	
	LOG4CPP_INFO( logger, "Camera #" << m_camid << " setup successful." );
	return 1;
}




void OpenCVFrameGrabber::ThreadProc()
{
	LOG4CPP_INFO(logger, "framegrabber thread starting...");
	m_running = true;

	cv::Mat frame;

	//	setup camera
	if (!camera_setup()) return;

	while ( !m_bStop )
	{
		/* acquire frame from camera */
		if( !m_videocapture.grab() )
		{
			LOG4CPP_WARN(logger, "Can not grab frame.");
			continue;
		}
		if ( !m_bStop ) {
			/* store the last frame's timestamp */
			Measurement::Timestamp time = Measurement::now();

			if( !m_videocapture.retrieve(frame) ) {
				LOG4CPP_WARN(logger, "Can not retrieve frame.");
				continue;
			}

			// check if we succeeded
			if (frame.empty()) {
				LOG4CPP_WARN(logger, "Received empty frame.");
				continue;
			}

#ifdef ENABLE_EVENT_TRACING
			TRACEPOINT_MEASUREMENT_CREATE(getEventDomain(), time, getName().c_str(), "VideoCapture")
#endif

			boost::shared_ptr< Image > pColorImage;
			boost::shared_ptr< Image > pGreyImage;

			// check CAP_PROP_FORMAT to find out what type of image is returned as frame
			pColorImage.reset( new Image( frame ) );
			pColorImage->set_origin(0);
			pColorImage->set_pixelFormat(Vision::Image::BGR);

			if (m_autoGPUUpload){
				Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
				if (oclManager.isInitialized()) {
					//force upload to the GPU
					pColorImage->uMat();
				}
			}
			pColorImage = m_undistorter->undistort( pColorImage );
			m_colorPort.send( Measurement::ImageMeasurement( time, pColorImage ) );

			if (m_greyPort.isConnected()) {
				pGreyImage = pColorImage->CvtColor( CV_RGB2GRAY, 1 );
				if (m_autoGPUUpload){
					Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
					if (oclManager.isInitialized()) {
						//force upload to the GPU
						pGreyImage->uMat();
					}
				}
				m_greyPort.send( Measurement::ImageMeasurement( time, pGreyImage ) );
			}
		}

	}
	LOG4CPP_INFO(logger, "framegrabber thread finished...");
	camera_cleanup();
}






} } // namespace Ubitrack::Driver


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::OpenCVFrameGrabber > ( "OpenCVFrameGrabber" );
}


