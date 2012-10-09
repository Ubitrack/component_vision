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
 * A component that reads images from a sequence of files.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#include <string>
#include <list>
#include <iostream>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <log4cpp/Category.hh>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utUtil/OS.h>
#include <utUtil/Exception.h>
#include <utVision/Image.h>

#include <opencv/highgui.h>

// define macros not present in all OpenCV versions
#ifndef CV_LOAD_IMAGE_GRAYSCALE
#define CV_LOAD_IMAGE_GRAYSCALE 0
#define CV_LOAD_IMAGE_UNCHANGED 1
#endif

using namespace Ubitrack;
using namespace Ubitrack::Vision;
using namespace Ubitrack::Measurement;

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.ImageFileFrameGrabber" ) );

namespace Ubitrack { namespace Drivers {

/**
 * @ingroup vision_components
 * Loads image files and sends them via push to simulate a real camera.
 * ***DEPRECATED***
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * PushSupplier< Measurement
Measurement::ImageMeasurement > port with name "Output"
 *
 * @par Configuration
 * The configuration looks like this:
 * \verbatim
 * <Configuration>
 *   <ImageFile name="1.png" duration="300"/>
 *   <ImageFile name="2.png" duration="300"/>
 * </Configuration>
 * \endverbatim
 * \p duration is given in ms
 */
class ImageFileFrameGrabber
	: public Dataflow::Component
{
public:

	/** constructor */
	ImageFileFrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > );

	/** destructor, waits until thread stops */
	~ImageFileFrameGrabber();

	/** component stop method. stops thread */
	virtual void stop();

	/** component start method. starts thread */
	virtual void start();

protected:
	// thread main loop
	void ThreadProc();

	// the thread
	boost::scoped_ptr< boost::thread > m_Thread;

	// stop the thread?
	volatile bool m_bStop;

	// list of images
	typedef std::list< std::pair< int, boost::shared_ptr< Image > > > ImageList;
	ImageList m_images;

	// the ports
	Dataflow::PushSupplier< ImageMeasurement > m_outPort;
};


void ImageFileFrameGrabber::stop()
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

void ImageFileFrameGrabber::start()
{
	if ( !m_running )
	{
		// do not start if no images are available
		if ( !m_images.empty() )
		{
			m_Thread.reset( new boost::thread( boost::bind ( &ImageFileFrameGrabber::ThreadProc, this ) ) );
			m_running = true;
		}
	}
}


ImageFileFrameGrabber::ImageFileFrameGrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_bStop( false )
	, m_outPort( "Output", *this )
{
	// read the list of images from the XML configuration
	const TiXmlElement* xmlConf( subgraph->m_DataflowConfiguration.getXML() );
	if ( !xmlConf )
		UBITRACK_THROW( "No configuration for ImageFileFrameGrabber" );
	for ( const TiXmlElement* xmlFile = xmlConf->FirstChildElement( "ImageFile" ); xmlFile;
		xmlFile = xmlFile->NextSiblingElement( "ImageFile" ) )
	{
		// parse duration
		int nDuration = 1000;
		xmlFile->QueryIntAttribute( "duration", &nDuration );

		// read filename
		const char* sFileName = xmlFile->Attribute( "name" );
		if ( sFileName )
		{
			// load file
			LOG4CPP_NOTICE( logger, "Loading image file " << sFileName );
			IplImage* pIpl = cvLoadImage( sFileName, CV_LOAD_IMAGE_GRAYSCALE ); // or CV_LOAD_IMAGE_UNCHANGED
			if ( pIpl )
				m_images.push_back( std::make_pair( nDuration, boost::shared_ptr< Image >( new Image( pIpl ) ) ) );
			else
				LOG4CPP_ERROR( logger, "Error loading image " << sFileName );
		}
	}
	stop();
}


ImageFileFrameGrabber::~ImageFileFrameGrabber()
{
	stop();
}


void ImageFileFrameGrabber::ThreadProc()
{
	ImageList::iterator it = m_images.begin();
	while ( !m_bStop )
	{
		// send the image and sleep
		m_outPort.send( ImageMeasurement( Measurement::now(), it->second ) );
		Util::sleep( it->first );

		// repeat endlessly
		if ( ++it == m_images.end() )
			it = m_images.begin();
	}
}

} } // namespace Ubitrack::Driver

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::ImageFileFrameGrabber > ( "ImageFileFrameGrabber" );
}
