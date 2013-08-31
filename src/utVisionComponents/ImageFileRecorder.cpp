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
 * A component that writes single images to the disk.
 *
 * @author Christian Waechter <christian.waechter@in.tum.de>
 */

#include <string>
#include <list>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <boost/bind.hpp>
#include <boost/version.hpp>
// Needed until boost version 1.45 because otherwise the deprecated version 2 would be used
#if BOOST_VERSION <= 104600
	#undef  BOOST_FILESYSTEM_VERSION
	#define BOOST_FILESYSTEM_VERSION 3
#endif
#include <boost/filesystem.hpp>

#include <log4cpp/Category.hh>

#include <utDataflow/PushConsumer.h>
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
using namespace Ubitrack::Measurement;
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.ImageFileRecorder" ) );

namespace Ubitrack { namespace Drivers {


class ImageFileRecorder
	: public Dataflow::Component
{
public:

	/** constructor */
	ImageFileRecorder( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > );

	/** destructor, waits until thread stops */
	~ImageFileRecorder();

	// /** component stop method. stops thread */
	virtual void stop();

	// /** component start method. starts thread */
	virtual void start();

protected:
	
	// enques images and sleeping time
	void saveImage( const ImageMeasurement &img );

	// stop the thread?
	volatile bool m_bStop;
	
	std::filebuf filebuffer;
	std::string m_prefix;
	std::string m_suffix;
	boost::filesystem::path m_imgDir;
	boost::filesystem::path m_logfile;
	std::size_t m_counter;

	// the ports
	Dataflow::PushConsumer< ImageMeasurement > m_inPort;
};


void ImageFileRecorder::stop()
{
	if ( m_running )
	{
		filebuffer.close();
		m_running = false;
	}
}


void ImageFileRecorder::start()
{
	if ( !m_running )
	{
		filebuffer.open ( m_logfile.string().c_str(), std::ios::out );
		if( !filebuffer.is_open()  )
			UBITRACK_THROW( "Error opening file" );
			
		m_running = true;
	}
}


ImageFileRecorder::ImageFileRecorder( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_bStop( false )
	, m_prefix( "img" )
	, m_suffix( "png" )
	, m_imgDir( "" )
	, m_logfile( "frames.txt" )
	, m_counter( 0 )
	, m_inPort( "Input", *this, boost::bind( &ImageFileRecorder::saveImage, this, _1 ) )
{
	LOG4CPP_DEBUG( logger, "ImageFileRecorder(): Configuring..." );

	if ( subgraph -> m_DataflowAttributes.hasAttribute( "file" ) ) {
		m_logfile = subgraph->m_DataflowAttributes.getAttributeString( "file" );
	}
	else 
	{
		m_logfile = "frames.txt";
	}
	
	if ( subgraph -> m_DataflowAttributes.hasAttribute( "prefix" ) )
		m_prefix = subgraph->m_DataflowAttributes.getAttributeString( "prefix" );
		
	if ( subgraph -> m_DataflowAttributes.hasAttribute( "suffix" ) )
		m_suffix = "." + subgraph->m_DataflowAttributes.getAttributeString( "suffix" );

	boost::filesystem::path path = "";
	if ( subgraph -> m_DataflowAttributes.hasAttribute( "path" ) )
		path = subgraph->m_DataflowAttributes.getAttributeString( "path" );
	m_imgDir /= path;
	
	if ( ! boost::filesystem::is_directory( m_imgDir ) )
		UBITRACK_THROW( "Invalid path specified for images." );
	
	boost::filesystem::path testPath = path;
	testPath /= m_prefix;
	testPath /= "XXXXX";
	testPath /= m_suffix;
	
	LOG4CPP_DEBUG( logger, "ImageFileRecorder(): Image path pattern: " << testPath );
	LOG4CPP_DEBUG( logger, "ImageFileRecorder(): Timestamp file: " << m_logfile );
}


ImageFileRecorder::~ImageFileRecorder()
{
	stop();
}


void ImageFileRecorder::saveImage( const ImageMeasurement &img )
{
	char fileName[256]; 

	LOG4CPP_DEBUG( logger, "saveImage(): ");

	std::ostream os( &filebuffer );
	std::sprintf ( fileName, "%s%.5u%s", m_prefix.c_str(), m_counter++, m_suffix.c_str() );
	boost::filesystem::path path = m_imgDir;
	path /= fileName;
	
	LOG4CPP_DEBUG( logger, "saveImage(): Saving image to file " << path );
	
	if( cvSaveImage( path.string().c_str(), *img ) == 0 )
		LOG4CPP_ERROR( logger, "Error saving image " << path );
	
	os << (unsigned long long)( img.time() / 1000000.)  << " " << path.filename() << std::endl;
}

} } // namespace Ubitrack::Driver


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf )
{
	cf->registerComponent< Ubitrack::Drivers::ImageFileRecorder > ( "ImageFileRecorder" );
}
