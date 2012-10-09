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
 * A component that recives colored images and converts them to grayscale.
 *
 * @author Christian Waechter <christian.waechter@in.tum.de>
 */

#include <string>
#include <list>
#include <iostream>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
// Needed until boost version 1.46 because otherwise the deprecated version 2 would be used
// (see also config/boost where version 2 is currently enforced)
#undef  BOOST_FILESYSTEM_VERSION
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

#include <log4cpp/Category.hh>

#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utUtil/OS.h>
#include <utUtil/Exception.h>
#include <utUtil/GlobFiles.h>
#include <utVision/Image.h>

#include <opencv/highgui.h>

using namespace Ubitrack;
using namespace Ubitrack::Vision;
using namespace boost::filesystem;
using namespace Ubitrack::Measurement;

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.ImageIO" ) );

namespace Ubitrack { namespace Drivers {


class ImageReader
	: public Dataflow::Component
{
public:

	/** constructor */
	ImageReader( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > );

	/** destructor, waits until thread stops */
	~ImageReader();

protected:
	// event handler
	ImageMeasurement pullImage( Ubitrack::Measurement::Timestamp t );
		
	// the ports
	Dataflow::PullSupplier< ImageMeasurement > m_outPort;
	
	// the file(s)
	std::list< boost::filesystem::path > m_files;
	std::list< boost::filesystem::path >::iterator m_fileIt;
};


ImageReader::ImageReader( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_outPort( "Output", *this, boost::bind( &ImageReader::pullImage, this, _1 ) )
{
	std::string path;
	if ( subgraph->m_DataflowAttributes.hasAttribute( "file" ) )
		path = subgraph->m_DataflowAttributes.getAttributeString( "file" );

	Util::globFiles( path, Util::PATTERN_OPENCV_IMAGE_FILES, m_files );
	
	LOG4CPP_DEBUG( logger, "Added " << m_files.size() << " images to list" );
	
	m_fileIt = m_files.begin();
}


ImageReader::~ImageReader(){};


ImageMeasurement ImageReader::pullImage( Ubitrack::Measurement::Timestamp t )
{
	if ( m_fileIt == m_files.end() ) {
		UBITRACK_THROW( "End of image list reached" );
	}
	
	std::string file = (*m_fileIt).string();

	LOG4CPP_DEBUG( logger, "Loading image " << file );

	m_fileIt++;
	boost::shared_ptr< Vision::Image > pImage( new Vision::Image( cvLoadImage( file.c_str(), CV_LOAD_IMAGE_UNCHANGED ) ) );

	return ImageMeasurement( t, pImage );
}



class ImageWriter
	: public Dataflow::Component
{
public:

	/** constructor */
	ImageWriter( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > );

	/** destructor, waits until thread stops */
	~ImageWriter();

protected:
	// event handler
	void pushImage( const ImageMeasurement& m );

	std::string m_filename;
	// the ports
	Dataflow::PushConsumer< ImageMeasurement > m_inPort;
};

ImageWriter::ImageWriter( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_inPort( "Input", *this, boost::bind( &ImageWriter::pushImage, this, _1 ) )
{
	if ( subgraph -> m_DataflowAttributes.hasAttribute( "file" ) )
		m_filename = subgraph -> m_DataflowAttributes.getAttributeString( "file" );
	else
		m_filename = "tmp.png";
}


ImageWriter::~ImageWriter(){};


void ImageWriter::pushImage( const ImageMeasurement& m )
{
	
	if( cvSaveImage( m_filename.c_str(), *m ) == 0 )
		LOG4CPP_ERROR( logger, "Error saving image " << m_filename );
}

} } // namespace Ubitrack::Driver



UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::ImageWriter > ( "ImageWriter" );
	cf->registerComponent< Ubitrack::Drivers::ImageReader > ( "ImageReader" );
}
