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
#include <log4cpp/Category.hh>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utUtil/OS.h>
#include <utUtil/Exception.h>
#include <utVision/Image.h>
#include <utVision/OpenCLManager.h>


#include <opencv/cv.h>

using namespace Ubitrack;
using namespace Ubitrack::Vision;
using namespace Ubitrack::Measurement;

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.ImageGPUUpload" ) );

namespace Ubitrack { namespace Drivers {


class ImageGPUUpload
	: public Dataflow::Component
{
public:

	/** constructor */
	ImageGPUUpload( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > );

	/** destructor, waits until thread stops */
	~ImageGPUUpload();

protected:
	// event handler
	void pushImage( const ImageMeasurement& m );

	unsigned m_factor;
	// the ports
	Dataflow::PushSupplier< ImageMeasurement > m_outPort;
	Dataflow::PushConsumer< ImageMeasurement > m_inPort;
};


ImageGPUUpload::ImageGPUUpload( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_factor ( 0 )
	, m_outPort( "Output", *this )
	, m_inPort( "Input", *this, boost::bind( &ImageGPUUpload::pushImage, this,  boost::placeholders::_1 ) )
{
}


ImageGPUUpload::~ImageGPUUpload()
{
}


void ImageGPUUpload::pushImage( const ImageMeasurement& m )
{
	Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
	if (oclManager.isInitialized()) {
		//force upload to the GPU
		m->uMat();
	} else {
		LOG4CPP_WARN(logger, "GPU Upload node active, but OpenCL is not initialized.");
	}
	m_outPort.send( m );
}

} } // namespace Ubitrack::Driver

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::ImageGPUUpload > ( "ImageGPUUpload" );
}
