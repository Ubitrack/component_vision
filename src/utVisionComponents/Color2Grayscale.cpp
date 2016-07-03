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
 * @author Benjamin Becker <benjamin.becker@eads.net>
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

#include <opencv/highgui.h>

using namespace Ubitrack;
using namespace Ubitrack::Vision;
using namespace Ubitrack::Measurement;
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.Color2Grayscale" ) );

namespace Ubitrack { namespace Drivers {

/**
 * @ingroup vision_components
 * Loads image files and sends them via push to simulate a real camera.
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * PushSupplier< Measurement
Measurement::ImageMeasurement > port with name "Output"
 */
class Color2Grayscale
	: public Dataflow::Component
{
public:

	/** constructor */
	Color2Grayscale( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > );

	/** destructor, waits until thread stops */
	~Color2Grayscale();

protected:
	// event handler
	void pushImage( const ImageMeasurement& m );

	// the ports
	Dataflow::PushSupplier< ImageMeasurement > m_outPort;
	Dataflow::PushConsumer< ImageMeasurement > m_inPort;
};


Color2Grayscale::Color2Grayscale( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_outPort( "Output", *this )
	, m_inPort( "Input", *this, boost::bind( &Color2Grayscale::pushImage, this, _1 ) )
{
}


Color2Grayscale::~Color2Grayscale()
{
}

void Color2Grayscale::pushImage(const ImageMeasurement& m )
{
	if(m->channels() == 1)
	{
		LOG4CPP_DEBUG( logger, "Got grayscale image: pushing unmodified" );
		m_outPort.send( m );
	} else {

		int cvtCode = 0;
		switch(m->pixelFormat()) {
		case Vision::Image::RGB:
			cvtCode = cv::COLOR_RGB2GRAY;
			break;
		case Vision::Image::RGBA:
			cvtCode = cv::COLOR_RGBA2GRAY;
			break;
		case Vision::Image::BGR:
			cvtCode = cv::COLOR_BGR2GRAY;
			break;
		case Vision::Image::BGRA:
			cvtCode = cv::COLOR_BGRA2GRAY;
			break;
		default:
			break;
		}
		boost::shared_ptr< Image > pImage;
		if (cvtCode != 0) {
			if (m->isOnGPU()) {
				cv::UMat tmp;
				cv::cvtColor(m->uMat(), tmp, cv::COLOR_RGB2GRAY);
				pImage.reset( new Image( tmp ) );
			}  else {
				cv::Mat tmp;
				cv::cvtColor(m->Mat(), tmp, cv::COLOR_RGB2GRAY);
				pImage.reset( new Image( tmp ) );
			}
			pImage->copyImageFormatFrom(*m);
			pImage->set_pixelFormat(Image::LUMINANCE);
			m_outPort.send( ImageMeasurement( m.time(), pImage ) );
		} else {
			LOG4CPP_ERROR(logger, "Unkown PixelColor for Grayscale converion...");
		}
	}
}

} } // namespace Ubitrack::Driver

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::Color2Grayscale > ( "Color2Grayscale" );
}
