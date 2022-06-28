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

#include <opencv/cv.h>

using namespace Ubitrack;
using namespace Ubitrack::Vision;
using namespace Ubitrack::Measurement;

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.ImageRotate" ) );

namespace Ubitrack { namespace Drivers {


class ImageRotate
	: public Dataflow::Component
{
public:

	/** constructor */
	ImageRotate( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > );

	/** destructor, waits until thread stops */
	~ImageRotate();

protected:
	// event handler
	void pushImage( const ImageMeasurement& m );

	// 0=no change, 1=CW, 2=CCW, 3=180
	unsigned m_rotation;
	// the ports
	Dataflow::PushSupplier< ImageMeasurement > m_outPort;
	Dataflow::PushConsumer< ImageMeasurement > m_inPort;
};


ImageRotate::ImageRotate( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_rotation ( 0 )
	, m_outPort( "Output", *this )
	, m_inPort( "Input", *this, boost::bind( &ImageRotate::pushImage, this,  boost::placeholders::_1 ) )
{
	if ( subgraph -> m_DataflowAttributes.hasAttribute( "imageRotation" ) )
		subgraph -> m_DataflowAttributes.getAttributeData( "imageRotation", m_rotation );
}


ImageRotate::~ImageRotate()
{
}

template<typename T>
boost::shared_ptr< Vision::Image > rotateImage(T& img, int rotate, Vision::Image::ImageFormatProperties& fmt) {
	T tmp;

	if (rotate == 0) {
		tmp = img; // do not copy if no rotation is applied
	} else if (rotate == 1){
		cv::transpose(img, tmp);
		cv::flip(tmp, tmp,1); //transpose+flip(1)=CW
	} else if (rotate == 2) {
		cv::transpose(img, tmp);
		cv::flip(tmp, tmp, 0); //transpose+flip(1)=CW
	} else if (rotate == 3){
		cv::flip(img, tmp, -1);    //flip(-1)=180
	} else { //if not 0,1,2,3:
		UBITRACK_THROW( "Invalid rotation specified" );
	}
	return boost::shared_ptr< Vision::Image >( new Image( tmp, fmt ) );
}

void ImageRotate::pushImage( const ImageMeasurement& m )
{

	Vision::Image::ImageFormatProperties fmt;
	m->getFormatProperties(fmt);

	boost::shared_ptr< Vision::Image > out;
	unsigned int width = m->width();
	unsigned int height = m->height();
	if ((m_rotation == 1) || (m_rotation == 2)) {
		width = m->height();
		height = m->width();
	}

	if (m->isOnGPU()) {
		out = rotateImage(m->uMat(), m_rotation, fmt);
	} else {
		out = rotateImage(m->Mat(), m_rotation, fmt);
	}

	m_outPort.send(ImageMeasurement(m.time(), out));
}

} } // namespace Ubitrack::Driver

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::ImageRotate > ( "ImageRotate" );
}
