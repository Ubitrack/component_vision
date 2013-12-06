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
 * A component that searches for conrnes using subpixel accuracy.
 *
 * @author Christian Waechter <christian.waechter@in.tum.de>
 */

#include <string>
#include <list>
#include <iostream>

#include <boost/thread.hpp>
#include <boost/scoped_array.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <log4cpp/Category.hh>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullConsumer.h>
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
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.CornerDetection" ) );

namespace Ubitrack { namespace Drivers {


class CornerDetectionSubPix
	: public Dataflow::Component
{
public:

	/** constructor */
	CornerDetectionSubPix( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > );

	/** destructor */
	~CornerDetectionSubPix(){};

protected:
	// event handler
	void pushImage( const ImageMeasurement& m );
	
	void pushPositions( const Measurement::Position2D& m );
	
	bool f_flipping;
	
	unsigned m_winSize;
	
	// the ports
	Dataflow::PullConsumer< ImageMeasurement > m_inPortImage;
	
	Dataflow::PushConsumer< Measurement::Position2D > m_inPortPosition;
	
	Dataflow::PushSupplier< ImageMeasurement > m_outPortImage;

	Dataflow::PushSupplier< Measurement::Position2D > m_outPortPosition;
	
};


CornerDetectionSubPix::CornerDetectionSubPix( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, f_flipping( true )
	, m_winSize( 5 )
	, m_inPortImage( "ImageIn", *this )
	, m_inPortPosition( "PositionIn", *this, boost::bind( &CornerDetectionSubPix::pushPositions, this, _1 ))
	, m_outPortImage( "ImageOut", *this )
	, m_outPortPosition( "PositionOut", *this )
{
	if ( subgraph->m_DataflowAttributes.hasAttribute( "flipYCoordinate" ) ) // enable Flipping of y Coordinate
		f_flipping = subgraph->m_DataflowAttributes.getAttributeString( "flipYCoordinate" ) == "true";
		
	if ( subgraph->m_DataflowAttributes.hasAttribute( "winSize" ) ) // enable Flipping of y Coordinate
		subgraph->m_DataflowAttributes.getAttributeData( "winSize", m_winSize );
}

void CornerDetectionSubPix::pushPositions( const Measurement::Position2D& m )
{
	try
	{
		boost::shared_ptr< Vision::Image > imgIn = ( m_inPortImage.get( m.time() )->Clone() );
		if( imgIn->nChannels != 1 )
			UBITRACK_THROW( "Expecting Grayscale Image, got color image!");
		
		Math::Vector< double, 2 > position = *m;
		boost::scoped_array< CvPoint2D32f > corners( new CvPoint2D32f[ 1 ] );
		corners[ 0 ].x = static_cast< float > ( position( 0 ) );
		corners[ 0 ].y = static_cast< float > ( position( 1 ) );
		cvFindCornerSubPix( *imgIn, corners.get(), 1, cvSize( m_winSize, m_winSize ), cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_ITER, 20, 0.01f ) );
		if( m_outPortImage.isConnected() )
		{
			boost::shared_ptr< Vision::Image > imgOut = imgIn->CvtColor ( CV_GRAY2RGB, 3, IPL_DEPTH_8U );
			cvCircle( *imgOut, cvPoint( static_cast< int > ( position( 0 ) ), static_cast< int > ( position( 1 ) ) ), 5, CV_RGB( 255, 0, 0 ) );
			cvCircle( *imgOut, cvPoint( static_cast< int > ( corners[ 0 ].x ), static_cast< int > ( corners[ 0 ].y ) ), 5, CV_RGB( 0, 255, 0 ) );
			m_outPortImage.send( ImageMeasurement( m.time(), imgOut ) );
			
		}
		if( f_flipping )
			corners[ 0 ].y = imgIn->height - corners[ 0 ].y;
		m_outPortPosition.send( Measurement::Position2D( m.time(), Math::Vector< double, 2 >( corners[ 0 ].x, corners[ 0 ].y ) ) );
		
		
	}
	catch( Ubitrack::Util::Exception e )
	{
		UBITRACK_THROW( "no image was given, exiting" );
	}
}

void CornerDetectionSubPix::pushImage( const ImageMeasurement& m )
{
	// if( m->nChannels != 1 )
		// UBITRACK_THROW( "Expecting Grayscale Image, got color image!");
	
	// try
	// {
		// Math::Vector< double, 2 > position = *m_inPortPosition.get( m.time() );
		// boost::scoped_array< CvPoint2D32f > corners( new CvPoint2D32f[ 1 ] );
		// corners[ 0 ].x = position( 0 );
		// corners[ 0 ].y = position( 1 );
		// cvFindCornerSubPix( *m, corners.get(), 1, cvSize( 5, 5 ), cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_ITER, 10, 0.1f ) );
		// m_outPort.send( Measurement::Position2D( m.time(), Math::Vector< double, 2 >( corners[ 0 ].x, corners[ 0 ].y ) ) );
	// }
	// catch( Ubitrack::Util::Exception e )
	// {
		// UBITRACK_THROW( "No 2d point given, exiting." );
	// }
}

} } // namespace Ubitrack::Driver

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::CornerDetectionSubPix > ( "CornerDetectionSubPix" );
}
