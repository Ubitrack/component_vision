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

// TODO: implement module start/stop mechanics

/**
 * @ingroup vision_components
 * @file
 * Implements a component that unwarps images.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#include <string>

#include <boost/scoped_ptr.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <log4cpp/Category.hh>

#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utVision/Image.h>

#include <cv.h>

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.UndistortImage" ) );

using namespace Ubitrack;
using namespace Ubitrack::Vision;
using namespace Ubitrack::Dataflow;

namespace Ubitrack { namespace Drivers {

class UndistortImage
	: public TriggerComponent
{
public:
	UndistortImage( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: TriggerComponent( sName, pConfig )
		, m_coeffPort( "Distortion", *this )
		, m_intrinsicsPort( "Intrinsic", *this )
		, m_imageIn( "Input", *this )
		, m_imageOut( "Output", *this )
	{
	}

	
	void initMap( int width, int height, const Math::Vector< 4 >& coeffs, const Math::Matrix< 3, 3 >& intrinsics )
	{
		// skip if already initialized with same values
		if ( m_pMapX && m_pMapX->width == width && m_pMapX->height == height && 
			coeffs == m_distortionCoeffs && intrinsics == m_intrinsicMatrix )
			return;
			
		LOG4CPP_INFO( logger, "Creating undistortion map" );
		LOG4CPP_DEBUG( logger, "coeffs=" << coeffs );
		LOG4CPP_DEBUG( logger, "intrinsic=" << intrinsics );
		
		m_distortionCoeffs = coeffs;
		m_intrinsicMatrix = intrinsics;
			
		// copy ublas to OpenCV parameters
		CvMat* pCvCoeffs = cvCreateMat( 1, 4, CV_32FC1 );
		for ( unsigned i = 0; i < 4; i++ )
			pCvCoeffs->data.fl[ i ] = static_cast< float >( coeffs( i ) );
			
		CvMat* pCvIntrinsics = cvCreateMat( 3, 3, CV_32FC1 );
		for ( unsigned i = 0; i < 3; i++ )
			for ( unsigned j = 0; j < 3; j++ )
				reinterpret_cast< float* >( pCvIntrinsics->data.ptr + i * pCvIntrinsics->step)[ j ] 
					= static_cast< float >( intrinsics( i, j ) );
		
		// create map images
		m_pMapX.reset( new Image( width, height, 1, IPL_DEPTH_32F ) );
		m_pMapY.reset( new Image( width, height, 1, IPL_DEPTH_32F ) );
		
		cvInitUndistortMap( pCvIntrinsics, pCvCoeffs, *m_pMapX, *m_pMapY );
		
 		LOG4CPP_DEBUG( logger, "first pixel mapped from " << 
			*reinterpret_cast< float* >( m_pMapX->imageData ) << ", " <<
			*reinterpret_cast< float* >( m_pMapY->imageData ) );
			
		// release data
		cvReleaseMat( &pCvCoeffs );
		cvReleaseMat( &pCvIntrinsics );
	}

	
	void compute( Measurement::Timestamp t )
	{
		// get the image
		Measurement::ImageMeasurement pImage = m_imageIn.get();

		// read parameters
		Math::Vector< 4 > coeffs = *m_coeffPort.get( t );
		Math::Matrix< 3, 3 > intrinsics = *m_intrinsicsPort.get( t );
		
		// compensate for left-handed OpenCV coordinate frame
		boost::numeric::ublas::column( intrinsics, 2 ) *= -1;
		
		// compensate if origin==0
		if ( !pImage->origin )
		{
			intrinsics( 1, 2 ) = pImage->height - 1 - intrinsics( 1, 2 );
			coeffs( 2 ) *= -1.0;
		}
	
		// initialize the distortion map
		initMap( pImage->width, pImage->height, coeffs, intrinsics );
		
		// undistort
		Measurement::ImageMeasurement pUndistorted( t, boost::shared_ptr< Image >( 
			new Image( pImage->width, pImage->height, pImage->nChannels, pImage->depth ) ) );
		pUndistorted->origin = pImage->origin;

		cvRemap( *pImage, *pUndistorted, *m_pMapX, *m_pMapY );
		
		// send result
		m_imageOut.send( pUndistorted );
	}

protected:
	PullConsumer< Measurement::Vector4D > m_coeffPort;
	PullConsumer< Measurement::Matrix3x3 > m_intrinsicsPort;
	
	TriggerInPort< Measurement::ImageMeasurement > m_imageIn;
	TriggerOutPort< Measurement::ImageMeasurement > m_imageOut;
	
	Math::Vector< 4 > m_distortionCoeffs;
	Math::Matrix< 3, 3 > m_intrinsicMatrix;
	
	boost::scoped_ptr< Image > m_pMapX;
	boost::scoped_ptr< Image > m_pMapY;
};

} } // namespace Ubitrack::Driver


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) 
{
	cf->registerComponent< Ubitrack::Drivers::UndistortImage > ( "UndistortImage" );
}
