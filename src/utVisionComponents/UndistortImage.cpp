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
 * Implements a component that unwarps images.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#include <string>

#include <boost/scoped_ptr.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

#include <opencv/cv.h>

#include <utMath/Vector.h>
#include <utMath/Matrix.h>
#include <utMath/CameraIntrinsics.h>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utVision/Image.h>

#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.UndistortImage" ) );

using namespace Ubitrack;
using namespace Ubitrack::Vision;
using namespace Ubitrack::Dataflow;

namespace Ubitrack { namespace Vision {

class UndistortImage
	: public TriggerComponent
{

protected:
	/** @deprecated (radial and tangential) distortion parameters */
	PullConsumer< Measurement::Vector4D > m_coeffPort;
	
	/** @deprecated intrinsic camera matrix */
	PullConsumer< Measurement::Matrix3x3 > m_matrixPort;
	
	/** intrinsic camera parameters: 3x3matrix + distortion parameters */
	PullConsumer< Measurement::CameraIntrinsics > m_intrinsicsPort;
	
	/** distorted incoming image */
	TriggerInPort< Measurement::ImageMeasurement > m_imageIn;
	
	/** undistorted outgoing image */
	TriggerOutPort< Measurement::ImageMeasurement > m_imageOut;
	
	/** mapping of the x coordinates */
	boost::scoped_ptr< Image > m_pMapX;
	
	/** mapping of the y coordinates */
	boost::scoped_ptr< Image > m_pMapY;
public:
	UndistortImage( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: TriggerComponent( sName, pConfig )
		, m_coeffPort( "Distortion", *this ) //old port
		, m_matrixPort( "Intrinsic", *this ) //old port
		, m_intrinsicsPort( "CameraIntrinsics", *this ) //new port
		, m_imageIn( "Input", *this )
		, m_imageOut( "Output", *this )
	{
	}

	
	void initMap( int width, int height, const Math::Vector< double, 8 >& coeffs, const Math::Matrix< double, 3, 3 >& intrinsics )
	{
		LOG4CPP_INFO( logger, "Creating undistortion map" );
		LOG4CPP_DEBUG( logger, "coeffs=" << coeffs );
		LOG4CPP_DEBUG( logger, "intrinsic=" << intrinsics );
	
#if (CV_MAJOR_VERSION>1) && (CV_MINOR_VERSION>2)
		const std::size_t dist_size = 8;
#else
		const std::size_t dist_size = 4;
#endif
		// copy ublas to OpenCV parameters
		CvMat* pCvCoeffs = cvCreateMat( 1, dist_size, CV_32FC1 );
		for ( std::size_t i = 0; i< dist_size; ++i )
			reinterpret_cast< float* >( pCvCoeffs->data.ptr )[ i ] = static_cast< float >( coeffs( i ) );
		
		CvMat* pCvIntrinsics = cvCreateMat( 3, 3, CV_32FC1 );
		for ( std::size_t i = 0; i < 3; i++ )
			for ( std::size_t j = 0; j < 3; j++ )
				reinterpret_cast< float* >( pCvIntrinsics->data.ptr + i * pCvIntrinsics->step)[ j ] 
					= static_cast< float >( intrinsics( i, j ) );
		
		// create map images
		m_pMapX.reset( new Image( width, height, 1, IPL_DEPTH_32F ) );
		m_pMapY.reset( new Image( width, height, 1, IPL_DEPTH_32F ) );
		
		cvInitUndistortMap( pCvIntrinsics, pCvCoeffs, *m_pMapX, *m_pMapY );

 		LOG4CPP_TRACE( logger, "first pixel mapped from " << 
			*reinterpret_cast< float* >( m_pMapX->imageData ) << ", " <<
			*reinterpret_cast< float* >( m_pMapY->imageData ) );
		
		cvReleaseMat( &pCvIntrinsics );
		cvReleaseMat( &pCvCoeffs );
	}

	
	void compute( Measurement::Timestamp t )
	{
		
		
		// get the image
		Measurement::ImageMeasurement pImage;
		try
		{
			pImage = m_imageIn.get( );
		}
		catch( ... )
		{
			LOG4CPP_WARN( logger, "Could not undistort an image, no image available." );
			return;
		}
		
		// skip if already initialized with same values
		if ( !m_pMapX || m_pMapX->width != pImage->width || m_pMapX->height != pImage->height )
		{
		
			// read parameters
			Math::Vector< double, 8 > coeffs;
			Math::Matrix< double, 3, 3 > intrinsics;
			
			try
			{
				//support for new camera intrinsics measurement
				if( m_intrinsicsPort.isConnected() )
				{
					Math::CameraIntrinsics< double > camIntrinsics = *m_intrinsicsPort.get( t );
					
					intrinsics = camIntrinsics.matrix;
					//scale the matrix, depending on the calibartion size
					intrinsics( 0, 0 ) *= pImage->width / static_cast< double >( camIntrinsics.dimension( 0 ) );
					intrinsics( 0, 2 ) *= pImage->width / static_cast< double >( camIntrinsics.dimension( 0 ) );
					intrinsics( 1, 1 ) *= pImage->height / static_cast< double >( camIntrinsics.dimension( 1 ) );
					intrinsics( 1, 2 ) *= pImage->height / static_cast< double >( camIntrinsics.dimension( 1 ) );

					Math::CameraIntrinsics< double >::radial_type radDist = camIntrinsics.radial_params;
					Math::CameraIntrinsics< double >::tangential_type tanDist =	camIntrinsics.tangential_params;
					coeffs( 0 )  = radDist[ 0 ];
					coeffs( 1 )  = radDist[ 1 ];
					coeffs( 2 )  = tanDist[ 0 ];
					coeffs( 3 )  = tanDist[ 1 ];
					coeffs( 4 ) = coeffs( 5 ) = coeffs( 6 ) = coeffs( 7 ) = 0;
					for(std::size_t i = 2; i < camIntrinsics.radial_size; ++i )
						coeffs( i+2 )  = radDist[ i ];
				}
				else //support for old pattern
				{
					intrinsics = *m_matrixPort.get( t );
					Math::Vector< double, 4 > dist = *m_coeffPort.get( t );
					coeffs( 0 )  = dist( 0 );
					coeffs( 1 )  = dist( 1 );
					coeffs( 2 )  = dist( 2 );
					coeffs( 3 )  = dist( 3 );
					coeffs( 4 ) = coeffs( 5 ) = coeffs( 6 ) = coeffs( 7 ) = 0;
				}
			}catch( ... )
			{
				coeffs( 0 ) = coeffs( 1 ) = coeffs( 2 ) = coeffs( 3 ) = 0;
				coeffs( 4 ) = coeffs( 5 ) = coeffs( 6 ) = coeffs( 7 ) = 0;
				intrinsics = Math::Matrix< double, 3, 3 >::identity();
				LOG4CPP_WARN( logger, "Setting default values for camera intrinsics." );
			}
			
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
		}
		
		// undistort
		boost::shared_ptr< Image > imgUndistorted( new Image( pImage->width, pImage->height, pImage->nChannels, pImage->depth ) );
		imgUndistorted->origin = pImage->origin;
		cvRemap( *pImage, *imgUndistorted, *m_pMapX, *m_pMapY );
		
		// send result
		m_imageOut.send( Measurement::ImageMeasurement( t, imgUndistorted ) );
	}
};

} } // namespace Ubitrack::Vision


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) 
{
	cf->registerComponent< Ubitrack::Vision::UndistortImage > ( "UndistortImage" );
}
