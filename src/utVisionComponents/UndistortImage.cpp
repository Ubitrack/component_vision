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

#include <boost/numeric/ublas/matrix_proxy.hpp>

#include <opencv/cv.h>

#include <utMath/Vector.h>
#include <utMath/Matrix.h>
#include <utMath/CameraIntrinsics.h>

#include <utVision/Image.h>
#include <utVision/Undistortion.h>

#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>

#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.Undistortion.Component" ) );

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

	TriggerOutPort<Measurement::Matrix3x3> m_intrinsicsOut;
	
	/** structure to handle the undistortion */
	Vision::Undistortion m_undistorter;
	
public:
	UndistortImage( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: TriggerComponent( sName, pConfig )
		, m_coeffPort( "Distortion", *this ) //old port
		, m_matrixPort( "Intrinsic", *this ) //old port
		, m_intrinsicsPort( "CameraIntrinsics", *this ) //new port
		, m_imageIn( "Input", *this )
		, m_imageOut( "Output", *this )
		, m_intrinsicsOut("OutIntrinsics", *this)
		, m_undistorter( )
	{}
	
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
		// check if we already set appropriate parameters for the incoming image
		if( !m_undistorter.isValid( *pImage ) )
			if( !reset( t ) ) //fetch new intrinsics
				return;
		
		boost::shared_ptr< Image > imgUndistorted = m_undistorter.undistort( *pImage );
		m_imageOut.send( Measurement::ImageMeasurement( t, imgUndistorted ) );

		m_intrinsicsOut.send(Measurement::Matrix3x3(t, m_undistorter.getMatrix()));
	}
	
	bool reset( const Measurement::Timestamp t )
	{
		Math::CameraIntrinsics< double > camIntrinsics;
		try
		{
			// old opencv/ubitrack intrinsic parameters
			if( m_matrixPort.isConnected() && m_coeffPort.isConnected() )
			{
				const Math::Matrix< double, 3, 3 > camMat = *m_matrixPort.get( t );
				const Math::Vector< double, 4 > camDis = *m_coeffPort.get( t );
				const Math::Vector< double, 2 > camRad( camDis( 0 ), camDis( 1 ) );
				const Math::Vector< double, 2 > camTan( camDis( 2 ), camDis( 3 ) );
				
				camIntrinsics = Math::CameraIntrinsics< double >( camMat, camRad, camTan );
			}
			
			// newer ubitrack format
			if( m_intrinsicsPort.isConnected() )
				camIntrinsics = *m_intrinsicsPort.get( t );	
		}
		catch( ... )
		{
			LOG4CPP_ERROR( logger, "Cannot fetch camera intrinsic parameters. Please verify if specified correctly." );
			return false;
		}
		
		m_undistorter.reset( camIntrinsics );
		return true;
	}

	Measurement::Matrix3x3 getIntrinsics(const Measurement::Timestamp t) {
		return Measurement::Matrix3x3(t, m_undistorter.getMatrix());
	}
};

} } // namespace Ubitrack::Vision


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) 
{
	cf->registerComponent< Ubitrack::Vision::UndistortImage > ( "UndistortImage" );
}
