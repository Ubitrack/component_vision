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
 * Computes the projection matrix from the camera intrinsics and the camera pose
 *
 * @author Frieder Pankratz <pankratz@in.tum.de>
 */

#include <string>
#include <iostream>

#include <log4cpp/Category.hh>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utCalibration/Projection.h>
#include <utMath/MatrixOperations.h>

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <utMath/cast_assign.h>


static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.CameraProjectionMatrix" ) );

using namespace Ubitrack;

namespace Ubitrack { namespace Vision {

class CameraProjectionMatrix
	:  public Dataflow::TriggerComponent
{
public:
	CameraProjectionMatrix(const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig  )
		:  Dataflow::TriggerComponent( sName, pConfig )	
		,  m_inPortIntrinsics( "CameraIntrinsics", *this)
		, m_inPortPose( "InputPose", *this )
		, m_outPort( "Output", *this )		
	{		
		
	
	}

	~CameraProjectionMatrix()
	{
		
	}
	
	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		Math::Matrix< double, 3, 3 > intrinsics = *m_inPortIntrinsics.get();		
		//intrinsics(1,1) = -intrinsics(1,1);
		Math::Pose pose = *m_inPortPose.get();
				
		Math::Matrix< double, 3, 4 > extrinsics(pose);			
		Math::Matrix< double, 3, 4 > mat = boost::numeric::ublas::prod(intrinsics , extrinsics);

		// print decomposed matrix to console if logging is enabled
		if ( logger.isDebugEnabled() )
		{
			Math::Matrix< double, 3, 3 > K;
			Math::Matrix< double, 3, 3 > R;
			Math::Vector< double, 3 > t;
			Calibration::decomposeProjection( K, R, t, mat );
			LOG4CPP_DEBUG( logger, "K: " << K );
			LOG4CPP_DEBUG( logger, "R: " << R );
			LOG4CPP_DEBUG( logger, "t: " << t );
		}

		m_outPort.send( Measurement::Matrix3x4( t, mat ) );
	}
	

protected:
	/** 2D Input port of the component. */
	Dataflow::TriggerInPort< Measurement::Matrix3x3 > m_inPortIntrinsics;

	/** 3D Input port of the component. */
	Dataflow::TriggerInPort< Measurement::Pose > m_inPortPose;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Matrix3x4 > m_outPort;
		
};

} } // namespace Ubitrack::Driver


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) 
{
	cf->registerComponent< Ubitrack::Vision::CameraProjectionMatrix > ( "CameraProjectionMatrix" );
}