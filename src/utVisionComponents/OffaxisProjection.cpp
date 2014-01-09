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
 * @ingroup driver_components
 * @file OffaxisProjection.cpp
 * Calculate an offaxis projection matrix based on eye and screen position.
 *
 * @author Nicolas Heuser (heuser@in.tum.de)
 */

#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMath/Matrix.h>
#include <utMeasurement/Measurement.h>
#include <utCalibration/Projection.h>

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Drivers.OffaxisProjection" ) );

using namespace Ubitrack::Dataflow;
namespace ublas = boost::numeric::ublas;

namespace Ubitrack { namespace Drivers {

class OffaxisProjectionComponent
	: public Dataflow::TriggerComponent
{
	/** Convert string into vector. For reading the XML attributes. */
	Math::Vector< double, 3 > stringToVector(const std::string& s) {
		double p[3];
		std::istringstream vecString( s );
		for (int i=0; i < 3; ++i)
		{
			vecString >> p[i];
		}
		Math::Vector< double, 3 > trans( p );
		return trans;
	} 

public:

	OffaxisProjectionComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: Dataflow::TriggerComponent( sName, subgraph )
		, m_inPortEyePos( "EyePosition", *this )
		, m_outPort( "ProjectionMatrix", *this )
		, m_near( 0.01 )
		, m_far( 10.0 )
	{
		Graph::UTQLSubgraph::NodePtr screenNode = subgraph->getNode( "Screen" );
		if ( !screenNode )
			UBITRACK_THROW( "No screen projection node in configuration" );

		// Screen corners in screen coordinates
		m_screen_ll = stringToVector( screenNode->getAttribute( "ScreenLowerLeft" ).getText());
		m_screen_ul = stringToVector( screenNode->getAttribute( "ScreenUpperLeft" ).getText());
		m_screen_lr = stringToVector( screenNode->getAttribute( "ScreenLowerRight" ).getText());

		// Normalize the resulting vvectors. Screen width and height are a byproduct.
		m_screen_width = ublas::norm_2( m_screen_lr - m_screen_ll );
		m_screen_height = ublas::norm_2( m_screen_ul - m_screen_ll );

		// There should be a better than have the clipping planes twice in the utql workflow.
		screenNode->getAttributeData( "NearClippingPlane", m_near );
		screenNode->getAttributeData( "FarClippingPlane", m_far );
	}

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t ) {

		Math::Vector< double, 3 > eye_pos = *(m_inPortEyePos.get());

		LOG4CPP_TRACE(logger, "Computing projection matrix for view at " << eye_pos << " to screen" );

		Math::Matrix< double, 4, 4 > projMatrix = Calibration::offAxisProjectionMatrix( eye_pos, m_screen_ll, m_screen_ul, m_screen_lr, m_near, m_far, m_screen_width, m_screen_height );
		m_outPort.send( Measurement::Matrix4x4(t, projMatrix) );
	}

protected:
	/** Eye position relative to screen port */
	Dataflow::TriggerInPort< Measurement::Position >	m_inPortEyePos;

	/** Output port of the component. */
	Dataflow::TriggerOutPort< Measurement::Matrix4x4 >	m_outPort;

	/** Offset of 3 screen corners in screen coordinates */
	Math::Vector< double, 3 >						m_screen_ll;
	Math::Vector< double, 3 >						m_screen_lr;
	Math::Vector< double, 3 >						m_screen_ul;

	/** Width and height of the screen */
	double							m_screen_width;
	double							m_screen_height;

	/** Near and far clipping plane dinstances */
	double							m_near;
	double							m_far;
};

} } // namespace Ubitrack::Drivers

UBITRACK_REGISTER_COMPONENT( Ubitrack::Dataflow::ComponentFactory* const cf )
{
	cf->registerComponent< Ubitrack::Drivers::OffaxisProjectionComponent >( "OffaxisProjection" );
}
