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
 * @ingroup dataflow_components
 * @file
 * Components that modifies timestamp
 *
 * @author Michael Schlegel <schlegem@in.tum.de>
 */

#include <log4cpp/Category.hh>

#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

#include <utVision/Image.h>


namespace Ubitrack { namespace Components {

/**
 * @ingroup dataflow_components
 * This component generates a signal event if a measurement arrives
 *
 * @par Input Ports
 * PushConsumer<Measurement> with name "Input".
 *
 * @par Output Ports
 * PushSupplier<Button> with name "Output".
 *
 * @par Configuration
 * none
 *
 * @par Operation
 *
 * @par Instances
 */
template< class EventType >
class SignalGeneratorVision
    : public Dataflow::Component
{
public:
    /**
     * UTQL component constructor.
     *
     * @param sName Unique name of the component.
     * @param subgraph UTQL subgraph
     */
    SignalGeneratorVision( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph  )
		: Dataflow::Component( sName )
		, m_inPort( "Input", *this, boost::bind( &SignalGeneratorVision::receiveMeasurement, this, _1 ) )
		, m_outPort( "Output", *this )
		, m_button( ' ' )
		, m_logger( log4cpp::Category::getInstance( "Ubitrack.Components.SignalGeneratorVision" ) )
    {
		// read button key
		std::string button( " " );

		if ( subgraph->m_DataflowAttributes.hasAttribute( "button" ) )
			button = subgraph->m_DataflowAttributes.getAttributeString( "button" );
			
		if ( button.empty() )
			m_button = Math::Scalar< int >( -1 );
		else
			m_button = Math::Scalar< int >( button[ 0 ] );
    }

    /** Method that computes the result. */
    void receiveMeasurement( const EventType& event )
    {
		m_outPort.send( Measurement::Button( event.time(), m_button ) );
    }

protected:
    /** Input port of the component. */
    Dataflow::PushConsumer< EventType > m_inPort;

    /// Output port of the component
	Dataflow::PushSupplier< Measurement::Button > m_outPort;

	Math::Scalar< int > m_button;

    /** log4cpp logger reference */
    log4cpp::Category& m_logger;	
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
    cf->registerComponent< SignalGeneratorVision< Measurement::ImageMeasurement > > ( "ButtonGeneratorImage" );
}

} } // namespace Ubitrack::Components
