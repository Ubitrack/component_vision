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
 * @file
 * NetworkSink component.
 *
 * @author Frieder Pankratz
 */

// WARNING: all boost/serialization headers should be
//          included AFTER all boost/archive headers
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/binary_object.hpp>

#include <string>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <iomanip>

// on windows, asio must be included before anything that possible includes windows.h
// don't ask why.
#include <boost/asio.hpp>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <boost/array.hpp>
#include <boost/thread.hpp>

#include <utDataflow/PushConsumer.h>
#include <utDataflow/ComponentFactory.h>
#include <utDataflow/Component.h>
#include <utMeasurement/Measurement.h>

#ifdef HAVE_OPENCV
	#include <utVision/Image.h>
	#include <opencv/highgui.h>
#endif
using namespace Ubitrack::Measurement;
namespace Ubitrack { namespace Vision {

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Drivers.NetworkSink" ) );
	
/**
 * @ingroup dataflow_components
 * Transmits measurements over the network.
 *
 * @par Input Ports
 * PushConsumer<EventType> port "Input"
 *
 * @par Output Ports
 * None.
 *
 * @par Configuration
 * - Edge configuration:
 * @verbatim
 * <Configuration port="..." destination="..."/>
 * @endverbatim
 *   - \c port: the TCP target port, default 0x5554 ("UT")
 *   - \c destination: the target machine or ip, default 127.0.0.1
 *
 * @par Instances
 * Registered for the following EventTypes and names:
 * - Ubitrack::Measurement::Position: NetworkPositionSink
 * - Ubitrack::Measurement::Rotation: NetworkRotationSink
 * - Ubitrack::Measurement::Pose : NetworkPoseSink
 */
class ImageNetworkSinkComponent
	: public Dataflow::Component
{

public:

	/** constructor */
	ImageNetworkSinkComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::Component( name )
		, m_inPort( "Input", *this, boost::bind( &ImageNetworkSinkComponent::eventIn, this, _1 ) )
		, m_IoService()
		, m_connected( false )
		, m_headerSend( false )
		, m_TCPPort( 0x5554 ) // default port is 0x5554 (UT)
		, m_Destination( "127.0.0.1" )
	{
		using boost::asio::ip::tcp;

		// check for configuration
		pConfig->m_DataflowAttributes.getAttributeData( "networkPort", m_TCPPort );
		if ( pConfig->m_DataflowAttributes.hasAttribute( "networkDestination" ) )
		{
			m_Destination = pConfig->m_DataflowAttributes.getAttributeString( "networkDestination" );
		}

		tcp::resolver resolver( m_IoService );
		
		// open new socket which we use for sending stuff
		m_SendSocket = boost::shared_ptr< tcp::socket >( new tcp::socket (m_IoService) );
		
		// resolve destination pair and store the remote endpoint
		

		std::ostringstream portString;
		portString << m_TCPPort;

		tcp::resolver::query query( tcp::v4(), m_Destination, portString.str() );
		resolver.async_resolve(query,  boost::bind(&ImageNetworkSinkComponent::resolve_handler, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::iterator)); 
		//m_IoService.run();
		boost::shared_ptr< boost::thread > m_NetworkThread = boost::shared_ptr< boost::thread >( new boost::thread( boost::bind( &boost::asio::io_service::run, &m_IoService ) ) );
				
	}

protected:
	

	void connect_handler(const boost::system::error_code& ec,
      boost::asio::ip::tcp::resolver::iterator endpoint_iterator) 
	{ 
		LOG4CPP_INFO(logger, "connect handler");
	  if (!ec) 
	  { 
		m_connected= true;
		LOG4CPP_INFO(logger, "Connected");
		
	  } 
	} 

	void resolve_handler(const boost::system::error_code &ec, boost::asio::ip::tcp::resolver::iterator it) 
	{ 
		LOG4CPP_INFO(logger, "resolve handler");
		if (!ec) 
		{ 
			m_SendSocket->async_connect(*it, boost::bind(&ImageNetworkSinkComponent::connect_handler, this,
			boost::asio::placeholders::error, ++it)); 
		} 
	}
	// receive a new pose from the dataflow
	void eventIn( const Measurement::ImageMeasurement& m )
	{
		if(!m_connected) return;
		
		
		

		std::string suffix("\n");
		Measurement::Timestamp sendtime = m.time();
				
		if(!m_headerSend) {
			/*
			std::ostringstream stream;
			boost::archive::text_oarchive packet( stream );
			// serialize the measurement, component name and current local time
			//packet << m_name;
			packet << m->width;
			packet << m->height;
			packet << m->nChannels;
			packet << m->depth;
			packet << m->origin;	
			m_headerSend = true;
			boost::asio::write(*m_SendSocket, boost::asio::buffer( stream.str().c_str(), stream.str().size() ));
			LOG4CPP_INFO(logger, "sending"<< stream.str().size());
			 */ 
			 int packet[5];
			packet[0] = m->width;
			packet[1] =  m->height;
			packet[2] =  m->nChannels;
			packet[3] =  m->depth;
			packet[4] = m->origin;	
			m_headerSend = true;
			boost::asio::write(*m_SendSocket, boost::asio::buffer( (char*) packet, 5*4 ));
		} else {
			//packet << boost::serialization::make_binary_object(m->imageData, m->imageSize );			
			boost::asio::write(*m_SendSocket, boost::asio::buffer( &sendtime, sizeof(sendtime) ));
			boost::asio::write(*m_SendSocket, boost::asio::buffer( m->imageData, m->imageSize ));
			//LOG4CPP_INFO(logger, "sending"<< m->imageSize);
		}
				
		
		//
		
				 
		//m_SendSocket->send_to( boost::asio::buffer( stream.str().c_str(), stream.str().size() ), *m_SendEndpoint );
	}

	// consumer port
	Dataflow::PushConsumer< ImageMeasurement > m_inPort;

	boost::asio::io_service m_IoService;
	boost::shared_ptr< boost::asio::ip::tcp::socket > m_SendSocket;	

	bool m_connected;
	bool m_headerSend;

	int m_TCPPort;
	std::string m_Destination;
};



// register module at factory
UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< ImageNetworkSinkComponent > ( "NetworkImageSink" );

}

} } // namespace Ubitrack::Drivers
