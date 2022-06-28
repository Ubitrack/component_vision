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

// std
#include <string>

// on windows, asio must be included before anything that possible includes windows.h
// don't ask why.
#include <boost/asio.hpp>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>

// Ubitrack
#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/ComponentFactory.h>

#ifdef HAVE_UTVISION
	#include <utVision/Image.h>
#endif

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

	// consumer port
	Dataflow::PushConsumer< Measurement::ImageMeasurement > m_inPort;

	/// the boost asio underlying network service
	boost::asio::io_service m_ioService;

	/// network socket
	boost::asio::ip::tcp::socket m_socket;

	/// thread that runs the boost asio network service, (if connections was established)
	boost::scoped_ptr< boost::thread > m_pNetworkThread;
	
	/// ip-address trying to reach
	std::string m_dstAddress;
	
	/// port at destination trying to reach
	std::string m_dstPort;

public:

	/** constructor */
	ImageNetworkSinkComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::Component( name )
		, m_inPort( "Input", *this, boost::bind( &ImageNetworkSinkComponent::eventIn, this,  boost::placeholders::_1 ) )
		, m_ioService()
		, m_socket( m_ioService )
		, m_dstAddress( "127.0.0.1" )
		, m_dstPort( "21844" ) // or 0x5554 as hex
	{
		// check for configurations and reset values if necessary
		pConfig->m_DataflowAttributes.getAttributeData( "networkPort", m_dstPort );
		if ( pConfig->m_DataflowAttributes.hasAttribute( "networkDestination" ) )
		{
			m_dstAddress = pConfig->m_DataflowAttributes.getAttributeString( "networkDestination" );
		}
	}
	
	virtual ~ImageNetworkSinkComponent()
	{
		m_ioService.stop();
	}

	virtual void start()
	{
		if( reset( m_dstAddress, m_dstPort ) )
			LOG4CPP_INFO( logger, "Starting to send image data to \"" << m_dstAddress << ":" << m_dstPort << "\"."  )
		else
			LOG4CPP_ERROR( logger, "Some problem occured, cannot start to send image data to \"" << m_dstAddress << ":" << m_dstPort << "\"."  )
	}
	
	virtual void stop()
	{
		if( m_socket.is_open() )
		{
			boost::system::error_code errCode;
			m_socket.shutdown( boost::asio::ip::tcp::socket::shutdown_both, errCode );
			if( errCode )
				LOG4CPP_ERROR( logger, "Could not shutdown the whole connection properly, got the following error:\n\n\"" << errCode.message() << "\" (\"" << errCode << "\")\n\n"  );
			m_socket.close();
		}
		
		LOG4CPP_TRACE( logger, "stopping thread to send to \"" << m_dstAddress << ":" << m_dstPort << "\"" );
		if( m_pNetworkThread )
			m_pNetworkThread->join();
	}
protected:
	
	bool reset( const std::string& dstIpAddress, const std::string& dstPortNr )
	{			
		boost::asio::ip::tcp::resolver resolver( m_ioService );
		
		boost::asio::ip::tcp::resolver::query query( boost::asio::ip::tcp::v4(), dstIpAddress, dstPortNr );
		boost::system::error_code errCode;
		boost::asio::ip::tcp::resolver::iterator result = resolver.resolve( query, errCode );
		
		if( errCode )
		{ 
			LOG4CPP_ERROR( logger, "Could not set the resolver for \"" << dstIpAddress << ":" << dstPortNr << "\" properly, got the following error:\n\n\"" << errCode.message() << "\" (\"" << errCode << "\")\n\n"  );
			return false;
		}
		
		LOG4CPP_INFO( logger, "Destination \"" << dstIpAddress << ":" << dstPortNr << "\" could be resolved, trying to establish a tcp connection." );
		
		// opening the socket to send some data
		m_socket.async_connect( *result
			, boost::bind( &ImageNetworkSinkComponent::connect_handler
			, this
			, boost::asio::placeholders::error, ++result ) ); 
		
		m_pNetworkThread.reset( new boost::thread( boost::bind( &boost::asio::io_service::run, &m_ioService ) ) );
		
		return true;
	}

	void connect_handler( const boost::system::error_code& errCode, boost::asio::ip::tcp::resolver::iterator endpoint_iterator ) 
	{ 
		if( errCode )
		{ 
			LOG4CPP_ERROR( logger, "Could not establish a tcp connection to \"" << m_dstAddress << ":" << m_dstPort << "\", got the following error:\n\n\"" << errCode.message() << "\" (\"" << errCode << "\")\n\n"  );
			return;
		}
		LOG4CPP_INFO( logger, "tcp connection to \"" << m_dstAddress << ":" << m_dstPort << "\" is established.\n" );
	}

	void eventIn( const Measurement::ImageMeasurement& m )
	{
		if( !m_socket.is_open() )
		{
			LOG4CPP_WARN( logger, "No connection to \"" << m_dstAddress << ":" << m_dstPort << "\", not sending any image." );
			return;
		}
		
		// remember internally if header was already send to destination
		static bool headerSend = false;
		
		if( !headerSend )
		{
			int imageHeaderInfos[ 7 ];
			imageHeaderInfos[ 0 ] = m->width();
			imageHeaderInfos[ 1 ] = m->height();
			imageHeaderInfos[ 2 ] = m->channels();
			imageHeaderInfos[ 3 ] = m->depth();
			imageHeaderInfos[ 4 ] = m->origin();
			imageHeaderInfos[ 5 ] = m->bitsPerPixel();
			imageHeaderInfos[ 6 ] = (int)m->pixelFormat();

			LOG4CPP_INFO( logger, "sending image header to \"" << m_dstAddress << ":" << m_dstPort << "\" :"
				<< "\nwidth   : " << imageHeaderInfos[ 0 ] << " [pixels]"
				<< "\nheight  : " << imageHeaderInfos[ 1 ] << " [pixels]"
				<< "\nchannels: " << imageHeaderInfos[ 2 ] 
				<< "\ndepth   : " << imageHeaderInfos[ 3 ] << " [bit]"
				<< "\norigin  : " << imageHeaderInfos[ 4 ] << " (0==top; 1==bottom)"
				<< "\nbitsPerPixel  : " << imageHeaderInfos[ 5 ] << " bits"
				<< "\nimageFormat  : " << imageHeaderInfos[ 6 ]
			);
				
			boost::asio::write( m_socket, boost::asio::buffer( &imageHeaderInfos[ 0 ], 7*sizeof(int) ) );
			headerSend = true;
		}
		else
		{
			LOG4CPP_TRACE( logger, "Sending image data to \"" << m_dstAddress << ":" << m_dstPort << "\"." );
			const Measurement::Timestamp t = m.time();
			boost::asio::write( m_socket, boost::asio::buffer( &t, sizeof( Measurement::Timestamp ) ) );
			boost::asio::write( m_socket, boost::asio::buffer( m->Mat().data, m->Mat().total()*m->Mat().elemSize()) );
		}
	}
};

// register module at factory
UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< ImageNetworkSinkComponent > ( "NetworkImageSink" );

}

} } // namespace Ubitrack::Drivers
