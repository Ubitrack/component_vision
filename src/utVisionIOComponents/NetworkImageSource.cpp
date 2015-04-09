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
 * NetworkSource component.
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

#include <utDataflow/Component.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>



#include <utVision/Image.h>
#include <opencv/highgui.h>

#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Drivers.NetworkSource" ) );

namespace Ubitrack { namespace Vision {


	
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
class ImageNetworkSourceComponent
	: public Dataflow::Component
{
	// consumer port
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;

	int m_listenPort;

	// boost asio service
	boost::asio::io_service m_ioService;

	// the socket for receiving image information
	boost::asio::ip::tcp::socket m_socket;

	// accepting incoming packages
	boost::asio::ip::tcp::acceptor m_acceptor;

	/// thread running the IO service
	boost::scoped_ptr< boost::thread > m_pNetworkThread;

	/**
	 * array to store image header values
	 * 
	 * contains the following elements in consecutive order
	 * - width
	 * - height
	 * - nChannels;
	 * - depth;
	 * - origin;
	 */
	int imageHeaderInfos[ 5 ];
	
public:

	/** constructor */
	ImageNetworkSourceComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::Component( name )
		, m_outPort( "Output", *this )
		, m_listenPort( 0x5554 ) // default port is 0x5554 (UT) 21844
		, m_ioService( )
		, m_socket ( m_ioService )
		, m_acceptor( m_ioService, boost::asio::ip::tcp::endpoint( boost::asio::ip::tcp::v4(), m_listenPort ) )
	{
		// check for configuration
		if( pConfig->m_DataflowAttributes.hasAttribute( "networkPort" ) )
		{
			LOG4CPP_WARN( logger, "Attention: if your port is different than " << m_listenPort << " the component will not work at the moment." );
			// pConfig->m_DataflowAttributes.getAttributeData( "networkPort", m_listenPort );
			// m_acceptor.bind( boost::asio::ip::tcp::endpoint( boost::asio::ip::tcp::v4(), m_listenPort ) );
		}
	}
	
	virtual void start()
	{
		restart();
	}
	
	virtual void stop()
	{
		
		if( m_socket.is_open() )
		{
			m_socket.cancel();
			LOG4CPP_NOTICE( logger, "tcp socket " << m_listenPort << ": cancelled" );
			boost::system::error_code errCode;
			m_socket.shutdown( boost::asio::ip::tcp::socket::shutdown_both, errCode );
			if( errCode )
				LOG4CPP_ERROR( logger, "Could not shutdown the whole connection properly, got the following error:\n\n\"" << errCode.message() << "\" (\"" << errCode << "\")\n\n"  );
			LOG4CPP_NOTICE( logger, "tcp socket " << m_listenPort << ": shutdown" );
			
			m_socket.close();
			LOG4CPP_NOTICE( logger, "tcp socket " << m_listenPort << ": closed" );
		}
		
		if( m_pNetworkThread )
		{
			m_ioService.stop();
			LOG4CPP_NOTICE( logger, "tcp socket " << m_listenPort << ": io service stopped" );
			
			
			if( m_pNetworkThread->joinable() )
			{
				
				LOG4CPP_NOTICE( logger, "tcp socket " << m_listenPort << ": listening thread asked to stop" );
				m_pNetworkThread->join(); ///< somehow this does not really work @todo have a look at closing the thread correctly
				LOG4CPP_NOTICE( logger, "tcp socket " << m_listenPort << ": listening thread stopped" );
				m_pNetworkThread.reset();
			}
		}
		LOG4CPP_INFO( logger, "tcp socket " << m_listenPort << ": stopped listening" );
	}

protected:
	
	void restart()
	{
		LOG4CPP_INFO( logger, "Starting network thread to listen on port " <<  m_listenPort << "." );
		// clean up if there is already a connection
		stop();
		
		// starting the handler thread for fetching images
		m_acceptor.listen();
		m_acceptor.async_accept( m_socket, boost::bind( &ImageNetworkSourceComponent::dataHeaderHandler, this, boost::asio::placeholders::error ) );
		m_pNetworkThread.reset( new boost::thread( boost::bind( &boost::asio::io_service::run, &m_ioService ) ) );
		
		LOG4CPP_INFO( logger, "thread started to listen on tcp port " << m_listenPort );
	}

	void dataHeaderHandler( const boost::system::error_code &errCode )
	{ 
		LOG4CPP_INFO( logger, "data handler called" );
		
		if( errCode )
		{
			LOG4CPP_ERROR( logger, "Could not read incoming data properly, got the following error:\n\n\"" << errCode.message() << "\" (\"" << errCode << "\")\n\n"  );
			return;
		}
		
		while( m_socket.is_open() )
		{
			const std::size_t arrivedBytes = m_socket.available();

			if ( !arrivedBytes )
			{
				LOG4CPP_TRACE( logger, "No packet arrived yet" );
				continue;
			}
			
			LOG4CPP_TRACE( logger, "Received " << arrivedBytes << " bytes of image data." );		
			
			static bool hasHeaderArrived = false;
			
			if( !hasHeaderArrived )
				hasHeaderArrived = fetchImageHeader();
			else
				fetchImageContent();
		}
		LOG4CPP_WARN( logger, "Socket has been closed." );
	}
	
	bool fetchImageHeader()
	{
		// boost::system::error_code msgError;
		const std::size_t sizeHeader = boost::asio::read( m_socket, boost::asio::buffer( &imageHeaderInfos[ 0 ], 5*sizeof(int) ) );//, msgError );
		if( sizeHeader != 5*sizeof(int) ) 
		{
			LOG4CPP_ERROR( logger, "Arrived packet did not contain enough data ("<< sizeHeader << " bytes, expected at least "<<  5*sizeof(int) << ") for image header, trying once more." );
			return false;
		}

		LOG4CPP_INFO( logger, "Received image header, expecting images of type" 
			<< "\nwidth   : " << imageHeaderInfos[ 0 ] << " [pixels]"
			<< "\nheight  : " << imageHeaderInfos[ 1 ] << " [pixels]"
			<< "\nchannels: " << imageHeaderInfos[ 2 ] 
			<< "\ndepth   : " << imageHeaderInfos[ 3 ] << " [bit]"
			<< "\norigin  : " << imageHeaderInfos[ 4 ] << " (0==top; 1==bottom)" );
			
		return true;
	}
	
	
	/// fetching the image data (+timestamp) from the socket
	bool fetchImageContent()
	{
		Measurement::Timestamp sendtime;
		{
			boost::system::error_code msgErrorTime;
			/*const std::size_t lenTime =*/ boost::asio::read( m_socket, boost::asio::buffer( &sendtime, sizeof( Measurement::Timestamp ) ) , boost::asio::transfer_all(), msgErrorTime );
			if( msgErrorTime )
			{
				LOG4CPP_ERROR( logger, "Could not read incoming timestamp properly, got the following error:\n\n\"" << msgErrorTime.message() << "\" (\"" << msgErrorTime << "\")\n\n"  );
				return false;
			}
		}
		
		Image::Ptr currentImage( new Image( imageHeaderInfos[ 0 ], imageHeaderInfos[ 1 ], imageHeaderInfos[ 2 ], imageHeaderInfos[ 3 ], imageHeaderInfos[ 4 ] ) );
		
		{	// preparing the reading buffer
			const std::size_t m_numBytes = static_cast< std::size_t >( imageHeaderInfos[ 0 ] * imageHeaderInfos[ 1 ] * imageHeaderInfos[ 2 ] );
			
			boost::system::error_code msgErrorData;
			/*const std::size_t lenData =*/ boost::asio::read( m_socket, boost::asio::buffer( currentImage->imageData, m_numBytes ) , boost::asio::transfer_all(), msgErrorData );
			if( msgErrorData )
			{
				LOG4CPP_ERROR( logger, "Could not read incoming image data properly, got the following error:\n\n\"" << msgErrorData.message() << "\" (\"" << msgErrorData << "\")\n\n"  );
				return false;
			}
		}
		
		m_outPort.send( Measurement::ImageMeasurement( sendtime, currentImage ) );
		return true;
	}
};

// register module at factory
UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< ImageNetworkSourceComponent > ( "NetworkImageSource" );

}

} } // namespace Ubitrack::Vision
