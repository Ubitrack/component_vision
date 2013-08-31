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

// WARNING: all boost/serialization headers should be
//          included AFTER all boost/archive headers
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
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

#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utDataflow/Component.h>
#include <utMeasurement/Measurement.h>

#include <utVision/Image.h>
#include <opencv/highgui.h>

namespace Ubitrack { namespace Vision {

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Drivers.NetworkSource" ) );
	
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

public:

	/** constructor */
	ImageNetworkSourceComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::Component( name )
		, m_outPort( "Output", *this )
		, m_IoService()
		, m_TCPPort( 0x5554 ) // default port is 0x5554 (UT)		
		, m_connected(false)
	{
		using boost::asio::ip::tcp;

		// check for configuration
		pConfig->m_DataflowAttributes.getAttributeData( "networkPort", m_TCPPort );
		
				
		m_acceptor = boost::shared_ptr<tcp::acceptor>(
        new tcp::acceptor(
            m_IoService,
            tcp::endpoint(tcp::v4(), m_TCPPort)));
		
		// open new socket which we use for sending stuff
		/*
		m_SendSocket = boost::shared_ptr< tcp::socket >( new tcp::socket (m_IoService) );
		
		m_acceptor->listen(); 
		m_acceptor->async_accept(*m_SendSocket, boost::bind(&ImageNetworkSourceComponent::accept_handler, this,
          boost::asio::placeholders::error)); 
		*/
		start_accept();
		boost::shared_ptr< boost::thread > m_NetworkThread = boost::shared_ptr< boost::thread >( new boost::thread( boost::bind( &boost::asio::io_service::run, &m_IoService ) ) );
				
	}

protected:
	
	void start_accept() {
		m_SendSocket = boost::shared_ptr< boost::asio::ip::tcp::socket >( new boost::asio::ip::tcp::socket (m_IoService) );
		
		m_acceptor->listen(); 
		m_acceptor->async_accept(*m_SendSocket, boost::bind(&ImageNetworkSourceComponent::accept_handler, this,
          boost::asio::placeholders::error)); 
	}
	

	void accept_handler(const boost::system::error_code &ec) 
	{ 
		LOG4CPP_INFO(logger, "accept handler");
		if (!ec) 
		{ 
			
			boost::system::error_code error;
			/*
			char headerBuffer[42];
			headerBuffer[41] = '\0';
			 
			size_t len  = boost::asio::read(*m_SendSocket, boost::asio::buffer(headerBuffer,41),boost::asio::transfer_all(), error);
			
			std::string data( headerBuffer );
			LOG4CPP_INFO(logger, "Header:" << data  );			
			std::istringstream stream( data );
			boost::archive::text_iarchive message( stream );
			
			int width, height, nChannels, depth, origin;
			
			message >> width;
			message >> height;
			message >> nChannels;
			message >> depth;
			message >> origin;	
			 */
			 int headerBuffer[5];
			 size_t len  = boost::asio::read(*m_SendSocket, boost::asio::buffer((char*)headerBuffer,5*4),boost::asio::transfer_all(), error);
			LOG4CPP_INFO(logger, "HeaderData:" << headerBuffer[0] << " : " << headerBuffer[1] << " : " << headerBuffer[2] << " : " << headerBuffer[3] << " : " << headerBuffer[4]);
				
			while(!error) {
				boost::shared_ptr< Image > currentImage(new Image(headerBuffer[0], headerBuffer[1], headerBuffer[2], headerBuffer[3], headerBuffer[4]));
				
				Measurement::Timestamp sendtime;
				size_t len_new  = boost::asio::read(*m_SendSocket, boost::asio::buffer(&sendtime, sizeof(sendtime)),boost::asio::transfer_all(), error);
				
				//Measurement::Timestamp timeNow = Measurement::now();				
				//LOG4CPP_INFO(logger, "timestamp:" << sendtime << " : "<< sendtime - timeNow << " : "<< timeNow - sendtime  );
				
				len_new  = boost::asio::read(*m_SendSocket, boost::asio::buffer(currentImage->imageData, currentImage->imageSize),boost::asio::transfer_all(), error);
				
		
				m_outPort.send( Measurement::ImageMeasurement( sendtime, currentImage ) );
				
				
				if(!error) {
					// There's no easy way to read into an std::string directly, but it's
					// possible to read into an std::vector and then use the fact that
					// vectors are guaranteed to be contiguous to construct a string from
					// the underlying raw string. It's also possible to use a boost::array
					// instead.
					//LOG4CPP_INFO(logger, "got data" << len_new  );
				}
				
				}
				LOG4CPP_INFO(logger, "done" << len  );
				
				start_accept();
		} 
	}
	// receive a new pose from the dataflow
	

	// consumer port
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;

	boost::asio::io_service m_IoService;
	boost::shared_ptr< boost::asio::ip::tcp::acceptor > m_acceptor;
	boost::shared_ptr< boost::asio::ip::tcp::socket > m_SendSocket;	
	
	int m_TCPPort;
	
	bool m_connected;
};



// register module at factory
UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< ImageNetworkSourceComponent > ( "NetworkImageSource" );

}

} } // namespace Ubitrack::Drivers
