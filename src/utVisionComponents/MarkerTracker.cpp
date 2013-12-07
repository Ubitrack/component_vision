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
 * A compontent that tracks square markers.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#include <stdlib.h>
#include <string>
#include <sstream>
#include <boost/foreach.hpp>
#include <log4cpp/Category.hh>

#include <math.h>

#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/Module.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMath/Scalar.h>
#include <utMath/cast_assign.h>

#include <utVision/Image.h>
#include <utVision/MarkerDetection.h>

#include <opencv/cv.h>

//#define DO_TIMING

#ifdef DO_TIMING
#include <utUtil/BlockTimer.h>
#endif

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.MarkerTracker" ) );

// namespace shortcuts
using namespace Ubitrack;
using namespace Ubitrack::Dataflow;
using namespace Ubitrack::Vision;
using namespace Ubitrack::Vision::Markers;
using namespace Ubitrack::Math;
namespace ublas = boost::numeric::ublas;

namespace Ubitrack { namespace Drivers {





/** the component key -- reads the marker ID */
class IdKey
	: public Math::Scalar< unsigned long long int >
{
public:
	/** extract id from configuration */
	IdKey( boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	{
		std::string sId = subgraph->getNode( "Marker" )->getAttributeString( "markerId" );
		if ( sId.empty() )
			UBITRACK_THROW( "Missing markerId attribute on node Marker" );
		std::istringstream idStream( sId );
		idStream >> std::hex >> m_value;
	}

	// create from unsignend long long int
	IdKey( unsigned long long int v )
	{ m_value = v; }
};

/** camera key just takes the node id */
MAKE_NODEIDKEY( CameraIdKey, "Camera" );

// forward declarations
class MarkerTracker;
class MarkerTrackerModule;

typedef Module< CameraIdKey, IdKey, MarkerTrackerModule, MarkerTracker > MarkerTrackerModuleBase;

/**
 * Tracks markers and send the results to the components.
 * This is done only once for each timestamp.
 */
class MarkerTrackerModule
	: public MarkerTrackerModuleBase
{
public:
	MarkerTrackerModule( const CameraIdKey& key, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, FactoryHelper* pFactory )
		: MarkerTrackerModuleBase( key, pFactory )
		, m_lastTimestamp( 0 )
		, m_markerSize( 6 )
		, m_codeSize( 4 )
		, m_codeMask( 0xFFFF )
		, m_useInnerEdgels( true )
#ifdef DO_TIMING
		, m_detectMarkersTimer( "detectMarkers", "Ubitrack.Timing" )
		, m_detectMarkersTimerRefine( "detectMarkersRefine", "Ubitrack.Timing" )
#endif
	{
		// get configuration
		std::string sMask = subgraph->getNode( "Camera" )->getAttributeString( "markerIdMask" );
		if ( ! sMask.empty() ) {
			std::istringstream idStream( sMask );
			idStream >> std::hex >> m_codeMask;
		}
		subgraph->getNode( "Camera" )->getAttributeData( "markerBitSize", m_markerSize );
		subgraph->getNode( "Camera" )->getAttributeData( "codeBitSize", m_codeSize );
		m_useInnerEdgels = subgraph->getNode( "Camera" )->getAttributeString( "enableInnerEdgels" ) == "true";

		LOG4CPP_DEBUG( logger, "Marker tracker configuration: marker bit size: " << m_markerSize << ", code bit size: " << m_codeSize << ", ID mask: " << m_codeMask << ", use inner edgelets: " << m_useInnerEdgels );

		bRefine = false;
		bPrevPoseVal = false;
		nframecounter = 0;
	}

	/** find markers in the image and send results to the components */
	void trackMarkers( const Measurement::ImageMeasurement& );

protected:

	// timestamp of the last measurement
	Measurement::Timestamp m_lastTimestamp;

	// timestamp of the last time the image was analysed fully 
	//Measurement::Timestamp m_fullAnalizeTimestep;

	// a flag that desides whether analize full image or just refine
	bool bRefine;

	bool bPrevPoseVal;
	//std::map< unsigned, MarkerInfo > m_markerMap;
	int nframecounter;

	/** Size of marker including its border, counted in bits */
	unsigned int m_markerSize;

	/** Size of marker bit pattern, counted in bits */
	unsigned int m_codeSize;
	
	/** Mask ANDed with bit pattern detected in image to obtain the final marker ID */
	unsigned long long int m_codeMask;
	
	/** Incorporate inner edgelets in pose refinement, may be unstable! */
	bool m_useInnerEdgels;

	#ifdef DO_TIMING
	Ubitrack::Util::BlockTimer m_detectMarkersTimer;
	Ubitrack::Util::BlockTimer m_detectMarkersTimerRefine;
	#endif
};


/**
 * @ingroup vision_components
 * Tracks markers in greyscale images.
 *
 * @par input ports
 * PushConsumer< Measurement
Measurement::ImageMeasurement > of name "Input", accepts
 * only greyscale images
 *
 * @par Output Ports
 * PushSupplier< Measurement::Pose > Output -- sends the pose of tracked marker \n
 * PushSupplier< ImageMeasurement > DebugOut -- sends a debug image if connected
 *
 * @par The Pattern
 * @verbatim
<Pattern name="MarkerTracker">
	<Input>
		<Node name="Camera"/>
		<Node name="ImagePlane"/>
		<Node name="Marker" id="someOtherId">
			<Predicate>markerType=="UbitrackSquareMarker"&amp;&amp;markerId!=""</Predicate> 
		</Node>
		<Edge name="Image" source="Camera" destination="ImagePlane">
			<Predicate>type=="image"&amp;&amp;mode="push"</Predicate>
		</Edge>
		<Edge name="Config" source="Camera" destination="Marker">
			<Predicate>trackable=="UbitrackSquareMarker"</Predicate>
		</Edge>
	</Input>
	<Output>
		<Edge name="Output" source="Camera" destination="Marker">
			<Attribute name="type" value="6D"/>
			<Attribute name="mode" value="push"/>
		</Edge>
	</Output>
	<DataflowConfiguration>
		<UbitrackLib class="MarkerTracker"/>
	</DataflowConfiguration>
</Pattern>
@endverbatim
 * Note that the "Config" edge is optional and can be used to restrict marker tracking to only "visible"
 * markers, where "visibility" is determined by some other process.
 *
 * @par Dataflow Configuration
 * @verbatim
<Pattern name="MarkerTracker">
	<Input>
		<Node name="Camera" id="someId1"/>
		<Node name="ImagePlane" id="someId2"/>
		<Edge name="Image" source="Camera" destination="ImagePlane" pattern-ref="..." edge-ref="..."/>
	</Input>
	<Output>
		<Node name="Marker" id="someOtherId">
			<Attribute name="markerId" value="0xID"/> 
			<Attribute name="markerSize" value="0.06"/>
		</Node>
		<Edge name="Output" source="Camera" destination="Marker">
			<Attribute name="type" value="6D"/>
			<Attribute name="mode" value="push"/>
		</Edge>
	</Output>
	<DataflowConfiguration>
		<UbitrackLib class="MarkerTracker"/>
	</DataflowConfiguration>
</Pattern>
@endverbatim
 *
 * \p id is the normalized hexadecimal ID of a marker. If marker codes are interpreted as
 * binary numbers read row-wise starting top-left, the normalized ID is the one which represents
 * the smallest number. The pose is also computed w.r.t the normal marker orientation.
 * \p markerSize is in meters
 *
 * There is also a \c DebugImage output port, which - when connected - creates debug images.
 * An additional \c CameraIntrinsics input port can be used to supply a 3x3 camera intrinsics
 * matrix as pull.
 *
 * The \c ErrorPose output port gives the resulting marker pose with a covariance matrix, 
 * if connected.
 *
 * The component also has an output port named \c Corners that only give the 2D corner positions
 * of the marker in counter-clockwise order, starting top-left.
 */
class MarkerTracker
	: public MarkerTrackerModule::Component
{
public:
	MarkerTracker( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const IdKey& componentKey, MarkerTrackerModule* pModule )
		: MarkerTrackerModule::Component( sName, componentKey, pModule )
		, m_inPort( "Image", *this, boost::bind( &MarkerTracker::pushImage, this, _1 ) )
		, m_inIntrinsics( "CameraIntrinsics", *this )
		, m_outCorners( "Corners", *this )
		, m_outPort( "Output", *this )
		, m_outErrorPose( "ErrorPose", *this )
		, m_debugPort( "DebugImage", *this )
		, m_bEdgeRefinement( true )
		, m_info( 0.06f )
		, m_lastTime( 0 )
	{
		// get configuration
		subgraph->getNode( "Marker" )->getAttributeData( "markerSize", m_info.fSize );

		if ( subgraph->m_DataflowAttributes.hasAttribute( "edgeRefinement" ) ) // enable Edge Refinement
			m_bEdgeRefinement = subgraph->m_DataflowAttributes.getAttributeString( "edgeRefinement" ) == "true";
		
		if ( subgraph->m_DataflowAttributes.hasAttribute( "enableTracking" ) ) // enable Tracking
			m_info.bEnableTracking = subgraph->m_DataflowAttributes.getAttributeString( "enableTracking" ) == "true";
		
		if ( subgraph->m_DataflowAttributes.hasAttribute( "enablePixelFlow" ) ) // enable Pixel Flow
			m_info.bEnablePixelFlow = subgraph->m_DataflowAttributes.getAttributeString( "enablePixelFlow" ) == "true";
		
		if ( subgraph->m_DataflowAttributes.hasAttribute( "enableFlipCheck" ) ) // enable Flip chaeck 
			m_info.bEnableFlipCheck = subgraph->m_DataflowAttributes.getAttributeString( "enableFlipCheck" ) == "true";
		
		if ( subgraph->m_DataflowAttributes.hasAttribute( "enableFastTracking" ) ) // enable Fast Tracking
			m_info.bEnableFastTracking = subgraph->m_DataflowAttributes.getAttributeString( "enableFastTracking" ) == "true";		
	}

	/** is the debug port connected? */
	bool debug()
	{ return m_debugPort.isConnected(); }

	bool isIntrinsics()
	{ return m_inIntrinsics.isConnected(); }
	
	/** returns the camera intrinsics if connected. throws otherwise */
	Measurement::Matrix3x3 intrinsics( Measurement::Timestamp t )
	{ return m_inIntrinsics.get( t ); }
	
	/** use edge refinement? */
	bool useEdgeRefinement() const
	{ return m_bEdgeRefinement; }

protected:
	/** method that receives events and displays the image */
	void pushImage( const Measurement::ImageMeasurement& m )
	{ getModule().trackMarkers( m ); }

	// ports
	Dataflow::PushConsumer< Measurement::ImageMeasurement > m_inPort;
	Dataflow::PullConsumer< Measurement::Matrix3x3 > m_inIntrinsics;
	Dataflow::PushSupplier< Measurement::PositionList2 > m_outCorners;
	Dataflow::PushSupplier< Measurement::Pose > m_outPort;
	Dataflow::PushSupplier< Measurement::ErrorPose > m_outErrorPose;
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_debugPort;

	// config
	bool m_bEdgeRefinement;
	
	/** Information about marker */
	MarkerInfo m_info;
	
	// some variables where the module stores information
	Measurement::Timestamp m_lastTime;
			
	friend class MarkerTrackerModule;
};

void MarkerTrackerModule::trackMarkers( const Measurement::ImageMeasurement& m )
{
	// check if image was already analyzed
	if ( m.time() == m_lastTimestamp )
		return;
	m_lastTimestamp = m.time();

	//declearation of iterator, will be used several times later on
	std::map<unsigned,MarkerInfo>::iterator iter;
	
	// debug image (if anybody is interested )
	boost::shared_ptr< Image > pDebugImg;
	{
		// check if debug image needs to be created
		ComponentList allComponents( getAllComponents() );
		for ( ComponentList::iterator it = allComponents.begin(); it != allComponents.end(); it++ )
			if ( (*it)->debug() )
			{
				pDebugImg = m->CvtColor( CV_GRAY2RGB, 3 );
				break;
			}
	}

	// get all components
	ComponentList components = getAllComponents();

	// get intrinsics matrix from first component
	Math::Matrix< float, 3, 3 > K;
	if ( components.front()->isIntrinsics() )
	{
		Math::matrix_cast_assign( K, *components.front()->intrinsics( m.time() ) );
	}
	else
	{
		LOG4CPP_DEBUG( logger, "No intrinsics matrix given" );
		
		// compute cheap camera matrix if none given
		float fTx = static_cast< float >( m->width / 2 );
		float fTy = static_cast< float >( m->height / 2 );
		float f = 1.25f * m->width;
		K( 0, 0 ) = f;
		K( 0, 1 ) = 0.0f;
		K( 0, 2 ) = -fTx;
		K( 1, 1 ) = f;
		K( 1, 2 ) = -fTy;
		K( 2, 2 ) = -1.0f;
		K( 1, 0 ) = K( 2, 0 ) = K( 2, 1 ) = 0.0f;
	}
	
	// create marker map
	MarkerInfoMap markerMap;
	MarkerInfoMap refineMarkerMap;
	
	// copy all nesesary information to a marker map
	BOOST_FOREACH( ComponentList::value_type pComp, components )
	{
		pComp->m_info.found = MarkerInfo::ENotFound;
		
		if ( pComp->m_info.nPrevPoseValidator < 3 || !pComp->m_info.bEnableTracking || !pComp->m_info.bEnableFastTracking )
		{	
			// set refinement level
			if ( pComp->m_outPort.isConnected() || pComp->m_outErrorPose.isConnected() || pDebugImg )
				if ( pComp->useEdgeRefinement() )
					pComp->m_info.refinement = MarkerInfo::EEdgeRefinedPose;
				else
					pComp->m_info.refinement = MarkerInfo::ERefinedPose;
			else
				pComp->m_info.refinement = MarkerInfo::ECorners;

			pComp->m_info.bUseInitialPose = m.time() < pComp->m_lastTime + 250000000;

			// covariance?
			pComp->m_info.bCalculateCovariance = pComp->m_outErrorPose.isConnected();
			
			// copy to full scan marker map
			markerMap[ pComp->getKey() ] = pComp->m_info;
		}
		else
			// copy to refinement map
			refineMarkerMap[ pComp->getKey() ] = pComp->m_info;
	}

	// try to refine markers with fast tracking enabled
	if ( !refineMarkerMap.empty() )
	{
		{
			#ifdef DO_TIMING
			UBITRACK_TIME( m_detectMarkersTimerRefine );
			#endif
			detectMarkers( *m, refineMarkerMap, K, pDebugImg.get(), true, m_codeSize, m_markerSize, m_codeMask, m_useInnerEdgels );
		}
	
		// copy not-found markers to marker map and update others
		BOOST_FOREACH( MarkerInfoMap::value_type& mapEl, refineMarkerMap )
		{
			// update found markers
			if ( mapEl.second.found != MarkerInfo::ENotFound )
				getComponent( mapEl.first )->m_info = mapEl.second;

			// if not or only found by pixel flow, add to full scan list
			if ( mapEl.second.found != MarkerInfo::ERefinementFound )
				markerMap[ mapEl.first ] = mapEl.second;
		}
		
		refineMarkerMap.clear();
	}
	// run marker tracker with full analysis on other and not-found markers
	if ( !markerMap.empty() )
	{
		{
			#ifdef DO_TIMING
			UBITRACK_TIME( m_detectMarkersTimer );
			#endif
			detectMarkers( *m, markerMap, K, pDebugImg.get(), false, m_codeSize, m_markerSize, m_codeMask, m_useInnerEdgels );
		}

		// update information of found markers and move not-found not-fast-tracking markers to refinement
		BOOST_FOREACH( MarkerInfoMap::value_type& mapEl, markerMap )
			if ( mapEl.second.found == MarkerInfo::EFullScanFound )
				getComponent( mapEl.first )->m_info = mapEl.second;
			else if ( mapEl.second.bEnableTracking && !mapEl.second.bEnableFastTracking && 
				mapEl.second.nPrevPoseValidator >= 3 )
				refineMarkerMap[ mapEl.first ] = mapEl.second;
	}
	// try to refine not-fast-tracking markers not found by full scan 
	if ( !refineMarkerMap.empty() )
	{
		{
			#ifdef DO_TIMING
			UBITRACK_TIME( m_detectMarkersTimerRefine );
			#endif
			detectMarkers( *m, refineMarkerMap, K, pDebugImg.get(), true, m_codeSize, m_markerSize, m_codeMask, m_useInnerEdgels );
		}
	
		// update found markers
		BOOST_FOREACH( MarkerInfoMap::value_type& mapEl, refineMarkerMap )
			// update found markers
			if ( mapEl.second.found != MarkerInfo::ENotFound )
				getComponent( mapEl.first )->m_info = mapEl.second;
	}

	// check which markers were found
	BOOST_FOREACH( ComponentList::value_type pComp, components )
	{
		MarkerInfo& info( pComp->m_info );
		
		// timeout pixelflow after 5 seconds. TODO: make configureable
		if ( info.found == MarkerInfo::EPixelFlowFound && pComp->m_lastTime + 5000000000LL < m.time() )
		{
			info.found = MarkerInfo::ENotFound;
			info.nPrevPoseValidator = 0;
		}
			
		// update lastTime only when found by refinement or full scan
		if ( info.found >= MarkerInfo::ERefinementFound )
			pComp->m_lastTime = m.time();
		
		if ( info.found >= MarkerInfo::ERefinementFound || ( info.found == MarkerInfo::EPixelFlowFound && info.bEnablePixelFlow ) )
		{
			// send corners
			if ( pComp->m_outCorners.isConnected() )
			{
				LOG4CPP_DEBUG( logger, "Marker " << std::hex << pComp->getKey() << ": sending corners [" 
					<< info.corners[ 0 ] << ", " << info.corners[ 1 ] << ", " << info.corners[ 2 ] << ", " << info.corners[ 3 ] << "]" );

				// convert from float to double
				boost::shared_ptr< std::vector< Math::Vector< double, 2 > > > pCorners( new std::vector< Math::Vector< double, 2 > > );
				for ( unsigned i = 0; i < 4; i++ )
					pCorners->push_back( Math::Vector< double, 2 >( info.corners[ i ] ) );

				pComp->m_outCorners.send( Measurement::PositionList2( m.time(), pCorners ) );
			}

			// send to component
			try
			{
				LOG4CPP_DEBUG( logger, "Marker " << std::hex << pComp->getKey() << ": sending pose " << info.pose );
				pComp->m_outPort.send( Measurement::Pose( m.time(), info.pose ) );
			}
			catch ( ... )
			{}

			// also send ErrorPose if anybody is connected
			if ( info.bCalculateCovariance )
				pComp->m_outErrorPose.send( Measurement::ErrorPose( m.time(), ErrorPose( info.pose, info.covariance ) ) );
		}
	}

	// push debug image
	if ( pDebugImg )
	{
		ComponentList components = getAllComponents();
		for ( ComponentList::iterator it = components.begin(); it != components.end(); it++ ) {
			(*it)->m_debugPort.send( Measurement::ImageMeasurement( m.time(), pDebugImg ) );
		}
	}
}


} } // namespace Ubitrack::Driver


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerModule< Ubitrack::Drivers::MarkerTrackerModule > ( "MarkerTracker" );
}
