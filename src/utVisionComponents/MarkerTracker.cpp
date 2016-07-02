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

#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/Module.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMath/Scalar.h>
#include <utMath/Util/cast_assign.h>

#include <utVision/Image.h>
#include <utVision/MarkerDetection.h>

#include <opencv/cv.h>

#include "MarkerTracker.h"
//#define DO_TIMING

#ifdef DO_TIMING
#include <utUtil/BlockTimer.h>
#endif
//MultiMarkerTracker
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/lexical_cast.hpp>
#include <utUtil/CalibFile.h>
#include <utAlgorithm/PoseEstimation2D3D/PlanarPoseEstimation.h>
#include <utAlgorithm/Homography.h>
#define _USE_MATH_DEFINES
#include <math.h>


// namespace shortcuts
using namespace Ubitrack;
using namespace Ubitrack::Dataflow;
using namespace Ubitrack::Vision;
using namespace Ubitrack::Vision::Markers;
using namespace Ubitrack::Math;
namespace ublas = boost::numeric::ublas;

namespace Ubitrack { namespace Drivers {


MarkerTrackerModule::MarkerTrackerModule( const CameraIdKey& key, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, FactoryHelper* pFactory )
		: Module< CameraIdKey, IdKey, MarkerTrackerModule, MarkerTrackerBase >( key, pFactory )
		, m_lastTimestamp( 0 )
		, m_markerSize( 6 )
		, m_codeSize( 4 )
		, m_codeMask( 0xFFFF )
		, m_useInnerEdgels( true )
		, m_binaryThresholdValue( 120 )
		, m_enableAdaptiveThreshold ( true )
#ifdef DO_TIMING
		, m_detectMarkersTimer( "detectMarkers", "Ubitrack.Timing" )
		, m_detectMarkersTimerRefine( "detectMarkersRefine", "Ubitrack.Timing" )
#endif
{
	// check if correct node is available
	if( !subgraph->hasNode( "Camera" ) )
	{
		LOG4CPP_ERROR( logger, "Cannot start marker tracker module, \"Camera\"-node is missing in dfg." );
		UBITRACK_THROW( "Cannot start marker tracker module, \"Camera\"-node is missing in dfg." );
	}
		
	// get configuration
	std::string sMask = subgraph->getNode( "Camera" )->getAttributeString( "markerIdMask" );
	if ( ! sMask.empty() )
	{
		std::istringstream idStream( sMask );
		idStream >> std::hex >> m_codeMask;
	}
	subgraph->getNode( "Camera" )->getAttributeData( "markerBitSize", m_markerSize );
	subgraph->getNode( "Camera" )->getAttributeData( "codeBitSize", m_codeSize );
	m_useInnerEdgels = subgraph->getNode( "Camera" )->getAttributeString( "enableInnerEdgels" ) == "true";

	m_enableAdaptiveThreshold = false;
	m_binaryThresholdValue = 120;
		
	subgraph->m_DataflowAttributes.getAttributeData( "enableAdaptiveThreshold", m_enableAdaptiveThreshold );
	subgraph->m_DataflowAttributes.getAttributeData( "ThresholdValue", m_binaryThresholdValue );
				
	LOG4CPP_DEBUG( logger, "Marker tracker configuration: marker bit size: " << m_markerSize << ", code bit size: " << m_codeSize << ", ID mask: " << m_codeMask << ", use inner edgelets: " << m_useInnerEdgels << " enableAdaptiveThreshold " << m_enableAdaptiveThreshold << " binaryThresholdValue " << m_binaryThresholdValue);
}

Math::Matrix< float, 3, 3 > MarkerTrackerModule::getIntrinsicsMatrix(const Measurement::ImageMeasurement& m)
{
	ComponentList components = getAllComponents();
	Math::Matrix< float, 3, 3 > K;
	// Since all components are stored as shared_ptr the following loop does not work :(
	// ComponentList::iterator it = std::find_if ( components.begin(), components.end() , std::mem_fun( &MarkerTracker::isIntrinsics ) );
	ComponentList::const_iterator it = components.begin();
	for( ; it != components.end(); ++it )
	{		
		if( (*it)->isIntrinsics() )
		{
			boost::shared_ptr<MarkerTracker> lMTrackerIntrinicsComp = boost::static_pointer_cast<MarkerTracker>(*it);
			
			Math::Util::matrix_cast_assign( K, * lMTrackerIntrinicsComp->intrinsics( m.time() ));
			break;
		}
	}

	if( it == components.end() )
	{
		LOG4CPP_WARN( logger, "Guessing an own 3-by-3 intrinsic matrix, since no matrix is provided." );
		
		// compute cheap camera matrix if none given
		float fTx = static_cast< float >( m->width() / 2 );
		float fTy = static_cast< float >( m->height() / 2 );
		float f = 1.25f * m->width();
		K( 0, 0 ) = f;
		K( 0, 1 ) = 0.0f;
		K( 0, 2 ) = -fTx;
		K( 1, 1 ) = f;
		K( 1, 2 ) = -fTy;
		K( 2, 2 ) = -1.0f;
		K( 1, 0 ) = K( 2, 0 ) = K( 2, 1 ) = 0.0f;
	}
	LOG4CPP_TRACE( logger, "Applying the following 3-by-3 intrinsic matrix to the marker tracker module:\n" << K );
	return K;
}

void MarkerTrackerModule::trackMarkers( const Measurement::ImageMeasurement& m )
{
	
	// check if image was already analysed
	if ( m.time() == m_lastTimestamp )
		return;
	m_lastTimestamp = m.time();

	// get all components, needed quite often
	ComponentList components = getAllComponents();
	ComponentList::const_iterator it = components.begin();

	// debug image (if anybody is interested )
	boost::shared_ptr< Image > pDebugImg;
	{
		// check if debug image needs to be created
		for ( it = components.begin(); it != components.end(); it++ )
			if ( (*it)->debug() )
			{
				//pDebugImg = m->CvtColor( CV_GRAY2RGB, 3 );
				if (!pDebugImg){
					pDebugImg.reset(new Vision::Image(m->width(), m->height(), 3));
					cv::cvtColor(m->Mat(), pDebugImg->Mat(), CV_GRAY2RGB);
					break;
				}
			}
	}	

	// fetch the intrinsic matrix from one of the components 
	Math::Matrix< float, 3, 3 > K = getIntrinsicsMatrix(m);	

	// create marker map
	MarkerInfoMap markerMap;
	MarkerInfoMap refineMarkerMap;
	
	// copy all necessary information to a marker map
	BOOST_FOREACH( ComponentList::value_type pComp, components )
	{
		//init for markertracker
		
		if( pComp->getKey().isMarker ){
			boost::shared_ptr<MarkerTracker> lMTrackerComp = boost::static_pointer_cast<MarkerTracker>(pComp);
			lMTrackerComp->m_info.found = MarkerInfo::ENotFound;
		
			if ( lMTrackerComp->m_info.nPrevPoseValidator < 3 || !lMTrackerComp->m_info.bEnableTracking || !lMTrackerComp->m_info.bEnableFastTracking )
			{	
				// set refinement level
				if ( lMTrackerComp->m_outPort.isConnected() || lMTrackerComp->m_outErrorPose.isConnected() || pDebugImg )
					if ( lMTrackerComp->useEdgeRefinement() )
						lMTrackerComp->m_info.refinement = MarkerInfo::EEdgeRefinedPose;
					else
						lMTrackerComp->m_info.refinement = MarkerInfo::ERefinedPose;
				else
					lMTrackerComp->m_info.refinement = MarkerInfo::ECorners;

				lMTrackerComp->m_info.bUseInitialPose = m.time() < lMTrackerComp->m_lastTime + 250000000;

				// covariance?
				lMTrackerComp->m_info.bCalculateCovariance = lMTrackerComp->m_outErrorPose.isConnected();
			
				// copy to full scan marker map
				markerMap[ pComp->getKey().nCode ] = lMTrackerComp->m_info;
			}
			else
				// copy to refinement map
				refineMarkerMap[ pComp->getKey().nCode ] = lMTrackerComp->m_info;
		}
		//init for multimarkertracker
		else if ( pComp->getKey().isMultiMarker ){
			// examine the image and store all found markers into the marker map
			boost::shared_ptr<MultiMarkerTracker> lMMTrackerComp = boost::static_pointer_cast<MultiMarkerTracker>(pComp);
			BOOST_FOREACH( MultiMarkerInfoMap::value_type& mapEl1, lMMTrackerComp->markerMap )
			{
				// set refinement level to EdgeRefinedPose (highest level)
				BOOST_FOREACH( MarkerInfoMap::value_type& refEl2, mapEl1.second )
				{
					refEl2.second.refinement = MarkerInfo::EEdgeRefinedPose;
					markerMap[ refEl2.first ] = refEl2.second;
				}
			}
		}
	}

	// try to refine markers with fast tracking enabled
	if ( !refineMarkerMap.empty() )
	{
		{
			#ifdef DO_TIMING
			UBITRACK_TIME( m_detectMarkersTimerRefine );
			#endif
			detectMarkers( *m, refineMarkerMap, K, pDebugImg.get(), true, m_codeSize, m_markerSize, m_codeMask, m_useInnerEdgels, m_enableAdaptiveThreshold, m_binaryThresholdValue );
		}
	
		// copy not-found markers to marker map and update others
		BOOST_FOREACH( MarkerInfoMap::value_type& mapEl, refineMarkerMap )
		{			
			// update found markers
			if ( mapEl.second.found != MarkerInfo::ENotFound && hasComponent( mapEl.first ) ){
				boost::shared_ptr<MarkerTracker> lMarkerComp = boost::static_pointer_cast<MarkerTracker>(getComponent( mapEl.first ));
				lMarkerComp->m_info = mapEl.second;
			}
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
			detectMarkers( *m, markerMap, K, pDebugImg.get(), false, m_codeSize, m_markerSize, m_codeMask, m_useInnerEdgels,
			 m_enableAdaptiveThreshold, m_binaryThresholdValue );
		}
		// update information of found markers and move not-found not-fast-tracking markers to refinement
		BOOST_FOREACH( MarkerInfoMap::value_type& mapEl, markerMap )
		{			
			if ( mapEl.second.found == MarkerInfo::EFullScanFound && hasComponent( mapEl.first ) ){
				boost::shared_ptr<MarkerTracker> lMarkerComp = boost::static_pointer_cast<MarkerTracker>(getComponent( mapEl.first ));
				lMarkerComp->m_info = mapEl.second;
			}else if ( mapEl.second.bEnableTracking && !mapEl.second.bEnableFastTracking && 
				mapEl.second.nPrevPoseValidator >= 3 && hasComponent( mapEl.first ) )
				refineMarkerMap[ mapEl.first ] = mapEl.second;
		}
	}
	// try to refine not-fast-tracking markers not found by full scan 
	if ( !refineMarkerMap.empty() )
	{
		{
			#ifdef DO_TIMING
			UBITRACK_TIME( m_detectMarkersTimerRefine );
			#endif
			detectMarkers( *m, refineMarkerMap, K, pDebugImg.get(), true, m_codeSize, m_markerSize, m_codeMask, m_useInnerEdgels, m_enableAdaptiveThreshold, m_binaryThresholdValue );
		}
	
		// update found markers
		BOOST_FOREACH( MarkerInfoMap::value_type& mapEl, refineMarkerMap )
			// update found markers
			if ( mapEl.second.found != MarkerInfo::ENotFound && hasComponent( mapEl.first) ){
				boost::shared_ptr<MarkerTracker> lMarkerComp = boost::static_pointer_cast<MarkerTracker>(getComponent( mapEl.first ));
				lMarkerComp->m_info = mapEl.second;
				
			}
	}

	// check which markers were found
	BOOST_FOREACH( ComponentList::value_type pComp, components )
	{
		if( pComp->getKey().isMultiMarker ){
			//forward the found markers to the multimarkertracker
			boost::shared_ptr<MultiMarkerTracker> lMMTrackerComp = boost::static_pointer_cast<MultiMarkerTracker>(pComp);
			lMMTrackerComp->foundMarkers(markerMap, K, m.time());

		}else if( pComp->getKey().isMarker ){
			
			//send the tracked markerpose to components connected to the marker tracker components

			boost::shared_ptr<MarkerTracker> lMTrackerComp = boost::static_pointer_cast<MarkerTracker>(pComp);
			MarkerInfo& info( lMTrackerComp->m_info );
			
		
			// timeout pixelflow after 5 seconds. TODO: make configureable
			if ( info.found == MarkerInfo::EPixelFlowFound && lMTrackerComp->m_lastTime + 5000000000LL < m.time() )
			{
				info.found = MarkerInfo::ENotFound;
				info.nPrevPoseValidator = 0;
			}
			
			// update lastTime only when found by refinement or full scan
			if ( info.found >= MarkerInfo::ERefinementFound )
				lMTrackerComp->m_lastTime = m.time();
		
			if ( info.found >= MarkerInfo::ERefinementFound || ( info.found == MarkerInfo::EPixelFlowFound && info.bEnablePixelFlow ) )
			{
		
				LOG4CPP_DEBUG( logger, "Marker " << std::hex << pComp->getKey() << " sending stuff ");
				
				// send corners
				if ( lMTrackerComp->m_outCorners.isConnected() )
				{
					LOG4CPP_DEBUG( logger, "Marker " << std::hex << pComp->getKey() << ": sending corners [" 
						<< info.corners[ 0 ] << ", " << info.corners[ 1 ] << ", " << info.corners[ 2 ] << ", " << info.corners[ 3 ] << "]" );

					// convert from float to double
					boost::shared_ptr< std::vector< Math::Vector< double, 2 > > > pCorners( new std::vector< Math::Vector< double, 2 > > );
					for ( unsigned i = 0; i < 4; i++ )
						pCorners->push_back( Math::Vector< double, 2 >( info.corners[ i ] ) );

					lMTrackerComp->m_outCorners.send( Measurement::PositionList2( m.time(), pCorners ) );
				}

				// send to component
				try
				{
					lMTrackerComp->m_outPort.send( Measurement::Pose( m.time(), info.pose ) );
				}
				catch ( ... )
				{}

				// also send ErrorPose if anybody is connected
				if ( info.bCalculateCovariance )
					lMTrackerComp->m_outErrorPose.send( Measurement::ErrorPose( m.time(), ErrorPose( info.pose, info.covariance ) ) );
			}
		}
	}

	// push debug image
	if ( pDebugImg )
	{
		for ( it = components.begin(); it != components.end(); it++ )
			if ( (*it)->debug() )
			{
				boost::shared_ptr<MarkerTracker> lMTrackerComp = boost::static_pointer_cast<MarkerTracker>(*it);
				lMTrackerComp->m_debugPort.send( Measurement::ImageMeasurement( m.time(), pDebugImg ) );
			}
			
		}
}



boost::shared_ptr< MarkerTrackerBase > MarkerTrackerModule::createComponent( const std::string& type, const std::string& name,
		boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const ComponentKey& key, ModuleClass* pModule )
{
	
	LOG4CPP_INFO( logger, "Class " + type + " created by Marker Tracker Module " );
	if ( type == "MarkerTracker" )
		return boost::shared_ptr< MarkerTrackerBase >(new MarkerTracker( name, subgraph, key, pModule ));
	else if ( type == "MultiMarkerTracker" )
		return boost::shared_ptr< MarkerTrackerBase >(new MultiMarkerTracker( name, subgraph, key, pModule ));
	else if ( type == "MultiMarkerTrackerBundleAdjustment" )
		return boost::shared_ptr< MarkerTrackerBase >(new MultiMarkerTrackerBundleAdustment( name, subgraph, key, pModule ));

	UBITRACK_THROW( "Class " + type + " not supported by Marker Tracker Module " );

}

MarkerTracker::MarkerTracker( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const IdKey& componentKey, MarkerTrackerModule* pModule )
	: MarkerTrackerBase( sName, subgraph, componentKey, pModule )
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
	if( subgraph->hasNode( "Marker" ) )
		subgraph->getNode( "Marker" )->getAttributeData( "markerSize", m_info.fSize );
	else
	{
		//LOG4CPP_ERROR( logger, "Cannot start component as a marker tracker, since there is no \"Marker\"-node in the dfg." );
		LOG4CPP_DEBUG( logger, "This is a \"" << getKey() << "\" component and no marker tracker, still no problem if it is the only one." );
	}
		
	if ( subgraph->m_DataflowAttributes.hasAttribute( "edgeRefinement" ) ) // enable Edge Refinement
		m_bEdgeRefinement = subgraph->m_DataflowAttributes.getAttributeString( "edgeRefinement" ) == "true";
		
	if ( subgraph->m_DataflowAttributes.hasAttribute( "enableTracking" ) ) // enable Tracking
		m_info.bEnableTracking = subgraph->m_DataflowAttributes.getAttributeString( "enableTracking" ) == "true";
		
	if ( subgraph->m_DataflowAttributes.hasAttribute( "enablePixelFlow" ) ) // enable Pixel Flow
		m_info.bEnablePixelFlow = subgraph->m_DataflowAttributes.getAttributeString( "enablePixelFlow" ) == "true";
		
	if ( subgraph->m_DataflowAttributes.hasAttribute( "enableFlipCheck" ) ) // enable Flip check 
		m_info.bEnableFlipCheck = subgraph->m_DataflowAttributes.getAttributeString( "enableFlipCheck" ) == "true";
		
	if ( subgraph->m_DataflowAttributes.hasAttribute( "enableFastTracking" ) ) // enable Fast Tracking
		m_info.bEnableFastTracking = subgraph->m_DataflowAttributes.getAttributeString( "enableFastTracking" ) == "true";		
	LOG4CPP_INFO( logger, "MarkerTracker is IntrinsicsEdge: " << subgraph->hasEdge( "CameraIntrinsics" ) << "isIntrinsics: " <<isIntrinsics() );
	LOG4CPP_INFO( logger, "MarkerTracker configuration: edgebased refinement: " << m_bEdgeRefinement << " enableTracking: " << m_info.bEnableTracking << " enablePixelFlow: " << m_info.bEnablePixelFlow << " enableFlipCheck " << m_info.bEnableFlipCheck << " bEnableFastTracking " << m_info.bEnableFastTracking );
}

bool MarkerTracker::debug()
{ 
	return m_debugPort.isConnected(); 
}

bool MarkerTracker::isIntrinsics()
{ 
	return m_inIntrinsics.isConnected(); 
}
Measurement::Matrix3x3 MarkerTracker::intrinsics( Measurement::Timestamp t )
{ 
	return m_inIntrinsics.get( t ); 
}

bool MarkerTracker::useEdgeRefinement() const
{ 
	return m_bEdgeRefinement; 
}
void MarkerTracker::pushImage( const Measurement::ImageMeasurement& m )
	{ getModule().trackMarkers( m ); }

//MultiMarkerTracker
MultiMarkerTracker::MultiMarkerTracker( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const IdKey& componentKey, MarkerTrackerModule* pModule )
		: MarkerTrackerBase( sName, subgraph, componentKey, pModule )
		, m_outErrorPose( "ErrorPose", *this )
//		, m_worldMapPort( "Output3DCorners", *this, boost::bind( &MultiMarkerTracker::getWorldMap, this, _1 ) )
		, m_useInnerEdgels( true )
	{
	
		LOG4CPP_INFO( logger, "info" );
		// get path to and read all external files
		m_markerConfig = subgraph->getNode( "MultiMarker" )->getAttributeString( "MarkerConfig" );
		LOG4CPP_INFO( logger, "info" );
		// default marker mask, if marker information was not found in the MarkerTrackerModule
		MarkerDetectionConfig config;
		config.priority = 0;
		config.defaultSize = 0.06;
		config.codeMask = 0xFFFF;
		config.codeSize = 4;
		config.markerSize = 6;
		if( subgraph->hasNode( "Camera" ) ){
			LOG4CPP_INFO(logger, "using markerdetection configuration of markertrackermodule");

			if (subgraph->getNode( "Camera" )->hasAttribute( "markerIdMask" )){
				std::string sMask = subgraph->getNode( "Camera" )->getAttributeString( "markerIdMask" );
				if ( ! sMask.empty() )
				{
					std::istringstream idStream( sMask );
					idStream >> std::hex >> config.codeMask;
				}
			}
			if (subgraph->getNode( "Camera" )->hasAttribute( "markerBitSize" ))
				subgraph->getNode( "Camera" )->getAttributeData( "markerBitSize", config.markerSize );
			
			if (subgraph->getNode( "Camera" )->hasAttribute( "codeBitSize" ))
				subgraph->getNode( "Camera" )->getAttributeData( "codeBitSize", config.codeSize );
			
		}else{
			LOG4CPP_INFO(logger, "using default markerdetection configuration");
		}
		LOG4CPP_INFO( logger, "info" );
		/*
		if ( subgraph->hasEdge( "Output3DCorners" ) ){
			m_worldMapPort = boost::shared_ptr< Dataflow::PullSupplier< Measurement::PositionList > >
			( new Dataflow::PullSupplier< Measurement::PositionList >( "Output3DCorners", *this, 
			boost::bind( &MultiMarkerTracker::getWorldMap, this, _1 ) ) );
		}*/
		readConfigFile(config);
		LOG4CPP_INFO( logger, "info" );
		readWorldFiles();
		if ( subgraph->m_DataflowAttributes.hasAttribute( "enableInnerEdgels" ) ) // enable inner edge list
			m_useInnerEdgels = subgraph->m_DataflowAttributes.getAttributeString( "enableInnerEdgels" ) == "true";
		LOG4CPP_INFO( logger, "info" );

	}

void MultiMarkerTracker::readConfigFile(MarkerDetectionConfig& config)
{

	// configurate the boost::regex string for the line recognition of the boost::smatch
	// (read and compare the content of each line, it has to match completely)
	std::string sComment( "\\s*(#.*)?" );
	
	boost::regex reMarkerConfig( "\\s*markerconfig\\s+([0-9]+)\\s+([0-9]+)\\s+(0x)?([0-9a-fA-F]+)\\s+([.0-9e]+)\\s+([0-9]+)" + sComment );
	boost::regex reMarker( "\\s*marker\\s+(0x)?([0-9a-fA-F]+)\\s+([.0-9e]+)" + sComment );

	// different pathes for win and unix
	#ifdef WIN32
	boost::regex reMarkerDir("\\s*markerdir\\s+(([a-zA-Z]:)(/.*)+)" + sComment);	
	#else
	boost::regex reMarkerDir( "\\s*markerdir\\s+((/.*)+)" + sComment );
	#endif

	boost::regex reEmpty( sComment );

	std::ifstream f( m_markerConfig.c_str() );

	boost::filesystem::path p(m_markerConfig);
	boost::filesystem::path dir = p.parent_path();
	m_markerDirectory = dir.string();
	
	
	// unable to read the file
	if (f.fail()){
		LOG4CPP_INFO(logger, "file not found for: " << m_markerConfig);
		throw std::runtime_error("unable to open markerbundle.conf");
	
	}

	// compare each line
	while ( !f.fail() && !f.eof() )
	{
		std::string buf;
		std::getline( f, buf );

		boost::smatch match;		
		
		// do nothing, if an empty line was found
		if ( boost::regex_match( buf, match, reEmpty ) )
			;

		// compare the marker information and store it into the marker map
		// init all flags for the marker tracker
		else if ( boost::regex_match( buf, match, reMarker ) )
		{	
			std::stringstream ss(match[2].str());
			unsigned long long int code;
			ss >> std::hex >> code;
			
			double size = boost::lexical_cast<double>(match[ 3 ]);
			MarkerInfo info;
			info.bCalculateCovariance = true;
			info.bEnableFlipCheck = true;
			info.fSize = float(size);
			
			markerMap[config][code] = info;			
			
			LOG4CPP_INFO(logger, "Looking for marker " << std::hex << code << " of size " << size );
		}

		//multimarkertracker was reduced to tracking the markerconfiguration configured in the markertracker module
		//so all markerconfigs in the MultimarkerConfig file are ignored
		else if ( boost::regex_match( buf, match, reMarkerConfig ) )
		{	
			
			/*
			// get the marker mask and configuration and store it into the marker map
			std::stringstream ss(match[4].str());
			unsigned long long int codeMask;
			ss >> std::hex >> codeMask;
			//just ignore the MarkerConfigurations in file and let the MarkerTrackerModule
			//set those fields by its global value
			
			double dSize = boost::lexical_cast<double>(match[5]);

			config.codeSize = boost::lexical_cast<unsigned long>(match[ 1 ]);
			config.markerSize = boost::lexical_cast<unsigned long>(match[ 2 ]);			
			config.codeMask =  codeMask;
			config.defaultSize = float(dSize);
			config.priority = boost::lexical_cast<unsigned long>(match[ 6 ]);
			*/
			//LOG4CPP_INFO(logger, "Using marker config CodeSize:" << config.codeSize << " MarkerSize: " << config.markerSize << " CodeMask: " << std::hex << config.codeMask << " DefaultSize: " << config.defaultSize << " Priority: " << config.priority);
			
			markerMap[config] = MarkerInfoMap();		
		}

		// get the directory, where the calibration files of the marker can be found and will be writen later on
		else if ( boost::regex_match( buf, match, reMarkerDir ) )
		{
			LOG4CPP_INFO(logger, "calib file dir:" << match[1].str());
			m_markerDirectory =  match[ 1 ].str();
		}

		// there was no match between the string of the line and the defined string in the config file
		else
			LOG4CPP_ERROR(logger, "unknown configuration string: " << buf );
	}

	// store the init marker information in m_writeConfig for later use with the config file
	// because already known marker (previous tracking session) would be deleted from the config file, if they are not seen in any images (actual tracking session)
	std::map<unsigned long long int, float> tmpMarker;
	m_writeConfig.clear();
	BOOST_FOREACH( MultiMarkerInfoMap::value_type& mapEl, markerMap )
	{
		BOOST_FOREACH( MarkerInfoMap::value_type& mapEl2, mapEl.second )
		{
			tmpMarker.insert( std::make_pair(mapEl2.first,mapEl2.second.fSize) );
		}

		m_writeConfig.insert( std::make_pair(mapEl.first,tmpMarker) );
		tmpMarker.clear();
	}
}


void MultiMarkerTracker::readWorldFiles(){

	// read the calibration files of the markers and put the information into a marker map
	BOOST_FOREACH( MultiMarkerInfoMap::value_type& mapEl, markerMap )
	{
		BOOST_FOREACH( MarkerInfoMap::value_type& mapEl2, mapEl.second )
		{
			// get the path name of the file
			std::ostringstream cornerFileName(std::ostringstream::out);
			cornerFileName << m_markerDirectory << "/marker" << std::hex << mapEl2.first << ".cal";
			LOG4CPP_INFO(logger, "Reading Positions for Marker:" << std::hex << mapEl2.first);

			// if file not found, gives error message
			if ( !boost::filesystem::exists( cornerFileName.str() ) )
			{
				LOG4CPP_INFO(logger, "Can't find my file:" << cornerFileName.str());
				continue;
			}

			Measurement::ErrorPose errorPose;

			
			// read the file and store the error pose
			
			Util::readCalibFile(cornerFileName.str(), errorPose);
			
			
			Pose tmpPose = Pose( errorPose->rotation(), errorPose->translation() );
			std::vector<Math::Vector< double, 3 > > corners;

			// calculate all four corner poses based on the marker pose and size (3D position list)
			for ( unsigned i = 0; i < 4; i++ ){
				Math::Vector< double, 3 > p = tmpPose * Math::Vector< double, 3 >( g_unitCorners[ i ] * mapEl2.second.fSize );
				corners.push_back( p );
			}

			// store the 3D position list of all four corners into the world map
			Measurement::PositionList tmpCorners( corners );
			worldMap[mapEl2.first] = *tmpCorners;

			// store the error pose list into the error map
			errorMap[mapEl2.first] = *errorPose;

			// define 2D corner values, all corners (0,0)
			// needed because of checkMarkerOverlap, which would crash, if no 2d corner values are available to check
			for( int i = 0; i < 4; i++ ){
				mapEl2.second.corners.push_back( Math::Vector< double, 2 >(0,0) );
			}
			
		}		
	}
}


void MultiMarkerTracker::foundMarkers(MarkerInfoMap& foundMarkerMap, const Math::Matrix< double, 3, 3 >& cam, const Measurement::Timestamp& t)
{
	int detectedMarkers = 0;
	// copy the results of the image analysis and get the number of marker, which has been found
	BOOST_FOREACH( MultiMarkerInfoMap::value_type& mapEl, markerMap )
	{
		BOOST_FOREACH( MarkerInfoMap::value_type& mapEl2, mapEl.second )
		{
			mapEl2.second = foundMarkerMap[mapEl2.first];
			if( mapEl2.second.found ){
				detectedMarkers++;
			}
		}
	}
	
	// create the data structure (vector) and reserve memory for the 2D and 3D position list
	std::vector< Math::Vector< double, 2 > > p2d;
	p2d.reserve(detectedMarkers * 4);
	std::vector< Math::Vector< double, 3 > > p3d;
	p3d.reserve(detectedMarkers * 4);

	
	// for each found marker in the marker map, store the corner values into a 2D position list
	// and get the poses from the world map, which was created in the doBundleAdjustment function
	// the reason is to get only the markers, which are known form the bundle
	BOOST_FOREACH( MultiMarkerInfoMap::value_type& mapEl, markerMap )
	{
		BOOST_FOREACH( MarkerInfoMap::value_type& mapEl2, mapEl.second )
		{
			if( mapEl2.second.found ){
				MultiMarker3DMap::iterator it = worldMap.find(mapEl2.first);
				if(it != worldMap.end()){
					
					p2d.insert( p2d.end(), mapEl2.second.corners.begin(), mapEl2.second.corners.end() );
					p3d.insert( p3d.end(), worldMap[mapEl2.first].begin() , worldMap[mapEl2.first].end() );				
				}
			}
		}		
	}
	
	// if the size of the position list is not correct, return the function
	if ( p2d.size() < 4 || p2d.size() != p3d.size()) {
		return;
	}
	
	// send the error pose of the marker
	Math::ErrorPose errPose = Algorithm::PoseEstimation2D3D::computePose( p2d, p3d, cam, true, Algorithm::PoseEstimation2D3D::PLANAR_HOMOGRAPHY );
	if(m_outErrorPose.isConnected()){
		m_outErrorPose.send( Measurement::ErrorPose( t, errPose ) );
	}
}

Measurement::PositionList MultiMarkerTracker::getWorldMap(Measurement::Timestamp t)
{
	// create a temporal data structure to store the world map information (vector for the pose)
	boost::shared_ptr< std::vector< Math::Vector< double, 3 > > > p3d(new std::vector< Math::Vector< double, 3 > >());	
	p3d->reserve(worldMap.size()*4);

	// transform the map information into a vector to get a 3D position list
	BOOST_FOREACH( MultiMarker3DMap::value_type& mapEl, worldMap ){
		p3d->insert( p3d->end(), mapEl.second.begin() , mapEl.second.end() );
	}

	// gives a position list back
	return Measurement::PositionList(t, p3d);
}


MultiMarkerTrackerBundleAdustment::MultiMarkerTrackerBundleAdustment( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const IdKey& componentKey, MarkerTrackerModule* pModule )
	: MultiMarkerTracker( sName, subgraph , componentKey, pModule )
	, m_findMarkers( false )
	, m_bEnableConsoleOutput(true)
	, m_State( state_stopped )
	, m_newMarker( false )
	, m_imageCount( 0 )
	, m_calibration( false )
	, m_tiltAngle( 70.0 )
	, m_minPixel( 60 )
	, m_maxImage( 5 )
{
	LOG4CPP_INFO(logger, "constructor");
	// get all attributes from the edges of the data flow graph
	if ( subgraph->m_DataflowAttributes.hasAttribute( "enableFlipCheck" ) ) // enable Flip check 
		m_bEnableFlipCheck = subgraph->m_DataflowAttributes.getAttributeString( "enableFlipCheck" ) == "true";		
	if ( subgraph->m_DataflowAttributes.hasAttribute( "enableFindMarkers" ) ) // enable find markers
		m_findMarkers = subgraph->m_DataflowAttributes.getAttributeString( "enableFindMarkers" ) == "true";		
	if ( subgraph->m_DataflowAttributes.hasAttribute( "enableConsoleOutput" ) ) // enable console output
		m_bEnableConsoleOutput = subgraph->m_DataflowAttributes.getAttributeString( "enableConsoleOutput" ) == "true";
	if ( subgraph->m_DataflowAttributes.hasAttribute( "tiltAngle" ) ) // get tilt angle
		subgraph->m_DataflowAttributes.getAttributeData( "tiltAngle", m_tiltAngle );
	if ( subgraph->m_DataflowAttributes.hasAttribute( "minPixel" ) ) // get min number of pixel
		subgraph->m_DataflowAttributes.getAttributeData( "minPixel", m_minPixel );
	if ( subgraph->m_DataflowAttributes.hasAttribute( "maxImage" ) ) // get max number of images
		subgraph->m_DataflowAttributes.getAttributeData( "maxImage", m_maxImage );
	// get all input edges of the camera bundle (edges do not need to be connected for the tracker, there can be any amount of edges)
	for ( Graph::UTQLSubgraph::EdgeMap::iterator it = subgraph->m_Edges.begin(); it != subgraph->m_Edges.end(); it++ ){
		if ( it->second->isInput() ){
			if ( 0 == it->first.compare( 0, 11, "ImageBundle" ) ){
				m_inBundleCamPort = boost::shared_ptr< Dataflow::PushConsumer< Measurement::ImageMeasurement > >
					( new Dataflow::PushConsumer< Measurement::ImageMeasurement >( it->first, *this, boost::bind( &MultiMarkerTrackerBundleAdustment::getImage, this, _1 ) ) );
			} else if( 0 == it->first.compare( 0, 22, "BundleCameraIntrinsics" ) ){
				m_inBundleCamIntrinsics = boost::shared_ptr< Dataflow::PullConsumer< Measurement::Matrix3x3 > >
					( new Dataflow::PullConsumer< Measurement::Matrix3x3 >( it->first, *this ) );
			} else if( 0 == it->first.compare( 0, 11, "ResetButton" ) ){
				m_resetPort = boost::shared_ptr< Dataflow::PushConsumer< Measurement::Button > >
					( new Dataflow::PushConsumer< Measurement::Button >( it->first, *this, boost::bind( &MultiMarkerTrackerBundleAdustment::resetMarkerBundle, this, _1 ) ) );
			}
		}
	}

	// create a new thread
	m_pThread = boost::shared_ptr< boost::thread >( new boost::thread( boost::bind( &MultiMarkerTrackerBundleAdustment::threadFunction, this ) ) );

	// init all global vectors and maps
	m_markerID.clear();
	m_tempIQ.clear();
	m_tempMarkerBundle.clear();
	m_separatedMarkers.clear();
}

// terminate the parallel process of the marker bundle, after the tracking has been closed
MultiMarkerTrackerBundleAdustment::~MultiMarkerTrackerBundleAdustment(){
	LOG4CPP_INFO( logger, "Destroying BundleAdjustment" );
	// tell thread to quit
	{
		boost::mutex::scoped_lock l( m_Mutex );
		m_State = state_end;
		m_NewEventCondition.notify_all();
	}

	// wait until thread has actually quit
	m_pThread->join();
	LOG4CPP_INFO( logger, "Destroyed BundleAdjustment" );
}


void MultiMarkerTrackerBundleAdustment::start(){
	Component::start();

	// init the marker bundle
	if(m_markerBundle.get() == 0 && m_inBundleCamIntrinsics.get() != 0){
		const Math::Matrix< double, 3, 3 > cam = *m_inBundleCamIntrinsics->get(Measurement::now());
		Math::Matrix< float, 3, 3 > intrinsics;
		Math::Util::matrix_cast_assign( intrinsics, cam );
		const Math::Vector4d distor = Math::Vector4d(0,0,0,0);
		m_markerBundle.reset(new BAInfo(intrinsics, distor));
	}

	LOG4CPP_NOTICE( logger, "Bundle adjustment started" );

	// tell thread to start
	boost::mutex::scoped_lock l( m_Mutex );
	m_State = state_running;
	m_NewEventCondition.notify_all();
}

void MultiMarkerTrackerBundleAdustment::stop(){
	LOG4CPP_DEBUG( logger, "Stopping bundle adjustment" );

	boost::mutex::scoped_lock l( m_Mutex );
	if ( m_State != state_stopped )
	{
		// tell thread to stop
		m_State = state_stopping;
		m_NewEventCondition.notify_all();

		// wait until thread has actually stopped
		while ( m_State != state_stopped )
			m_NewEventCondition.wait( l );
	}
	
	LOG4CPP_NOTICE( logger, "Bundle adjustment stopped" );
}
	
void MultiMarkerTrackerBundleAdustment::threadFunction()
{
	
	while ( true )
	{

		// get the current image from the queue
		Measurement::ImageMeasurement currentImage;		
		{
			// lock the mutex
			boost::mutex::scoped_lock l( m_Mutex );

			if ( m_State == state_running && m_tempIQ.size() > 0 )
			{
				// get image and delete it from the queue
				currentImage = m_tempIQ.back();
				m_tempIQ.pop_back();
			}
			else if ( m_State == state_end )
			{
				LOG4CPP_DEBUG( logger, "Ending bundle adjustment thread");
				// end this thread
				return;
			}
			else if ( m_State == state_stopping )
			{
				LOG4CPP_DEBUG( logger, "Ending other bundle adjustment threads");
				// stop and wait for something to happen
				m_State = state_stopped;

				// tell other threads we have stopped
				m_NewEventCondition.notify_all();
			}
			else
			{				
				// wait for something to happen
				m_NewEventCondition.wait( l );
			}
		}

		// do bundle adjustment if images were taken from the queue
		if ( currentImage )
		{
			try
			{
				// start the image queue function if a new image was received
				imageQueue(currentImage,m_maxImage);
			}
			catch ( const Ubitrack::Util::Exception& e )
			{
				LOG4CPP_WARN( logger, e );
			}
			catch ( const std::exception& e )
			{
				LOG4CPP_WARN( logger, "Caught std::exception: " << e.what() << " occurred in BundleAdjustment " );
			}
			catch ( ... )
			{
				LOG4CPP_WARN( logger, "Caught unknown exception in BundleAdjustment" );
			}
		}
	}
}

void MultiMarkerTrackerBundleAdustment::imageQueue(const Measurement::ImageMeasurement& image, int maxImage) {
	LOG4CPP_INFO(logger, "imageQueue");
	try
	{
		// use detectMarkers to get the IDs of all Markers in the image
		const Math::Matrix< double, 3, 3 > cam = getModule().getIntrinsicsMatrix(image);
		

		// test if the image is a grayscale image
		if( image->channels() != 1 ){
			UBITRACK_THROW( "The MarkerTracker requires a grayscale image" );
		}

		// temp parameter for findMarkers
		MarkerInfo info;
		info.bCalculateCovariance = true;
		info.bEnableFlipCheck = true;

		// use detectMarkers to find possible markers in the image and store the imformation in the marker map
		BOOST_FOREACH( MultiMarkerInfoMap::value_type& mapEl, markerMap )
		{

			// set the refinement level to EdgeRefinedPose (highest level)
			BOOST_FOREACH( MarkerInfoMap::value_type& refEl, mapEl.second )
			{
				refEl.second.refinement = MarkerInfo::EEdgeRefinedPose;
			}

			// two different modes are possible:
			// findMarkers tries to find unknown markers in the image and put them together for the marker bundle
			// the normal mode searchs the image for all markers, which were defined in the config file, but can not find new marker
			if(m_findMarkers) {

				info.fSize = mapEl.first.defaultSize;

				// temp map for detectMarkers
				std::map<unsigned long long,MarkerInfo> fmMap;
				fmMap[0] = info;

				detectMarkers(*image, fmMap, cam, 0, false, mapEl.first.codeSize, mapEl.first.markerSize, mapEl.first.codeMask, m_useInnerEdgels); 

				// all new found markers will be copied into the markerMap
				BOOST_FOREACH( MarkerInfoMap::value_type& mapEl2, fmMap ){
					if( mapEl2.first != 0 )
						mapEl.second[mapEl2.first] = mapEl2.second;
				}
			} else {
				LOG4CPP_INFO( logger, "imagequeue: detectmarkers" );
				detectMarkers(*image, mapEl.second, cam, 0, false, mapEl.first.codeSize, mapEl.first.markerSize, mapEl.first.codeMask, m_useInnerEdgels); 
			}
		}

		int markerCount = 0;
		std::vector<TempIDConfig> tmpMarkerID;
		WriteMarkerConfig tmpMarkerMask;
		MarkerIDSize tmpMarkerSize;
		bool prevMarker = false;
		TempIDConfig tmpID;

		// checks if findMarkers is activated
		if( m_findMarkers ){

			MultiMarkerInfoMap tempMap = markerMap;

			// check all found markers from detectMarker, if certain conditions are met
			// this is a preselection to discard false markers, before the overlapping of markers will be checked (validateMarker)
			BOOST_FOREACH( MultiMarkerInfoMap::value_type& mapEl, tempMap )
			{
				BOOST_FOREACH( MarkerInfoMap::value_type& mapEl2, mapEl.second )
				{
					// only found markers are used
					if( mapEl2.second.found ){

						// test if the marker is known from previous iterations, => do not delete the marker from the markerMap
						prevMarker = false;
						BOOST_FOREACH( WriteMarkerConfig::value_type& mapEl3, m_writeConfig )
						{
							BOOST_FOREACH( MarkerIDSize::value_type& mapEl4, mapEl3.second )
							{
								if( mapEl3.first.codeMask == mapEl.first.codeMask && mapEl3.first.codeSize == mapEl.first.codeSize && mapEl3.first.markerSize == mapEl.first.markerSize && mapEl4.first == mapEl2.first ){
									prevMarker = true;
									break;
								}
							}
							if( prevMarker )
								break;
						}

						// only valid markers are used
						if( !validateMarker(mapEl2.second,mapEl2.first,mapEl.first,image->width(),image->height()) && !prevMarker ){

							// delete false markers from the map
							BOOST_FOREACH( MultiMarkerInfoMap::value_type& el, markerMap )
							{
								if( el.first.codeMask == mapEl.first.codeMask && el.first.codeSize == mapEl.first.codeSize && el.first.markerSize == mapEl.first.markerSize )
									el.second.erase( mapEl2.first );
							}
						}
					}
				}
			}

			tempMap = markerMap;
			
			// additionally check for overlapping markers and discard the final false marker
			BOOST_FOREACH( MultiMarkerInfoMap::value_type& mapEl, tempMap )
			{
				BOOST_FOREACH( MarkerInfoMap::value_type& mapEl2, mapEl.second )
				{
					// only found markers are used
					if( mapEl2.second.found ){

						// only valid markers are used
						if( checkMarkerOverlap(mapEl2.second,mapEl2.first,mapEl.first,markerMap) ){

							// count the number of markers in the image
							markerCount++;

							// save the marker IDs in a vector
							tmpID.ID = mapEl2.first;
							tmpID.codeMask = mapEl.first.codeMask;
							tmpID.codeSize = mapEl.first.codeSize;
							tmpID.markerSize = mapEl.first.markerSize;
							tmpMarkerID.push_back(tmpID);

							// save the marker and the size
							tmpMarkerSize.insert(std::make_pair(mapEl2.first,mapEl2.second.fSize));

						} else {

							// delete false markers from the map
							BOOST_FOREACH( MultiMarkerInfoMap::value_type& el, markerMap )
							{
								if( el.first.codeMask == mapEl.first.codeMask && el.first.codeSize == mapEl.first.codeSize && el.first.markerSize == mapEl.first.markerSize )
									el.second.erase( mapEl2.first );
							}
						}
					}
				}

				// temp map for writeConfigFile (see few rows below)
				tmpMarkerMask.insert(std::make_pair(mapEl.first,tmpMarkerSize));
				tmpMarkerSize.clear();
			}

		// findMarkers is deactivated => no false marker possible, therefore no check for valid markers needed
		} else {
			LOG4CPP_INFO( logger, "imagequeue: get some marker informations for later use" );
			// get some marker informations for later use
			BOOST_FOREACH( MultiMarkerInfoMap::value_type& mapEl, markerMap )
			{
				BOOST_FOREACH( MarkerInfoMap::value_type& mapEl2, mapEl.second )
				{
					// only found markers are used
					if( mapEl2.second.found ){

						// count the number of markers in the image
						markerCount++;

						// save the marker IDs in a vector
						tmpID.ID = mapEl2.first;
						tmpID.codeMask = mapEl.first.codeMask;
						tmpID.codeSize = mapEl.first.codeSize;
						tmpID.markerSize = mapEl.first.markerSize;
						tmpMarkerID.push_back(tmpID);
					}
				}
			}
		}

		// write to config if findMarkers is activated
		if( m_findMarkers ){

			// stores the marker info of current image in global parameter for writeConfigFile
			// needed because writeConfigFile would only write the marker from the last image without a global parameter
			if( m_writeConfig.empty() ){
				m_writeConfig.insert(tmpMarkerMask.begin(), tmpMarkerMask.end());
			} else {
				BOOST_FOREACH( WriteMarkerConfig::value_type& mapEl, m_writeConfig )
				{
					mapEl.second.insert(tmpMarkerMask.at(mapEl.first).begin(), tmpMarkerMask.at(mapEl.first).end());
				}
			}

			// write all found markers to config
			writeConfigFile();
		}

		LOG4CPP_INFO( logger, "imagequeue: imagecount>2" );
		// the image will only be stored in the image queue, if at least two markers can be seen
		if( markerCount >= 2 ){

			// count the number of images for the next marker bundle
			m_imageCount++;

			// store the amount of new markers
			int newMarkerCount = 0;
			
			// store the amount of known separated markers
			int separatedMarkerCount = 0;

			// if the image is stored in the image queue, then increase the counter of the markerID map
			// if the marker ID is new, then put in a new pair of key and value with the count of one
			BOOST_FOREACH( TempIDConfig vecEl, tmpMarkerID )
			{
				// no new marker
				MarkerIDCount::iterator it = m_markerID.find(vecEl);
				if( it != m_markerID.end() ){
					m_markerID[vecEl]++;

					// count all known separated markers in the image
					BOOST_FOREACH( SeparatedMarkers::value_type& mapEl, m_separatedMarkers )
					{
						if( mapEl.first.ID == vecEl.ID && mapEl.first.codeMask == vecEl.codeMask && mapEl.first.codeSize == vecEl.codeSize && mapEl.first.markerSize == vecEl.markerSize )
							separatedMarkerCount++;
					}

				// new marker
				} else {
					m_markerID.insert(std::make_pair(vecEl,1));
					m_newMarker = true;

					// count all new markers in the image
					newMarkerCount++;
				} 
			}

			// no separation possible in first image
			if( !m_tempMarkerBundle.empty() ){

				// if the amount of new and known separated markers is the same as all markers in the image, than a separation of markers has occured
				// all markers, which can be seen in the image, are not from the previous iteration (start point is the first image, which are never separated)
				if( newMarkerCount + separatedMarkerCount == markerCount ){

					// insert all separated markers
					// each marker have a list of markers, which is known to him
					BOOST_FOREACH( TempIDConfig vecEl, tmpMarkerID )
					{
						SeparatedMarkers::iterator it = m_separatedMarkers.find( vecEl );
						if( it == m_separatedMarkers.end() )
							m_separatedMarkers.insert( std::make_pair( vecEl,std::set<TempIDConfig>() ) );

						m_separatedMarkers[vecEl].insert( tmpMarkerID.begin(), tmpMarkerID.end() );
						m_separatedMarkers[vecEl].erase( vecEl );
					}

					// create the transitive hull by forwarding all marker, which one marker knowns, to all the others
					BOOST_FOREACH( SeparatedMarkers::value_type& mapEl, m_separatedMarkers )
					{
						BOOST_FOREACH( SeparatedMarkers::value_type& mapEl2, m_separatedMarkers )
						{
							if( ( mapEl.first.ID != mapEl2.first.ID || mapEl.first.codeMask != mapEl2.first.codeMask || mapEl.first.codeSize != mapEl2.first.codeSize || 
									mapEl.first.markerSize != mapEl2.first.markerSize ) && mapEl.second.find(mapEl2.first) != mapEl.second.end() )
								mapEl2.second.insert( mapEl.second.begin(), mapEl.second.end() );
						}
					}

				// no separation, check the relation with other known markers and delete the marker from the map
				// (each marker from the image and each marker from their list of knowing marker)
				} else {

					// all marker in the image
					std::set<TempIDConfig> deleteID;
					deleteID.insert( tmpMarkerID.begin(), tmpMarkerID.end() );

					// from the list of knowing marker
					BOOST_FOREACH( TempIDConfig vecEl, tmpMarkerID )
					{
						deleteID.insert( m_separatedMarkers[vecEl].begin(), m_separatedMarkers[vecEl].end() );
					}

					// delete marker, which are no longer separated from the map
					BOOST_FOREACH( TempIDConfig vecEl, deleteID )
					{
						m_separatedMarkers.erase( vecEl );
					}
				}
			}

			// put all needed data for the marker bundle in the temp data structure MarkerBundleConfig
			MarkerBundleConfig tmpConfig;
			tmpConfig.camID = m_tempMarkerBundle.size();
			std::ostringstream tmpImage(std::ostringstream::out);
			tmpImage << image.time();
			tmpConfig.imageName = tmpImage.str();
			tmpConfig.markerMap = markerMap;
			m_tempMarkerBundle.push_back( tmpConfig );

			// write info about found marker to console
			BOOST_FOREACH( MultiMarkerInfoMap::value_type& mapEl, markerMap )
			{
				BOOST_FOREACH( MarkerInfoMap::value_type& mapEl2, mapEl.second )
				{
					if ( mapEl2.second.found != Vision::Markers::MarkerInfo::ENotFound )
					{
						LOG4CPP_INFO(logger, "Found marker " << std::hex << mapEl2.first << " in " << tmpImage.str() << " (camera " << std::dec << tmpConfig.camID << ")" );
					}
				}
			}

			// check if all known markers can be seen at least in two images
			bool enoughMarkers = true;
			BOOST_FOREACH( MarkerIDCount::value_type& mapEl, m_markerID )
			{
				if( mapEl.second < 2 ){
					enoughMarkers = false;
					break;
				}
			}

			// start the bundle adjustment, if all new markers can be seen in at least two different set of images,
			// or if the number of images in the queue is to long
			// and no separated markers can be seen in the image
			if( (m_imageCount >= maxImage || m_newMarker) && enoughMarkers && m_separatedMarkers.empty() ){ 

				// write all data in m_markerBundle
				writeMarkerBundle();

				// do bundle adjustment
				doBundleAdjustment();

				// set flags to default
				m_newMarker = false;
				m_imageCount = 0;
			}
		}
	}
	catch ( const std::string& s )
	{ LOG4CPP_ERROR(logger,"Error: " << s ); }
	catch ( const std::runtime_error& e )
	{ LOG4CPP_ERROR(logger,"Error: " << e.what()); }
	return;
}

void MultiMarkerTrackerBundleAdustment::getImage( const Measurement::ImageMeasurement& image ){
	LOG4CPP_INFO(logger, "getImage");
	// delete old calibration data, before starting the marker bundle (you would get wrong parameter)
	// this is executed only once in run-time (the first time you start a new marker bundle)
	if( !m_calibration ){

		m_calibration = true;
		worldMap.clear();
		errorMap.clear();

		// delete old calib files
		deleteWorldFile();
	}

	// lock the image queue
	boost::mutex::scoped_lock l( m_Mutex );

	// get the image from the dfg edge and store it in a global vector and wake up the bundle adjustment thread
	m_tempIQ.push_back(image);

	// notify waiting thread
	if ( m_State == state_running )
		m_NewEventCondition.notify_all();
}

bool MultiMarkerTrackerBundleAdustment::validateMarker( MarkerInfo marker, unsigned long long int ID, MarkerDetectionConfig markerMask, int imageWidth, int imageHeight ){
	LOG4CPP_INFO(logger, "validateMarker");
	// to validate if one marker, which was found from detectMarkers, is valid, some conditions have to be met
	// 1) orientation
	//		the pose of the marker has to point in camera direction all the time (not downwards)
	// 2) visibility
	//		the visibility of the found marker needs to have a certain level (discard marker with low values)
	// 3) translation
	//		the z-direction of the camera is pointing backwards, therefore all markers need to have a negative z-value
	// 4) cornerlist
	//		x- and y-values need to be in range of the image resolution
	// 5) single colored patches
	//		discard found markers, which are complete white or black (ID may not be the codeMask or zero)
	// 6) nearly black markers with large mask size (resolution)
	//		discard found markers, which are nearly black (ID and codeMask too similar) (check only large mask sizes)
	// 7) small markers in the image (too far away from the camera)
	//		discard too small markers, which cannot be seen very well, therefore false IDs can be found (compare the number of pixels the marker consists of)
	//
	// 8) overlap (this part will be checked separately, because all other tests use the marker alone and do not need the markerMap (see checkMarkerOverlap))
	//		two markers are not allowed to overlap
	//		two cases need to be checked: markers, which are within each other, or markers with the same position (possible because of different masks, one need to be chosen)


	// check cornerlist
	bool cornerCheck = true;
	for( int i = 0; i < 4; i++ ){
		if( marker.corners[i][0] < 0 || marker.corners[i][0] > imageWidth || marker.corners[i][1] < 0 || marker.corners[i][1] > imageHeight ){
			cornerCheck = false;
			break;
		}
	}

	// check on nearly black markers
	bool maskCheck = true;
	int count = 0;
	std::string completeString = "";

	std::ostringstream idString(std::ostringstream::out);
	std::ostringstream maskString(std::ostringstream::out);
	std::ostringstream tempString(std::ostringstream::out);

	// get string from ID and codeMask
	maskString << std::hex << markerMask.codeMask;
	tempString << std::hex << ID;

	// this test is only needed for markers with large resolution (because the bits of the marker is not recognized otherwise)
	if( maskString.str().size() > 8 ){

		// get front zeros for the ID string
		for( int c = tempString.str().size(); c < maskString.str().size(); c++ ){
			completeString.append("0");
		}

		// append zeros and ID
		completeString.append(tempString.str());
		idString << completeString;

		// test each character of the ID and codeMask on difference
		for( int i = 0; i < maskString.str().size(); i++ ){
			if( idString.str().at(i) != maskString.str().at(i) )
				count++;
		}

		// discard all markers with only one or zero different hex value compared to the mask
		if( count < 2 )
			maskCheck = false;
	}

	// calculate the min and max values of the 2D corners in x- and y- direction for the input marker
	// the number of pixels one marker should at least have, is user defined from the data flow graph
	// markers, which are too far away from the camera, cannot be detected very well, therefore this test is needed
	float minX, maxX, minY, maxY, x, y;
	minX = marker.corners[0][0];
	minY = marker.corners[0][1];
	maxX = minX;
	maxY = minY;
	for( int i = 1; i < 4; i++ ){
		x = marker.corners[i][0];
		y = marker.corners[i][1];
		if( x < minX )
			minX = x;

		if( y < minY )
			minY = y;
					
		if( x > maxX )
			maxX = x;
					
		if( y > maxY )
			maxY = y;
	}

	// compare all tests 
	if( marker.nVisibility < 90 || !maskCheck || !cornerCheck || ID == markerMask.codeMask || ID == 0 || marker.pose.translation()[2] > 0.0 || 
			!checkMarkerOrientation(marker,(90.0 - m_tiltAngle)*M_PI/180.0) || (abs(maxX-minX) < m_minPixel) || (abs(maxY-minY) < m_minPixel) )
		return false;
	else
		return true;
}

bool MultiMarkerTrackerBundleAdustment::checkMarkerOrientation( MarkerInfo marker, float flatness ){
	LOG4CPP_INFO(logger, "checkMarkerOrientation");
	// flatness is the value to eliminate flat camera poses where the marker cannot always be seen correctly
	// therefore you do not want to use this marker for the bundle
	// the unit of flatness is rad (NOT degree)
	double x = marker.pose.rotation().x();
	double y = marker.pose.rotation().y();
	double z = marker.pose.rotation().z();
	double w = marker.pose.rotation().w();

	// based on the camera position, the amount of rotation in x- and y- direction is limited until the marker would be seen from underneath, which would not be possible
	// (rotation is only allowed in the upper half sphere of the marker)
	double angleX = atan2( 2*(w*x+y*z), 1-2*(x*x+y*y) );
	double angleY = asin( 2*(w*y-z*x) );

	return pow(angleX,2) + pow(angleY,2) <= pow(M_PI_2 - flatness, 2);
}

bool MultiMarkerTrackerBundleAdustment::checkMarkerOverlap( MarkerInfo marker, unsigned long long int ID, MarkerDetectionConfig markerMask, MultiMarkerInfoMap infoMap ){
	LOG4CPP_INFO(logger, "checkMarkerOverlap");
	// checks two possibilities for marker overlap
	// with different marker masks, one marker can be found more then once, therefore chose one mask based on user defined priorities
	// there is the possibility that small rectangles can be seen within a marker, therefore all markers, which are within an already found marker, are discarded based on 2D corner values
	bool sameMarker = false;
	bool sameCorner = false;
	bool outMarker = false;
	float minX, minY, maxX, maxY, x, y, toleranceX, toleranceY;

	// calculate the min and max values of the 2D corners in x- and y- direction for the input marker
	// this is needed to get the tolerance value (compare the corners with the whole markerMap later)
	// the tolerance value is based on the amount of pixels, the marker is seen in the image (the tolerance will get bigger, if the camera gets closer to the marker)
	minX = marker.corners[0][0];
	minY = marker.corners[0][1];
	maxX = minX;
	maxY = minY;
	for( int i = 1; i < 4; i++ ){
		x = marker.corners[i][0];
		y = marker.corners[i][1];
		if( x < minX )
			minX = x;

		if( y < minY )
			minY = y;
					
		if( x > maxX )
			maxX = x;
					
		if( y > maxY )
			maxY = y;
	}

	toleranceX = (maxX - minX) / 10;
	toleranceY = (maxY - minY) / 10;

	// checks the input marker with the whole MultiMarkerInfoMap
	BOOST_FOREACH( MultiMarkerInfoMap::value_type& mapEl, infoMap )
	{
		BOOST_FOREACH( MarkerInfoMap::value_type& mapEl2, mapEl.second )
		{
			// check input marker only with marker from current image
			if( mapEl2.second.found ){

				// do not check one marker with itself
				if( ID != mapEl2.first || markerMask.codeMask != mapEl.first.codeMask || markerMask.codeSize != mapEl.first.codeSize || markerMask.markerSize != mapEl.first.markerSize ){

					// calculate the min and max values of the 2D corners in x- and y- direction for the whole marker area
					minX = mapEl2.second.corners[0][0];
					minY = mapEl2.second.corners[0][1];
					maxX = minX;
					maxY = minY;
					for( int i = 1; i < 4; i++ ){
						x = mapEl2.second.corners[i][0];
						y = mapEl2.second.corners[i][1];
						if( x < minX )
							minX = x;

						if( y < minY )
							minY = y;
					
						if( x > maxX )
							maxX = x;
					
						if( y > maxY )
							maxY = y;
					}

					// compare the input marker with its corner values with the min and max values
					// if all four corners are within the min/max interval, the marker is false and therefore outMarker remains unchanged
					// if one corner is outside then the parameter is set to true
					outMarker = false;
					for( int i = 0; i < 4; i++ ){
						if( marker.corners[i][0] <= minX || marker.corners[i][0] >= maxX || marker.corners[i][1] <= minY || marker.corners[i][1] >= maxY )
							outMarker = true;
					}

					// compare all four corners of the input marker separately with the cornerlist of the infoMap
					// two markers do not always have the same order in the cornerlist, therefore you need to check every corner with the whole cornerlist of the infoMap
					// if all four corners have the same position (with a little tolerance value), they are the same marker
					// if one corner differs, then you can stop the loop and continue with sameMarker = false
					sameMarker = true;
					for( int i = 0; i < 4; i++ ){
						sameCorner = false;
						for( int j = 0; j < 4 ; j++ ){
							if( abs(marker.corners[i][0] - mapEl2.second.corners[j][0]) < toleranceX && abs(marker.corners[i][1] - mapEl2.second.corners[j][1]) < toleranceY )
								sameCorner = true;
						}

						if( !sameCorner ){
							sameMarker = false;
							break;
						}
					}

					// if the input marker has got the same 2D corner position as the marker from infoMap, you have to decide which one to use
					// this is done based on the priority value from the config file (higher int value means higher priority)
					// if the priority is to low, it is the wrong marker => return false
					// if the input marker has got a different 2D cornerlist, then you need to check, if it has been within another marker (outMarker) => wrong marker and return false
					if( sameMarker && (markerMask.priority <= mapEl.first.priority) )
						return false;
					else if( !sameMarker && !outMarker )
						return false;
				}
			}
		}
	}

	// if the input marker do not overlap with other markers (the previous conditions in the loop are never true), => valid marker and return true
	return true;
}

void MultiMarkerTrackerBundleAdustment::writeConfigFile(){
	LOG4CPP_INFO(logger, "writeConfigFile");
	// get the path name of the file
	std::ofstream outFile( m_markerConfig.c_str() );

	// each value element of m_writeConfig consists of one marker mask
	// it is a global parameter to store all information of the marker (see imageQueue)
	// output the marker configuration, directory and the marker information itself
	BOOST_FOREACH( WriteMarkerConfig::value_type& mapEl, m_writeConfig )
	{
		outFile << "#markerconfig [codeSize] [markerSize] [codeMask] [defaultSize] [priority]" << std::endl;
		outFile << "markerconfig " << mapEl.first.codeSize << " " << std::dec << mapEl.first.markerSize << " " << std::hex << mapEl.first.codeMask << " " << mapEl.first.defaultSize << " " << mapEl.first.priority << std::endl;
		outFile << "#marker [MarkerID] [MarkerSize]" << std::endl;

		// each mask consists of several marker, which are writen in the config file (ID and size)
		BOOST_FOREACH( MarkerIDSize::value_type& mapEl2, mapEl.second )
		{
			outFile << "marker " << std::hex << mapEl2.first << " " << mapEl2.second << std::endl;
		}
		outFile << "#markerdir [directory]" << std::endl;
		outFile << "markerdir " << m_markerDirectory.c_str() << std::endl;
	}
	outFile.close();
	
}

void MultiMarkerTrackerBundleAdustment::writeMarkerBundle(){
	LOG4CPP_INFO(logger, "writeMarkerBundle");
	// delete all data in m_markerBundle to start with a clean structure
	m_markerBundle->cameras.clear();
	m_markerBundle->imageToCam.clear();
	m_markerBundle->markers.clear();
	const Math::Matrix< double, 3, 3 > cam = *m_inBundleCamIntrinsics->get(Measurement::now()); 
	Math::Matrix< float, 3, 3 > intrinsics;
	Math::Util::matrix_cast_assign( intrinsics, cam );
	const Math::Vector4d distor =  Math::Vector4d(0,0,0,0);
	m_markerBundle.reset(new BAInfo(intrinsics, distor));

	// delete old calib files
	deleteWorldFile();

	// put all needed information for the marker bundle in the parameter m_markerBundle
	BOOST_FOREACH( MarkerBundleConfig vecEl, m_tempMarkerBundle )
	{
		m_markerBundle->cameras.push_back( BACameraInfo() );
		m_markerBundle->cameras.back().name = vecEl.imageName;
		m_markerBundle->imageToCam[ vecEl.imageName ] = vecEl.camID;

		std::map< unsigned long long int, Markers::MarkerInfo > tmpMarkerMap;		
		BOOST_FOREACH( MultiMarkerInfoMap::value_type& mapEl, vecEl.markerMap )
		{
			BOOST_FOREACH( MarkerInfoMap::value_type& mapEl2, mapEl.second )
			{
				if ( mapEl2.second.found != Vision::Markers::MarkerInfo::ENotFound )
				{
					m_markerBundle->markers[ mapEl2.first ].fSize = mapEl2.second.fSize;
					m_markerBundle->markers[ mapEl2.first ].cameras.insert( vecEl.camID );
					tmpMarkerMap[mapEl2.first] = mapEl2.second;
				}				
			}		
		}

		// copy remaining marker infos
		m_markerBundle->cameras[ vecEl.camID ].measMarker.swap( tmpMarkerMap );
	}
}

void MultiMarkerTrackerBundleAdustment::doBundleAdjustment(){
	LOG4CPP_INFO(logger, "doBundleAdjustment");
	try
	{	
		// initialize poses
		m_markerBundle->initMarkers();

		// do bundle adjustment
		m_markerBundle->bundleAdjustment( false );

		// turning the console output on/off
		if( m_bEnableConsoleOutput == true ){
			m_markerBundle->printConfiguration();
			m_markerBundle->printResiduals();
		}

		// get all needed marker informations from the bundle and write them to an external file
		BOOST_FOREACH( BAInfo::MarkerMap::value_type& mapEl, m_markerBundle->markers ){

			std::vector< Math::Vector< double, 3 > > markerCorners;
			Math::Matrix< double, 6, 6 > tmpCovariance = Math::Matrix< double, 6, 6 >::zeros();
			int markerCount = 0;
			bool valid = true;

			// get the covariance matrix of the actual marker
			BOOST_FOREACH( BACameraInfo vecEl, m_markerBundle->cameras )
			{
				BOOST_FOREACH( BACameraInfo::MarkerMeasMap::value_type& mapEl2, vecEl.measMarker )
				{
					// get the covariance matrix of one marker from each image
					if(mapEl.first == mapEl2.first){

						// test if one value of the covariance matrix has a QNAN value
						// these covariances will not be used for the average of all images
						valid = true;
						for( int i = 0; i < 6; i++ ){
							for( int j = 0; j < 6; j++ ){
								if( mapEl2.second.covariance.at_element(i,j) == std::numeric_limits<double>::quiet_NaN() ){
									valid = false;
									break;
								}
							}
							if( !valid )
								break;
						}

						// get sum of all covariance values of one marker
						if( valid ){
							tmpCovariance += mapEl2.second.covariance;
							markerCount++;
						}
					}
				}
			}

			// calculate average of covariance matrix
			if( markerCount != 0 )
				tmpCovariance = tmpCovariance / markerCount;

			// calculate all four corner poses based on the marker pose and size (3D position list)
			for ( unsigned i = 0; i < 4; i++ )
			{
				Math::Vector< double, 3 > p = mapEl.second.pose * Math::Vector< double, 3 >( g_unitCorners[ i ] * mapEl.second.fSize );	    		
				markerCorners.push_back( p );
				
			}

			// write marker pose of all four corners to the world map for later use
			worldMap[mapEl.first] = markerCorners;

			// write marker error pose to the error map for later use
			errorMap[mapEl.first] = ErrorPose( mapEl.second.pose, tmpCovariance );

			// output marker error pose to Ubitrack calibration files suitable for manual configuration of a marker bundle (quaternion, translation and covariance)
			writeWorldFile(mapEl.first, ErrorPose( mapEl.second.pose, tmpCovariance ));			
		}

	}
	catch ( const std::string& s )
	{ LOG4CPP_ERROR(logger,"Error: " << s ); }
	catch ( const std::runtime_error& e )
	{ LOG4CPP_ERROR(logger,"Error: " << e.what()); }
	return;
}

void MultiMarkerTrackerBundleAdustment::writeWorldFile(unsigned long long id, const Math::ErrorPose& errorPose){
	// get the path name of the file
	std::ostringstream cornerFileName(std::ostringstream::out);
	cornerFileName << m_markerDirectory << "/marker" << std::hex << id << ".cal";
	LOG4CPP_INFO(logger, "writeWorldFile: " << cornerFileName.str() );
	// output the world map information to a calibration file
	Util::writeCalibFile( cornerFileName.str(), Measurement::ErrorPose( Measurement::now(), errorPose ) );
}

void MultiMarkerTrackerBundleAdustment::deleteWorldFile(){
	LOG4CPP_INFO(logger, "deleteWorldFile");
	// delete all calib files from all known markers
	BOOST_FOREACH( WriteMarkerConfig::value_type& mapEl, m_writeConfig )
	{
		BOOST_FOREACH( MarkerIDSize::value_type& mapEl2, mapEl.second )
		{
			// get path
			std::ostringstream calibFileName(std::ostringstream::out);
			calibFileName << m_markerDirectory << "/marker" << std::hex << mapEl2.first << ".cal";

			// if file not found, gives error message
			if ( !boost::filesystem::exists( calibFileName.str() ) )
				continue;

			// delete calib file
			boost::filesystem::remove( calibFileName.str() );
		}
	}
}


void MultiMarkerTrackerBundleAdustment::resetMarkerBundle( const Measurement::Button& button ){
	LOG4CPP_INFO(logger, "resetMarkerBundle");
	// reset all collected data from the marker bundle and world map
	if( *button == 114 ){

		// reset all global parameter
		m_separatedMarkers.clear();
		m_tempMarkerBundle.clear();
		m_markerID.clear();
		worldMap.clear();
		errorMap.clear();

		// reset markerMap (delete this loop if not wanted)
		BOOST_FOREACH( MultiMarkerInfoMap::value_type& mapEl, markerMap )
		{
			mapEl.second.clear();
		}

		// delete all data in marker bundle and init all camera parameter
		m_markerBundle->markers.clear();
		m_markerBundle->cameras.clear();
		m_markerBundle->imageToCam.clear();
		const Math::Matrix< double, 3, 3 > cam = *m_inBundleCamIntrinsics->get(Measurement::now()); 
		Math::Matrix< float, 3, 3 > intrinsics;
		Math::Util::matrix_cast_assign( intrinsics, cam );
		const Math::Vector4d distor =  Math::Vector4d(0,0,0,0);
		m_markerBundle.reset(new BAInfo(intrinsics, distor));

		// delete old calib files
		deleteWorldFile();

		// clear config file (delete this loop if not wanted)
		BOOST_FOREACH( WriteMarkerConfig::value_type& mapEl, m_writeConfig )
		{
			mapEl.second.clear();
		}
		writeConfigFile();
	}
}


} } // namespace Ubitrack::Driver


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	std::vector< std::string > markerTrackerComponents;
	markerTrackerComponents.push_back( "MarkerTracker" );	
	markerTrackerComponents.push_back( "MultiMarkerTracker" );
	markerTrackerComponents.push_back( "MultiMarkerTrackerBundleAdjustment" );
	cf->registerModule< Ubitrack::Drivers::MarkerTrackerModule > ( markerTrackerComponents );
}
