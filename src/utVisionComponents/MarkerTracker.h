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
 * @author Daniel Pustka <daniel.pustka@in.tum.de> (MarkerTracker)
 * @author Martin Schwörer <schwoere@in.tum.de> (MultiMarkerTracker)
 */


#ifndef __MarkerTracker_h_INCLUDED__
#define __MarkerTracker_h_INCLUDED__

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
#include <utVision/MarkerBundle.h>
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
	// : public Math::Scalar< unsigned long long int >
	: public std::string
{

public:
	const bool isMarker;
	const bool isMultiMarker;
	unsigned long long int nCode;
	
	/** extract id from configuration */
	IdKey( boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
		: isMarker( subgraph->hasNode( "Marker" ) )
		, isMultiMarker( subgraph->hasNode( "MultiMarker" ) )
		, nCode( 0 )
	{
		
		if( isMarker )
		{
			
			this->assign( subgraph->getNode( "Marker" )->getAttributeString( "markerId" ) );
			
			if ( this->empty() )
				UBITRACK_THROW( "Missing markerId attribute on node Marker" );
			
			
			std::istringstream idStream( *this );
			idStream >> std::hex >> nCode;
			
			// generate string id from nCode, the same way that is used when using the int constructor
			// otherwise problems with marker Ids with 0 in the front
			std::string id;
			std::ostringstream idStream2( id );
			idStream2<< "0x" << std::hex << nCode;
			this->assign( idStream2.str() );
			
			
			return;
	}
		if( isMultiMarker ){
			this->assign( subgraph->getNode( "MultiMarker" )->getAttributeString( "MarkerConfig" ));
			return;
		}

		if( subgraph->hasEdge( "Image" ) )
		{
			this->assign( "Image" );
			return;
		}
		if( subgraph->hasEdge( "DebugImage" ) )
		{
			this->assign( "DebugImage" );
			return;
		}
		if( subgraph->hasEdge( "CameraIntrinsics" ) )
		{
			this->assign( "CameraIntrinsics" );
			return;
		}
		
		UBITRACK_THROW( "Could not assign a valid marker module component key." );
	}

	// create from unsigned long long int
	IdKey( unsigned long long int value )
		: isMarker( true )
		, nCode( value )
		, isMultiMarker( false )
	{
		std::string id;
		std::ostringstream idStream( id );
		idStream << "0x" << std::hex << value;
		this->assign( idStream.str() );
	}
};




/** camera key just takes the node id */
MAKE_NODEIDKEY( CameraIdKey, "Camera" );

// forward declarations
class MarkerTrackerBase;
class MarkerTrackerModule;

//typedef Module< CameraIdKey, IdKey, MarkerTrackerModule, MarkerTrackerBase > MarkerTrackerModuleBase;


/**
 * Tracks markers and send the results to the components.
 * This is done only once for each timestamp.
 */
class MarkerTrackerModule
	: public Module< CameraIdKey, IdKey, MarkerTrackerModule, MarkerTrackerBase >
{
public:
	MarkerTrackerModule( const CameraIdKey& key, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, FactoryHelper* pFactory );

	/** find markers in the image and send results to the components */
	void trackMarkers( const Measurement::ImageMeasurement& );

	/** fetch the intrinsic matrix from one of the components **/
	Math::Matrix< float, 3, 3 > MarkerTrackerModule::getIntrinsicsMatrix(const Measurement::ImageMeasurement& time);

	/** create new components. Necessary to support multiple component types. */
	//boost::shared_ptr< MarkerTrackerBase > createComponent( const std::string& type, const std::string& name, 
	//boost::shared_ptr< Graph::UTQLSubgraph > pConfig, const IdKey& key, MarkerTrackerModule* pModule );
	
	boost::shared_ptr< MarkerTrackerBase > createComponent( const std::string& type, const std::string& name,
		boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const ComponentKey& key, ModuleClass* pModule );
	
protected:

	// timestamp of the last measurement
	Measurement::Timestamp m_lastTimestamp;

	// timestamp of the last time the image was analysed fully 
	//Measurement::Timestamp m_fullAnalizeTimestep;

	/** Size of marker including its border, counted in bits */
	unsigned int m_markerSize;

	/** Size of marker bit pattern, counted in bits */
	unsigned int m_codeSize;
	
	/** Mask ANDed with bit pattern detected in image to obtain the final marker ID */
	unsigned long long int m_codeMask;
	
	/** Incorporate inner edgelets in pose refinement, may be unstable! */
	bool m_useInnerEdgels;
	
	/** using adaptive thresholding for thresholding the image*/
	bool m_enableAdaptiveThreshold;

	/** if m_enableAdaptiveThreshold not enabled, binary thresholding with m_binaryThresholdValue is used*/
	int m_binaryThresholdValue;
	

	#ifdef DO_TIMING
	Ubitrack::Util::BlockTimer m_detectMarkersTimer;
	Ubitrack::Util::BlockTimer m_detectMarkersTimerRefine;
	#endif
	friend class MultiMarkerTrackerBundleAdustment;


};

class MarkerTrackerBase
	: public MarkerTrackerModule::Component
{
public:
	MarkerTrackerBase( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const IdKey& componentKey, MarkerTrackerModule* pModule )
	//MarkerTrackerBase( const std::string& sName, const IdKey& componentKey, MarkerTrackerModule* pModule )
		: MarkerTrackerModule::Component( sName, componentKey, pModule )
		, m_lastTime( 0 )
	{
	}

	virtual bool debug(){ return false; }
	virtual bool isIntrinsics(){ return false; }

	
private:
	
	// some variables where the module stores information
	Measurement::Timestamp m_lastTime;
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
	: public MarkerTrackerBase
{
public:
	MarkerTracker( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const IdKey& componentKey, MarkerTrackerModule* pModule );

	/** is the debug port connected? */
	bool debug();
	

	bool isIntrinsics();
	
	/** returns the camera intrinsics if connected. throws otherwise */
	Measurement::Matrix3x3 intrinsics( Measurement::Timestamp t );
	
	/** use edge refinement? */
	bool useEdgeRefinement() const;


protected:
	/** method that receives events and displays the image */
	void pushImage( const Measurement::ImageMeasurement& m );

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


 

typedef struct MarkerDetectionConfig{
	/** Size of marker including its border, counted in bits */
	unsigned int markerSize;

	/** Size of marker bit pattern, counted in bits */
	unsigned int codeSize;
	
	/** Mask ANDed with bit pattern detected in image to obtain the final marker ID */
	unsigned long long int codeMask;

	/** Default size of markers, measured in meters */
	float defaultSize;

	/** Priority value of the marker mask to use one mask, if two or more markers are found in the same image position */
	/** Higher int value means higher priority */
	unsigned int priority;

} MarkerDetectionConfig;

// vector to get the corner position based on the central marker pose
static const Math::Vector< double, 3 > g_unitCorners[ 4 ] = 
{
	Math::Vector< double, 3 >( -0.5,  0.5, 0.0 ),
	Math::Vector< double, 3 >( -0.5, -0.5, 0.0 ),
	Math::Vector< double, 3 >(  0.5, -0.5, 0.0 ),
	Math::Vector< double, 3 >(  0.5,  0.5, 0.0 ) 
};

typedef std::map<MarkerDetectionConfig,std::map<unsigned long long int,MarkerInfo> > MultiMarkerInfoMap;;
typedef std::map<unsigned long long, std::vector<Math::Vector< double, 3 > > > MultiMarker3DMap;
typedef std::map<unsigned long long, Math::ErrorPose> MultiMarkerErrorMap;

typedef std::map<unsigned long long int, float> MarkerIDSize;
typedef std::map<MarkerDetectionConfig,MarkerIDSize> WriteMarkerConfig;

// compare different marker configuration to order them in an explicit sequence
bool operator<(const MarkerDetectionConfig &a, const MarkerDetectionConfig &b)
{
	if(a.codeMask == b.codeMask){
		if(a.codeSize == b.codeSize){			
			return a.markerSize < b.markerSize;
		}
		return a.codeSize < b.codeSize;
	}
    return a.codeMask < b.codeMask;
}

class MultiMarkerTracker
	: public MarkerTrackerBase
{
public:
	
	MultiMarkerTracker( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const IdKey& componentKey, MarkerTrackerModule* pModule );
	
	bool debug()
	{ return false; }

	bool isIntrinsics()
	{ return false; };
	
protected:
	
	void pushImage( const Measurement::ImageMeasurement& m )
	{ getModule().trackMarkers( m ); }

	void foundMarkers(MarkerInfoMap& foundMarkerMap, const Math::Matrix< double, 3, 3 >& cam, const Measurement::Timestamp& t);

	void readConfigFile(MarkerDetectionConfig& config);
	void readWorldFiles();
	
	// get the 3D position list of the markers from the world map
	Measurement::PositionList getWorldMap(Measurement::Timestamp t);

	// ports
	Dataflow::PushSupplier< Measurement::ErrorPose > m_outErrorPose;
	//boost::shared_ptr< Dataflow::PullSupplier< Measurement::PositionList >> m_worldMapPort;

	// Incorporate inner edgelets in pose refinement, may be unstable!
	bool m_useInnerEdgels;

	// parameter for storing the marker informations
	std::string m_markerConfig;
	std::string m_markerDirectory;
	
	// map for writeConfigFile to gather all markers, not just the info from the last image
	WriteMarkerConfig m_writeConfig;

	// the global marker, world map and error map to store all found marker informations in run-time
	MultiMarkerInfoMap markerMap;
	
	MultiMarker3DMap worldMap;
	MultiMarkerErrorMap errorMap;
	friend class MarkerTrackerModule;
};
typedef struct TempIDConfig{
	/** ID of the marker */
	unsigned long long int ID;

	/** Size of marker including its border, counted in bits */
	unsigned int markerSize;

	/** Size of marker bit pattern, counted in bits */
	unsigned int codeSize;
	
	/** Mask ANDed with bit pattern detected in image to obtain the final marker ID */
	unsigned long long int codeMask;

} TempIDConfig;
typedef struct MarkerBundleConfig{
	/** ID and number of the image */
	size_t camID;

	/** to get the timestamp of the image */
	std::string imageName;

	/** the whole markerMap of the image */
	MultiMarkerInfoMap markerMap;

} MarkerBundleConfig;

// compare different TempID configuration to order them in an explicit sequence
bool operator<(const TempIDConfig &a, const TempIDConfig &b)
{
	if(a.codeMask == b.codeMask){
		if(a.codeSize == b.codeSize){
			if(a.markerSize == b.markerSize){			
				return a.ID < b.ID;
			}
			return a.markerSize < b.markerSize;
		}
		return a.codeSize < b.codeSize;
	}
	return a.codeMask < b.codeMask;
}


typedef std::map<TempIDConfig, std::set<TempIDConfig> > SeparatedMarkers;
typedef std::map<MarkerDetectionConfig,MarkerIDSize> WriteMarkerConfig;
typedef std::map<TempIDConfig, int> MarkerIDCount;

class MultiMarkerTrackerBundleAdustment: public MultiMarkerTracker{

public:
	MultiMarkerTrackerBundleAdustment( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const IdKey& componentKey, MarkerTrackerModule* pModule );
	~MultiMarkerTrackerBundleAdustment();

	// start and stop functions for all processes
	virtual void start();
	virtual void stop();

	// get the image from the edge of the data flow graph, which is used for the bundle
	void getImage(const Measurement::ImageMeasurement& image);
	
	// reset all collected datas of the marker bundle
	// you can start without the interference of old datas, which would lead to false results
	void resetMarkerBundle(const Measurement::Button& button);

	// queue thread function 
	void threadFunction();


protected:
	// read and write all files, which are used for the image tracking
	void deleteWorldFile();
	void writeConfigFile();
	void writeWorldFile(unsigned long long id, const Math::ErrorPose& errorPose);

	// examine the actual image and store marker informations for the marker bundle
	void imageQueue(const Measurement::ImageMeasurement& image, int maxImage);

	// checks if the found marker is valid based on several conditions (see the comment in the source file)
	bool validateMarker( MarkerInfo marker, unsigned long long int ID, MarkerDetectionConfig markerMask, int imageWidth, int imageHeight );

	// checks if an marker was found within another marker or the same marker was found from different masks
	bool checkMarkerOverlap( MarkerInfo marker, unsigned long long int ID, MarkerDetectionConfig markerMask, MultiMarkerInfoMap infoMap );

	// checks if the final pose of the marker is pointing in the direction of the camera
	bool checkMarkerOrientation( MarkerInfo marker, float flatness );

	// reorder the images, which are stored in m_markerBundle, so the marker bundle can work without crashing
	void writeMarkerBundle();

	// start the marker bundle
	void doBundleAdjustment();

	
	// input ports (do not need to be connected, any amount of edges possible)
	boost::shared_ptr< Dataflow::PushConsumer< Measurement::ImageMeasurement > > m_inBundleCamPort;
	boost::shared_ptr< Dataflow::PullConsumer< Measurement::Matrix3x3 > > m_inBundleCamIntrinsics;
	boost::shared_ptr< Dataflow::PushConsumer< Measurement::Button > > m_resetPort;

	// config parameter for data flow graph
	bool m_bEnableFlipCheck;
	bool m_findMarkers;
	double m_tiltAngle;
	int m_minPixel;
	int m_maxImage;
		// all markers, which are separated from other markers (there needs to be a transitive hull of all markers for the marker bundle)
	SeparatedMarkers m_separatedMarkers;

	// vector for all images, which are used for the marker bundle later on
	std::vector<MarkerBundleConfig> m_tempMarkerBundle;

	// parameter for turning on/off of the markerbundle console output
	bool m_bEnableConsoleOutput;

	// parameter for recognizing, if a new Marker for the bundle were found
	bool m_newMarker;

	// parameter for recognizing, if old calibration datas should be deleted (when new images for marker bundle were received)
	bool m_calibration;

	// map for writeConfigFile to gather all markers, not just the info from the last image
	WriteMarkerConfig m_writeConfig;

	// store the marker IDs of all found markers and count how often they are found in the images
	MarkerIDCount m_markerID;

	// number of image for the next marker bundle adjustment
	int m_imageCount;

	// temp image for the image queue function
	std::vector<Measurement::ImageMeasurement > m_tempIQ;

	// mutex for thread synchronization 
	boost::mutex m_Mutex;

	// condition variable for thread synchronization 
	boost::condition m_NewEventCondition;

	// the event dispatching thread 
	boost::shared_ptr< boost::thread > m_pThread;

	// current state of the event thread 
	enum { state_running, state_stopping, state_stopped, state_end } m_State;

	// data structure for the marker bundle
	boost::shared_ptr< BAInfo > m_markerBundle;

};


}}
#endif