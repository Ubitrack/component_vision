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
 * Finding the edges of a chessboard in an image
 *
 * @author Daniel Muhra <muhra@in.tum.de>
 * @author Christian Waechter <christian.waechter@in.tum.de>
 */

// std
#include <fstream>
#include <sstream>

// OpenCV
#include <utVision/Image.h>
#include <opencv/cv.h>
 
// boost
#include <boost/scoped_array.hpp>
 
#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMath/Vector.h>
#include <utUtil/Exception.h>

//Log4Ccpp
#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.ChessboardFunctions" ) );
namespace Ubitrack { namespace Vision {

/**
 * @ingroup vision_components
 * Chessboard detection component.
 * This class contains a chessboard detection implemented as a \c TriggerComponent.
 *
 * @par Input Ports
 * PushConsumer< Measurement
Measurement::ImageMeasurement > of name "Input", accepts only greyscale images
 *
 * @par Output Ports
 * PushSupplier<Measurement::PositionList2> with name "Output".
 * PushSupplier<ImageMeasurement> with name "DebugImage".
 *
 * @par Configuration
 * \c chessBoardHeight: number of edges the chessboard has (height). 
 * \c chessBoardWidth: number of edges the chessboard has (width).
 * \c Debug: if set to 1, image output is enabled.
 *
 * @par Operation
 * The component returns an vector of 2D coordinates which represents the edges of an given chessboard
 *
 * @par Example Configuration
 * \verbatim 
<Pattern name="ChessBoardTracking" id="someId">
	<Input>
		<Node name="ChessBoard" id="someId">
			<Attribute name="chessBoardHeight" value="6"/>
			<Attribute name="chessBoardWidth" value="7"/>
			<Attribute name="chessBoardWidth" value="0.025"/>
		</Node>
		<Node name="ImagePlane" id="someId2"/>
		<Node name="Intrinsic" id="someId3"/>
		<Node name="Distortion" id="someId4"/>
		<Edge name="Image" source="ChessBoard" destination="ImagePlane" pattern-ref="..." edge-ref="..."/>
		<Edge name="Intrinsic" source="ImagePlane" destination="Intrinsic" pattern-ref="..." edge-ref="..."/>
		<Edge name="Distortion" source="ImagePlane" destination="Distortion" pattern-ref="..." edge-ref="..."/>
	</Input>
	<Output>
		<Node name="Values" id="someId5"/>
		<Node name="Consumer" id="someId6"/>
		<Edge name="Output" source="Values" destination="Consumer"/>
	</Output>
	<DataflowConfiguration>
		<UbitrackLib class="ChessBoardTracking"/>
	</DataflowConfiguration>
</Pattern> 
\endverbatim
 */
class ChessboardFunctionsComponent
	: public Dataflow::Component
{
public:
	/**
	 * Standard component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	ChessboardFunctionsComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
		: Dataflow::Component( sName )
		, m_grid_type( 0 )
		, m_width( 0 )
		, m_height( 0 )
		, m_sizeX( 0 )
		, m_sizeY( 0 )
		, m_edges( 0 )
		, m_scaleFactor( 0 )
		, m_normalize ( false )
		, m_inPort( "Image", *this, boost::bind( &ChessboardFunctionsComponent::pushImage, this, _1 ) )
		, m_intrinsicPort( "Intrinsic", *this )
		, m_distortionPort( "Distortion", *this )
		// the old files ChessBoardDetection and ChessBoardTracking used the same name "Output" for their different Output Ports
		// thats why here is a distinction for the name
		, m_outPosePort( ( 0 == pCfg->m_DataflowClass.compare( 0, 18, "ChessBoardTracking" )  ) ? "Output" : "Pose", *this )
		, m_outPoints2DPort( ( 0 == pCfg->m_DataflowClass.compare( 0, 19, "ChessBoardDetection" )  ) ? "Output" : "Corners", *this )
		, m_outPoints3DPort(  "Corners3D", *this )
		, m_debugPort( "DebugImage", *this )
		
    {
		if(	0 == pCfg->m_DataflowClass.compare( 0, 19, "ChessBoardDetection" ) )
			LOG4CPP_ERROR( logger, "ChessBoardDetection is a deprecated Component.\nPlease switch to the newer ChessboardFunctions Component and use the according pattern." );
			
		if(	0 == pCfg->m_DataflowClass.compare( 0, 18, "ChessBoardTracking" ) )
			LOG4CPP_ERROR( logger, "ChessBoardTracking is a deprecated Component.\nPlease switch to the newer ChessboardFunctions Component and use the according pattern." );

			
		// same stuff for the chessboard detection (very old (deprecated) + old)
		if ( pCfg->hasNode( "ChessBoard" ) )
		{
			// get height and width attributes from ChessBoard node
			Graph::UTQLSubgraph::NodePtr pCbNode = pCfg->getNode( "ChessBoard" );
			pCbNode->getAttributeData( "chessBoardHeight", m_height );
			pCbNode->getAttributeData( "chessBoardWidth", m_width );
		
		
			if ( pCbNode->hasAttribute( "gridtype" ) ) // chosse the type of the calibration grid
			{
				if( pCbNode->getAttributeString( "gridtype" ) == "chessboard" ) //also the default value
					m_grid_type = 0;
				if( pCbNode->getAttributeString( "gridtype" ) == "circularsymmetric" )
					m_grid_type = 1;
				if( pCbNode->getAttributeString( "gridtype" ) == "circularasymmetric" )
					m_grid_type = 2;
			}
			
			//stuff of the old calibration pattern
			{
				if ( pCbNode->hasAttribute( "xAxisLength" ) ) // overall size of xAxis
					pCbNode->getAttributeData( "xAxisLength", m_sizeX );
				m_sizeX /= ( m_width - 1 );
				
				if ( pCbNode->hasAttribute( "yAxisLength" ) ) // overall size of yAxis
					pCbNode->getAttributeData( "yAxisLength", m_sizeY );
				m_sizeY /= ( m_height - 1 );
				
				if ( pCbNode->hasAttribute( "scaleFactor" ) ) // scalefactor for smaller images
					pCbNode->getAttributeData( "scaleFactor", m_scaleFactor );
			}

			//stuff of the very old (deprecated) pattern
			{
				if ( pCbNode->hasAttribute( "chessBoardSize" ) ) // old parameters override new ones
				{
					pCbNode->getAttributeData( "chessBoardSize", m_sizeX );
					pCbNode->getAttributeData( "chessBoardSize", m_sizeY );
				}
			}

			
				

		}
		
		// very new pattern, uses no own chessboard size
		// scaling was thrown away, this should be handled somewhere else
		if ( pCfg->hasNode( "CalibrationGridPoints" ) )
		{
			Graph::UTQLSubgraph::NodePtr pCbNode = pCfg->getNode( "CalibrationGridPoints" );
			std::size_t width, height = 0;
			pCbNode->getAttributeData( "gridPointsX", m_width );
			pCbNode->getAttributeData( "gridPointsY", m_height );
			
			// value_type m_sizeX, m_sizeY, lastDim = 0;
			pCbNode->getAttributeData( "gridAxisLengthX", m_sizeX );
			pCbNode->getAttributeData( "gridAxisLengthY", m_sizeY );
			// pCbNode->getAttributeData( "gridThirdDimension", lastDim );
			
			// grid type is set to circular as default:)
			m_grid_type = 1;
			if ( pCbNode->hasAttribute( "gridType" ) )
				if( pCbNode->getAttributeString( "gridType" ) == "asymmetric" )
					m_grid_type = 2;
					
			if ( pCbNode->hasAttribute( "boardType" ) )
				if( pCbNode->getAttributeString( "boardType" ) == "chessboard" )
					m_grid_type = 0;
			
			
			Graph::UTQLSubgraph::EdgePtr edge = pCfg->getEdge( "Corners" );
			if ( edge->hasAttribute( "normalize" ) )
				if( edge->getAttributeString( "normalize" ) == "true" )
					m_normalize = true;
				
		}
		
		if ( m_height <= 0 || m_width <= 0  )
			UBITRACK_THROW( "Could not perform calibration grid detection, specify the number of features to detect using the chessBoardHeight, chessBoardWidth parameters." );
		
		if ( !( m_sizeY > 0 && m_sizeX > 0 ) )
			UBITRACK_THROW( "Could not perform calibration grid detection, calibration grid axis size was not spezified correctly, use the chessBoardSize, yAxisLength or xAxisLength attribute." );
		
		
		// prepare the object points, that always stay fix
		m_edges = m_height * m_width;

		objPoints.reset( new float[ 3 * m_edges ] );
		const float offset = ( m_grid_type == 2 ) ? m_sizeX * 0.5 : 0;
		for( int i = 0; i < m_edges ; ++i )
		{
			objPoints[ 3 * i + 0 ] = static_cast< float >( i % m_width ) * m_sizeX;
			if( ( ( i / m_width ) % 2 ) == 1 )
				objPoints[ 3 * i + 0 ] += offset;
			objPoints[ 3 * i + 1 ] = static_cast< float >( i / m_width ) * m_sizeY;
			objPoints[ 3 * i + 2 ] = 0.0;
		}
    }

	/** Method that computes the result. */
	void pushImage( const Measurement::ImageMeasurement& img )
	{
		boost::scoped_array< CvPoint2D32f > corners( new CvPoint2D32f[ m_edges ] );
		
		int found;
		int pattern_was_found;
		if( m_scaleFactor > 0 )
		{
			/**
			Problem: cvFindChessboardCorners takes ages on large images.
			Idea: Shrink image for corner detection and do the subpixel refinement on the
			original fullsize image.
			m_scaleFactor is optional, but when specified the image is shrinked m_scaleFactor -times.
			CW@2013-06-11: there is also the shrink image component doing a similar job.
			however, I leave this in here, just changed the code to use the already existing image function
			*/
			Vision::Image::Ptr tmpa = img->Clone();
			for( int i( 0 ) ; i < m_scaleFactor; ++i ) 
				tmpa = tmpa->PyrDown();
			pattern_was_found = cvFindChessboardCorners( *tmpa, cvSize( m_width, m_height ) , corners.get(), &found, CV_CALIB_CB_ADAPTIVE_THRESH );

			// Rescale 2d measurements to original image size
			for( int i=0; i<m_edges; i++ )
			{
				corners[ i ].x *= 1<<m_scaleFactor;
				corners[ i ].y *= 1<<m_scaleFactor;
			}
		}
		else  // m_scaleFactor == 0
		{
#if CV_MAJOR_VERSION > 1 && CV_MINOR_VERSION > 2
			cv::Mat calib_image( *img, false );
			
			std::vector< cv::Point2f > centers; //( CvPoint2D32f == cv::Point2f )
			centers.reserve( m_edges ); //allocate some space for the values
			
			switch( m_grid_type ) //switch for correct detection method
			{
			case 0:
				pattern_was_found = cv::findChessboardCorners( calib_image, cvSize( m_width, m_height ), centers );
				break;
			case 1:
				pattern_was_found = cv::findCirclesGrid( calib_image, cvSize( m_width, m_height ), centers, cv::CALIB_CB_SYMMETRIC_GRID );
				break;
			case 2:
				pattern_was_found = cv::findCirclesGrid( calib_image, cvSize( m_width, m_height ), centers, cv::CALIB_CB_ASYMMETRIC_GRID );
				break;
			default:
				LOG4CPP_ERROR( logger, "Cannot perform calibration grid detection, the type of the calibration grid is not specified correctly." );
			}
			//assgin the values to the other array again -> will get better with c++11
			if( pattern_was_found )
			{
				found = m_edges;
				for( int i( 0 ); i < m_edges; ++i )
					corners[ i ] = centers[ i ];
			}
			
#else
			pattern_was_found = cvFindChessboardCorners( *img, cvSize( m_width, m_height ) , corners.get(), &found, CV_CALIB_CB_ADAPTIVE_THRESH );
#endif
		}
		
		if( pattern_was_found && found == m_edges )
		{
			if( m_grid_type == 0 )
				cvFindCornerSubPix( *img, corners.get(), m_edges, cvSize( 10, 10 ), cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_ITER, 10, 0.1f ) );

			/** copies all values into OpenCV compatible formats */
			boost::scoped_array< float > imgPoints( new float[ 2 * m_edges ] );
			CvMat image_points = cvMat ( m_edges, 2, CV_32F, imgPoints.get() );
			CvMat object_points = cvMat ( m_edges, 3, CV_32F, objPoints.get() );

			if( img->origin ) // == IPL_ORIGIN_BL
			{
				LOG4CPP_DEBUG( logger, "Origin flag set" );
				
				for( int i = 0; i < m_edges ; ++i )
				{
					imgPoints[ 2 * i     ] = corners[ i ].x;
					imgPoints[ 2 * i + 1 ] = corners[ i ].y;
				}
			}
			else // == IPL_ORIGIN_TL
			{
				LOG4CPP_DEBUG( logger, "Origin flag NOT set, mirror y-coordinates" );
				
				for( int i = 0; i < m_edges ; ++i )
				{
					/* Commented out by Peter Keitler on 23.11.2011:
					It is enough here to flip the y-coordinates (as the two lines below do). 
					OpenCV takes care of providing the proper sequence
					of points already! This does not depend on the origin of the image, but on the physical layout of the
					checkerboard. If the latter is unique, as e.g. for 5x7 checkerboards, then the first detected
					corner will always reside in the same checkerboard corner, regardless imaging parameters, and 
					corners will always be detected row-by-row. */
					/*
					CW@2013-06-11 still the problem exists that whenever you use the chessboard for tracking it will
					result in a different pose for the chessboard depending on the image orientation.
					If you want to have the chessboard-origin always at the same chessboard corner uncomment the following code:
					*/
					
					/*
					int index = ( m_height - 1 ) * m_width - i + ( ( i % m_width ) << 1 );
					//equivalent to:
					// int index = ( m_height - ( i / m_width ) - 1 ) * m_width + ( i % m_width );
					// int index = ( m_height * m_width - m_width - i ) + ( i % m_width ) + ( i % m_width );
					// int index = ( m_height - 1 ) * m_width - i + ( i % m_width ) + ( i % m_width );
					// int index = ( m_height - 1 ) * m_width - i + 2 * ( i % m_width );
				
					imgPoints[ 2 * index     ] = corners[ i ].x;
					imgPoints[ 2 * index + 1 ] = ( img->height - 1 - corners[ i ].y );
					*/
					
					imgPoints[ 2 * i     ] = corners[ i ].x;
					imgPoints[ 2 * i + 1 ] = img->height - 1 - corners[ i ].y;
					
					LOG4CPP_TRACE( logger, "point: " << corners[i].x << ", " << corners[i].y );
				}
			}
			
			if( m_outPoints2DPort.isConnected() )
			{
				std::vector< Math::Vector< double, 2 > > positions;
				positions.reserve( m_edges );
				if( m_normalize )				
					for( int i = 0; i < m_edges; ++i )
						positions.push_back( Math::Vector< double, 2 >( imgPoints[ 2*i ] / (img->width-1), imgPoints[ 2*i+1 ] / (img->height-1) ) );
				else
					for( int i = 0; i < m_edges; ++i )
						positions.push_back( Math::Vector< double, 2 >( imgPoints[ 2*i ] , imgPoints[ 2*i+1 ] ) );
							
				m_outPoints2DPort.send( Measurement::PositionList2( img.time(), positions ) );
			}
			
			if( m_outPoints3DPort.isConnected() )
			{
				std::vector< Math::Vector< double, 3 > > positions;
				
				positions.reserve( m_edges );
				for( int i = 0; i < m_edges; ++i )
					positions.push_back( Math::Vector< double, 3 >( objPoints[ 2*i ], objPoints[ 2*i+1 ], objPoints[ 2*i+2 ] ) );

				m_outPoints3DPort.send( Measurement::PositionList( img.time(), positions ) );
			}
			
			if( m_outPosePort.isConnected() )
			{
				float disVal[4];
				CvMat distortion_coeffs = cvMat( 4, 1, CV_32FC1, disVal );
				try
				{
					Math::Vector< double, 4 > distortion = *m_distortionPort.get( img.time() );
					disVal[ 0 ] = static_cast< float > ( distortion( 0 ) );
					disVal[ 1 ] = static_cast< float > ( distortion( 1 ) );
					disVal[ 2 ] = static_cast< float > ( distortion( 2 ) );
					disVal[ 3 ] = static_cast< float > ( distortion( 3 ) );
				}
				catch( const Ubitrack::Util::Exception& )
				{
					disVal[ 0 ] = disVal[ 1 ] = disVal[ 2 ] = disVal[ 3 ] = 0.0f;
				}
				
				float intrVal[9];
				CvMat intrinsic_matrix = cvMat ( 3, 3, CV_32FC1, intrVal );
				try
				{
					// compensate for left-handed OpenCV coordinate frame
					Math::Matrix< double, 3, 3 > intrinsic =  *m_intrinsicPort.get( img.time() );
					intrVal[ 0 ] = static_cast< float > ( intrinsic( 0, 0 ) );
					intrVal[ 1 ] = static_cast< float > ( intrinsic( 0, 1 ) );
					intrVal[ 2 ] = static_cast< float > ( intrinsic( 0, 2 ) ) * -1.0f ;
					intrVal[ 3 ] = static_cast< float > ( intrinsic( 1, 0 ) );
					intrVal[ 4 ] = static_cast< float > ( intrinsic( 1, 1 ) );
					intrVal[ 5 ] = static_cast< float > ( intrinsic( 1, 2 ) ) * -1.0f ;
					intrVal[ 6 ] = static_cast< float > ( intrinsic( 2, 0 ) );
					intrVal[ 7 ] = static_cast< float > ( intrinsic( 2, 1 ) );
					intrVal[ 8 ] = static_cast< float > ( intrinsic( 2, 2 ) ) * -1.0f ;
				}
				catch( const Ubitrack::Util::Exception& )
				{
					intrVal[ 0 ] = intrVal[ 4 ] = 650.0f;
					intrVal[ 2 ] = static_cast< float > ( ( img->width - 1 ) >> 2 );
					intrVal[ 5 ] = static_cast< float > ( ( img->height - 1 ) >> 2 );
					intrVal[ 8 ] = 1.0f;
					intrVal[ 1 ] = intrVal[ 3 ] = intrVal[ 6 ] = intrVal[ 7 ] = 0.0f;
				}

				// Commented out by Peter Keitler on Sept 6, 2011:
				// This statement is redundant as 2D coordinates are flipped above, already
				// (see code directly after cvFindCornerSubPix() )
				/*
				// compensate if origin==0
				if ( !( img->origin ) )
				{
					intrVal[5] = img->height - 1 - intrVal[5];
					disVal[2] *= -1.0;
				}
				*/

				/** rotation and translation vectors for OpenCV  output */
				float m_current_rot_vector[3];	
				CvMat current_rot_vec = cvMat( 1, 3, CV_32FC1, m_current_rot_vector );
				
				float m_current_translation_vector[3];	
				CvMat current_trans_vec = cvMat (3, 1, CV_32FC1, m_current_translation_vector);

				/** actual computation */
				cvFindExtrinsicCameraParams2(
							&object_points,
							&image_points,
							&intrinsic_matrix,
							&distortion_coeffs,
							&current_rot_vec,
							&current_trans_vec);


				/** convert rotation vector into a matrix */
				float m_current_rotation_matrix[9];
				CvMat current_rot_mat = cvMat ( 3, 3, CV_32FC1, m_current_rotation_matrix);
				
				cvRodrigues2( &current_rot_vec , &current_rot_mat);

				/** copying into Ubitrack formats */
				double rot_mat[ 9 ];

				for( int i = 0; i < 9; ++i )
					rot_mat[ i ] = double( m_current_rotation_matrix[ i ] ); 

				/** conversion to right-handed coordinate-system */
				Math::Vector< double, 3 > trans( m_current_translation_vector[0], m_current_translation_vector[1], -m_current_translation_vector[2] );
				Math::Matrix< double, 3, 3 > rot( rot_mat );
				
				rot( 2, 0 ) = - rot( 2, 0 );
				rot( 2, 1 ) = - rot( 2, 1 );
				rot( 0, 2 ) = - rot( 0, 2 );
				rot( 1, 2 ) = - rot( 1, 2 );

				Math::Pose pose( Math::Quaternion(rot), trans );
				
				m_outPosePort.send( Measurement::Pose( img.time(), pose ) );
			}
		}
		
		// debug image
		if( m_debugPort.isConnected() )
		{
			boost::shared_ptr< Image > debugImg;

			if( img->nChannels == 1 )
				debugImg = img->CvtColor( CV_GRAY2RGB, 3 );
			else
				debugImg = img->Clone();
				
			debugImg->origin = img->origin; // it should not flip.....
			
			cvDrawChessboardCorners( *debugImg, cvSize( m_width, m_height ), corners.get(), m_edges, pattern_was_found );

			m_debugPort.send( Measurement::ImageMeasurement( img.time(), debugImg ) );
		}
    }

protected:	

	/** specifies the type of the grid (corners, (asymetric-)circles */
	int m_grid_type;
	
	/** number of inner corners in x-direction ( equals width or rows ) */
	int m_width;
	
	/** number of inner corners in y-direction ( equals height or columns ) */
	int m_height;
	
	/** stepsize in x-direction ( equals width or rows ) */
	float m_sizeX;
	
	/** stepsize in y-direction ( equals height or columns ) */
	float m_sizeY;
	
	/** number of inner corners */
	int m_edges;

	/** number of pyrDown before chessboard detection */
	int m_scaleFactor;
	
	bool m_normalize;
	
	/** the chessboard's object points */
	boost::scoped_array< float > objPoints;
	
	/** Image to search for chessboard corners. */
	Dataflow::PushConsumer< Measurement::ImageMeasurement > m_inPort;
	
	/** Intrinsic matrix for chessboard tracking*/
	Dataflow::PullConsumer< Measurement::Matrix3x3 > m_intrinsicPort;
	
	/** distortion parameters for chessboard tracking */
	Dataflow::PullConsumer< Measurement::Vector4D > m_distortionPort;
	
	/** pose of the calibration board */
	Dataflow::PushSupplier< Measurement::Pose > m_outPosePort;
	
	/** Chessboard Corners in image coordinates */
	Dataflow::PushSupplier< Measurement::PositionList2 > m_outPoints2DPort;
	
	/** Chessboard Corners in object coordinates */
	Dataflow::PushSupplier< Measurement::PositionList > m_outPoints3DPort;
	
	/** Image for debugging purposes */
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_debugPort;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf )
{
	cf->registerComponent< ChessboardFunctionsComponent > ( "ChessboardFunctions" );
	//Deprecated, just for compatibility:
	cf->registerComponent< ChessboardFunctionsComponent > ( "ChessBoardTracking" );
	cf->registerComponent< ChessboardFunctionsComponent > ( "ChessBoardDetection" );	
}

} } // namespace Ubitrack::Components
