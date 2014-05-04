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
 * Computes the intrinsic matrix and the distortion coefficients
 *
 * @author Daniel Muhra <muhra@in.tum.de>
 */
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/ExpansionInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMath/Vector.h>

#include <utVision/Image.h>
#include <boost/scoped_array.hpp>
#include <opencv/cv.h>

namespace ublas = boost::numeric::ublas;
using namespace Ubitrack::Measurement;

namespace Ubitrack { namespace Vision {

/**
 * @ingroup vision_components
 * Intrinsic calibration component.
 * This class contains a intrinsic calibration implemented as a \c TriggerComponent.
 *
 * @par Input Ports
 * ExpansionInPort< Math::PositionList2 > of name "Corners", corner points of the chessboard
 *
 * @par Output Ports
 * TriggerOutPort<Measurement::Matrix3x3>> with name "Intrinsic".
 * TriggerOutPort<Measurement::Vector4D>> with name "Distortion".
 *
 * @par Configuration
 * \c chessHeight: number of squares of the chessboard in y direction
 * \c chessWidth: number of squares of the chessboard in x direction
 * \c imgHeight: height of the camera image
 * \c imgWidth: width of the camera image
 * \c chessSize: length of an chessboard square.
 *
 * @par Instances
 * Registered for the following behaviour and names:
 * - push-input, pull-output : IntrinsicCalibrationPushPull
 * - push-input, push-output : IntrinsicCalibrationPushPush
 *
 * @par Operation
 * The component returns an 3 x 3 intrinsic matrix and the distortion coefficients
 *
 * @par Example Configuration
 * \verbatim
<Pattern name="Intrinsic" id="someId">
	<Input>
		<Node name="ChessBoard" id="someId">
			<Attribute name="chessBoardHeight" value="6"/>
			<Attribute name="chessBoardWidth" value="7"/>
			<Attribute name="chessBoardSize" value="0.025"/>
		</Node>
		<Node name="ImagePlane" id="someId2"/>
		<Edge name="Corners" source="ChessBoard" destination="ImagePlane" pattern-ref="..." edge-ref="..."/>
	</Input>
	<Output>
		<Node name="Values" id="someId3"/>
		<Node name="Consumer" id="someId4"/>
		<Edge name="Intrinsic" source="Values" destination="Consumer"/>
		<Edge name="Distortion" source="Values" destination="Consumer"/>
	</Output>
	<DataflowConfiguration>
		<UbitrackLib class="IntrinsicCalibrationPushPull"/>
			<Attribute name="imgHeight" value="480"/>
			<Attribute name="imgWidth" value="640"/>
	</DataflowConfiguration>
</Pattern>
\endverbatim
 */

class IntrinsicComponent
	: public Dataflow::TriggerComponent
{
public:
	/**
	 * Standard component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	IntrinsicComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
		: Dataflow::TriggerComponent( sName, pCfg )
		, m_height( -1 )
		, m_width( -1 )
		, m_sizeX( -1 )
		, m_sizeY( -1 )
		, m_flags( 0 )
		, m_inPort( "Corners", *this )
		, m_intrPort( "Intrinsic", *this )
		, m_distPort( "Distortion", *this )
    {
		// get height and width attributes from ChessBoard node
		Graph::UTQLSubgraph::NodePtr pCbNode = pCfg->getNode( "ChessBoard" );
		pCbNode->getAttributeData( "chessBoardHeight", m_height );
		pCbNode->getAttributeData( "chessBoardWidth", m_width );
		
		if ( m_height <= 0 || m_width <= 0  )
			UBITRACK_THROW( "ChessBoard nodes has no valid chessBoardHeight, chessBoardWidth" );
		
		if ( pCbNode->hasAttribute( "xAxisLength" ) ) // overall size of xAxis
			pCbNode->getAttributeData( "xAxisLength", m_sizeX );
		m_sizeX /= ( m_width - 1 );
			
		if ( pCbNode->hasAttribute( "yAxisLength" ) ) // overall size of yAxis
			pCbNode->getAttributeData( "yAxisLength", m_sizeY );
		m_sizeY /= ( m_height - 1 );
		
		if ( pCbNode->hasAttribute( "chessBoardSize" ) ) // old parameters override new ones
		{
			pCbNode->getAttributeData( "chessBoardSize", m_sizeX );
			pCbNode->getAttributeData( "chessBoardSize", m_sizeY );
		}

		if ( m_sizeY < 0 || m_sizeX < 0 )
			UBITRACK_THROW( "ChessBoard node has no chessBoardSize, yAxisLength or xAxisLength attribute" );

		// TODO: get image width and height from some image edge
		pCfg->m_DataflowAttributes.getAttributeData( "imgHeight", m_imgHeight );
		pCfg->m_DataflowAttributes.getAttributeData( "imgWidth", m_imgWidth );

		m_corners = m_height*m_width;
		m_values = 0;
		
		// look for some flags which can be specified...
		if ( pCfg->m_DataflowAttributes.hasAttribute( "fixPrincipalPoint" ) ) // fixes principal point to centre of image plane
			if( pCfg->m_DataflowAttributes.getAttributeString( "fixPrincipalPoint" ) == "true" )
				m_flags |= CV_CALIB_FIX_PRINCIPAL_POINT;
				
		if ( pCfg->m_DataflowAttributes.hasAttribute( "noTangentialDistortion" ) ) // assumes no tangential distortion
			if( pCfg->m_DataflowAttributes.getAttributeString( "noTangentialDistortion" ) == "true" )
				m_flags |= CV_CALIB_ZERO_TANGENT_DIST;
		
		if ( pCfg->m_DataflowAttributes.hasAttribute( "fixAspectRatio" ) ) // fixes aspect ratio, such that fx/fy
			if( pCfg->m_DataflowAttributes.getAttributeString( "fixAspectRatio" ) == "true" )
				m_flags |= CV_CALIB_FIX_ASPECT_RATIO;
		
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		if( hasNewPush() )
			computeIntrinsic();
		m_intrPort.send(Measurement::Matrix3x3 ( t, intrinsic ) );
		m_distPort.send(Measurement::Vector4D ( t, distortion ) );
	}

	void computeIntrinsic()
	{
		m_values = m_inPort.get()->size();

		boost::scoped_array< float > imgPoints( new float[2*m_values*m_corners] );
		boost::scoped_array< float > objPoints( new float[3*m_values*m_corners] );
		boost::scoped_array< int > chessNumber( new int[ m_values ] );

		// copy image corners to big array
		for( int i = 0; i < m_values; ++i )
		{
			for( int j = 0; j < m_corners; ++j )
			{
				imgPoints[2*i*m_corners + 2*j] = static_cast< float > ( m_inPort.get()->at( i ).at( j )( 0 ) );
				imgPoints[2*i*m_corners + 2*j + 1] = static_cast< float> ( m_inPort.get()->at( i ).at( j )( 1 ) );
			}
		}

		// generate object points
		for( int i = 0; i < m_values; ++i )
			for ( int y = 0; y < m_height; ++y )
				for ( int x = 0; x < m_width; ++x )
				{
					unsigned j = ( i * m_corners + y * m_width + x ) * 3;
					objPoints[ j     ] = m_sizeX * x;
					objPoints[ j + 1 ] = m_sizeY * y;
					objPoints[ j + 2 ] = 0;
				}

		for( int i = 0; i < m_values; ++i )
			chessNumber[i] = m_corners;


		if( m_values > 1 )
		{
			CvMat object_points = cvMat ( m_corners*m_values, 3, CV_32F, objPoints.get() );
			CvMat image_points = cvMat ( m_corners*m_values, 2, CV_32F, imgPoints.get() );
			CvMat point_counts = cvMat( 1, m_values, CV_32S, chessNumber.get() );

			float intrVal[9];
			CvMat intrinsic_matrix = cvMat ( 3, 3, CV_32FC1, intrVal );

			float disVal[4];
			CvMat distortion_coeffs = cvMat( 4, 1, CV_32FC1, disVal );


			cvCalibrateCamera2(
							&object_points,
							&image_points,
							&point_counts,
							cvSize( m_imgWidth, m_imgHeight ),
							&intrinsic_matrix,
							&distortion_coeffs,
							NULL ,		//	NULL for no output
							NULL ,	//	NULL for no output
							m_flags );

			intrinsic( 0, 0 ) = static_cast< double >( intrVal[0] );
			intrinsic( 0, 1 ) = static_cast< double >( intrVal[1] );
			intrinsic( 0, 2 ) = -static_cast< double >( intrVal[2] );
			intrinsic( 1, 0 ) = static_cast< double >( intrVal[3] );
			intrinsic( 1, 1 ) = static_cast< double >( intrVal[4] );
			intrinsic( 1, 2 ) = -static_cast< double >( intrVal[5] );
			intrinsic( 2, 0 ) = 0.0;
			intrinsic( 2, 1 ) = 0.0;
			intrinsic( 2, 2 ) = -1.0;

			distortion( 0 ) = static_cast< double >( disVal[0] );
			distortion( 1 ) = static_cast< double >( disVal[1] );
			distortion( 2 ) = static_cast< double >( disVal[2] );
			distortion( 3 ) = static_cast< double >( disVal[3] );
		}
		else UBITRACK_THROW( "Illegal number of poses" );
    }

protected:
	/** defines the used image */
	int m_imgHeight;
	int	m_imgWidth;

	/** defines the used chessboard */
	int m_height;
	int m_width;
	float m_sizeX;
	float m_sizeY;
	int m_corners;

	/** number of used images */
	int m_values;
	
	/** signs constraints for the calibration, look at OpenCV Doku for explanations */
	int m_flags;

	/** currently computed intrinsic values. */
	Math::Matrix< double, 3, 3 > intrinsic;
	Math::Vector< double, 4 > distortion;

	/** Input port of the component. */
	Dataflow::ExpansionInPort< std::vector < Math::Vector< double, 2 > > > m_inPort;

	/** Output ports of the component. */
	Dataflow::TriggerOutPort< Measurement::Matrix3x3 > m_intrPort;
	Dataflow::TriggerOutPort< Measurement::Vector4D > m_distPort;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< IntrinsicComponent > ( "IntrinsicCalibration" );
}

} } // namespace Ubitrack::Components

