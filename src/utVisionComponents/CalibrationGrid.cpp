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
 * Generating 3d points of a planar grid like structure. Can be used
 * to generate correspondences for a calibration grid, like a chessboard.
 *
 * @author Christian Waechter <christian.waechter@cs.tum.edu>
 */
 
//std
#include <vector>

// Ubitrack
#include <utMath/Vector.h>
#include <utMeasurement/Measurement.h>
#include <utDataflow/Component.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/ComponentFactory.h>


namespace Ubitrack { namespace Vision {

class CalibrationGrid
	: public Dataflow::Component
{
protected:
	
	typedef double value_type;
	
	/** the object points describing the grid in a 3-dimensional Euclidean space */
	std::vector< Math::Vector< double, 3 > > m_objPoints;
	
	/** Output ports of the component. */
	Dataflow::PullSupplier< Measurement::PositionList > m_outPort;
	
public:
	/**
	 * Standard component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	CalibrationGrid( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
		: Dataflow::Component( sName )
		, m_outPort( "Output", *this, boost::bind( &CalibrationGrid::getGrid, this, _1 ) )
    {

		if( !pCfg->hasNode( "CalibrationGridPoints" ) )
			UBITRACK_THROW( "Cannot generate 3d points of a planaer calibration grid, Node \"CalibrationGridPoints\" was not specified.");
		
		Graph::UTQLSubgraph::NodePtr pCbNode = pCfg->getNode( "CalibrationGridPoints" );
		std::size_t width, height = 0;
		pCbNode->getAttributeData( "gridPointsX", width );
		pCbNode->getAttributeData( "gridPointsY", height );
		
		value_type m_sizeX, m_sizeY, lastDim = 0;
		pCbNode->getAttributeData( "gridAxisLengthX", m_sizeX );
		pCbNode->getAttributeData( "gridAxisLengthY", m_sizeY );
		pCbNode->getAttributeData( "gridThirdDimension", lastDim );
		
		bool symmetric ( true );
		if ( pCbNode->hasAttribute( "gridType" ) )
			if( pCbNode->getAttributeString( "gridType" ) == "asymmetric" )
				symmetric = false;

		reset< value_type >( width, height, m_sizeX, m_sizeY, lastDim, symmetric );
    }
	
	template< typename T >
	void reset( const std::size_t width, const std::size_t height, const T x_axis, const T y_axis, const T z, const bool symmetric )
	{
		const std::size_t n = width * height;
		if( n < 1 )
			UBITRACK_THROW( "Calibration grid dimension parameters are not set properly, height and width should be different from zero." );
		
		if( ( x_axis * y_axis ) == 0 )
			UBITRACK_THROW( "Calibration grid axis length parameters are not set properly, parameters should be different from zero." );
			
		const T size_x = x_axis / ( width - 1 );
		const T size_y = y_axis / ( height - 1 );
		//there should be an offset when the grid is asymmetric:
		const T asymm_offset_x = symmetric ? 0 : size_x * 0.5;
		
		m_objPoints.reserve( n );
		for( std::size_t i( 0 ); i < n ; ++i )
		{
			const T x = static_cast< T >( i % width ) * size_x;
			const T y  = static_cast< T >( i / width ) * size_y;
			if( ( ( i / width ) % 2 ) == 0 )
				m_objPoints.push_back( Math::Vector< double, 3 > ( x, y, z ) );
			else
				m_objPoints.push_back( Math::Vector< double, 3 > ( asymm_offset_x + x, y, z ) );
			
		}		
	}
	
	Measurement::PositionList getGrid( const Measurement::Timestamp t ) const
	{
		if( !m_objPoints.empty() )
			return Measurement::PositionList( t, m_objPoints );
		else
			UBITRACK_THROW( "Cannot provide calibration grid coordinates, grid was not initialized correctly." );
	}


};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< CalibrationGrid > ( "CalibrationGrid" );
}

} } // namespace Ubitrack::Vision

