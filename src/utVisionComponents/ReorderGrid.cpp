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
 */
#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMath/Vector.h>
#include <utMath/Graph/Munkres.h>
#include <utCalibration/Homography.h>

#include <fstream>
#include <sstream>
#include <boost/scoped_array.hpp>

#include <utVision/Image.h>
#include <opencv/cv.h>

namespace ublas = boost::numeric::ublas;

namespace Ubitrack { namespace Vision {

/**
 * @ingroup vision_components
 * ReorderGrid component.
 * This class contains a interprets a number of 2D Points as Grid and reorders them in an arranged, constant form.
 *
 * @par Input Ports
 * PushConsumer< Measurement::PositionList2 > of name "Input"
 *
 * @par Output Ports
 * PushSupplier<Measurement::PositionList2> with name "Output".
 *
 * @par Configuration
 * \c gridHeight: number of edges the grid has (height). 
 * \c gridWidth: number of edges the grid has (width).
  *
 * @par Operation
 * The component returns a vector of 2D coordinates which represents the edges of an given grid
 *
 * @par Example Configuration
 * \verbatim
 <Pattern name="ReorderGrid" id="ReorderGrid1">
	<Input>
		<Node name="Camera" id="Camera"/>
		<Node name="Grid" id="Grid">
			<Attribute name="gridHeight" value="4"/>
			<Attribute name="gridWidth" value="4"/>
		</Node>
		<Node name="ImagePlane" id="ImagePlane"/>
		<Edge name="Input" source="Camera" destination="ImagePlane" pattern-ref="..." edge-ref="..."/>
	</Input>
	
	<Output>
		<Edge name="Output" source="Camera" destination="ImagePlane"/>
	</Output>
	
	<DataflowConfiguration>
		<UbitrackLib class="ReorderGrid"/>
	</DataflowConfiguration>
</Pattern>
\endverbatim
 */
class ReorderGridComponent
	: public Dataflow::Component
{
public:
	/**
	 * Standard component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	ReorderGridComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
		: Dataflow::Component( sName )
		, m_height( -1 )
		, m_width( -1 )
		, m_inPort( "Input", *this, boost::bind( &ReorderGridComponent::pushPoints, this, _1 ) )
		, m_outPort( "Output", *this )
    {

		// get height and width attributes from ChessBoard node
		Graph::UTQLSubgraph::NodePtr pCbNode = pCfg->getNode( "Grid" );
		pCbNode->getAttributeData( "gridHeight", m_height );
		pCbNode->getAttributeData( "gridWidth", m_width );
		
		m_edges = m_height * m_width;

		for(int i=0; i < m_edges ;i++)
		{
			objPoints.push_back( Math::Vector< double, 2 >( double( i % m_height), double( i / m_height) ) );
		}
		
		if ( m_height < 0 || m_width < 0 )
			UBITRACK_THROW( "Grid nodes has no gridHeight or gridWidth attribute" );
    }

	/** computes the angle */ 
	double theta( const Math::Vector< double, 2 > & p1, const Math::Vector< double, 2 > & p2 )
	{
		Math::Vector< double, 2 > dist = p2 - p1;
		double ax = abs( dist( 0 ) );
		double ay = abs( dist( 1 ) );
		double t = 0.0;

		if( ax + ay  == 0.0 )
			return 0.0;
		else t = dist( 1 ) / ( ax + ay );

		if( dist( 0 ) < 0.0 )
			t = 2 - t;
		else if( dist( 1 ) < 0.0 )
			t = 4 + t;
		return t * 90.0;
	}

	/** computes the convex hull*/
	unsigned int wrap( std::vector< Math::Vector< double, 2 > > & pts )
	{
		Math::Vector< double, 2 > temp;
		unsigned int min = 0;
		unsigned int m = 0;
		unsigned int n = pts.size();
		double minangle = 0.0;
		double v;
		
		for( unsigned int i = 1; i < pts.size(); i++ )
		{
			if( pts.at( i )( 1 ) < pts.at( min )( 1 ) )
				min = i;
		}

		pts.push_back( pts.at( min ) );

		while( min != n )
		{
			m++;
			temp = pts.at( m - 1  );
			pts.at( m - 1 ) = pts.at( min );
			pts.at( min ) = temp;

			min = n;
			v = minangle;
			minangle = 360.0;

			for( unsigned int i = m; i < n+1; i++ )
			{
				double w = theta( pts.at( m - 1 ), pts.at( i ) );
				if( w > v && w < minangle )
				{
					min = i;
					minangle = w;
				}
			}
		}
		pts.pop_back();

		return m;
	}

	/** computes the angle described by three points*/
	double radiant( const Math::Vector< double, 2 > & x, const Math::Vector< double, 2 > & y, const Math::Vector< double, 2 > & z )
	{
		double val = ublas::norm_2( z - y );
		double val2 = ublas::norm_2( x - y );
		if( val != 0 && val2 != 0)
		{
			val = ublas::inner_prod( ( z - y ) / val, ( x - y ) / val2 );
			if( val < 0 )
				return -val;
			else return val;
		}
		return 1;

	}

	/** computes the projected points*/ 
	std::vector< Math::Vector< double, 2 > > getProjectedPoints( const std::vector< Math::Vector< double, 2 > > & pts, const unsigned int m )
	{
		Math::Matrix< double, 3, 3 > H;

		std::vector< Math::Vector< double, 2 > > from;
		std::vector< Math::Vector< double, 2 > > to;
		std::vector< Math::Vector< double, 2 > > projected;

		Math::Vector< double, 2 > temp;

		temp( 0 ) = 0.0;
		temp( 1 ) = 0.0;
		to.push_back( temp );

		temp( 0 ) = double( m_width - 1 );
		temp( 1 ) = 0.0;
		to.push_back( temp );

		temp( 0 ) = double( m_width - 1 );
		temp( 1 ) = double( m_height - 1 );
		to.push_back( temp );

		temp( 0 ) = 0.0;
		temp( 1 ) = double( m_height - 1 );
		to.push_back( temp );

		if( m < 4 )
		{
			return projected;
		}
		else if( m == 4 )
		{
			from.push_back( pts.at( 0 ) );
			from.push_back( pts.at( 1 ) );
			from.push_back( pts.at( 2 ) );
			from.push_back( pts.at( 3 ) );
		}
		else	//find 4 points with angle about 90 degrees
		{
			bool* picked = new bool[ m ];

			//for initialization
			for( unsigned int i=0; i< m; i++ )
			{
				picked[i] = false;
			}

			for( unsigned int i=0; i < 4; i++ )
			{
				//set to any (not picked) val
				int index = 0;
				while( picked[index] )
				{
					index++;
				}
				double min = 1.0;
				
				//look for smallest angle
				for( unsigned int j=m; j< 2*m; j++ )
				{
					if( !picked[ j % m ] )
					{
						double rad = radiant( pts.at( ( j - 1 ) % m ), pts.at( j % m ), pts.at( ( j + 1 ) % m ) );

						if( rad < min )
						{
							min = rad;
							index = j % m;
						}
					}
				}

				picked[ index ] = true;
			}

			//pick those 4 edge points in their oder
			for( unsigned int i=0; i< m; i++ )
			{
				if( picked[i] )
					from.push_back( pts.at( i ) );
			}	
		}			

		H = Calibration::homographyDLT( from, to );
		
		for( unsigned int i=0; i < pts.size(); i++ )
		{
			Math::Vector< double, 3 > p( pts.at( i )( 0 ), pts.at( i )( 1 ), 1.0 );
			p = ublas::prod( H, p );
			projected.push_back( Math::Vector< double, 2 >( p( 0 ) / p ( 2 ), p( 1 ) / p( 2 ) ) );
		}

		return projected;
	}
	
	/** computes the correspondences*/
	std::vector< std::size_t > getCorrespondences( std::vector< Math::Vector< double, 2 > > & from, std::vector< Math::Vector< double, 2 > > & to )
	{
		std::size_t fSize = from.size();
		std::size_t tSize = to.size();
		
		Math::Matrix< double, 0, 0 > matrix( fSize, tSize );

		for( std::size_t row=0; row<fSize; row++ )
		{
			for( std::size_t col=0; col<tSize; col++ )
			{
				matrix( row, col ) = ublas::norm_2( Math::Vector< double, 2 >( from.at(row) ) - to.at(col) );
			}
		}

		Math::Graph::Munkres< double > m( matrix );
		m.solve();

		//match for constructed chessboard
		return m.getColMatchList();
	}

	/** Method that computes the result. */
	void pushPoints( const Measurement::PositionList2 pm )
	{
		if( ( *pm ).size() == (unsigned int)m_edges )
		{
			std::vector< Math::Vector< double, 2 > > pts = *pm;

			std::vector< Math::Vector< double, 2 > > lol;

			//sort in a way, that the hull points are at the beginning
			unsigned int m = wrap( pts );

			//get the projected points
			std::vector< Math::Vector< double, 2 > > projected = getProjectedPoints( pts, m );

			//to find out, how to sort the new points
			std::vector< std::size_t > corr = getCorrespondences( projected, objPoints );

			std::vector< Math::Vector< double, 2 > > pts2;

			for( std::size_t i( 0 ); i < m_edges; ++i )
			{
				pts2.push_back( pts.at( corr.at( i ) ) );
			}

			//m_outPort.send(Measurement::PositionList2( pm.time(), projected ) );
			m_outPort.send(Measurement::PositionList2( pm.time(), pts2 ) );
		}
    }

protected:
	/** defines the chessboard to be tracked */
	int m_height;
	int m_width;
	int m_edges;
	std::vector< Math::Vector< double, 2 > > objPoints;
	/** Input port of the component. */
	Dataflow::PushConsumer< Measurement::PositionList2 > m_inPort;
	/** Output ports of the component. */
	Dataflow::PushSupplier< Measurement::PositionList2 > m_outPort;
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< ReorderGridComponent > ( "ReorderGrid" );
}

} } // namespace Ubitrack::Components

