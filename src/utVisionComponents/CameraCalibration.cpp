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
 * @author Christian Waechter <christian.waechter@cs.tum.edu>
 */

//std
#include <vector> 
#include <numeric>
#include <algorithm>

// OpenCV (for the calibration function)
#include <opencv/cv.h> 
#include <opencv2/calib3d/calib3d_c.h> 
#include <opencv/cv.hpp>

// Boost (for data preperation)
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>

// Ubitrack
#include <utUtil/OS.h>
#include <utMath/Vector.h>
#include <utMath/Matrix.h>
#include <utMath/CameraIntrinsics.h>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/ExpansionInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>

#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.CameraCalibration" ) );

using namespace cv;

namespace Ubitrack { namespace Vision {

class CameraCalibration
	: public Dataflow::TriggerComponent
{
protected:
	
	///shortcut for the type of 2d measurements
	typedef std::vector < Math::Vector< double, 2 > > vector_2d_type;
	
	///shortcut for the type of 3d measurements
	typedef std::vector < Math::Vector< double, 3 > > vector_3d_type;
	
	/** the intrinsic parameters, estimated so far */
	Math::CameraIntrinsics< double > m_camIntrinsics;
	
	/** signs constraints for the calibration, 
	please have a look at the OpenCV Doku for explanations */
	int m_flags;
	
	/** Input port of the component for 2D measurements. */
	Dataflow::ExpansionInPort< vector_2d_type > m_inPort2D;
	
	/** Input port of the component for 3D measurements. */
	Dataflow::ExpansionInPort< vector_3d_type > m_inPort3D;

	/** Output port of the component providing the camera intrinsic parameters. */
	Dataflow::TriggerOutPort< Measurement::CameraIntrinsics > m_intrPort;
	
	/** signs if the thread is running. */
	boost::mutex m_mutexThread;
	
	/** thread performing camera calibration in the background. */
	boost::scoped_ptr< boost::thread > m_pThread;

	int m_imgHeight;
	int m_imgWidth;
	bool m_useRationalModel;
	
public:
	/**
	 * Standard component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param cfg ComponentConfiguration containing all configuration.
	 */
	CameraCalibration( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pCfg )
		: Dataflow::TriggerComponent( sName, pCfg )
		, m_flags( 0 ) // for using 6 instead of 3 distortion parameters
		, m_useRationalModel(false)
		, m_imgWidth(640)
		, m_imgHeight(480)
		, m_inPort2D( "Points2D", *this )
		, m_inPort3D( "Points3D", *this )
		, m_intrPort( "CameraIntrinsics", *this )
		, m_mutexThread( )
    {
	


		// look for some flags which can be specified...

		if (pCfg->m_DataflowAttributes.hasAttribute("useRationalModel")) // fixes principal point to centre of image plane
			if (pCfg->m_DataflowAttributes.getAttributeString("useRationalModel") == "true"){
				m_flags = CV_CALIB_RATIONAL_MODEL;
				m_useRationalModel = true;
			}
				
		


		if ( pCfg->m_DataflowAttributes.hasAttribute( "fixPrincipalPoint" ) ) // fixes principal point to centre of image plane
			if( pCfg->m_DataflowAttributes.getAttributeString( "fixPrincipalPoint" ) == "true" )
				m_flags |= CV_CALIB_FIX_PRINCIPAL_POINT;
				
		if ( pCfg->m_DataflowAttributes.hasAttribute( "noTangentialDistortion" ) ) // assumes no tangential distortion
			if( pCfg->m_DataflowAttributes.getAttributeString( "noTangentialDistortion" ) == "true" )
				m_flags |= CV_CALIB_ZERO_TANGENT_DIST;
		
		if ( pCfg->m_DataflowAttributes.hasAttribute( "fixAspectRatio" ) ) // fixes aspect ratio, such that fx/fy
			if( pCfg->m_DataflowAttributes.getAttributeString( "fixAspectRatio" ) == "true" )
				m_flags |= CV_CALIB_FIX_ASPECT_RATIO;
		
		pCfg->m_DataflowAttributes.getAttributeData("imgHeight", m_imgHeight);
		pCfg->m_DataflowAttributes.getAttributeData("imgWidth", m_imgWidth);
		
		///@todo add some other flags, that are possible with newer versions of OpenCV
    }

	/** Method that computes the result. */
	void compute( Measurement::Timestamp t )
	{
		if( hasNewPush() )		
		{
			if( !boost::mutex::scoped_try_lock ( m_mutexThread ) )
			{
				LOG4CPP_WARN( logger, "Cannot perform camera calibration, thread is still busy, sending old calibration data." );
				return;
			}
			else
			{
				const std::vector< vector_2d_type >& points2D = *m_inPort2D.get();
				const std::vector< vector_3d_type >& points3D = *m_inPort3D.get();
				m_pThread.reset( new boost::thread( boost::bind( &CameraCalibration::computeIntrinsic, this, points3D, points2D )));
			}
		}
			
		// no need to send the old intrinsics, once the thread finishes the calculation it will push the result
		//m_intrPort.send( Measurement::CameraIntrinsics ( t, m_camIntrinsics ) );
	}

	void computeIntrinsic( const std::vector< vector_3d_type > points3D, const std::vector< vector_2d_type > points2D )
	{
		boost::mutex::scoped_lock lock( m_mutexThread );
		const std::size_t m_values = points2D.size();
		
		if( m_values < 2 )
		{
			// UBITRACK_THROW( "Cannot perform camera calibration, need at lest two different views." );
			LOG4CPP_ERROR( logger, "Cannot perform camera calibration, need at lest two different views." );
			return;
		}
			
		
		if( m_values != points3D.size() )
		{
			// UBITRACK_THROW( "Cannot perform camera calibration, number of views in 2D does not match number of 3D grids." );
			LOG4CPP_ERROR( logger, "Cannot perform camera calibration, number of views in 2D does not match number of 3D grids." );
			return;
		}	

		//count the number of available correspondences
		boost::scoped_array< int > chessNumber( new int[ m_values ] );
		
		
		std::vector< std::size_t > num_points;
		num_points.reserve( m_values );
		
		for( std::size_t i( 0 ); i < m_values; ++i )
		{
			const std::size_t n2D = points2D.at( i ).size();
			const std::size_t n3D = points3D.at( i ).size();
			if( n2D != n3D )
			{
				// UBITRACK_THROW( "Cannot perform camera calibration, number of corresponding 2D/3D measurements does not match." );
				LOG4CPP_ERROR( logger, "Cannot perform camera calibration, number of corresponding 2D/3D measurements does not match." );
				return;
			}
				
			num_points.push_back( n2D );
			chessNumber[ i ] = static_cast< int > ( n2D );
		}
		
		const std::size_t summe2D3D = std::accumulate( num_points.begin(), num_points.end(), 0 );
			
		
		// everything is fine so far, prepare copying the data		
		std::vector<std::vector<Point2f> > imgPoints(m_values);
		std::vector<std::vector<Point3f> > objPoints(m_values);
		

		// copy image corners to big array
		for( std::size_t i ( 0 ); i < m_values; ++i )
		{
			const std::size_t corners2D = points2D.at( i ).size();
			imgPoints[i].resize(corners2D);
			for( std::size_t j ( 0 ) ; j < corners2D; ++j )
			{
				imgPoints[i][j].x = static_cast< float > ( points2D.at( i ).at( j )( 0 ) );
				imgPoints[i][j].y = static_cast< float > ( points2D.at(i).at(j)(1));
			}
			
			const std::size_t corners3D = points3D.at( i ).size();
			objPoints[i].resize(corners3D);
			for( std::size_t j ( 0 ) ; j < corners3D; ++j )
			{
				objPoints[i][j].x = static_cast< float > ( points3D.at( i ).at( j )( 0 ) );
				objPoints[i][j].y = static_cast< float> (points3D.at(i).at(j)(1));
				objPoints[i][j].z = static_cast< float > (points3D.at(i).at(j)(2));
			}
		}

		
		Mat intrinsic_matrix = Mat::eye(3, 3, CV_64F);

		Mat distortion_coeffs;
		if (m_useRationalModel) {
			distortion_coeffs = Mat::zeros(8, 1, CV_64F);
		}
		else {
			distortion_coeffs = Mat::zeros(4, 1, CV_64F);
		}
			
		
		
		
		try
		{

			std::vector<Mat> rvecs, tvecs;
			Size imageSize = cv::Size(m_imgWidth, m_imgHeight);
			
			double reprojectionError = calibrateCamera(objPoints, imgPoints, imageSize, intrinsic_matrix, distortion_coeffs, rvecs, tvecs, m_flags);
			LOG4CPP_INFO(logger, "Final reprojeciton error:" << reprojectionError << " using " << m_values << " views");
		
		}
		catch( const std::exception & e )
		{
			LOG4CPP_ERROR( logger, "Cannot perform camera calibration, error in OpenCV function call.\n" << e.what() );
			return;
		}



		///@todo check if the paramrers should not be flipped, as it is done at some other places.
		Math::Matrix< double, 3, 3 > intrinsic;
		intrinsic(0, 0) = intrinsic_matrix.at<double>(0, 0);
		intrinsic(0, 1) = intrinsic_matrix.at<double>(0, 1);
		intrinsic(0, 2) = -intrinsic_matrix.at<double>(0, 2);
		intrinsic(1, 0) = intrinsic_matrix.at<double>(1, 0);
		intrinsic(1, 1) = intrinsic_matrix.at<double>(1, 1);
		intrinsic(1, 2) = -intrinsic_matrix.at<double>(1, 2);
		intrinsic(2, 0) = intrinsic_matrix.at<double>(2, 0);
		intrinsic(2, 1) = intrinsic_matrix.at<double>(2, 1);
		intrinsic(2, 2) = -intrinsic_matrix.at<double>(2, 2);

		if (m_useRationalModel) {			
			Math::Vector< double, 6 > radial;
			radial(0) = distortion_coeffs.at<double>(0);
			radial(1) = distortion_coeffs.at<double>(1);
			radial(2) = distortion_coeffs.at<double>(4);
			radial(3) = distortion_coeffs.at<double>(5);
			radial(4) = distortion_coeffs.at<double>(6);
			radial(5) = distortion_coeffs.at<double>(7);

			const Math::Vector< double, 2 > tangential(distortion_coeffs.at<double>(2), distortion_coeffs.at<double>(3));

			m_camIntrinsics = Math::CameraIntrinsics< double >(intrinsic, radial, tangential, (std::size_t)m_imgWidth, (std::size_t)m_imgHeight);
		}
		else {			
			Math::Vector< double, 2 > radial;
			radial(0) = distortion_coeffs.at<double>(0);
			radial(1) = distortion_coeffs.at<double>(1);

			const Math::Vector< double, 2 > tangential(distortion_coeffs.at<double>(2), distortion_coeffs.at<double>(3));

			m_camIntrinsics = Math::CameraIntrinsics< double >(intrinsic, radial, tangential, (std::size_t)m_imgWidth, (std::size_t)m_imgHeight);
		}
		
		m_intrPort.send( Measurement::CameraIntrinsics ( Measurement::now(), m_camIntrinsics ) );		
		LOG4CPP_INFO( logger, "Finished camera calibration using " << m_values << " views." );
    }
};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< CameraCalibration > ( "CameraCalibration" );
}

} } // namespace Ubitrack::Vision

