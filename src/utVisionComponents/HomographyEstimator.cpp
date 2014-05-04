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
 * A component that estimates Homography
 *
 * @author Yuta Itoh <yuta.itoh@in.tum.de>
 */

#include <log4cpp/Category.hh>

#include <utVision/Image.h>
 
#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMath/Vector.h>
#include <utUtil/Exception.h>

#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.HomographyEstimator" ) );

namespace Ubitrack { namespace Vision {


/**
 * @ingroup vision_components
 * @file
 * A component that estimate homography from 2 projection matrices and a plane which is recieved as a "6DoF pose".
 * The plane pose is defined on the source coordinate system.
 *
 * @author Yuta Itoh <yuta.itoh@in.tum.de>
 * @date 2014
 */
class HomographyEstimator
	: public Dataflow::Component
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	HomographyEstimator( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::Component( sName )
		, m_inPlanePosePort(  "InputPlanePose", *this, boost::bind( &HomographyEstimator::compute, this, _1 ) )
		, m_inRelativePosePort("InputRelative", *this )
		, m_inIntrinsicSrcPort("InputIntrinsicSrc", *this )
		, m_inIntrinsicDstPort("InputIntrinsicDst", *this )
		, m_outHomographyPort( "OutputHomography", *this )
	{
		pConfig->m_DataflowAttributes.getAttributeData( "target_width", m_w );
		pConfig->m_DataflowAttributes.getAttributeData( "target_height", m_h );
		m_size.width  = m_w;
		m_size.height = m_h;
	}

	void computeHomographyFrom2PAndPlane(const cv::Matx34d P1, const cv::Matx34d P2, const cv::Matx14d &plane,
		cv::Matx33d &H)
	{
		// Search null space of the plane (4x3 matrix)
		cv::SVD svd(plane,cv::SVD::FULL_UV);
		cv::Mat nL0 = svd.vt.rowRange(cv::Range(1,4));
		nL0=nL0.t();
		cv::Matx43d nL=nL0;  // the null space found (4x3 matrix)

		cv::Matx33d tmp = P1*nL;
		H = P2*nL*tmp.inv();
	}

	/** Method that computes the result. */
	void compute(const Measurement::Pose& pose )
	{

		LOG4CPP_DEBUG( logger, "Estimate Homography" );

		// Intrinsic matrices
		const Ubitrack::Math::Matrix3x3  &Ka = m_inIntrinsicSrcPort.get(pose.time())->content();
		const Ubitrack::Math::Matrix3x3  &Kb = m_inIntrinsicDstPort.get(pose.time())->content();
		// 6DoF pose to the plane in the source coordinate system
		Ubitrack::Math::Matrix3x3  Rp;
		pose->rotation().toMatrix(Rp); /// This operation accumrates numerical error about 0.5 %
		const Ubitrack::Math::Vector<3, double> &tp = pose->translation();
		// 6DoF pose from the source view camera to the target 
		Ubitrack::Math::Matrix3x3  R;
		m_inRelativePosePort.get(pose.time())->rotation().toMatrix(R); /// This operation accumrates numerical error about 0.5 %
		const Ubitrack::Math::Vector<3, double> &t = m_inRelativePosePort.get(pose.time())->translation();
		
		// Plane normal vector ([R_MW(:,3);-R_MW(:,3)'*t_MW]')
		cv::Matx14d plane( Rp(2,0),Rp(2,1),Rp(2,2), -(Rp(2,0)*tp[0]+Rp(2,1)*tp[1]+Rp(2,2)*tp[2]) );

		/// Compute a projection matrix of the source camera
		/// Note the conversion of the data row/column major
		const cv::Matx34d P1( 
			Ka(0,0), Ka(1,0),Ka(2,0), 0.0,
			Ka(0,1),-Ka(1,1),Ka(2,1), 0.0, ///  -fy --> fy
			Ka(0,2), Ka(1,2),Ka(2,2), 0.0
			);

		/// Compute a projection matrix of the distination camera
		/// Note the conversion of the data row/column major
		const cv::Matx33d Kb33d( 
			Kb(0,0), Kb(1,0),Kb(2,0),
			Kb(0,1),-Kb(1,1),Kb(2,1), ///  -fy --> fy
			Kb(0,2), Kb(1,2),Kb(2,2)
			);
		R = boost::numeric::ublas::trans(R);
		const Ubitrack::Math::Vector<3, double> t2 = -boost::numeric::ublas::prod(R,t);
		R = boost::numeric::ublas::trans(R); /// This makes the result correct, but Why????
		/// Note the conversion of the data row/column major
		const cv::Matx34d P( 
			R(0,0),R(1,0),R(2,0), t2[0],
			R(0,1),R(1,1),R(2,1), t2[1],
			R(0,2),R(1,2),R(2,2), t2[2]
			);
		const cv::Matx34d P2=Kb33d*P;


		/// P1, P2, and plane are defined in the same coordinate system--that of the source
		cv::Matx33d H;
		computeHomographyFrom2PAndPlane( P1, P2, plane, H);
		Ubitrack::Math::Matrix3x3 H0;
		for( int i=0;i<3;i++)
			for( int j=0;j<3;j++)
				H0(i,j)=H(j,i);// row major --> column major
		
#if 0
		std::cout<<std::endl;
		cv::Mat plane_(plane);
		std::cout<<"plane_"<<std::endl;
		std::cout<<plane_<<std::endl;

		cv::Mat P1_(P1);
		std::cout<<"P1_"<<std::endl;
		std::cout<<P1_<<std::endl;
		
		cv::Mat K_(Kb33d);
		std::cout<<"K_"<<std::endl;
		std::cout<<K_<<std::endl;

		cv::Mat P_(P);
		std::cout<<"P_"<<std::endl;
		std::cout<<P_<<std::endl;

		cv::Mat P2_(P2);
		std::cout<<"P2_"<<std::endl;
		std::cout<<P2_<<std::endl;

		cv::Mat H_(H);
		std::cout<<"H_"<<std::endl;
		std::cout<<H_<<std::endl;
		std::cout<<std::endl;
#endif

		m_outHomographyPort.send( Measurement::Matrix3x3( pose.time(), H0 ) );
	}

protected:
	unsigned int m_w; // Width of the warped image
	unsigned int m_h; // Height of the warped image
	cv::Size2i m_size;
	// the ports
	Dataflow::PushConsumer< Measurement::Pose >       m_inPlanePosePort;    // 6DoF poose to the plane
	Dataflow::PullConsumer< Measurement::Pose >       m_inRelativePosePort;	// 6DoF poose to the target view camera
	Dataflow::PullConsumer< Measurement::Matrix3x3 >  m_inIntrinsicSrcPort;
	Dataflow::PullConsumer< Measurement::Matrix3x3 >  m_inIntrinsicDstPort;
	Dataflow::PushSupplier< Measurement::Matrix3x3 >  m_outHomographyPort;

};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	// Pose * Pose = Pose
	cf->registerComponent< Ubitrack::Vision::HomographyEstimator > ( "HomographyEstimator" );

}


} } // namespace Ubitrack::Components
