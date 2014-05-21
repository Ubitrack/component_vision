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
 * Warp a source image by Homography matrix
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

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.ImageWarpingByHomography" ) );

namespace Ubitrack { namespace Vision {


/**
 * @ingroup vision_components
 * @file
 * A component that warp a source image by 3x3 homography matrix
 *
 * @author Yuta Itoh <yuta.itoh@in.tum.de>
 * @date 2014
 */
class ImageWarpingByHomography
	: public Dataflow::Component
{
public:
	/**
	 * UTQL component constructor.
	 *
	 * @param sName Unique name of the component.
	 * @param subgraph UTQL subgraph
	 */
	ImageWarpingByHomography( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::Component( sName )
		, m_w(640)
		, m_h(480)
		, m_inImagePort( "InputImage", *this, boost::bind( &ImageWarpingByHomography::compute, this, _1 ) )
		, m_outImagePort("OutputImage", *this )
		, m_inHomographyPort( "InputHomography", *this )
	{
		pConfig->m_DataflowAttributes.getAttributeData( "target_width", m_w );
		pConfig->m_DataflowAttributes.getAttributeData( "target_height", m_h );
		m_size.width  = m_w;
		m_size.height = m_h;
	}

	/** Method that computes the result. */
	void compute(const Measurement::ImageMeasurement& img )
	{

		// Pull a 3x3 homography matrix
		const Ubitrack::Math::Matrix3x3d  &H0 = m_inHomographyPort.get(img.time())->content();
		cv::Matx33d H( H0(0,0),H0(1,0),H0(2,0),
			H0(0,1),H0(1,1),H0(2,1),
			H0(0,2),H0(1,2),H0(2,2)
			);

		LOG4CPP_DEBUG( logger, "Create a warped iamge" );
		int nChannels = 3;
		if(img->nChannels == 1)
		{
			nChannels = 1;
		}
		boost::shared_ptr< Image > pImage( new Image( m_w, m_h, nChannels ) );

		cv::Mat dst( *pImage );
		cv::Mat src( *img    );

#if 0
		//		cv::imshow("ImageWarp src",src);
		//		cv::imshow("ImageWarp dst",dst);
		cv::waitKey(5);
		std::cout<<std::endl;
		cv::Mat H_(H);
		std::cout<<"H used in the image warping:"<<std::endl;
		std::cout<<H_<<std::endl;
		std::cout<<"width : "<< dst.cols <<std::endl;
		std::cout<<"height: "<< dst.rows <<std::endl;
		std::cout<<std::endl;
#endif

		H = H.t();
		cv::flip(src,src,0); /// Origin convertin between Ubitrack and Opencv
		cv::warpPerspective( src, dst, H, m_size );
		cv::flip(src,src,0);
		cv::flip(dst,dst,0); /// OpenCV --> Ubitrack

		pImage->origin = img->origin; //inserted by CW to have correct origin
		m_outImagePort.send( Measurement::ImageMeasurement( img.time(), pImage ) );
	}

protected:
	unsigned int m_w; // Width of the warped image
	unsigned int m_h; // Height of the warped image
	cv::Size2i m_size;
	// the ports
	Dataflow::PushConsumer< Measurement::ImageMeasurement > m_inImagePort;
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outImagePort;
	Dataflow::PullConsumer< Measurement::Matrix3x3 >  m_inHomographyPort;


};


UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	// Pose * Pose = Pose
	cf->registerComponent< Ubitrack::Vision::ImageWarpingByHomography > ( "ImageWarpingByHomography" );

}


} } // namespace Ubitrack::Components
