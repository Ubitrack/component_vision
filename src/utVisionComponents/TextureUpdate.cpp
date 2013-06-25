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
 * @ingroup dataflow_components
 * @file
 * Components that modifies timestamp
 *
 * @author Michael Schlegel <schlegem@in.tum.de>
 */

#include <log4cpp/Category.hh>

#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utVision/Image.h>
#include <opencv/cv.h>

#ifdef _WIN32
	#include "GL/freeglut.h"	
#elif __APPLE__
	#include <OpenGL/OpenGL.h>
	#include <GLUT/glut.h>
#elif ANDROID
#include <GLES2/gl2.h>
#else
	#include <GL/glx.h>
#endif


namespace Ubitrack { namespace Components {

  /**
   * @ingroup dataflow_components
   * This component adds a temporal offset to pushed measurements. 
   * It can be used to delay a measurement by the specified amount of time.
   *
   * @par Input Ports
   * PushConsumer<Measurement> with name "Input".
   *
   * @par Output Ports
   * PushSupplier<Measurement> with name "Output".
   *
   */  
  class TextureUpdateOpenGL
    : public Dataflow::Component
  {
  public:
    /**
     * UTQL component constructor.
     *
     * @param sName Unique name of the component.
     * @param subgraph UTQL subgraph
     */
    TextureUpdateOpenGL( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph  )
      : Dataflow::Component( sName )      
      , m_inPort( "Input", *this, boost::bind( &TextureUpdateOpenGL::receiveImage, this, _1 ) )
	  , m_inPortTextureID( "Input", *this, boost::bind( &TextureUpdateOpenGL::receiveUpdateTexture, this, _1 ) )      
      , m_logger( log4cpp::Category::getInstance( "Ubitrack.Components.TextureUpdate" ) )
    {
      		

		//LOG4CPP_INFO( m_logger, "Setting delay time" << m_delayTime );
    }

    /** Method that computes the result. */
    void receiveImage( const Measurement::ImageMeasurement& image )
    {
		boost::mutex::scoped_lock l( m_mutex );
		currentImage = image;
    }
	
	void receiveUpdateTexture( const Measurement::Button& textureID )
    {
		boost::shared_ptr< Vision::Image > rgbaImage;
		{
			boost::mutex::scoped_lock l( m_mutex );
			
			rgbaImage = currentImage->CvtColor( CV_RGB2RGBA, 4, IPL_DEPTH_8U );
		}
		glBindTexture( GL_TEXTURE_2D, (GLuint) textureID.get() );
		glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, rgbaImage->width, rgbaImage->height, 
			GL_RGBA, GL_UNSIGNED_BYTE, rgbaImage->imageData );
    }

  protected:
    
    Measurement::ImageMeasurement currentImage;

    /** Input port of the component. */
    Dataflow::PushConsumer< Measurement::ImageMeasurement > m_inPort;
	
	Dataflow::PushConsumer< Measurement::Button > m_inPortTextureID;


    /** log4cpp logger reference */
    log4cpp::Category& m_logger;	
	
	boost::mutex m_mutex;

  };


  UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
    cf->registerComponent< TextureUpdateOpenGL >   ( "TextureUpdateOpenGL" ); 
  }

} } // namespace Ubitrack::Components
