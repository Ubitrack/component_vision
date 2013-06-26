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
#include <utDataflow/PullSupplier.h>
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
	  , m_inPortTextureID( "InputTextureID", *this, boost::bind( &TextureUpdateOpenGL::receiveUpdateTexture, this, _1 ) )      
      , m_logger( log4cpp::Category::getInstance( "Ubitrack.Components.TextureUpdate" ) )
//	,m_lastUpdate(0)
	,rgbaImage(new Vision::Image(320,240,4))
    {
      		

		//LOG4CPP_INFO( m_logger, "Setting delay time" << m_delayTime );
    }

    /** Method that computes the result. */
    void receiveImage( const Measurement::ImageMeasurement& image )
    {
		LOG4CPP_INFO(m_logger, "receiveImage");
		boost::mutex::scoped_lock l( m_mutex );
		currentImage = image;
		LOG4CPP_INFO(m_logger, "receiveImage:"<<currentImage.get());
    }
	
	Measurement::Position receiveUpdateTexture( Measurement::Timestamp textureID )
    {
		
		Measurement::Position b(textureID, Math::Vector<3>(0,0,0));
			LOG4CPP_INFO(m_logger, "receiveUpdateTexture");
		{

			boost::mutex::scoped_lock l( m_mutex );
			
			LOG4CPP_INFO(m_logger, "receiveUpdateTexture:"<<currentImage.get());
			if(currentImage.get() == NULL){
				LOG4CPP_INFO(m_logger, "receiveUpdateTexture: return, no new data");
				return b;
			}

	
			cvCvtColor(currentImage.get(), rgbaImage.get(), CV_RGB2RGBA);
			currentImage.reset();

		}
		LOG4CPP_INFO(m_logger, "receiveUpdateTexture ID:"<<textureID);
		glBindTexture( GL_TEXTURE_2D, (GLuint) textureID );
		glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, rgbaImage->width, rgbaImage->height, 
			GL_RGBA, GL_UNSIGNED_BYTE, rgbaImage->imageData );
		LOG4CPP_ERROR(m_logger, "value"<<rgbaImage->imageData[4960]);
		return b;

    }

  protected:
    
    boost::shared_ptr< Vision::Image >  currentImage;

    /** Input port of the component. */
    Dataflow::PushConsumer< Measurement::ImageMeasurement > m_inPort;
	
	Dataflow::PullSupplier< Measurement::Position > m_inPortTextureID;


    /** log4cpp logger reference */
    log4cpp::Category& m_logger;	
	
	boost::mutex m_mutex;
	Measurement::Timestamp m_lastUpdate;
	boost::shared_ptr< Vision::Image >  rgbaImage;

  };


  UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
    cf->registerComponent< TextureUpdateOpenGL >   ( "TextureUpdateOpenGL" ); 
  }

} } // namespace Ubitrack::Components
