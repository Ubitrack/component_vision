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
 * Components that updates a texture using opengl
 *
 * @author Frieder Pankratz <pankratz@in.tum.de>
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
	,rgbaImage()
    {
      		

		//LOG4CPP_INFO( m_logger, "Setting delay time" << m_delayTime );
    }

    /** Method that computes the result. */
    void receiveImage( const Measurement::ImageMeasurement& image )
    {
		LOG4CPP_DEBUG(m_logger, "receiveImage start");
		boost::mutex::scoped_lock l( m_mutex );
		currentImage = image;
		LOG4CPP_DEBUG(m_logger, "receiveImage:"<<currentImage.get());
    }
	
	Measurement::Position receiveUpdateTexture( Measurement::Timestamp textureID )
    {
		


		if( textureID == 0 && currentImage.get() != NULL){
			Measurement::Position result(textureID, Math::Vector< double, 3 >(currentImage->width,currentImage->height,currentImage->nChannels));
			return result;
		}

		boost::shared_ptr< Vision::Image > sourceImage;
			LOG4CPP_DEBUG(m_logger, "receiveUpdateTexture start");
		{

			boost::mutex::scoped_lock l( m_mutex );
			if(currentImage.get() == NULL){
				LOG4CPP_WARN(m_logger, "receiveUpdateTexture: return, no new data");
				return Measurement::Position(0, Math::Vector< double, 3 >(0,0,0));
			}

			LOG4CPP_DEBUG(m_logger, "receiveUpdateTextureASDF:"<<currentImage.get());
			if(currentImage->nChannels == 4){
				LOG4CPP_DEBUG(m_logger, "image correct channels");
				sourceImage = currentImage;
				currentImage.reset();
			}
			else{
				LOG4CPP_DEBUG(m_logger, "convert else");
				if(rgbaImage.get() == 0){
					LOG4CPP_INFO(m_logger, "create buffer image");
					rgbaImage.reset(new Vision::Image(currentImage->width, currentImage->height, 4));
					
				}
				LOG4CPP_DEBUG(m_logger, "convert image");
				cvCvtColor(currentImage.get(), rgbaImage.get(), CV_BGR2RGBA);
				sourceImage = rgbaImage;
				currentImage.reset();
			}
			
			

	
			

		}
		LOG4CPP_DEBUG(m_logger, "receiveUpdateTexture ID:"<<textureID);
		glBindTexture( GL_TEXTURE_2D, (GLuint) textureID );
		glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, sourceImage->width, sourceImage->height, 
			GL_RGBA, GL_UNSIGNED_BYTE, sourceImage->imageData );		
		return Measurement::Position(textureID, Math::Vector< double, 3 >(0,0,0));;

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
