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

#define USE_UMAT
#ifdef USE_UMAT
#include <utVision/OpenCLManager.h>
#include <CL/cl_gl.h>
#endif

#define WITH_OPENGL_TEXTURE_UPDATE 1
#define WITH_DIRECTX11_TEXTURE_UPDATE 1

#ifdef WITH_DIRECTX11_TEXTURE_UPDATE
#include <d3d11.h>
#endif

#ifdef WITH_OPENGL_TEXTURE_UPDATE

#ifdef _WIN32
	#include "GL/freeglut.h"	
#elif __APPLE__
  #ifdef HAVE_FREEGLUT
        #include "GL/freeglut.h"
  #else
	#include <OpenGL/OpenGL.h>
	#include <GLUT/glut.h>
  #endif
#elif ANDROID
#include <GLES2/gl2.h>
#else
	#include <GL/glx.h>
#endif

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
      , m_logger( log4cpp::Category::getInstance( "Ubitrack.Components.TextureUpdate:" + subgraph->m_ID) )
	  , useOpenGL(true)
//	,m_lastUpdate(0)
	,rgbaImage()
#ifdef USE_UMAT
	  , m_clImageInitialized(false)
#endif
    {
		std::string frameworkPara;
		subgraph->m_DataflowAttributes.getAttributeData("Framework", frameworkPara);
		if (frameworkPara == "OpenGL")
			useOpenGL = true;
		else
			useOpenGL = false;

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
			Measurement::Position result(textureID, Math::Vector< double, 3 >(currentImage->width(),currentImage->height(),currentImage->channels()));
			return result;
		}

	
			LOG4CPP_DEBUG(m_logger, "receiveUpdateTexture start");
		{

			boost::mutex::scoped_lock l( m_mutex );


			if(currentImage.get() == NULL){
				LOG4CPP_WARN(m_logger, "receiveUpdateTexture: return, no new data");
				return Measurement::Position(0, Math::Vector< double, 3 >(0,0,0));
			}

			
			LOG4CPP_DEBUG(m_logger, "Update texture");
	
			if (useOpenGL)
				updateTextuteOpenGL(textureID);
#ifdef WITH_DIRECTX11_TEXTURE_UPDATE
			else
				updateTextureDirectX11(textureID);
#endif

			currentImage.reset();
		}
			/*
		
		*/

	
		return Measurement::Position(textureID, Math::Vector< double, 3 >(0,0,0));;

    }

  protected:
    
    boost::shared_ptr< Vision::Image >  currentImage;

    /** Input port of the component. */
    Dataflow::PushConsumer< Measurement::ImageMeasurement > m_inPort;
	
	Dataflow::PullSupplier< Measurement::Position > m_inPortTextureID;

	void updateTextuteOpenGL(Measurement::Timestamp textureID){
		boost::shared_ptr< Vision::Image > sourceImage;

		if (currentImage->channels() == 4){
			LOG4CPP_DEBUG(m_logger, "image correct channels");
			sourceImage = currentImage;			
		}
		else{
			LOG4CPP_DEBUG(m_logger, "convert else");
			if (rgbaImage.get() == 0){
				LOG4CPP_INFO(m_logger, "create buffer image");
				rgbaImage.reset(new Vision::Image(currentImage->width(), currentImage->height(), 4));

			}
			LOG4CPP_DEBUG(m_logger, "convert image");

#ifdef USE_UMAT
			cv::cvtColor(currentImage->uMat(), rgbaImage->uMat(), cv::COLOR_BGR2RGBA);
#else
			cvCvtColor(currentImage.get(), rgbaImage.get(), CV_BGR2RGBA);
#endif		
			
			sourceImage = rgbaImage;			
		}

		LOG4CPP_DEBUG(m_logger, "receiveUpdateTexture ID:" << textureID);
		glBindTexture(GL_TEXTURE_2D, (GLuint)textureID);

#ifdef USE_UMAT
		if( !m_clImageInitialized )
		{
			m_clImageInitialized = true;

			//define texture parameters like in BackgroundImage?
			cl_int err;
			Ubitrack::Vision::OpenCLManager& oclManager = Ubitrack::Vision::OpenCLManager::singleton();
			m_commandQueue = oclManager.getCommandQueue();
			m_clImage = clCreateFromGLTexture2D( oclManager.getContext(), CL_MEM_WRITE_ONLY, GL_TEXTURE_2D, 0, (GLuint) textureID, &err);
			if(err != CL_SUCCESS)
			{
				LOG4CPP_INFO( m_logger, "error at  clCreateFromGLTexture2D:" << err );
				m_clImageInitialized = false;
			}
		}

		glEnable(GL_TEXTURE_2D);
		cl_mem clBuffer = (cl_mem) sourceImage->uMat().handle(cv::ACCESS_READ);
	
		

		cl_int err;
		err = clEnqueueAcquireGLObjects(m_commandQueue, 1, &m_clImage, 0, NULL, NULL);
		if(err != CL_SUCCESS)
		{
			LOG4CPP_INFO( m_logger, "error at  clEnqueueAcquireGLObjects:" << err );
		}
		size_t offset = 0; 
		size_t dst_origin[3] = {0, 0, 0};
		size_t region[3] = {sourceImage->width(), sourceImage->height(), 1};

		if(sourceImage->uMat().isContinuous()){
			err = clEnqueueCopyBufferToImage(m_commandQueue, clBuffer, m_clImage, 0, dst_origin, region, 0, NULL, NULL);
		}
	
		if (err != CL_SUCCESS)
		{
			LOG4CPP_INFO( m_logger, "error at  clEnqueueCopyBufferToImage:" << err );
		}

		err = clEnqueueReleaseGLObjects(m_commandQueue, 1, &m_clImage, 0, NULL, NULL);
		if(err != CL_SUCCESS) 
		{
			LOG4CPP_INFO( m_logger, "error at  clEnqueueReleaseGLObjects:" << err );
		}

		err = clFinish(m_commandQueue);

		if (err != CL_SUCCESS)
		{
			LOG4CPP_INFO( m_logger, "error at  clFinish:" << err );
		}
		glDisable( GL_TEXTURE_2D );
#else
		if (sourceImage->depth() == IPL_DEPTH_32F) {
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, sourceImage->width(), sourceImage->height(),
				GL_RGBA, GL_FLOAT, sourceImage->iplImage()->imageData);
		}
		else if (sourceImage->depth() == IPL_DEPTH_8U) {
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, sourceImage->width(), sourceImage->height(),
				GL_RGBA, GL_UNSIGNED_BYTE, sourceImage->iplImage()->imageData);
		}
#endif
	}


#ifdef WITH_DIRECTX11_TEXTURE_UPDATE

	void updateTextureDirectX11(Measurement::Timestamp texturePtr){
		try{
			ID3D11Resource* textureResource = (ID3D11Resource*)texturePtr;
			ID3D11DeviceContext* ctx = NULL;
			ID3D11Device* device = NULL;
			
			textureResource->GetDevice(&device);
			if (device == NULL){
				LOG4CPP_ERROR(m_logger, "GetDevice == NULL");
				return;
			}
			device->GetImmediateContext(&ctx);
			if (ctx == NULL){
				LOG4CPP_ERROR(m_logger, "GetImmediateContext == NULL");
				return;
			}

			ID3D11Texture2D* d3dtex = (ID3D11Texture2D*)texturePtr;
			D3D11_TEXTURE2D_DESC desc;
			d3dtex->GetDesc(&desc);



			/*LOG4CPP_INFO(m_logger, "	desc.Usage : " << desc.Usage);
			LOG4CPP_INFO(m_logger, "	desc.Width : " << desc.Width << " : " <<currentImage->width);
			LOG4CPP_INFO(m_logger, "	desc.Height : " << desc.Height << " : " << currentImage->height);
			LOG4CPP_INFO(m_logger, "	desc.Format : " << DXGI_FORMAT_R32_SINT << " : " << desc.Format << " : " << currentImage->nChannels << " : " << currentImage->depth);
*/




			unsigned char* data = (unsigned char*)currentImage->iplImage()->imageData;
			boost::shared_ptr<D3D11_BOX> box;
			int bytePerRow=4;

			if (desc.Height == currentImage->height() && desc.Width == currentImage->width()) {
				//LOG4CPP_INFO(m_logger, "same width and height  ");
				
			} else{
				boost::shared_ptr< Vision::Image > sourceImage;
				
				bytePerRow = 4;
				box.reset(new D3D11_BOX());
					
				box->back = 1;
				box->front = 0;
				box->left = 0;
				box->right = sourceImage->width();
				box->bottom = 0;
				box->top = sourceImage->height();				
				//ctx->UpdateSubresource(d3dtex, 0, 0, data, sourceImage->width * 4, 0);
			}

			if ((desc.Format == DXGI_FORMAT_R32_SINT || desc.Format == DXGI_FORMAT_R32_TYPELESS) && currentImage->channels() == 1 && currentImage->depth() == IPL_DEPTH_32S) {
				bytePerRow = 4*currentImage->width();


				//LOG4CPP_INFO(m_logger, "same byte layout, nothing to do  ");



			}
			else if (currentImage->channels() == 3) {
				//LOG4CPP_DEBUG(m_logger, "convert else");
				if (rgbaImage.get() == 0){
					LOG4CPP_INFO(m_logger, "create buffer image");
					rgbaImage.reset(new Vision::Image(currentImage->width(), currentImage->height(), 4, 8, currentImage->origin()));

				}
				//LOG4CPP_DEBUG(m_logger, "convert image");
				cvCvtColor(currentImage.get(), rgbaImage.get(), CV_BGR2RGBA);
				data = (unsigned char*)rgbaImage->iplImage()->imageData;
				bytePerRow = 4 * currentImage->width();
			}

		

			//data = new unsigned char[desc.Width*desc.Height * 4];
			//FillTextureFromCode(desc.Width, desc.Height, desc.Width * 4, data);
			
			ctx->UpdateSubresource(d3dtex, 0, box.get(), data, bytePerRow, 0);
			
			//delete[] data;

			ctx->Release();
		}
		catch (...) {
			LOG4CPP_ERROR(m_logger, "exception");
			return;
		}
		
		
	}

	HRESULT UpdateSubresource_Workaround(
		ID3D11Device *pDevice,
		ID3D11DeviceContext *pDeviceContext,
		ID3D11Resource *pDstResource,
		UINT dstSubresource,
		const D3D11_BOX *pDstBox,
		const void *pSrcData,
		UINT srcBytesPerElement,
		UINT srcRowPitch,
		UINT srcDepthPitch,
		bool* pDidWorkAround)
	{
		HRESULT hr = S_OK;
		bool needWorkaround = false;
		D3D11_DEVICE_CONTEXT_TYPE contextType = pDeviceContext->GetType();

		if (pDstBox && (D3D11_DEVICE_CONTEXT_DEFERRED == contextType))
		{
			D3D11_FEATURE_DATA_THREADING threadingCaps = { FALSE, FALSE };

			hr = pDevice->CheckFeatureSupport(D3D11_FEATURE_THREADING, &threadingCaps, sizeof(threadingCaps));
			if (SUCCEEDED(hr))
			{
				if (!threadingCaps.DriverCommandLists)
				{
					needWorkaround = true;
				}
			}
		}

		const void* pAdjustedSrcData = pSrcData;

		if (needWorkaround)
		{
			D3D11_BOX alignedBox = *pDstBox;

			// convert from pixels to blocks
			if (false)
			{
				alignedBox.left /= 4;
				alignedBox.right /= 4;
				alignedBox.top /= 4;
				alignedBox.bottom /= 4;
			}

			pAdjustedSrcData = ((const BYTE*)pSrcData) - (alignedBox.front * srcDepthPitch) - (alignedBox.top * srcRowPitch) - (alignedBox.left * srcBytesPerElement);
		}

		pDeviceContext->UpdateSubresource(pDstResource, dstSubresource, pDstBox, pAdjustedSrcData, srcRowPitch, srcDepthPitch);

		if (pDidWorkAround)
		{
			*pDidWorkAround = needWorkaround;
		}

		return hr;
	}

#endif
    /** log4cpp logger reference */
    log4cpp::Category& m_logger;	
	
	boost::mutex m_mutex;
	Measurement::Timestamp m_lastUpdate;
	boost::shared_ptr< Vision::Image >  rgbaImage;
	bool useOpenGL;

#ifdef USE_UMAT
	cl_mem m_clImage;
	cl_command_queue m_commandQueue;
	bool m_clImageInitialized;
#endif

  };


  UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
    cf->registerComponent< TextureUpdateOpenGL >   ( "TextureUpdateOpenGL" ); 
  }

} } // namespace Ubitrack::Components
