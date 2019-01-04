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

#include <utVision/OpenGLPlatform.h>

#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utVision/Image.h>
#include <opencv/cv.h>

// @todo should that be a compiletime option??

#ifdef HAVE_OPENCL

#ifdef WIN32
#define USE_UMAT
#endif


#ifdef USE_UMAT
#include <utVision/OpenCLManager.h>

#ifdef __APPLE__
    #include "OpenCL/cl_gl.h"
#else
    #include "CL/cl_gl.h"
	#ifdef WIN32
		#include <CL/cl_d3d11_ext.h>
	#endif
#endif

#endif

#define WITH_OPENGL_TEXTURE_UPDATE 1

// DirectX support is disabled for now
// specifically, because it requires NVIDIA OpenCL
// #ifdef _WIN32
// #define WITH_DIRECTX11_TEXTURE_UPDATE 1
// #endif

// #ifdef WITH_DIRECTX11_TEXTURE_UPDATE
// #include <d3d11.h>
// #endif

#include <opencv/highgui.h>


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
	,convertedImage()
#ifdef USE_UMAT
	  , m_clImageInitialized(false)

#endif
    {
		
		subgraph->getEdge("Input")->getAttributeData("channels", m_textureChannels);
		//imageformats like RGB are not supported and have to be converted to RGBA
		if(m_textureChannels == 3)
		{
			m_textureChannels = 4;
		}
		subgraph->getEdge("Input")->getAttributeData("imageWidth", m_textureWidth);
		subgraph->getEdge("Input")->getAttributeData("imageHeight", m_textureHeight);

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
		static int received = 0;

		boost::mutex::scoped_lock l( m_mutex );
		
		currentImage = image;

		m_textureWidth = currentImage->width();
		m_textureHeight = currentImage->height();
		m_textureChannels = currentImage->channels();

		//LOG4CPP_INFO(m_logger, "new Image with timestamp: " << image.time() );
		
    }
	
	Measurement::Position receiveUpdateTexture( Measurement::Timestamp textureID )
    {
		
		//LOG4CPP_INFO(m_logger, "receiveUpdateTexture TextureID: " << textureID);	

		if (currentImage.get() == NULL)
		{
			return Measurement::Position(0, Math::Vector< double, 3 >(0, 0, 0));
		}

		if( textureID == 0){
			LOG4CPP_INFO(m_logger, "no texture id provided ");
			Measurement::Position result(textureID, Math::Vector< double, 3 >(m_textureWidth, m_textureHeight, m_textureChannels));
			return result;
		}
		
//#ifdef USE_UMAT
#ifdef DONTUSE

		if( !m_clImageInitialized )
		{
			LOG4CPP_DEBUG(m_logger, "initializeing OCL Manager");
			m_clImageInitialized = true;

			//define texture parameters like in BackgroundImage?
			cl_int err;
			Ubitrack::Vision::OpenCLManager& oclManager = Ubitrack::Vision::OpenCLManager::singleton();
			
			
			if( useOpenGL )
			{
				oclManager.initializeOpenGL();
				m_clImage = clCreateFromGLTexture2D( oclManager.getContext(), CL_MEM_WRITE_ONLY, GL_TEXTURE_2D, 0, (GLuint) textureID, &err);
			} else
			{
				throw std::runtime_error("DirectX is currently not supported.");
				// ID3D11Texture2D* textureResource = (ID3D11Texture2D*) textureID;
				// D3D11_TEXTURE2D_DESC textureDesc;
				// textureResource->GetDesc(&textureDesc);
				// LOG4CPP_DEBUG(m_logger, "D3DX11Info: width: " << textureDesc.Width << " height: " << textureDesc.Height << " format: " << textureDesc.Format);
				

				// //ID3D11Resource* textureResource = (ID3D11Resource*)textureID;
				// ID3D11DeviceContext* ctx = NULL;
				// ID3D11Device* device = NULL;
			
				// textureResource->GetDevice(&device);
				// if (device == NULL){
				// 	LOG4CPP_ERROR(m_logger, "GetDevice == NULL");
				// }

				LOG4CPP_INFO(m_logger, "call initializeDirectX");
				oclManager.initializeDirectX(device);

			

				// clCreateFromD3D11Texture2DNV_fn clCreateFromD3D11Texture2DNV = (clCreateFromD3D11Texture2DNV_fn) clGetExtensionFunctionAddress("clCreateFromD3D11Texture2DNV");
				// //clCreateFromD3D11Texture2DKHR = (clCreateFromD3D11Texture2DNV_fn) clGetExtensionFunctionAddress("clCreateFromD3D11Texture2DKHR");
				// LOG4CPP_INFO(m_logger, "func pointer to clCreateFromD3D11Texture2DNV" << clCreateFromD3D11Texture2DNV);

				// ID3D11Texture2D* d3dtex = (ID3D11Texture2D*)textureID;
				// m_clImage = clCreateFromD3D11Texture2DNV( oclManager.getContext(), CL_MEM_WRITE_ONLY, d3dtex, 0, &err);
				
				// if(err != CL_SUCCESS){
				// 	m_clImageInitialized = false;
				// 	LOG4CPP_INFO(m_logger, "Error creating clImage from Texture: " <<err);
				// }else{
				// 	m_clImageInitialized = true;
				// 	LOG4CPP_INFO(m_logger, "success: created From D3DX11 Texture");
				// }

				// clEnqueueAcquireD3D11ObjectsNV = (clEnqueueAcquireD3D11ObjectsNV_fn) clGetExtensionFunctionAddress("clEnqueueAcquireD3D11ObjectsNV");
				// LOG4CPP_INFO(m_logger, "func pointer to clEnqueueAcquireD3D11ObjectsNV" << clCreateFromD3D11Texture2DNV);
				// if(!clEnqueueAcquireD3D11ObjectsNV)
				// {
				// 	LOG4CPP_ERROR(m_logger, "clEnqueueAcquireD3D11ObjectsNV NULL");
				// 	m_clImageInitialized = false;
				// }

				// clEnqueueReleaseD3D11ObjectsNV = (clEnqueueReleaseD3D11ObjectsNV_fn) clGetExtensionFunctionAddress("clEnqueueReleaseD3D11ObjectsNV");
				// LOG4CPP_INFO(m_logger, "func pointer to clEnqueueReleaseD3D11ObjectsNV" << clCreateFromD3D11Texture2DNV);
				// if(!clEnqueueReleaseD3D11ObjectsNV)
				// {
				// 	LOG4CPP_ERROR(m_logger, "clEnqueueReleaseD3D11ObjectsNV NULL");
				// 	m_clImageInitialized = false;
				// }
			}
			
			m_commandQueue = oclManager.getCommandQueue();
			if(err != CL_SUCCESS || !m_clImageInitialized)
			{
				LOG4CPP_INFO( m_logger, "error at  clCreateFromGL\\DirectXTexture2D:" << err );
				m_clImageInitialized = false;
				return Measurement::Position(0, Math::Vector< double, 3 >(0, 0, 0));
			}

			LOG4CPP_INFO(m_logger, "initializeing finished: " << oclManager.isInitialized() );
		}
#endif

		
			
		//LOG4CPP_INFO(m_logger, "receiveUpdateTexture start");
		{

			boost::mutex::scoped_lock l( m_mutex );


			if(currentImage.get() == NULL){
				LOG4CPP_WARN(m_logger, "receiveUpdateTexture: return, no new data");
				return Measurement::Position(0, Math::Vector< double, 3 >(0,0,0));
			}

			//LOG4CPP_DEBUG(m_logger, "Update texture");

		
			if (useOpenGL)
				updateTextuteOpenGL(textureID);
#ifdef WITH_DIRECTX11_TEXTURE_UPDATE
			else
				updateTextureDirectX11CPU(textureID);
				//updateTextureDirectX11(textureID);
#endif
		
			currentImage.reset();
		}

		
		return Measurement::Position( textureID, Math::Vector< double, 3 >(0, 0, 0) );

    }

  protected:
    boost::shared_ptr< Vision::Image >  currentImage;

    /** Input port of the component. */
    Dataflow::PushConsumer< Measurement::ImageMeasurement > m_inPort;
	
	Dataflow::PullSupplier< Measurement::Position > m_inPortTextureID;

	void updateTextuteOpenGL(Measurement::Timestamp textureID){
		boost::shared_ptr< Vision::Image > sourceImage;

		if (currentImage->channels() == 4 || currentImage->channels() == 1){
			LOG4CPP_DEBUG(m_logger, "image correct channels");
			sourceImage = currentImage;			
		}
		else{
			LOG4CPP_DEBUG(m_logger, "convert else");
			if (convertedImage.get() == 0){
				LOG4CPP_INFO(m_logger, "create buffer image");
				convertedImage.reset(new Vision::Image(currentImage->width(), currentImage->height(), 4));

			}
			LOG4CPP_DEBUG(m_logger, "convert image");
			
#ifdef USE_UMAT
			cv::cvtColor(currentImage->uMat(), convertedImage->uMat(), cv::COLOR_BGR2RGBA);
#else
			cvCvtColor(currentImage.get(), convertedImage.get(), CV_BGR2RGBA);
#endif		
			
			sourceImage = convertedImage;			
		}

		LOG4CPP_DEBUG(m_logger, "receiveUpdateTexture ID:" << textureID);
		glBindTexture(GL_TEXTURE_2D, (GLuint)textureID);

#ifdef USE_UMAT

		glEnable(GL_TEXTURE_2D);
		cl_mem clBuffer = (cl_mem) sourceImage->uMat().handle(cv::ACCESS_READ);
		
		LOG4CPP_INFO(m_logger, "starting aquire");
		cl_int err;
		err = clEnqueueAcquireGLObjects(m_commandQueue, 1, &m_clImage, 0, NULL, NULL);
		if(err != CL_SUCCESS)
		{
			LOG4CPP_INFO( m_logger, "error at  clEnqueueAcquireGLObjects:" << err );
			return;
		}
		size_t offset = 0; 
		size_t dst_origin[3] = {0, 0, 0};
		size_t region[3] = {sourceImage->width(), sourceImage->height(), 1};

		LOG4CPP_INFO(m_logger, "starting copy: "<< sourceImage->uMat().size().width << "x" << sourceImage->uMat().size().height << "x" << sourceImage->uMat().channels() );

		if(sourceImage->uMat().isContinuous()){
			err = clEnqueueCopyBufferToImage(m_commandQueue, clBuffer, m_clImage, 0, dst_origin, region, 0, NULL, NULL);
		}
	
		if (err != CL_SUCCESS)
		{
			LOG4CPP_INFO( m_logger, "error at  clEnqueueCopyBufferToImage:" << err );
			return;
		}
		LOG4CPP_INFO(m_logger, "starting release");
		err = clEnqueueReleaseGLObjects(m_commandQueue, 1, &m_clImage, 0, NULL, NULL);
		if(err != CL_SUCCESS) 
		{
			LOG4CPP_INFO( m_logger, "error at  clEnqueueReleaseGLObjects:" << err );
			return;
		}
		LOG4CPP_INFO(m_logger, "starting to finish");
		err = clFinish(m_commandQueue);

		if (err != CL_SUCCESS)
		{
			LOG4CPP_INFO( m_logger, "error at  clFinish:" << err );
			return;
		}
		LOG4CPP_INFO(m_logger, "CL done");
		glDisable( GL_TEXTURE_2D );
		LOG4CPP_INFO(m_logger, "return");
#else
		if (sourceImage->depth() == CV_32F) {
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, sourceImage->width(), sourceImage->height(),
				GL_RGBA, GL_FLOAT, sourceImage->Mat().data);
		}
		else if (sourceImage->depth() == CV_8U) {
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, sourceImage->width(), sourceImage->height(),
				GL_RGBA, GL_UNSIGNED_BYTE, sourceImage->Mat().data);
		}
#endif
	}

#ifdef WITH_DIRECTX11_TEXTURE_UPDATE
#ifdef USE_UMAT
void updateTextureDirectX11(Measurement::Timestamp texturePtr)
{
	LOG4CPP_DEBUG(m_logger, "updateTextureD11");
	boost::shared_ptr< Vision::Image > sourceImage;

	if (currentImage->channels() == 4){
		LOG4CPP_DEBUG(m_logger, "image correct channels");
		sourceImage = currentImage;			
	}
	else if(currentImage->channels() == 3){
		if (convertedImage.get() == 0){
			convertedImage.reset(new Vision::Image(currentImage->width(), currentImage->height(), 4));
		}
		LOG4CPP_DEBUG(m_logger, "convert image");
		cv::cvtColor(currentImage->uMat(), convertedImage->uMat(), cv::COLOR_BGR2RGBA);			
		sourceImage = convertedImage;
	}
	else if(currentImage->channels() == 1 && currentImage->uMat().type() != CV_32FC1)
	{
		if (convertedImage.get() == 0){
			LOG4CPP_INFO(m_logger, "create buffer image");
			convertedImage.reset(new Vision::Image(currentImage->width(), currentImage->height(), 1));
		}
		currentImage->uMat().convertTo(convertedImage->uMat(), CV_32FC1, 1.0/255.0);
		sourceImage = convertedImage;
	}
		
	cl_mem clBuffer = (cl_mem) sourceImage->uMat().handle(cv::ACCESS_READ);	
	ID3D11DeviceContext* ctx = NULL;
	ID3D11Device* device = NULL;
	

	ID3D11Texture2D* d3dtex = (ID3D11Texture2D*)texturePtr;
	D3D11_TEXTURE2D_DESC desc;
	d3dtex->GetDesc(&desc);

	//recover device
	d3dtex->GetDevice(&device);
	if (device == NULL){
		LOG4CPP_ERROR(m_logger, "GetDevice == NULL");
		return;
	}

	//recover context
	device->GetImmediateContext(&ctx);
	if (ctx == NULL){
		LOG4CPP_ERROR(m_logger, "GetImmediateContext == NULL");
		return;
	}
	
	//aquire the clImage 
	//LOG4CPP_INFO(m_logger, "starting aquire");
	cl_int err;
	
	err = clEnqueueAcquireD3D11ObjectsNV(m_commandQueue, 1, &m_clImage, 0, NULL, NULL);
	if(err != CL_SUCCESS)
	{
		LOG4CPP_INFO( m_logger, "error at  clEnqueueAcquireGLObjects:" << err );
		return;
	}
	//LOG4CPP_INFO(m_logger, "done aquire");
	size_t offset = 0; 
	size_t dst_origin[3] = {0, 0, 0};
	size_t region[3] = {sourceImage->width(), sourceImage->height(), 1};

	//LOG4CPP_INFO(m_logger, "starting copy");
	//LOG4CPP_INFO(m_logger, "starting copy: offset: " << sourceImage->uMat().offset <<  ": "<< sourceImage->uMat().size().width << "x" << sourceImage->uMat().size().height << "x" << sourceImage->uMat().channels() );
	if(sourceImage->uMat().isContinuous()){
		err = clEnqueueCopyBufferToImage(m_commandQueue, clBuffer, m_clImage, 0, dst_origin, region, 0, NULL, NULL);
	}
	
	if (err != CL_SUCCESS)
	{
		//LOG4CPP_INFO( m_logger, "error at  clEnqueueCopyBufferToImage:" << err );
		err = clEnqueueReleaseD3D11ObjectsNV(m_commandQueue, 1, &m_clImage, 0, NULL, NULL);
		//LOG4CPP_INFO(m_logger, "releasing status: " << (err == CL_SUCCESS ? "SUCCESS" : "FAILED") );
		return;
	}
	LOG4CPP_INFO(m_logger, "starting release");
	err = clEnqueueReleaseD3D11ObjectsNV(m_commandQueue, 1, &m_clImage, 0, NULL, NULL);
	if(err != CL_SUCCESS) 
	{
		LOG4CPP_INFO( m_logger, "error at  clEnqueueReleaseGLObjects:" << err );
		return;
	}
	LOG4CPP_INFO(m_logger, "starting to finish");
	err = clFinish(m_commandQueue);

	if (err != CL_SUCCESS)
	{
		LOG4CPP_INFO( m_logger, "error at  clFinish:" << err );
		return;
	}
	static int receivedAndREnder = 0;
	LOG4CPP_INFO(m_logger, "CL done: " << receivedAndREnder++);
}

#endif



	void updateTextureDirectX11CPU(Measurement::Timestamp texturePtr)
	{
		try{
			
			LOG4CPP_INFO(m_logger, "updateTextureDirectX11");
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



			//LOG4CPP_INFO(m_logger, "	desc.Usage : " << desc.Usage);
			//LOG4CPP_INFO(m_logger, "	desc.Width : " << desc.Width << " : " <<currentImage->width());
			//LOG4CPP_INFO(m_logger, "	desc.Height : " << desc.Height << " : " << currentImage->height());
			//LOG4CPP_INFO(m_logger, "	desc.Format : " << DXGI_FORMAT_R32_SINT << " : " << desc.Format << " : " << currentImage->nChannels << " : " << currentImage->depth);





			unsigned char* data = (unsigned char*)currentImage->Mat().data;
			boost::shared_ptr<D3D11_BOX> box;
			int bytePerRow=4;

			if (desc.Height == currentImage->height() && desc.Width == currentImage->width()) {
				LOG4CPP_INFO(m_logger, "same width and height  ");
				
			} else{
				LOG4CPP_INFO(m_logger, "different width and height  ");
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

			if ((desc.Format == DXGI_FORMAT_R32_SINT || desc.Format == DXGI_FORMAT_R32_TYPELESS) && currentImage->channels() == 1 && currentImage->depth() == CV_32S) {
				bytePerRow = 4*currentImage->width();


				LOG4CPP_INFO(m_logger, "4 byte layout, nothing to do  ");



			}
			else if (currentImage->channels() == 3) {
				//LOG4CPP_INFO(m_logger, "no convert");
				if (convertedImage.get() == 0){
					LOG4CPP_INFO(m_logger, "create buffer image");
					convertedImage.reset(new Vision::Image(currentImage->width(), currentImage->height(), 4, 8, currentImage->origin()));

				}
				//LOG4CPP_INFO(m_logger, "convert image");
				cvtColor(currentImage->Mat(), convertedImage->Mat(), CV_BGR2RGBA);
				data = (unsigned char*)convertedImage->Mat().data;
				bytePerRow = 4 * currentImage->width();

				//LOG4CPP_INFO(m_logger, "3 byte layout, convert  ");
				
				//data = (unsigned char*)currentImage->Mat().data;
				//bytePerRow = 3 * currentImage->width();
			}

		

			
			
			//LOG4CPP_INFO(m_logger, "no convert1 " );
			ctx->UpdateSubresource(d3dtex, 0, box.get(), data, bytePerRow, 0);
			bool res;
			//UpdateSubresource_Workaround(device, ctx, d3dtex, 0, 0, data, 3, bytePerRow, 0, &res);
			//LOG4CPP_INFO(m_logger, "no convert2:" << res);
			//delete[] data;
			
			ctx->Release();
			//LOG4CPP_INFO(m_logger, "no convert3");
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
	boost::shared_ptr< Vision::Image >  convertedImage;
	bool useOpenGL;
	int m_textureChannels;
	int m_textureWidth;
	int m_textureHeight;
#ifdef USE_UMAT
	cl_mem m_clImage;
	cl_command_queue m_commandQueue;
	bool m_clImageInitialized;
// #ifdef WITH_DIRECTX11_TEXTURE_UPDATE
// 	clEnqueueAcquireD3D11ObjectsNV_fn clEnqueueAcquireD3D11ObjectsNV;
// 	clEnqueueReleaseD3D11ObjectsNV_fn clEnqueueReleaseD3D11ObjectsNV;
// #endif
#endif

  };


  UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
    cf->registerComponent< TextureUpdateOpenGL >   ( "TextureUpdateOpenGL" ); 
  }

} } // namespace Ubitrack::Components

#endif //HAVE_OPENCL