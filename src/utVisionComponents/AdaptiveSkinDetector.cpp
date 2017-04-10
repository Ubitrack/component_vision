//#define ENABLE_ASS_CODE
#ifdef ENABLE_ASS_CODE


#include <string>
#include <list>
#include <iostream>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <log4cpp/Category.hh>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utUtil/OS.h>
#include <utUtil/Exception.h>
#include <utVision/Image.h>

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>


using namespace Ubitrack;
using namespace Ubitrack::Measurement;

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.AdaptiveSkinDetector" ) );

namespace Ubitrack { namespace Vision {


#define ASD_INTENSITY_SET_PIXEL(pointer, qq) {(*pointer) = (unsigned char)qq;}

#define ASD_IS_IN_MOTION(pointer, v, threshold)	((abs((*(pointer)) - (v)) > (threshold)) ? true : false)


class AdaptiveSkinDetector
	: public Dataflow::Component
{
public:

	/** constructor */
	AdaptiveSkinDetector( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > );

	/** destructor, waits until thread stops */
	~AdaptiveSkinDetector();

	enum {
	        MORPHING_METHOD_NONE = 0,
	        MORPHING_METHOD_ERODE = 1,
	        MORPHING_METHOD_ERODE_ERODE = 2,
	        MORPHING_METHOD_ERODE_DILATE = 3
	    };


protected:
	// event handler
	void pushImage( const ImageMeasurement& m );
	void receiveConfigMorphingMode( const Ubitrack::Measurement::Button& m );

    void initData(const Vision::Image* src, int widthDivider, int heightDivider);

	unsigned m_factor;
	// the ports
	Dataflow::PushSupplier< ImageMeasurement > m_outPort;
	Dataflow::PushConsumer< ImageMeasurement > m_inPort;
	Dataflow::PushConsumer< Ubitrack::Measurement::Button > m_configMorphingMode;

private:

    enum {
        GSD_HUE_LT = 3,
        GSD_HUE_UT = 33,
        GSD_INTENSITY_LT = 15,
        GSD_INTENSITY_UT = 250
    };

    class Histogram
    {
    private:
        enum {
            HistogramSize = (GSD_HUE_UT - GSD_HUE_LT + 1)
        };

    protected:
        int findCoverageIndex(double surfaceToCover, int defaultValue = 0);

    public:
		cv::MatND  fHistogram;
		int histSize[1];
		float* ranges[1];

        Histogram();
        virtual ~Histogram();

        void findCurveThresholds(int &x1, int &x2, double percent = 0.05);
        void mergeWith(Histogram *source, double weight);
    };


    int nStartCounter, nFrameCount, nSkinHueLowerBound, nSkinHueUpperBound,
        nSkinIntensityLowerBound, nSkinIntensityUpperBound,
        nMorphingMethod, nSamplingDivider;
    double fHistogramMergeFactor, fHuePercentCovered;
    Histogram histogramHueMotion, skinHueHistogram;
    cv::Mat imgHueFrame, imgSaturationFrame, imgLastGrayFrame, imgMotionFrame;
	cv::Mat imgShrinked, imgTemp, imgGrayFrame, imgHSVFrame;
	cv::Mat erode_kernel, dilate_kernel;

};


AdaptiveSkinDetector::AdaptiveSkinDetector( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_factor ( 0 )
	, m_outPort( "Output", *this )
	, m_inPort( "Input", *this, boost::bind( &AdaptiveSkinDetector::pushImage, this, _1 ) )
	, m_configMorphingMode("MorphingMode", *this, boost::bind( &AdaptiveSkinDetector::receiveConfigMorphingMode, this, _1 ) )
	, nStartCounter( 0 )
	, nFrameCount( 0 )
	, nSkinHueLowerBound( GSD_HUE_LT )
	, nSkinHueUpperBound( GSD_HUE_UT )
    , nSkinIntensityLowerBound( GSD_INTENSITY_LT )
    , nSkinIntensityUpperBound( GSD_INTENSITY_UT )
	, nMorphingMethod( MORPHING_METHOD_ERODE_DILATE )
	, nSamplingDivider( 1 )
	, fHistogramMergeFactor( 0.05 )
	, fHuePercentCovered( 0.95 )
{
	if ( subgraph -> m_DataflowAttributes.hasAttribute( "samplingDivider" ) )
		subgraph -> m_DataflowAttributes.getAttributeData( "samplingDivider", nSamplingDivider );

	if ( subgraph -> m_DataflowAttributes.hasAttribute( "morphingMethod" ) )
		subgraph -> m_DataflowAttributes.getAttributeData( "morphingMethod", nMorphingMethod );

    if ( subgraph -> m_DataflowAttributes.hasAttribute( "skinHueLB" ) )
        subgraph -> m_DataflowAttributes.getAttributeData( "skinHueLB", nSkinHueLowerBound );
    if ( subgraph -> m_DataflowAttributes.hasAttribute( "skinHueUB" ) )
        subgraph -> m_DataflowAttributes.getAttributeData( "skinHueUB", nSkinHueUpperBound );

	if ( subgraph -> m_DataflowAttributes.hasAttribute( "skinIntensityLB" ) )
        subgraph -> m_DataflowAttributes.getAttributeData( "skinIntensityLB", nSkinIntensityLowerBound );
    if ( subgraph -> m_DataflowAttributes.hasAttribute( "skinIntensityUB" ) )
        subgraph -> m_DataflowAttributes.getAttributeData( "skinIntensityUB", nSkinIntensityUpperBound );


	int erosion_elem = 0; // configurable
	int erosion_size = 0; // configurable
	int dilation_elem = 0;
	int dilation_size = 0;
	int const max_elem = 2;
	int const max_kernel_size = 21;

	int erosion_type = 0;
	if (erosion_elem == 0){ erosion_type = cv::MORPH_RECT; }
	else if (erosion_elem == 1){ erosion_type = cv::MORPH_CROSS; }
	else if (erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }
	erode_kernel = cv::getStructuringElement(erosion_type,
		cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		cv::Point(erosion_size, erosion_size));


	int dilation_type = 0;
	if (dilation_elem == 0){ dilation_type = cv::MORPH_RECT; }
	else if (dilation_elem == 1){ dilation_type = cv::MORPH_CROSS; }
	else if (dilation_elem == 2) { dilation_type = cv::MORPH_ELLIPSE; }
	dilate_kernel = cv::getStructuringElement(dilation_type,
		cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		cv::Point(dilation_size, dilation_size));


}

AdaptiveSkinDetector::~AdaptiveSkinDetector()
{
}

void AdaptiveSkinDetector::initData(const Vision::Image* src, int widthDivider, int heightDivider)
{
    LOG4CPP_INFO(logger, "Initialize Skin detector: " << src->width/widthDivider << "," << src->height/heightDivider);
	 
	cv::Size imageSize = cv::Size(src->width() / widthDivider, src->height() / heightDivider);

	imgHueFrame = cv::Mat(imageSize, CV_8UC(1));
	imgShrinked = cv::Mat(imageSize, CV_8UC(src->channels()));
	imgSaturationFrame = cv::Mat(imageSize, CV_8UC(1));
	imgTemp = cv::Mat(imageSize, CV_8UC(1));
	imgGrayFrame = cv::Mat(imageSize, CV_8UC(1));
	imgLastGrayFrame = cv::Mat(imageSize, CV_8UC(1));
	imgHSVFrame = cv::Mat(imageSize, CV_8UC(1));
}

void AdaptiveSkinDetector::receiveConfigMorphingMode( const Ubitrack::Measurement::Button& m )
{
	int value = *(m.get());
	if ((value >= MORPHING_METHOD_NONE) && (value <= MORPHING_METHOD_ERODE_DILATE)) {
		nMorphingMethod = value;
	}
}


void AdaptiveSkinDetector::pushImage( const ImageMeasurement& m )
{
    Vision::Image* src = m.get();

    int h, v, i, l;
    bool isInit = false;

    nFrameCount++;

    if (imgHueFrame.empty())
    {
        isInit = true;
        initData(src, nSamplingDivider, nSamplingDivider);
    }

    boost::shared_ptr< Image > imgFilteredFrame( new Image( src->width()/nSamplingDivider, src->height()/nSamplingDivider, 1) );
    unsigned char *pShrinked, *pHueFrame, *pMotionFrame, *pLastGrayFrame, *pFilteredFrame, *pGrayFrame;


    // assumes BGR image here - how do we know what img-type we received (maybe look into texture node ..)
	if ((src->width() != imgHueFrame.cols) || (src->height() != imgHueFrame.rows))
    {
		cv::resize(src->Mat(), imgShrinked, imgShrinked.size());
		if (src->pixelFormat == Vision::Image::BGR)
			cv::cvtColor(imgShrinked, imgHSVFrame, CV_BGR2HSV);
		else
			cv::cvtColor(imgShrinked, imgHSVFrame, CV_RGB2HSV);
    }
    else
    {
		if (src->pixelFormat == Vision::Image::BGR)
			cv::cvtColor(src->Mat(), imgHSVFrame, CV_BGR2HSV);
		else
			cv::cvtColor(src->Mat(), imgHSVFrame, CV_RGB2HSV);
    }
	std::vector<cv::Mat> hsvvec;
	hsvvec.push_back(imgHueFrame);
	hsvvec.push_back(imgSaturationFrame);
	hsvvec.push_back(imgGrayFrame);
	cv::split(imgHSVFrame, hsvvec);

	cv::Size imageSize = cv::Size(imgHueFrame.rows, imgHueFrame.cols);
	imgMotionFrame = cv::Mat::zeros(imageSize, CV_8UC(1));
	imgFilteredFrame->Mat() = cv::Mat::zeros(imageSize, CV_8UC(1));


	pShrinked = imgShrinked.data;
	pHueFrame = imgHueFrame.data;
	pLastGrayFrame = imgLastGrayFrame.data;
	pGrayFrame = imgGrayFrame.data;
	pMotionFrame = imgMotionFrame.data;
	pFilteredFrame = imgFilteredFrame->Mat().data;

    l = imgHueFrame.rows * imgHueFrame.cols;

    for (i = 0; i < l; i++)
    {
        v = (*pGrayFrame);
        if ((v >= GSD_INTENSITY_LT) && (v <= GSD_INTENSITY_UT))
        {
            h = (*pHueFrame);
            if ((h >= GSD_HUE_LT) && (h <= GSD_HUE_UT))
            {
                if ((h >= nSkinHueLowerBound) && (h <= nSkinHueUpperBound))
                    ASD_INTENSITY_SET_PIXEL(pFilteredFrame, h);
//                    ASD_INTENSITY_SET_PIXEL(pFilteredFrame, 0xFF);

                if (ASD_IS_IN_MOTION(pLastGrayFrame, v, 7))
                    ASD_INTENSITY_SET_PIXEL(pMotionFrame, h);
            }
        }

        pShrinked += 3;
        pGrayFrame++;
        pLastGrayFrame++;
        pMotionFrame++;
        pHueFrame++;
        pFilteredFrame++;
    }

	int channels[3];
	channels[0] = 0;

	if (isInit) {
		cv::calcHist(&imgHueFrame, 1, channels, cv::Mat(), skinHueHistogram.fHistogram, 1, &(histogramHueMotion.histSize[0]), &(histogramHueMotion.ranges[0]));
	}

	imgLastGrayFrame = imgGrayFrame.clone();

	cv::erode(imgMotionFrame, imgTemp, erode_kernel);
	cv::dilate(imgTemp, imgMotionFrame, dilate_kernel);

	cv::calcHist(&imgMotionFrame, 1, channels, cv::Mat(), histogramHueMotion.fHistogram, 1, &(histogramHueMotion.histSize[0]), &(histogramHueMotion.ranges[0]));

	skinHueHistogram.mergeWith(&histogramHueMotion, fHistogramMergeFactor);
    skinHueHistogram.findCurveThresholds(nSkinHueLowerBound, nSkinHueUpperBound, 1 - fHuePercentCovered);

    switch (nMorphingMethod)
    {
        case MORPHING_METHOD_ERODE :
			cv::erode(imgFilteredFrame->Mat(), imgTemp, erode_kernel);
			imgFilteredFrame->Mat() = imgTemp.clone();
            break;
        case MORPHING_METHOD_ERODE_ERODE :
			cv::erode(imgFilteredFrame->Mat(), imgTemp, erode_kernel);
			cv::erode(imgTemp, imgFilteredFrame->Mat(), erode_kernel);
            break;
        case MORPHING_METHOD_ERODE_DILATE :
			cv::erode(imgFilteredFrame->Mat(), imgTemp, erode_kernel);
			cv::dilate(imgTemp, imgFilteredFrame->Mat(), dilate_kernel);
            break;
    }
    m_outPort.send( ImageMeasurement( m.time(), imgFilteredFrame ) );
}


//------------------------- Histogram for Adaptive Skin Detector -------------------------//

AdaptiveSkinDetector::Histogram::Histogram()
{
	histSize[0] = HistogramSize;
	float hranges[2];
	hranges[0] = GSD_HUE_LT;    // BRG range
	hranges[1] = GSD_HUE_UT;
	ranges[0] = hranges;

}

AdaptiveSkinDetector::Histogram::~Histogram()
{
}

int AdaptiveSkinDetector::Histogram::findCoverageIndex(double surfaceToCover, int defaultValue)
{
    double s = 0;
    for (int i = 0; i < HistogramSize; i++)
    {
        s += cvGetReal1D( fHistogram->bins, i );
        if (s >= surfaceToCover)
        {
            return i;
        }
    }
    return defaultValue;
}

void AdaptiveSkinDetector::Histogram::findCurveThresholds(int &x1, int &x2, double percent)
{
    double sum = 0;

    for (int i = 0; i < HistogramSize; i++)
    {
        sum += cvGetReal1D( fHistogram->bins, i );
    }

    x1 = findCoverageIndex(sum * percent, -1);
    x2 = findCoverageIndex(sum * (1-percent), -1);

    if (x1 == -1)
        x1 = GSD_HUE_LT;
    else
        x1 += GSD_HUE_LT;

    if (x2 == -1)
        x2 = GSD_HUE_UT;
    else
        x2 += GSD_HUE_LT;
}

void AdaptiveSkinDetector::Histogram::mergeWith(AdaptiveSkinDetector::Histogram *source, double weight)
{
    float myweight = (float)(1-weight);
    float maxVal1 = 0, maxVal2 = 0, *f1, *f2, ff1, ff2;

    cvGetMinMaxHistValue(source->fHistogram, NULL, &maxVal2);

    if (maxVal2 > 0 )
    {
        cvGetMinMaxHistValue(fHistogram, NULL, &maxVal1);
        if (maxVal1 <= 0)
        {
            for (int i = 0; i < HistogramSize; i++)
            {
                f1 = (float*)cvPtr1D(fHistogram->bins, i);
                f2 = (float*)cvPtr1D(source->fHistogram->bins, i);
                (*f1) = (*f2);
            }
        }
        else
        {
            for (int i = 0; i < HistogramSize; i++)
            {
                f1 = (float*)cvPtr1D(fHistogram->bins, i);
                f2 = (float*)cvPtr1D(source->fHistogram->bins, i);

                ff1 = ((*f1)/maxVal1)*myweight;
                if (ff1 < 0)
                    ff1 = -ff1;

                ff2 = (float)(((*f2)/maxVal2)*weight);
                if (ff2 < 0)
                    ff2 = -ff2;

                (*f1) = (ff1 + ff2);

            }
        }
    }
}



} } // namespace Ubitrack::Vision

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Vision::AdaptiveSkinDetector > ( "AdaptiveSkinDetector" );
}


#endif // ENABLE_ASS_CODE