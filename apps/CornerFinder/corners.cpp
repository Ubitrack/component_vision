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
 * @ingroup vision
 * @file
 * 
 *
 * @author Christian Waechter <christian.waechter@in.tum.de>
 */
 
#include <iostream>
#include <fstream>
#include <vector>
#include <string>


#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/numeric/ublas/io.hpp>

//highgui includes windows.h with the wrong parameters
#ifdef _WIN32
#include <utUtil/CleanWindows.h>
#endif
#include <opencv/highgui.h>
#include <opencv/cv.h>

#include <utMath/NewFunction/Function.h>
#include <utMath/NewFunction/Addition.h>
#include <utMath/NewFunction/Dehomogenization.h>
#include <utMath/NewFunction/LieRotation.h>
#include <utMath/BackwardPropagation.h>
#include <utCalibration/NewFunction/CameraIntrinsicsMultiplication.h>
#include <utCalibration/AbsoluteOrientation.h>
#include <utCalibration/3DPointReconstruction.h>
 #include <utCalibration/LensDistortion.h>

#include <utVision/Undistortion.h>

#include <utUtil/Logging.h>
#include <log4cpp/Category.hh>
//static log4cpp::Category& optLogger( log4cpp::Category::getInstance( "Ubitrack.Utils.MarkerFinder" ) );


using namespace Ubitrack;

IplImage*		img_gray = 0, *img_col = 0;
IplImage*		tmp_img = 0;
int				win_size = 10, target = 1, height = 0;

double scale = 4.;
double			xPos, yPos;
std::string			tmp_name;

std::vector <double>	pointsX;
std::vector <double>	pointsY;
std::vector <std::string>	names;
std::vector <std::string>	marker;

void cvCross ( IplImage* img, CvPoint point, CvScalar farbe )
{
	int length = 8;
    cvLine(img, cvPoint(point.x-length,point.y-length), cvPoint(point.x+length,point.y+length), farbe);
	cvLine(img, cvPoint(point.x-length,point.y+length), cvPoint(point.x+length,point.y-length), farbe);
}

void on_mouse( int event, int x, int y, int flags, void* param)
{
	CvSize window			= cvSize( win_size , win_size );
	CvPoint2D32f corner_pt	= cvPoint2D32f ( x * scale , y * scale );
	CvTermCriteria termcrit	= cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,10,1);
	
	switch( event )
	{
	case CV_EVENT_LBUTTONDOWN:
		cvReleaseImage( &img_col );
		
		img_col = cvCloneImage( tmp_img );
		cvCircle( img_col, cvPoint( x , y ), 8, CV_RGB( 250, 50, 50 ) );
		std::cout << "new Mouse position at " << x  << " and " << y << std::endl;
		cvFindCornerSubPix( img_gray, &corner_pt, 1, window, cvSize ( -1, -1 ), termcrit );
		//std::cout << "cliked at Pos " << x << " and " << y << std::endl;
		cvCross( img_col, cvPoint( (1./scale) * corner_pt.x, (1./scale) * corner_pt.y ), CV_RGB( 50, 250, 50 ) );
		xPos = (corner_pt.x);// * 2.; //because of downsample
		yPos = abs( (corner_pt.y) - height); //because of counting from upper edge, not lower
		
		std::cout << "new x Pos at " << xPos  << " and y Pos at " << yPos << std::endl;
		break;
	case CV_EVENT_RBUTTONDOWN:
		char buffer[10];
		sprintf( buffer, "%02d", target );
		//printf("%s\n", buffer); 
		pointsX.push_back ( xPos );
		pointsY.push_back ( yPos );
		names.push_back ( tmp_name );
		marker.push_back ( std::string( buffer ) );
		break;
	default:
		;//std::cout << "something" << std::endl;
	}
}



void createImageList( std::vector< std::string >& l )
{
	using namespace boost::filesystem;
	
	boost::regex ext( ".*\\.(jpg|JPG|png|PNG|bmp|BMP|PPM|ppm)" );
#if BOOST_FILESYSTEM_VERSION == 3	
	path compPath( "." );
#else
	path compPath( ".", native );
#endif
	if ( !exists( compPath ) )
		throw std::string( "path does not exist" );
	
	//iterate directory
	directory_iterator dirEnd;
	for ( directory_iterator it( compPath ); it != dirEnd; it++ )
	{
#ifdef BOOST_FILESYSTEM_I18N
		path p( it->path() );
#else
		path p( *it );
#endif

		std::string suffix = ".ppm";
		
		//check if file of suitable extension
#if BOOST_FILESYSTEM_VERSION == 3
		if ( exists( p ) && !is_directory( p ) && regex_match( p.leaf().string(), ext ) )
			l.push_back( p.leaf().string() );
#else
		if ( exists( p ) && !is_directory( p ) && regex_match( p.leaf(), ext ) )
			l.push_back( p.leaf() );
#endif
	}

}

void writeToFile( std::string filename )
{
	std::ofstream outfile ( filename.c_str() );
	// >> i/o operations here <<
	// refPointMeasurement P01 IMAG0177.jpg 821.2 427.5
	for (int n=0; n<marker.size(); n++)
	{
		outfile << "refPointMeasurement P";
		outfile << marker[n] << " ";
		outfile << names[n] << " ";
		outfile << pointsX[n] << " ";
		outfile << pointsY[n] << "\n";
	}
	outfile.close();

}


int main( int, char** )
{
	try
	{
		Util::initLogging();
		std::cout << "starting Corner Finder" << std::endl;
		// load intrinsics
		// Vision::Undistortion undistorter( "CamMatrix.calib", "CamCoeffs.calib" );
		// Math::Matrix< float, 3, 3 > intrinsics;
		// Math::matrix_cast_assign( intrinsics, undistorter.getIntrinsics() );
		// std::cout << "Undistortion started" << std::endl;
		// find image files in directories
		std::vector< std::string > imageNames;
		std::cout << "Searching for Images" << std::endl;
		createImageList( imageNames );
		std::cout << "Number of Images : " << imageNames.size() << std::endl;
		
		//initialize openCv Window and Mosue Callback
		cvNamedWindow ( "CornerFinder", CV_WINDOW_AUTOSIZE );
		cvCreateTrackbar( "Window", "CornerFinder", &win_size, 50, NULL );
		cvCreateTrackbar( "Target", "CornerFinder", &target, 100, NULL );
		cvSetMouseCallback ( "CornerFinder" , &::on_mouse , 0 );

		
		// open each file and search for markers
		for ( std::vector< std::string >::iterator itImage = imageNames.begin(); itImage != imageNames.end(); itImage++ )
		{

			boost::shared_ptr< Vision::Image > pImage( new Vision::Image( cvLoadImage( itImage->c_str(), CV_LOAD_IMAGE_GRAYSCALE ) ) );
			tmp_name =  std::string( itImage-> c_str() );
			
			std::cout << "Bild " << itImage->c_str() << std::endl;
			// undistort
			//pImage = undistorter.undistort( pImage );
			
			
			img_gray			= cvCloneImage( *pImage );
			CvSize img_size		= cvGetSize( img_gray );
			height				= img_size.height;
			// img_size.width		= 
			// img_size.height		=  ) ;
			CvSize size_scaled1	= cvSize( ( int ) img_size.width * ( 1. / 2 ), ( int ) img_size.height * ( 1. / 2 ) );
			CvSize size_scaled2	= cvSize( ( int ) img_size.width * ( 1. / 4 ), ( int ) img_size.height * ( 1. / 4 ) );
			IplImage *img_sml1	= cvCreateImage( size_scaled1, IPL_DEPTH_8U, 1 );
			IplImage *img_sml2	= cvCreateImage( size_scaled2, IPL_DEPTH_8U, 1 );
			img_col				= cvCreateImage( size_scaled2, IPL_DEPTH_8U, 3 );
			cvPyrDown( img_gray, img_sml1, CV_GAUSSIAN_5x5 );
			cvPyrDown( img_sml1, img_sml2, CV_GAUSSIAN_5x5 );
			cvCvtColor( img_sml2, img_col, CV_GRAY2BGR );
			cvReleaseImage( &img_sml1 );
			cvReleaseImage( &img_sml2 );
			tmp_img = cvCloneImage( img_col ); // tmp Image to be used to paint crosse and circles
			
			std::cout << "scaled picture Size " << img_size.width << " " << img_size.height << std::endl;

			cvLine( img_col, cvPoint(0, img_size.height-10), cvPoint(300,img_size.height-10), CV_RGB( 255, 255, 255)  );
			//Showing them for clicking the Corners
			int key = 0;
			while( key != 'n')
			{
				cvShowImage( "CornerFinder",  img_col );
				
				key = cvWaitKey( 20 );
				if('w' == key)
				    writeToFile("markerRefs");
				if('q' == key)
				    std::exit(0);
					
				
				//std::cout << "Suche Corner"  << std::endl;
			}
			cvReleaseImage( &img_col );
			cvReleaseImage( &tmp_img );
		}
	}
	catch ( const std::string& s )
	{ std::cout << "Error: " << s << std::endl; }
	catch ( const std::runtime_error& e )
	 { std::cout << "Error: " << e.what() << std::endl; }
	cvDestroyAllWindows (  );
	writeToFile( "points.log" );

}
