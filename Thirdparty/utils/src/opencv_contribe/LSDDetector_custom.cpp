/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2014, Biagio Montesano, all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#include "lsd_c.h"
#include "descriptor_custom.hpp"
#include "opencv_contribe.h"
// #include "LineDescriptor.h"
// #include "LineStructure.h"
// #include "PairwiseLineMatching.h"

//using namespace cv;
namespace cv
{
namespace line_descriptor_custom
{

Ptr<LSDDetectorC> LSDDetectorC::createLSDDetectorC()
{
  return Ptr<LSDDetectorC>( new LSDDetectorC() );
}

/* compute Gaussian pyramid of input image */
void LSDDetectorC::computeGaussianPyramid( const Mat& image, int numOctaves, int scale )
{
  /* clear class fields */
  gaussianPyrs.clear();

  /* insert input image into pyramid */
  cv::Mat currentMat = image.clone();
  //cv::GaussianBlur( currentMat, currentMat, cv::Size( 5, 5 ), 1 );
  gaussianPyrs.push_back( currentMat );

  /* fill Gaussian pyramid */
  for ( int pyrCounter = 1; pyrCounter < numOctaves; pyrCounter++ )
  {
    /* compute and store next image in pyramid and its size */
    pyrDown( currentMat, currentMat, Size( currentMat.cols / scale, currentMat.rows / scale ) );
    gaussianPyrs.push_back( currentMat );
  }
}

/* check lines' extremes */
inline void checkLineExtremes( cv::Vec4f& extremes, cv::Size imageSize )
{

  if( extremes[0] < 0 )
    extremes[0] = 0;

  if( extremes[0] >= imageSize.width )
    extremes[0] = (float)imageSize.width - 1.0f;

  if( extremes[2] < 0 )
    extremes[2] = 0;

  if( extremes[2] >= imageSize.width )
    extremes[2] = (float)imageSize.width - 1.0f;

  if( extremes[1] < 0 )
    extremes[1] = 0;

  if( extremes[1] >= imageSize.height )
    extremes[1] = (float)imageSize.height - 1.0f;

  if( extremes[3] < 0 )
    extremes[3] = 0;

  if( extremes[3] >= imageSize.height )
    extremes[3] = (float)imageSize.height - 1.0f;
}

/* requires line detection (only one image) */
void LSDDetectorC::detect( const Mat& image, CV_OUT std::vector<KeyLine>& keylines, int scale, int numOctaves, const Mat& mask )
{
  if( mask.data != NULL && ( mask.size() != image.size() || mask.type() != CV_8UC1 ) )
    throw std::runtime_error( "Mask error while detecting lines: please check its dimensions and that data type is CV_8UC1" );

  else
    detectImpl( image, keylines, numOctaves, scale, mask );
}

/* requires line detection (more than one image) */
void LSDDetectorC::detect( const std::vector<Mat>& images, std::vector<std::vector<KeyLine> >& keylines, int scale, int numOctaves,
                          const std::vector<Mat>& masks ) const
{
  /* detect lines from each image */
  for ( size_t counter = 0; counter < images.size(); counter++ )
  {
    if( masks[counter].data != NULL && ( masks[counter].size() != images[counter].size() || masks[counter].type() != CV_8UC1 ) )
      throw std::runtime_error( "Masks error while detecting lines: please check their dimensions and that data types are CV_8UC1" );

    else
      detectImpl( images[counter], keylines[counter], numOctaves, scale, masks[counter] );
  }
}

/* implementation of line detection */
void LSDDetectorC::detectImpl( const Mat& imageSrc, std::vector<KeyLine>& keylines, int numOctaves, int scale, const Mat& mask ) const
{
  cv::Mat image;
  if( imageSrc.channels() != 1 )
    cvtColor( imageSrc, image, COLOR_BGR2GRAY );
  else
    image = imageSrc.clone();

  /*check whether image depth is different from 0 */
  if( image.depth() != 0 )
    throw std::runtime_error( "Error, depth image!= 0" );

  /* create a pointer to self */
  LSDDetectorC *lsd = const_cast<LSDDetectorC*>( this );

  /* compute Gaussian pyramids */
  lsd->computeGaussianPyramid( image, numOctaves, scale );

  /* create an LSD extractor */
  cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector();

  /* prepare a vector to host extracted segments */
  std::vector<std::vector<cv::Vec4f> > lines_lsd;

  /* extract lines */
  for ( int i = 0; i < numOctaves; i++ )
  {
    std::vector<Vec4f> octave_lines;
    ls->detect( gaussianPyrs[i], octave_lines );
    lines_lsd.push_back( octave_lines );
  }
  /* create keylines */
  int class_counter = -1;
  for ( int octaveIdx = 0; octaveIdx < (int) lines_lsd.size(); octaveIdx++ )
  {
    float octaveScale = pow( (float)scale, octaveIdx );
    for ( int k = 0; k < (int) lines_lsd[octaveIdx].size(); k++ )
    {
      KeyLine kl;
      cv::Vec4f extremes = lines_lsd[octaveIdx][k];

      /* check data validity */
      checkLineExtremes( extremes, gaussianPyrs[octaveIdx].size() );

      /* fill KeyLine's fields */
      kl.startPointX = extremes[0] * octaveScale;
      kl.startPointY = extremes[1] * octaveScale;
      kl.endPointX = extremes[2] * octaveScale;
      kl.endPointY = extremes[3] * octaveScale;
      kl.sPointInOctaveX = extremes[0];
      kl.sPointInOctaveY = extremes[1];
      kl.ePointInOctaveX = extremes[2];
      kl.ePointInOctaveY = extremes[3];
      kl.lineLength = (float) sqrt( pow( extremes[0] - extremes[2], 2 ) + pow( extremes[1] - extremes[3], 2 ) );

      /* compute number of pixels covered by line */
      LineIterator li( gaussianPyrs[octaveIdx], Point2f( extremes[0], extremes[1] ), Point2f( extremes[2], extremes[3] ) );
      kl.numOfPixels = li.count;

      kl.angle = atan2( ( kl.endPointY - kl.startPointY ), ( kl.endPointX - kl.startPointX ) );
      kl.class_id = ++class_counter;
      kl.octave = octaveIdx;
      kl.size = ( kl.endPointX - kl.startPointX ) * ( kl.endPointY - kl.startPointY );
      kl.response = kl.lineLength / max( gaussianPyrs[octaveIdx].cols, gaussianPyrs[octaveIdx].rows );
      kl.pt = Point2f( ( kl.endPointX + kl.startPointX ) / 2, ( kl.endPointY + kl.startPointY ) / 2 );

      keylines.push_back( kl );
    }
  }

  /* delete undesired KeyLines, according to input mask */
  if( !mask.empty() )
  {
    for ( size_t keyCounter = 0; keyCounter < keylines.size(); keyCounter++ )
    {
      KeyLine kl = keylines[keyCounter];
      if( mask.at<uchar>( (int) kl.startPointY, (int) kl.startPointX ) == 0 && mask.at<uchar>( (int) kl.endPointY, (int) kl.endPointX ) == 0 )
      {
        keylines.erase( keylines.begin() + keyCounter );
        keyCounter--;
      }
    }
  }

}

// Overload detect and detectImpl with LSDDetectorC Options
void LSDDetectorC::detect( const Mat& image, CV_OUT std::vector<KeyLine>& keylines, int scale, int numOctaves, LSDOptions opts, const Mat& mask )
{
  if( mask.data != NULL && ( mask.size() != image.size() || mask.type() != CV_8UC1 ) )
    throw std::runtime_error( "Mask error while detecting lines: please check its dimensions and that data type is CV_8UC1" );

  else
    detectImpl( image, keylines, numOctaves, scale, opts, mask );
}
#pragma GCC push_options
#pragma GCC optimize (0)
void LSDDetectorC::detectImpl( const Mat& imageSrc, std::vector<KeyLine>& keylines, int numOctaves, int scale, LSDOptions opts, const Mat& mask ) const
{
  cv::Mat image;
  if( imageSrc.channels() != 1 )
    cvtColor( imageSrc, image, COLOR_BGR2GRAY );
  else
    image = imageSrc.clone();

  /*check whether image depth is different from 0 */
  if( image.depth() != 0 )
    throw std::runtime_error( "Error, depth image!= 0" );

  /* create a pointer to self */
  LSDDetectorC *lsd = const_cast<LSDDetectorC*>( this );

  /* compute Gaussian pyramids */
  lsd->computeGaussianPyramid( image, numOctaves, scale );

  /* create an LSD extractor */
  cv::Ptr<cv::cv_contribe::LineSegmentDetector> ls;
  ls = cv::cv_contribe::createLineSegmentDetector( opts.refine,
                                         opts.scale,
                                         opts.sigma_scale,
                                         opts.quant,
                                         opts.ang_th,
                                         opts.log_eps,
                                         opts.density_th,
                                         opts.n_bins);

  /* prepare a vector to host extracted segments */
  std::vector<std::vector<cv::Vec4f> > lines_lsd;

  /* extract lines */
  int _cols = image.cols;
  int _rows = image.rows;
  for ( int i = 0; i < numOctaves; i++ )
  {
    std::vector<Vec4f> octave_lines;
    ls->detect( gaussianPyrs[i], octave_lines );
    // int n;
    // int reg = 0;
    // double *lsd_image = new double[image.cols * image.rows];
    // for (int c = 0; c < _cols; ++c) {
    //   for (int r = 0; r < _rows; ++r) {
    //     lsd_image[c + r * _cols] = image.at<uchar>(r, c);
    //   }
    // }
    // int * region;
    // int regX,regY;
    // double * segs = LineSegmentDetection( 
    // &n, lsd_image, image.cols, image.rows, opts.scale, opts.sigma_scale, opts.quant, 
    // opts.ang_th, opts.log_eps, opts.density_th, opts.n_bins,reg ? &region : NULL, &regX, &regY );
    // cv::Mat show_image;
    // cvtColor( image, show_image, COLOR_GRAY2BGR );
    // image.convertTo(show_image, CV_8UC3);
    // for (int l = 0; l < n; ++l) {
    //   double x1 = segs[7*l];
    //   double y1 = segs[7*l + 1];
    //   double x2 = segs[7*l + 2];
    //   double y2 = segs[7*l + 3];
    //   octave_lines.push_back(cv::Vec4f(x1, y1, x2, y2));
    //   int thickness = segs[7*l + 4];
    //   thickness = thickness > 0 ? thickness:1;
    //   // std::cout << thickness << "\n";
    //   cv::Scalar color((rand() % 255), (rand() % 255), (rand() % 255));
    //   cv::line(show_image, cv::Point2f(x1, y1), cv::Point2f(x2, y2), color, thickness);
    // }
    // cv::imshow("line_cus", show_image);
    // cv::waitKey();
    lines_lsd.push_back( octave_lines );
  }
  /* create keylines */
  int class_counter = -1;
  for ( int octaveIdx = 0; octaveIdx < (int) lines_lsd.size(); octaveIdx++ )
  {
    float octaveScale = pow( (float)scale, octaveIdx );
    for ( int k = 0; k < (int) lines_lsd[octaveIdx].size(); k++ )
    {
      KeyLine kl;
      cv::Vec4f extremes = lines_lsd[octaveIdx][k];

      /* check data validity */
      checkLineExtremes( extremes, gaussianPyrs[octaveIdx].size() );

      /* check line segment min length */
      double length = (float) sqrt( pow( extremes[0] - extremes[2], 2 ) + pow( extremes[1] - extremes[3], 2 ) );
      if( length > opts.min_length )
      {
          /* fill KeyLine's fields */
          kl.startPointX = extremes[0] * octaveScale;
          kl.startPointY = extremes[1] * octaveScale;
          kl.endPointX   = extremes[2] * octaveScale;
          kl.endPointY   = extremes[3] * octaveScale;
          kl.sPointInOctaveX = extremes[0];
          kl.sPointInOctaveY = extremes[1];
          kl.ePointInOctaveX = extremes[2];
          kl.ePointInOctaveY = extremes[3];
          kl.lineLength = length;

          /* compute number of pixels covered by line */
          LineIterator li( gaussianPyrs[octaveIdx], Point2f( extremes[0], extremes[1] ), Point2f( extremes[2], extremes[3] ) );
          kl.numOfPixels = li.count;

          kl.angle = atan2( ( kl.endPointY - kl.startPointY ), ( kl.endPointX - kl.startPointX ) );
          kl.class_id = ++class_counter;
          kl.octave = octaveIdx;
          kl.size = ( kl.endPointX - kl.startPointX ) * ( kl.endPointY - kl.startPointY );
          kl.response = kl.lineLength / max( gaussianPyrs[octaveIdx].cols, gaussianPyrs[octaveIdx].rows );
          kl.pt = Point2f( ( kl.endPointX + kl.startPointX ) / 2, ( kl.endPointY + kl.startPointY ) / 2 );

          keylines.push_back( kl );
      }
    }
  }

  /* delete undesired KeyLines, according to input mask */
  if( !mask.empty() )
  {
    for ( size_t keyCounter = 0; keyCounter < keylines.size(); keyCounter++ )
    {
      KeyLine kl = keylines[keyCounter];
      if( mask.at<uchar>( (int) kl.startPointY, (int) kl.startPointX ) == 0 && mask.at<uchar>( (int) kl.endPointY, (int) kl.endPointX ) == 0 )
      {
        keylines.erase( keylines.begin() + keyCounter );
        keyCounter--;
      }
    }
  }

}
#pragma GCC pop_options
// Overload with fast detection
void LSDDetectorC::detectFast( const Mat& image, CV_OUT std::vector<KeyLine>& keylines, int scale, int numOctaves, LSDOptions opts, const Mat& mask )
{
  if( mask.data != NULL && ( mask.size() != image.size() || mask.type() != CV_8UC1 ) )
    throw std::runtime_error( "Mask error while detecting lines: please check its dimensions and that data type is CV_8UC1" );

  else
    detectImplFast( image, keylines, numOctaves, scale, opts, mask );
}

void LSDDetectorC::detectImplFast( const Mat& imageSrc, std::vector<KeyLine>& keylines, int numOctaves, int scale, LSDOptions opts, const Mat& mask ) const
{
  cv::Mat image;
  if( imageSrc.channels() != 1 )
    cvtColor( imageSrc, image, COLOR_BGR2GRAY );
  else
    image = imageSrc.clone();

  /*check whether image depth is different from 0 */
  if( image.depth() != 0 )
    throw std::runtime_error( "Error, depth image!= 0" );

  /* create a pointer to self */
  LSDDetectorC *lsd = const_cast<LSDDetectorC*>( this );

  /* compute Gaussian pyramids */
  lsd->computeGaussianPyramid( image, numOctaves, scale );

  /* create an LSD extractor */
  cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector( opts.refine,
                                                                       opts.scale,
                                                                       opts.sigma_scale,
                                                                       opts.quant,
                                                                       opts.ang_th,
                                                                       opts.log_eps,
                                                                       opts.density_th,
                                                                       opts.n_bins);

  /* prepare a vector to host extracted segments */
  std::vector<std::vector<cv::Vec4f> > lines_lsd;

  /* extract lines */
  for ( int i = 0; i < numOctaves; i++ )
  {
    std::vector<Vec4f> octave_lines;
    ls->detect( gaussianPyrs[i], octave_lines );
    lines_lsd.push_back( octave_lines );
  }

  /* create keylines */
  int class_counter = -1;
  for ( int octaveIdx = 0; octaveIdx < (int) lines_lsd.size(); octaveIdx++ )
  {
    float octaveScale = pow( (float)scale, octaveIdx );
    for ( int k = 0; k < (int) lines_lsd[octaveIdx].size(); k++ )
    {
      KeyLine kl;
      cv::Vec4f extremes = lines_lsd[octaveIdx][k];

      /* check data validity */
      checkLineExtremes( extremes, gaussianPyrs[octaveIdx].size() );

      /* check line segment min length */
      double length = (float) sqrt( pow( extremes[0] - extremes[2], 2 ) + pow( extremes[1] - extremes[3], 2 ) );
      if( length > opts.min_length )
      {
          /* fill KeyLine's fields */
          kl.startPointX = extremes[0] * octaveScale;
          kl.startPointY = extremes[1] * octaveScale;
          kl.endPointX   = extremes[2] * octaveScale;
          kl.endPointY   = extremes[3] * octaveScale;
          kl.sPointInOctaveX = extremes[0];
          kl.sPointInOctaveY = extremes[1];
          kl.ePointInOctaveX = extremes[2];
          kl.ePointInOctaveY = extremes[3];
          kl.lineLength = length;

          /* compute number of pixels covered by line */
          LineIterator li( gaussianPyrs[octaveIdx], Point2f( extremes[0], extremes[1] ), Point2f( extremes[2], extremes[3] ) );
          kl.numOfPixels = li.count;

          kl.angle = atan2( ( kl.endPointY - kl.startPointY ), ( kl.endPointX - kl.startPointX ) );
          kl.class_id = ++class_counter;
          kl.octave = octaveIdx;
          kl.size = ( kl.endPointX - kl.startPointX ) * ( kl.endPointY - kl.startPointY );
          kl.response = kl.lineLength / max( gaussianPyrs[octaveIdx].cols, gaussianPyrs[octaveIdx].rows );
          kl.pt = Point2f( ( kl.endPointX + kl.startPointX ) / 2, ( kl.endPointY + kl.startPointY ) / 2 );

          keylines.push_back( kl );
      }
    }
  }

  /* delete undesired KeyLines, according to input mask */
  if( !mask.empty() )
  {
    for ( size_t keyCounter = 0; keyCounter < keylines.size(); keyCounter++ )
    {
      KeyLine kl = keylines[keyCounter];
      if( mask.at<uchar>( (int) kl.startPointY, (int) kl.startPointX ) == 0 && mask.at<uchar>( (int) kl.endPointY, (int) kl.endPointX ) == 0 )
      {
        keylines.erase( keylines.begin() + keyCounter );
        keyCounter--;
      }
    }
  }

}
}


}

