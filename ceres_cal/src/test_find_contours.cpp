//test findCountours; building up to camera calibration


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"

#include <roadprintz_camera_calibration/circle_detector.hpp>

#define DEBUG_CIRCLE_DETECTOR

//static const std::string OPENCV_WINDOW = "Open-CV display window";
using namespace std;
using namespace cv;

//const int w = 500;
int levels = 6; //5;
int g_width;
int g_height;

int THRESH = 150;
int g_ans;

vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

vector < Mat  > g_vec_of_images;
int g_n_images;
int g_imagenum=0;

  struct CV_EXPORTS Center
  {
    Point2d location;
    double radius;
    double confidence;
  };
  
  void read_images() {
      Mat image;
      image = imread("image_0_0_0.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image); 
      image = imread("image_1_0_0.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_2_0_0.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_3_0_0.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_4_0_0.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      
      image = imread("image_0_1_0.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image); 
      image = imread("image_1_1_0.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_2_1_0.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_3_1_0.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_4_1_0.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);       
      
      image = imread("image_0_2_0.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image); 
      image = imread("image_1_2_0.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_2_2_0.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_3_2_0.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_4_2_0.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);       

      image = imread("image_0_0_1.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image); 
      image = imread("image_1_0_1.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_2_0_1.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_3_0_1.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_4_0_1.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      
      image = imread("image_0_1_1.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image); 
      image = imread("image_1_1_1.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_2_1_1.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_3_1_1.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_4_1_1.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);       
      
      image = imread("image_0_2_1.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image); 
      image = imread("image_1_2_1.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_2_2_1.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_3_2_1.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_4_2_1.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image); 

      image = imread("image_0_0_2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image); 
      image = imread("image_1_0_2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_2_0_2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_3_0_2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_4_0_2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      
      image = imread("image_0_1_2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image); 
      image = imread("image_1_1_2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_2_1_2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_3_1_2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_4_1_2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);       
      
      image = imread("image_0_2_2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image); 
      image = imread("image_1_2_2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_2_2_2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_3_2_2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image);  
      image = imread("image_4_2_2.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file
      g_vec_of_images.push_back(image); 
      
      g_n_images = g_vec_of_images.size();
      cout<<"loaded "<<g_n_images<<endl;
  }

typedef struct
{
  int pattern_rows;        // number of rows
  int pattern_cols;        // number of colulmns
  bool is_symmetric;       // not sure
  double circle_diameter;  // size of each circle
  double spacing;          // spacing between circle centers
} CircleGridParameters;

struct Params {
  int thresholdStep = 10;
  int minThreshold = 50;
  int maxThreshold = 220;
  int minRepeatability = 2;
  int minDistBetweenCircles = 10;
  int minRadiusDiff = 10;

  bool filterByColor = true;
  int circleColor = 0;

 bool filterByArea = true;
  int minArea = 25; //25;  //int
  int maxArea = 50000;  //int

  bool filterByCircularity = false;
  double minCircularity = 0;  //double
  double maxCircularity = 1;

  bool filterByInertia = true;
  int minInertiaRatio = 0.1; //double
  int maxInertiaRatio = 1;

  bool filterByConvexity = false;
  double minConvexity = 0.95;
  double maxConvexity = 1;    
};
Params params;
/*
  thresholdStep = 10;
  minThreshold = 50;
  maxThreshold = 220;
  minRepeatability = 2;
  minDistBetweenCircles = 10;
  minRadiusDiff = 10;

  filterByColor = true;
  circleColor = 0;

  filterByArea = true;
  minArea = 25;  //int
  maxArea = 5000;  //int

  filterByCircularity = false;
  minCircularity = 0.8f;  //double
  maxCircularity = std::numeric_limits<float>::max();

  filterByInertia = true;
  minInertiaRatio = 0.1f; //double
  maxInertiaRatio = std::numeric_limits<float>::max();

  filterByConvexity = true;
  minConvexity = 0.95f;
  maxConvexity = std::numeric_limits<float>::max(); 


bool   filterByArea = true;
int  minArea = 25;  //int
int  maxArea = 5000;  //int

bool  filterByColor = true;
int  circleColor = 0;

bool   filterByInertia = true;
double minInertiaRatio = 0.1;
double maxInertiaRatio = 10;

bool filterByConvexity = true;
double  minConvexity = 0;
double  maxConvexity = 0; 
 */
Mat g_cnt_img;

//void CircleDetectorImpl::findCircles(InputArray _image, InputArray _binaryImage, std::vector<Center>& centers) const

void findCircles(InputArray _image, InputArray _binaryImage, std::vector<Center>& centers) 
{
  //  CV_INSTRUMENT_REGION()

  Mat image = _image.getMat();  // Oh so much  cleaner this way :(
  Mat binaryImage = _binaryImage.getMat();

  (void)image;
  centers.clear();

  vector<vector<Point> > contours;
  Mat tmpBinaryImage = binaryImage.clone();
  findContours(tmpBinaryImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  // loop on all contours
  for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
  {
    // each if statement may eliminate a contour through the continue function
    // some if statements may also set the confidence whose default is 1.0
    Center center;
    center.confidence = 1;
    Moments moms = moments(Mat(contours[contourIdx]));
    
    if (params.filterByArea)
    {
      double area = moms.m00;
      if (area < params.minArea || area >= params.maxArea) continue;
    }

    if (params.filterByCircularity)        
    {
      double area = moms.m00;
      double perimeter = arcLength(Mat(contours[contourIdx]), true);
      double ratio = 4 * CV_PI * area / (perimeter * perimeter);
      if (ratio < params.minCircularity || ratio >= params.maxCircularity) continue;
    }

    if (params.filterByInertia)
    {
      double denominator = sqrt(pow(2 * moms.mu11, 2) + pow(moms.mu20 - moms.mu02, 2));
      const double eps = 1e-2;
      double ratio;
      if (denominator > eps)
      {
        double cosmin = (moms.mu20 - moms.mu02) / denominator;
        double sinmin = 2 * moms.mu11 / denominator;
        double cosmax = -cosmin;
        double sinmax = -sinmin;

        double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
        double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
        ratio = imin / imax;
      }
      else
      {
        ratio = 1;
      }

      if (ratio < params.minInertiaRatio || ratio >= params.maxInertiaRatio) continue;

      center.confidence = ratio * ratio;
    }

    if (params.filterByConvexity)
    {
      vector<Point> hull;
      convexHull(Mat(contours[contourIdx]), hull);
      double area = contourArea(Mat(contours[contourIdx]));
      double hullArea = contourArea(Mat(hull));
      double ratio = area / hullArea;
      if (ratio < params.minConvexity || ratio >= params.maxConvexity) continue;      
    }
    Mat pointsf;
    Mat(contours[contourIdx]).convertTo(pointsf, CV_32F);
    if (pointsf.rows < 5) continue;
    RotatedRect box = fitEllipse(pointsf);

    // find center
    // center.location = Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);
    center.location = box.center;

    // one more filter by color of central pixel
    if (params.filterByColor)
    {
      if (binaryImage.at<uchar>(cvRound(center.location.y), cvRound(center.location.x)) != params.circleColor) continue;      
    }

    // compute circle radius
    //	{
    //	vector<double> dists;
    //	for (size_t pointIdx = 0; pointIdx < contours[contourIdx].size(); pointIdx++)
    //	{
    //		Point2d pt = contours[contourIdx][pointIdx];
    //		dists.push_back(norm(center.location - pt));
    //	}
    //	std::sort(dists.begin(), dists.end());
    //	center.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
    //}
    center.radius = (box.size.height + box.size.width) / 4.0;
    centers.push_back(center);

  }
}



//note: required level 5,6 or 7 to see contours of circles
static void on_trackbar(int, void*)
{
    Mat cnt_img = Mat::zeros(g_height,g_width,CV_8UC3);
    //levels=5 seems good--finds circle contours at min of tiny contours
    cout<<"levels = "<<levels<<endl;
    int _levels = levels - 3;
    drawContours( cnt_img, contours, _levels <= 0 ? 3 : -1, Scalar(128,255,255),
                  3, LINE_AA, hierarchy, std::abs(_levels) );

    
    imshow("contours", cnt_img);
    g_cnt_img = cnt_img;
}

static void on_imagebar(int, void*)
{
    Mat image = g_vec_of_images[g_imagenum];
    cout<<"g_imagenum = "<<g_imagenum<<endl;
    imshow("images", image);
}


int main(int argc, char** argv) {
    //ros::init(argc, argv, "contour_finder");
    //ros::NodeHandle n; //        

    /*
    if( argc != 2)
    {
     cout <<" Usage: rosrun example_opencv test_find_contours ImageToLoadAndDisplay" << endl;
     return -1;
    }
    */
     Mat image;
    read_images();
    //image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file
    image = g_vec_of_images[0];

    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    g_width=image.cols;
    g_height=image.rows;
    cout << "Width : " << g_width << endl;
    cout << "Height: " << g_height << endl;

    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", image );                   // Show our image inside it.
    
    //convert image to binary:
    Mat grayscaleImage;
    cvtColor(image, grayscaleImage, CV_BGR2GRAY);
    
    Mat binarizedImage;
    double thresh=THRESH;
    cout<<"enter thresh (e.g. 125): ";
    //cin>>thresh;    //looked OK at 100 and 150; NOT 50  or 200; try 125
    threshold(grayscaleImage, binarizedImage, thresh, 255, THRESH_BINARY);

    namedWindow( "binary", 1 );
    imshow( "binary", binarizedImage );  
    
     vector<vector<Point> > contours0;
    findContours( binarizedImage, contours0, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    contours.resize(contours0.size());
    double eps = 0.1;
    for( size_t k = 0; k < contours0.size(); k++ ) 
        approxPolyDP(Mat(contours0[k]), contours[k], eps, true);

    namedWindow( "contours", 1 );
    namedWindow( "images", 1 );
    //levels=5;
    
    createTrackbar( "levels+3", "contours", &levels, 7, on_trackbar );
    createTrackbar( "image_num", "images", &g_imagenum, g_n_images-1, on_imagebar );


    on_trackbar(0,0);   
    on_imagebar(0,0);
    cout<<"finding circles..."<<endl;
    //vector<Center> curCenters;
    //findCircles(grayscaleImage, binarizedImage, curCenters);
    
    //find a 5x5 grid of circles
    //Size patternsize(5,5); //number of centers
    Size patternsize(5,5); //DEBUG TEST
    
    
    //Mat gray = ....; //source image
    vector<Point2f> centers; //this will be filled by the detected centers
    //	auto success = findCirclesGrid(cvImage, cv::Size(5, 7), this->centers, CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING, blobDetector);

    //bool patternfound = findCirclesGrid(grayscaleImage, patternsize, centers);
    //NEWER: find grid of circles:
    
 
//SimpleBlobDetector::Params params;
//params.maxArea = 10e4;

// Set up the detector with default parameters.
CircleDetector circleDetector; 

//SimpleBlobDetector detector;
//const Ptr<FeatureDetector>& blobDetector=new SimpleBlobDetector();
//Ptr<FeatureDetector> blobDetector = new SimpleBlobDetector(params);

  /**
   *  @brief circle_detector_ptr_ is a custom blob detector which localizes circles better than simple blob detection
   */
  cv::Ptr<cv::CircleDetector> circle_detector_ptr_;

  /**
   *  @brief blob_detector_ptr_ is a simple blob detector
   */
  //cv::Ptr<cv::FeatureDetector> blob_detector_ptr_;
  
  circle_detector_ptr_ = cv::CircleDetector::create();
  //blob_detector_ptr_ = cv::SimpleBlobDetector::create(simple_blob_params);
 /*      
  *         Mat image;
        image = g_vec_of_images[16];
     bool isFound = findCirclesGrid(image, patternSize, centers, CALIB_CB_ASYMMETRIC_GRID, blobDetector);
*/
     
     cout<<"processing "<<g_n_images<<" images"<<endl;
     int failures=0;
    //for (int i_image = 0; i_image < g_n_images; i_image++) {
    for (int i_image = 42; i_image < g_n_images; i_image++) {
        
        Mat image;
        image = g_vec_of_images[i_image];
        cout<<"i_image = "<<i_image<<endl;
        Point2f center; // = centers[i];
        cvtColor(image, grayscaleImage, CV_BGR2GRAY);
        /*
       Mat binarizedImage;
        double thresh=THRESH;
        //cout<<"enter thresh (e.g. 125): ";
        //cin>>thresh;    //looked OK at 100 and 150; NOT 50  or 200; try 125
        threshold(grayscaleImage, binarizedImage, thresh, 255, THRESH_BINARY);   
        */
        //bool patternfound = findCirclesGrid(grayscaleImage, patternsize, centers, CALIB_CB_CLUSTERING);  //CALIB_CB_CLUSTERING
        //bool patternfound = findCirclesGrid(grayscaleImage, patternsize, centers, CALIB_CB_ASYMMETRIC_GRID);  //CALIB_CB_CLUSTERING
        //bool patternfound = findCirclesGrid(grayscaleImage, patternsize, centers, CALIB_CB_SYMMETRIC_GRID);  //CALIB_CB_CLUSTERING
        //bool patternfound = findCirclesGrid(binarizedImage, patternsize, centers, CALIB_CB_SYMMETRIC_GRID);  //CALIB_CB_CLUSTERING
        //bool patternfound = findCirclesGrid(grayscaleImage, patternsize, centers, CALIB_CB_SYMMETRIC_GRID,blobDetector);  //CALIB_CB_CLUSTERING
         //          successful_find = cv::findCirclesGrid(image_roi_, pattern_size, observation_pts_, cv::CALIB_CB_SYMMETRIC_GRID,
                     //                           circle_detector_ptr_);
        bool patternfound = findCirclesGrid(grayscaleImage, patternsize, centers, CALIB_CB_SYMMETRIC_GRID, circle_detector_ptr_);  //CALIB_CB_CLUSTERING
                                                
        if (patternfound) {
            cout << "pattern found in image "<<i_image << endl;
            int ncircles = centers.size();
            cout << "found " << ncircles << " circles" << endl;
            for (int i_circle=0;i_circle<ncircles;i_circle++) {
                center = centers[i_circle];
                circle( image, center, 2, Scalar(0,0,255), 2 );
            }
        } else {
            cout << "pattern not found in image"<<i_image << endl;
            failures++;
           cout<<"enter 1 for next image: ";
           cin>>g_ans;           
        }
        //center = centers[i_image];
        //circle( image, center, 2, Scalar(0,0,255), 2 );
        g_vec_of_images[i_image]= image;
        //cout<<"enter 1 for next image: ";
        //cin>>g_ans;
    }
    //drawChessboardCorners(img, patternsize, Mat(centers), patternfound);
     cout<<" there were "<<failures<<" failures"<<endl;
    //int ncircles = curCenters.size();
     /*
    int ncircles = centers.size();

    cout<<"found "<<ncircles<<" centers"<<endl;
    for (int i=0;i<ncircles;i++) {
        //Center center = curCenters[i];
        //Center center = centers[i];
        Point2f center = centers[i];
        
        cout<<"drawing circle "<<i<<endl;
        //circle( image, center.location, 2, Scalar(0,0,255), 2 );
        //circle( g_cnt_img, center.location, 3, Scalar(0,0,255), 3 );
        circle( image, center, 2, Scalar(0,0,255), 2 );
        circle( g_cnt_img, center, 3, Scalar(0,0,255), 3 );        

    } */

  imshow("Display window", image );
  imshow("contours", g_cnt_img);
  waitKey();
    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}

