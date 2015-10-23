#ifndef MARKER_HPP
#define MARKER_HPP

#include <vector>

#include <opencv2/opencv.hpp>

namespace nxtar{

    /**
     * Termination criteria for OpenCV's iterative algorithms.
     */
    const cv::TermCriteria TERM_CRITERIA = cv::TermCriteria(CV_TERMCRIT_EPS +
                                                            CV_TERMCRIT_ITER,
                                                            30,
                                                            0.1);

    class Marker;

    typedef std::vector<cv::Point2f>             points_vector;
    typedef std::vector<cv::Point3f>             points_vector_3D;
    typedef std::vector<std::vector<cv::Point> > contours_vector;
    typedef std::vector<Marker>                  markers_vector;

    class Marker{
        public:
            ~Marker();
            points_vector points;
            cv::Mat       translation;
            cv::Mat       rotation;
            int code;
    };

    /**
     * Detect all 5x5 markers in the input image and return their codes in the
     * output vector. 
     */
    void getAllMarkers(markers_vector &, cv::Mat &);

    /**
     * Find a chessboard calibration pattern in the input image. Returns true
     * if the pattern was found, false otherwise. The calibration points
     * detected on the image are saved in the output vector.
     */
    bool findCalibrationPattern(points_vector &, cv::Mat &);

    /**
     * Sets the camera matrix and the distortion coefficients for the camera
     * that captured the input image points into the output matrices. Returns
     * the reprojection error as returned by cv::calibrateCamera.
     */
    double getCameraParameters(cv::Mat &,
                               cv::Mat &,
                               std::vector<points_vector> &,
                               cv::Size);

    /**
     * Obtains the necesary geometric transformations necessary to move a reference
     * unitary polygon to the position and rotation of the markers passed as input.
     * The obtained transformations are given relative to a camera centered in the
     * origin and are saved inside the input markers.
     */
    void estimateMarkerPosition(markers_vector &, cv::Mat &, cv::Mat &);

}

#endif
