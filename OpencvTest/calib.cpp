#ifdef DESKTOP
#include <iostream>
#endif

#include "marker.hpp"

namespace nxtar{
/******************************************************************************
 * PRIVATE CONSTANTS                                                          *
 ******************************************************************************/

    /**
     * Flags for the calibration pattern detection function.
     */
    static const int PATTERN_DETECTION_FLAGS = cv::CALIB_CB_ADAPTIVE_THRESH +
                                        cv::CALIB_CB_NORMALIZE_IMAGE + 
                                        cv::CALIB_CB_FAST_CHECK;

    /**
     * Size of the chessboard pattern image (columns, rows).
     */
    static const cv::Size CHESSBOARD_PATTERN_SIZE(6, 9);

    /**
     * Size of a square cell in the calibration pattern.
     */
    static const float SQUARE_SIZE = 1.0f;

/******************************************************************************
 * PUBLIC API                                                                 *
 ******************************************************************************/


    bool findCalibrationPattern(points_vector & corners, cv::Mat & img){
        bool patternfound;
        cv::Mat gray;

        // Convert the input image to grayscale and attempt to find the
        // calibration pattern.
        cv::cvtColor(img, gray, CV_BGR2GRAY);
        patternfound = cv::findChessboardCorners(gray, CHESSBOARD_PATTERN_SIZE, corners, PATTERN_DETECTION_FLAGS);

        // If the pattern was found then fix the detected points a bit.
        if(patternfound)
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), TERM_CRITERIA);

        // Render the detected pattern.
        cv::drawChessboardCorners(img, CHESSBOARD_PATTERN_SIZE, cv::Mat(corners), patternfound);

        return patternfound;
    }

    double getCameraParameters(cv::Mat & camera_matrix, cv::Mat & dist_coeffs, std::vector<points_vector> & image_points, cv::Size image_size){
        std::vector<cv::Mat> rvecs, tvecs;
        std::vector<points_vector_3D> object_points;
        points_vector_3D corner_points;

        // Build the reference object points vector.
        for(int i = 0; i < CHESSBOARD_PATTERN_SIZE.height; i++){
            for(int j = 0; j < CHESSBOARD_PATTERN_SIZE.width; j++){
                corner_points.push_back(cv::Point3f(float( j * SQUARE_SIZE ), float( i * SQUARE_SIZE ), 0));
            }
        }
        object_points.push_back(corner_points);
        object_points.resize(image_points.size(), object_points[0]);

        // Build a camera matrix.
        camera_matrix = cv::Mat::eye(3, 3, CV_64F);

        // Build the distortion coefficients matrix.
        dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);

        // Calibrate and return the reprojection error.
        return cv::calibrateCamera(object_points, image_points, image_size, camera_matrix, dist_coeffs, rvecs, tvecs, 0, TERM_CRITERIA);
    }
}
