#include <iostream>
#include <cstdlib>
#include <ctime>
#include <csignal>
#include <vector>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "marker.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::vector;

static bool done = false;

void manageSignal(int signal){
    // On SIGINT mark the application as ready to terminate.
    if(signal == SIGINT)
        done = true;
}

int main(int argc, char** argv){
    bool bSuccess, camera_calibrated = false;
    char key;
    cv::Mat frame, camera, distortion, out_frame;
    nxtar::markers_vector codes;
    nxtar::points_vector calibrationPoints;
    vector<nxtar::points_vector> calibrationSamples;
    time_t time_a = 0;
    double error;

    // Add signal handler.
    signal(SIGINT, manageSignal);

    // Open video camera. Exit if failed to open it.
    cv::VideoCapture cap("data/calib.mp4");

    if(!cap.isOpened()){
         cerr << "Could not open the default camera." << endl;
         return EXIT_FAILURE;
    }

    // Create the main application window.
    cv::namedWindow("OpenCV", CV_WINDOW_AUTOSIZE);
    
    /**************************************************************************
     * CAMERA CALIBRATION STEP.                                               *
     **************************************************************************/
    try{
        while(!done){
            // Read a frame from the camera.
            bSuccess = cap.read(frame);

            // If the camera failed to return a frame, exit.
            if (!bSuccess){
                cap.set(CV_CAP_PROP_POS_FRAMES, 1);
                continue;
            } else {
                cv::resize(frame, frame, cv::Size(), 0.5, 0.5);
            }
            
            // Detect the camera calibration pattern.
            if(nxtar::findCalibrationPattern(calibrationPoints, frame)){

                if((time(NULL) - time_a) > 2){
                    nxtar::points_vector tempVector;

                    // Save the calibration points to the samples list.
                    for(size_t i = 0; i < calibrationPoints.size(); i++)
                        tempVector.push_back(calibrationPoints[i]);
                    calibrationPoints.clear();
                    calibrationSamples.push_back(tempVector);

                    std::cout << "Sample "
                              << calibrationSamples.size()
                              <<" saved."
                              << std::endl;

                    // If 10 samples have been saved.
                    if(calibrationSamples.size() >= 10){
                        std::cout << "Enough samples taken." << std::endl;

                        // Get the camera parameters and calibrate.
                        cv::Size image_size = frame.size();

                        error = nxtar::getCameraParameters(camera,
                                                           distortion,
                                                           calibrationSamples,
                                                           image_size);
                        camera_calibrated = true;
                        std::cout << "getCameraParameters returned "
                                  << error
                                  << std::endl;

                        // Clear the calibration samples list.
                        for(size_t i = 0; i < calibrationSamples.size(); i++){
                            calibrationSamples[i].clear();
                        }
                        calibrationSamples.clear();
                        
                        done = true;
                    }
                    time_a = time(NULL);
                }
            }
            calibrationPoints.clear();

            // Show the original frame for reference.
            cv::imshow("OpenCV", frame);

            // Wait for a key press. Exit if the user pressed the escape key.
            key = cv::waitKey(1);
            if(key == 27 /* escape key */)
                done = true;
        }
    }catch(cv::Exception e){
        std::cerr << e.msg << std::endl;
        std::cerr << e.err << std::endl;
        goto finish;
    }

    done = false;
    cap.open("data/marker.mp4");

    /**************************************************************************
     * MARKER DETECTION STEP.                                                 *
     **************************************************************************/
    try{
        while(!done){
            // Read a frame from the camera.
            bSuccess = cap.read(frame);

            // If the camera failed to return a frame, exit.
            if (!bSuccess){
                cap.set(CV_CAP_PROP_POS_FRAMES, 1);
                continue;
            } else {
                cv::resize(frame, frame, cv::Size(), 0.5, 0.5);
            }

            // Detect all markers in the video frame.
            nxtar::getAllMarkers(codes, frame);

            // If the camera has been calibrated, show the undistorted image.
            if(camera_calibrated){
                cv::undistort(frame, out_frame, camera, distortion);
                cv::imshow("Undistorted", out_frame);
                cv::imshow("Diff", frame - out_frame);
            }

            // Show the original frame for reference.
            cv::imshow("OpenCV", frame);

            // Wait for a key press. Exit if the user pressed the escape key.
            key = cv::waitKey(1);
            if(key == 27 /* escape key */)
                done = true;
        }
    }catch(cv::Exception e){
        std::cerr << e.msg << std::endl;
        std::cerr << e.err << std::endl;
    }

 finish:
    cap.release();
    
    return EXIT_SUCCESS;
}
