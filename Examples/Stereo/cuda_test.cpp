#include <iostream>
#include <ctime>
#include <cmath>
#include "bits/time.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>

int main() {

    std::clock_t begin = std::clock();

    try {
        cv::String filename = "";
        cv::Mat srcHost = cv::imread(filename, cv::IMREAD_GRAYSCALE);

        for (int i = 0; i < 100; i++) 
        {
            cv::cuda::GpuMat dst, src;
            src.upload(srcHost);

            //cv::cuda::threshold(src,dst,128.0,255.0, CV_THRESH_BINARY);
            cv::cuda::bilateralFilter(src,dst,3,1,1);

            cv::Mat resultHost;
            dst.download(resultHost);
        }

        std::clock_t end = std::clock();
        std::cout << "GPU time: " << double(end-begin) / CLOCKS_PER_SEC  << std::endl;

        std::clock_t begin2 = std::clock();
        for (int i = 0; i < 100; i++) 
        {
            cv::Mat dst;
            cv::bilateralFilter(srcHost,dst,3,1,1);
        }
        std::clock_t end2 = std::clock();
        std::cout << "CPU time: " << double(end2-begin2) / CLOCKS_PER_SEC  << std::endl;

        //cv::imshow("Result",resultHost);
        //cv::waitKey();

    } catch (const cv::Exception& ex) {
        std::cout << "Error: " << ex.what() << std::endl;
    }
}