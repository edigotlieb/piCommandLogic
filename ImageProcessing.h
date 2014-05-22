/* 
 * File:   ImageProcessing.h
 * Author: danarad
 *
 * Created on May 8, 2014, 1:07 PM
 */

#include <list>
#include <limits>
#include <string>
#include <fstream>
#include <cmath>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <climits>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>

#ifndef IMAGEPROCESSING_H
#define	IMAGEPROCESSING_H

using namespace std;
using namespace cv;

// Image Constants
#define IMAGE_WIDTH 160
#define IMAGE_HEIGHT 120
#define PIXEL_TO_ANGLE 0.3375
#define GRAYSCALE_MAX_VALUE 256

// General
#define BUFFER_LENGTH 128
#define DEG_TO_RAD (CV_PI / 180)
#define RAD_TO_DEG (180 / CV_PI)
#define RIGHT_ANGLE 90

// Median Blur
#define MEDIAN_FILTER_KERNEL_SIZE 3

// Threshold
#define THRESHOLD_VALUE 10

// Morphologies
#define DIAMOND_SIZE 3
#define NUM_OF_LINES 181
#define SMALL_HOLES_CONSTANT 125
#define SMALL_TREES_CONSTANT 1000
#define LINE_TYPE 8
#define IMFILL_THRESHOLD 100

// Hough Transform
#define NUM_OF_PEAKS 5
#define THETA_RANGE 30
#define EPS 5
#define MAX_NUM_OF_TREES 10
#define HOUGH_THERSHOLD 75

// ERRORS
#define MEMORY_ALLOC_ERR 1
#define FILE_OPEN_ERR 2


struct FilterApplier 
{
    /**
     * Creates a filter applier. Imports data files in locations: ./ConfFiles/mahalanobis.conf, 
     * ./ConfFiles/lines/matSizes.conf and ./ConfFiles/lines/line%d (where %d is a number btween 0 
     * and 180). 
     */
    FilterApplier();
    
    /**
     * Frees all data allocated in constructor. 
     */
    ~FilterApplier();
    
    /**
     * Checks if the object was created without errors. 
     * @return true if object was created without errors and false otherwise. 
     */
    bool isOpened();
    
    /**
     * Takes a photo through the camera and finds trees in it. 
     * @param treeLocations - Array that will contain pairs of tree left border and right border. 
     * Each row is a pair and there are MAX_NUM_OF_TREES rows. If less trees than MAX_NUM_OF_TREES 
     * were found, the rest of the rows will be filled with zeros. 
     */
    void findTrees(int treeLocations[MAX_NUM_OF_TREES][2]);

private:
    // Complete processing of the image found in _matOrig. 
    void applyAllFilters(int treeLocations[MAX_NUM_OF_TREES][2]);
    // Transfers _matNew to _matOrig and clears _matNew. 
    void new2orig();
    // Imports the data for mahalDistance from ./ConfFiles/mahalanobis.conf. 
    int importMahalanobis();
    // Imports the data for applyMorphologies from ./ConfFiles/lines/matSizes.conf and 
    // ./ConfFiles/lines/line%d (where %d is a number between 0 and 180).
    int importLines();
    // Calculates the "color distance" of each pixel from the given color profiles (configured in 
    // the calibration). 
    void mahalDistance();
    // Makes guesses for angles in which trees can be found. 
    void houghTransform(int *thetas, int *numUsedThetas);
    // Helper functions for houghTransform()
    bool containsTheta(int *usedThetas, int numUsedThetas, float eps, float theta);
    bool thetaInRange(int theta, int theta_center, int theta_range);
    // Applies the open morphology for a set of given angles. 
    void applyMorphologies(int *peaks, int numOfPeaks);
    // Creates a 3x3 diamond shaped matrix used for dilation. 
    void createDiamond();
    // Removes blobs which are too small to be considered trees. 
    void removeSmallTrees(double size);
    // Fills small holes in trees. 
    void fillSmallHoles(double size);
    // Detects blobs, confines them in rectangles, and inserts left and right borders into the given 
    // matrix. 
    void blobAnalysis(int treeLocations[MAX_NUM_OF_TREES][2]);

    VideoCapture _vidCap;
    int ***_mahalanobisData;
    Mat *_matOrig;
    Mat *_matNew;
    Mat _diamond;
    Mat _lines[NUM_OF_LINES];
    bool _error;
};

#endif	/* IMAGEPROCESSING_H */