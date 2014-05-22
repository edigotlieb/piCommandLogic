#include "ImageProcessing.h"

using namespace std;
using namespace cv;

bool showPhotos = false;

FilterApplier::FilterApplier()
{
    _error = false;
    
    // Set up VideoCapture
    _vidCap.open(0);
    _error = !_vidCap.isOpened()
    
    _vidCap.set(CV_CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH);
    _vidCap.set(CV_CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT);
    
    // Set up FilterApplier
    _error = _error || (!FilterApplier::importMahalanobis()) && (!FilterApplier::importLines());
    FilterApplier::createDiamond();
    _matOrig = new Mat();
    _matNew = new Mat();
}

FilterApplier::~FilterApplier()
{
    for(int i = 0; i < GRAYSCALE_MAX_VALUE; i++)
    {
	for(int j = 0; j < GRAYSCALE_MAX_VALUE; j++)
	{
	    delete [] _mahalanobisData[i][j];
	}
    }
    for(int i = 0; i < GRAYSCALE_MAX_VALUE; i++)
    {
	delete [] _mahalanobisData[i];
    }
    delete [] _mahalanobisData;
    delete _matOrig;
    delete _matNew;
}

bool FilterApplier::isOpened()
{
    return !_error;
}

void FilterApplier::findTrees(int treeLocations[MAX_NUM_OF_TREES][2])
{
    _vidCap >> *_matOrig;
    FilterApplier::applyAllFilters(treeLocations);
}

void FilterApplier::applyAllFilters(int treeLocations[MAX_NUM_OF_TREES][2])
{
    if(showPhotos)
	imshow("Original Image", *_matOrig);
    
    // Phase 1: Median Filter
    medianBlur(*_matOrig, *_matNew, MEDIAN_FILTER_KERNEL_SIZE);
    new2orig();
    if(showPhotos)
	imshow("Blurred Image", *_matOrig);

    // Phase 2: Color Distance
    FilterApplier::mahalDistance();
    if(showPhotos)
	imshow("Mahal Image", *_matOrig);
    
    // Phase 3: Threshold
    threshold(*_matOrig, *_matNew, THRESHOLD_VALUE, GRAYSCALE_MAX_VALUE - 1, THRESH_BINARY_INV);
    new2orig();
    if(showPhotos)
	imshow("Thresh Image", *_matOrig);

    // Phase 4: Cleanup
    dilate(*_matOrig, *_matNew, _diamond);
    new2orig();
    FilterApplier::fillSmallHoles(SMALL_HOLES_CONSTANT);
    FilterApplier::removeSmallTrees(SMALL_TREES_CONSTANT);
    if(showPhotos)
	imshow("Cleanup Image", *_matOrig);

    // Phase 4: Hough Transform and Peek finding
    int peaks[NUM_OF_PEAKS];
    int numOfPeaks;
    FilterApplier::houghTransform(peaks, &numOfPeaks);
    if(showPhotos)
	imshow("Morph Image", *_matOrig);

    // Phase 5: Morphologies
    FilterApplier::applyMorphologies(peaks, numOfPeaks);
    FilterApplier::removeSmallTrees(SMALL_TREES_CONSTANT);
    if(showPhotos)
    {
	imshow("Final Image", *_matOrig);
	waitKey(0);
    }

    // Phase 6: Blob Detection
    FilterApplier::blobAnalysis(treeLocations);
}

void FilterApplier::new2orig()
{
    Mat *tmp = _matOrig;
    ~*_matOrig;
    _matOrig = _matNew;
    _matNew = tmp;
}

int FilterApplier::importMahalanobis()
{
    // Allocate memory for all the data (Not enough space on stack)
    _mahalanobisData = new int **[GRAYSCALE_MAX_VALUE];
    if(_mahalanobisData == NULL)
    {
	return MEMORY_ALLOC_ERR;
    }
    for(int i = 0; i < GRAYSCALE_MAX_VALUE; i++)
    {
	_mahalanobisData[i] = new int *[GRAYSCALE_MAX_VALUE];
	if(_mahalanobisData[i] == NULL)
	{
	    return MEMORY_ALLOC_ERR;
	}
    }
    for(int i = 0; i < GRAYSCALE_MAX_VALUE; i++)
    {
	for(int j = 0; j < GRAYSCALE_MAX_VALUE; j++)
	{
	    _mahalanobisData[i][j] = new int[GRAYSCALE_MAX_VALUE];
	    if(_mahalanobisData[i][j] == NULL)
	    {
		return MEMORY_ALLOC_ERR;
	    }
	}
    }

    // Open file for reading
    ifstream input;
    input.open("./ConfFiles/mahalanobis.conf", ios::in | ios::binary);
    if(!input.is_open())
    {
	return FILE_OPEN_ERR;
    }
    char num[BUFFER_LENGTH];
    
    // Read all the values
    for(int i = 0; i < GRAYSCALE_MAX_VALUE; i++)
    {
	for(int j = 0; j < GRAYSCALE_MAX_VALUE; j++)
	{
	    for(int k = 0; k < GRAYSCALE_MAX_VALUE; k++)
	    {
		// Note: Matlab saves col by col and not row by row => index order is (i, k, j)
		input.read(num, sizeof (int));
		_mahalanobisData[i][k][j] = *((int *) num);
	    }
	}
    }
    input.close();
    return 0;
}

int FilterApplier::importLines()
{
    // Create streams for reading the files containing the line matrices and the file containing 
    // their sizes. Also create buffers for reading from them
    ifstream sizes, line;
    char sizeBuf[BUFFER_LENGTH], lineBuf[BUFFER_LENGTH];
    
    // Holds current matrix size (given by file)
    uint16_t rows, cols;
    
    // A char* that holds the current filename of line to read from
    char fileName[BUFFER_LENGTH];
    
    // Pointer to the current matrix line (efficient access to matrix)
    uchar *data;
    
    // Open the sizes file
    sizes.open("./ConfFiles/lines/matSizes.conf", ios::in | ios::binary);
    if(!sizes.is_open())
    {
	return FILE_OPEN_ERR;
    }
    
    for(int i = 0; i < NUM_OF_LINES; i++)
    {
	// Read sizes of the current matrix and create it
	sizes.read(sizeBuf, sizeof(uint16_t));
	cols = *((uint16_t *) sizeBuf);
	sizes.read(sizeBuf, sizeof(uint16_t));
	rows = *((uint16_t *) sizeBuf);
	_lines[i] = Mat(rows, cols, CV_8UC1);
	
	// Create matching filename and open it
	sprintf(fileName, "./ConfFiles/lines/line%d", i);
	line.open(fileName, ios::in | ios::binary);
	if(!line.is_open())
	{
	    return FILE_OPEN_ERR;
	}
	
	// Read matrix data
	for(int j = 0; j < rows; j++)
	{
	    data = _lines[i].ptr<uchar>(j);
	    for(int k = 0; k < cols; k++)
	    {
		line.read(lineBuf, sizeof(uint8_t));
		data[k] = (unsigned char) *((uint8_t *) lineBuf);
	    }
	}
	line.close();
    }
    sizes.close();
    return 0;
}

void FilterApplier::mahalDistance()
{
    // Mahalanobis requires int values. In the end we convert to unsigned char (grayscale)
    *_matNew = Mat(_matOrig->rows, _matOrig->cols, CV_8UC1);
    Mat tmp = Mat(_matOrig->rows, _matOrig->cols, CV_32SC1);

    int *pTmp;
    unsigned char *pOrig;
    
    // Max and min are essential for the conversion to grayscale
    int mahal, max = 0, min = INT_MAX;
    
    // Assigns for every pixel the mahalanobis value as appears in _mahalanobisData (imported from 
    // file using importMahalanobis() in the ctor)
    for(int i = 0; i < _matOrig->rows; i++)
    {
	pTmp = tmp.ptr<int>(i);
	pOrig = _matOrig->ptr<unsigned char>(i);
	for(int j = 0; j < _matOrig->cols; j++)
	{
	    int b = pOrig[3 * j], g = pOrig[3 * j + 1], r = pOrig[3 * j + 2];
	    mahal = _mahalanobisData[r][g][b];
	    pTmp[j] = mahal;
	    if(mahal > max)
	    {
		max = mahal;
	    }
	    if(mahal < min)
	    {
		min = mahal;
	    }
	}
    }
    
    // Converts the given matrix to grayscale
    tmp.convertTo(*_matNew, CV_8UC1, 255.0 / (max - min), -255.0 * min / (max - min));
    new2orig();
}

void FilterApplier::houghTransform(int *thetas, int *numOfUsedThetas)
{
    // Find guesses for angles of trees
    vector<Vec2f> lines;
    HoughLines(*_matOrig, lines, 1, DEG_TO_RAD, HOUGH_THERSHOLD);
    *numOfUsedThetas = 0;

    // Remove irrelevant guesses
    float theta, thetaCenter = RIGHT_ANGLE;
    for(int i = 0; i < lines.size() && *numOfUsedThetas < NUM_OF_PEAKS; i++)
    {
	theta = RIGHT_ANGLE - lines[i][1] * RAD_TO_DEG;

	if((!containsTheta(thetas, *numOfUsedThetas, EPS, theta)) && (thetaInRange(theta, 
		thetaCenter, THETA_RANGE) || thetaInRange(theta, -thetaCenter, THETA_RANGE)) && 
		(*numOfUsedThetas < NUM_OF_PEAKS))
	{
	    thetas[(*numOfUsedThetas)++] = theta;
	}
    }
}

bool FilterApplier::containsTheta(int *usedThetas, int numUsedThetas, float eps, float theta)
{
    float upperBound, lowerBound;
    for(uint8_t i = 0; i < numUsedThetas; i++)
    {
	bool upper = false, lower = false;
	upperBound = usedThetas[i] + eps;
	if(upperBound > RIGHT_ANGLE)
	{
	    upper = (theta >= usedThetas[i] && theta <= RIGHT_ANGLE) || (theta <= upperBound - 2 * 
		    RIGHT_ANGLE);
	}
	else
	{
	    upper = theta <= upperBound;
	}
	lowerBound = usedThetas[i] - eps;
	if(lowerBound < -RIGHT_ANGLE)
	{
	    lower = (theta <= usedThetas[i] && theta >= -RIGHT_ANGLE) || (theta >= upperBound + 2 * 
		    RIGHT_ANGLE);
	}
	else
	{
	    lower = theta >= lowerBound;
	}
	if(upper && lower)
	{
	    return true;
	}
    }
    return false;
}

bool FilterApplier::thetaInRange(int theta, int theta_center, int theta_range)
{
    return (theta > theta_center - theta_range) && (theta < theta_center + theta_range);
}

void FilterApplier::applyMorphologies(int *peaks, int size)
{
    Mat matDst;
    *_matNew = Mat::zeros(_matOrig->rows, _matOrig->cols, _matOrig->type());
    for(int i = 0; i < size; i++)
    {
	morphologyEx(*_matOrig, matDst, MORPH_OPEN, _lines[peaks[i] + RIGHT_ANGLE]);
	bitwise_or(matDst, *_matNew, *_matNew);
    }
    new2orig();
}

void FilterApplier::createDiamond()
{
    _diamond = Mat(DIAMOND_SIZE, DIAMOND_SIZE, CV_8UC1);
    for(int i = 0; i < DIAMOND_SIZE; i++)
    {
	for(int j = 0; j < DIAMOND_SIZE; j++)
	{
	    bool b = !((i == 0 && j == 0) || (i == 0 && j == DIAMOND_SIZE - 1) ||
		    (i == DIAMOND_SIZE - 1 && j == 0) || (i == DIAMOND_SIZE - 1 && j == DIAMOND_SIZE
		    - 1));
	    _diamond.at<bool>(i, j) = b;
	}
    }
}

void FilterApplier::removeSmallTrees(double size)
{
    // Find contours of all objects
    vector<Vec4i> hierarchy;
    vector<vector <Point> > contours;
    _matOrig->copyTo(*_matNew);

    findContours(*_matOrig, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Remove all objects with an area too small for a tree
    double area;
    for(int i = 0; i < contours.size(); i++)
    {
	area = contourArea(contours[i]);
	if(area >= 0 && area <= size)
	{
	    drawContours(*_matNew, contours, i, 0, CV_FILLED, LINE_TYPE, hierarchy);
	}
    }
    new2orig();
}

void FilterApplier::fillSmallHoles(double size)
{
    // Find contours of all holes (by reversing the image and finding contours)
    vector<Vec4i> hierarchy;
    vector<vector <Point> > contours;

    threshold(*_matOrig, *_matNew, IMFILL_THRESHOLD, GRAYSCALE_MAX_VALUE - 1, THRESH_BINARY_INV);
    findContours(*_matNew, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Remove all small holes
    for(int i = 0; i < contours.size(); i++)
    {
	double area = contourArea(contours[i]);
	if(area >= 0 && area <= size)
	{
	    drawContours(*_matOrig, contours, i, GRAYSCALE_MAX_VALUE - 1, CV_FILLED, LINE_TYPE, 
		    hierarchy);
	}
    }
    ~*_matNew;
}

void FilterApplier::blobAnalysis(int treeLocations[MAX_NUM_OF_TREES][2])
{
    // Find contours of all objects
    vector<vector <Point> > contours;
    findContours(*_matOrig, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    // For each tree find a binding rectangle and put its left border and right border into array
    int i;
    for(i = 0; i < contours.size() && i < MAX_NUM_OF_TREES; i++)
    {
	Rect r = boundingRect(contours.at(i));
	treeLocations[i][0] = r.x * PIXEL_TO_ANGLE;
	treeLocations[i][1] = (r.x + r.width) * PIXEL_TO_ANGLE;
    }
    
    // Fill the rest of the array with zeros
    for(; i < MAX_NUM_OF_TREES; i++)
    {
	treeLocations[i][0] = (treeLocations[i][1] = 0);
    }
}