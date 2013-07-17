#include <iostream>
#include <time.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\opencv.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <Windows.h>
#include <FC1\PGRFlyCapture.h>
#include <FlyCapture2.h>
#include <math.h>
#include <ostream>
#include <fstream>

#define DEBUG(x) cout << "DEBUG: " << x << endl;
static const int OUTPUT_RATIO = 4;
using namespace std;
using namespace cv;
using namespace FlyCapture2;

int str_to_int(std::string str){
	int res;
	std::istringstream is(str);
	if(!(is >> res)){
		cout << "Could not format string: " << str << ", to an int." << endl;
		throw "Could not format string: " + str + ", to an int";
	}else {
		return res;
	}
}

bool find_movement(const IplImage * prev, const IplImage * curr, const IplImage * next, int threshold){

	int window = 200;
	bool movement = false;
	IplImage *d1,*d2,*result;
	d1 = cvCreateImage(cvSize(prev->width,prev->height),IPL_DEPTH_8U,1);
	d2 = cvCreateImage(cvSize(prev->width,prev->height),IPL_DEPTH_8U,1);
	result = cvCreateImage(cvSize(prev->width,prev->height),IPL_DEPTH_8U,1);

	cvAbsDiff(next,curr,d1);
	cvAbsDiff(curr,prev,d2);

	cvXor(d1,d2,result);
	int middle_y = result->height/2;
	int middle_x = result->width/2;

	// Center window
	int x1=0,y1=0,x2=0,y2=0;
	cvThreshold(result,result,threshold,255,CV_THRESH_BINARY);

	Mat result2(result);
	//	for(int i = middle_x-window; i < middle_x+window; i++){
	//		for(int j = middle_y-window; j < middle_y+window; j++){
	for(int i = 0; i < result->height; i++){
		for(int j = 0; j < result->width;j++){
			if(result2.at<uchar>(i,j) > 0)
			{
				y1 = i;
				x1 = j;
				movement = true;
				i = result2.cols;
				break;
			}
		}
	}

	return movement;
}



void find_center(const Mat & mat, Point & p){
	int x1=0,x2=0,y1=0,y2=0;
	for(int i = 0; i < mat.rows; i++){
		for(int j = 0; j < mat.cols; j++){
			if(mat.at<uchar>(i,j) > 0){
				x1 = j, y1 = i;
				goto next;		
			}
		}
	}
next:
	for(int i = mat.rows - 1; i >= 0; i--){
		for(int j = mat.cols - 1; j >= 0; j--){
			if(mat.at<uchar>(i,j) > 0){
				x2 = j, y2 = i;
				goto final;		
			}
		}
	}
	final:

	int avg_x = (x1 + x2) / 2;
	int avg_y = (y1 + y1) / 2;

	p = Point(avg_x,avg_y);
}
void analyze_subtraction(Mat & aprox,string path){
	Mat res(aprox.size(), CV_8UC1, Scalar::all(0));

	int ymax=0,y1=0,y2=0;
	for(int x = 0; x < aprox.cols; x++){
		for(int y = 0; y < aprox.rows; y++){
			if(aprox.at<uchar>(y,x) > 0){
				ymax = y;
				break;
			}
		}

		if(ymax == 0) continue;
		int min_y = (res.rows + ymax)/2;
		int max_y = (3*res.rows + ymax)/4;

		for(int y = min_y; y < max_y;y++){
			res.at<uchar>(y,x) = 255;
		}
	}

	string fn = path + "/res(1).bmp";
	imwrite(fn,res);

}

void analyze_subtraction(vector<Mat> & results, string path){

	Mat res(results[1].size(), CV_8UC1, Scalar::all(0));

	int ymax=0,y1=0,y2=0;
	for(size_t i  = 0; i < results.size(); i++){

		for(int x = 0; x < results[i].cols; x++){
			ymax = 0;
			for(int y = 0; y < results[i].rows; y++){
				if(results[i].at<uchar>(y,x) > 0){
					ymax = y;
					break;
				}
			}

			if(ymax == 0) continue;
			int min_y = (res.rows + ymax - 1)/2;
			int max_y = (3*res.rows + ymax - 3)/4;

			for(int y = min_y; y < max_y;y++){
				res.at<uchar>(y,x) = 255;
			}
		}
	}

	string fn = path + "/res(2).bmp";
	imwrite(fn,res);
}

bool background_subtraction(vector<int> & counters, string path){
	if(counters.size() < 3) 
		return false;
	Mat prev,curr,next,d1,d2,result,aprox(0,0,CV_8U);

	char f1[100],f2[100],f3[100];
	int i = 0;

	vector<int> tmp;
	vector<Mat> results;
	while(counters.size() - i >= 3){
		sprintf(f1,"%s/img (%d).bmp",path.c_str(),counters[i]);
		sprintf(f2,"%s/img (%d).bmp",path.c_str(),counters[i+1]);
		sprintf(f3,"%s/img (%d).bmp",path.c_str(),counters[i+2]);
		prev = imread(String(f1),CV_LOAD_IMAGE_GRAYSCALE);
		curr = imread(String(f2),CV_LOAD_IMAGE_GRAYSCALE);
		next = imread(String(f3),CV_LOAD_IMAGE_GRAYSCALE);


		if(!prev.data || !curr.data || !next.data){
			i++;
			continue;
		}
		if(aprox.rows == 0)
			aprox = Mat(prev.size(),CV_8UC1,Scalar::all(0));

		absdiff(next,curr,d1);
		absdiff(curr,prev,d2);
		result = Mat(curr.size(),CV_8UC1);
		bitwise_xor(d1,d2,result);

		threshold(result, result, 100, 255, CV_THRESH_BINARY);
		bitwise_or(result,aprox,aprox);
		sprintf(f1,"%s/result(%d).bmp",path.c_str(),i);
		tmp.push_back(i);
		results.push_back(result);
		imwrite(f1,result);
		i += 3;
	}
	string fn = path + "/aprox.bmp";
	imwrite(fn,aprox);
	analyze_subtraction(results,path);
	analyze_subtraction(aprox,path);
	counters.clear();
	counters = tmp;
	return true;
}

void print_to_file(string file_name,vector<int> & counters){
	ofstream myfile;

	myfile.open(file_name.c_str(),ios::out);
	int count = 0;

	for(unsigned int i = 0; i < counters.size(); i++){
		count = counters[i];
		myfile << count << endl;
	}
	myfile.close();
}

void run_from_file(vector<int> & counters, string path){

	cout << "run_from_file: " << path << endl;
	string line;
	string file_name = path + "/counters.txt";
	ifstream myfile (file_name);
	if (myfile.is_open()){
		int count = 0;
		while ( myfile.good()){
			getline (myfile,line);
			if(line == "") break;
			count = str_to_int(line);
			counters.push_back(count);
			cout << line << endl;
		}
		myfile.close();
	} else {
		cout << "Unable to open file"; 
		return;
	}
}


void run(vector<int> & counters,string path,int threshold){
	cout << "hej" << endl;
	Camera cam;
	clock_t t1, t2;
	Error error = cam.Connect(0);
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return;
	}
	cout << "Got connection" << endl;

	Image image,convertedImage;
	IplImage *dest,*prev,*curr,*next,*output,*orig;
	error = cam.StartCapture();

	if (error != PGRERROR_OK){
		error.PrintErrorTrace();
		return;
	}
	error = cam.RetrieveBuffer(&image);
	if (error != PGRERROR_OK){
		error.PrintErrorTrace();
		return;
	}

	dest = cvCreateImage(cvSize(image.GetCols(),image.GetRows()),IPL_DEPTH_8U,1);
	prev = cvCreateImage(cvSize(image.GetCols(),image.GetRows()),IPL_DEPTH_8U,1);
	curr = cvCreateImage(cvSize(image.GetCols(),image.GetRows()),IPL_DEPTH_8U,1);
	next = cvCreateImage(cvSize(image.GetCols(),image.GetRows()),IPL_DEPTH_8U,1);
	output = cvCreateImage(cvSize(image.GetCols()/4,image.GetRows()/4),IPL_DEPTH_8U,1);

	error = image.Convert( PIXEL_FORMAT_MONO8, &convertedImage );	
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return;
	}

	convertedImage.Save("image.bmp");

	int counter = 0;
	string win_name = "Free XXX WebCam Only 5 dollah!";
	char img_name[200];
	unsigned int rowBytes = 0;
	namedWindow(win_name,CV_WINDOW_AUTOSIZE);

	int res = 0;
	t1 = clock();
	while(2*CLOCKS_PER_SEC - res > 0){
		cam.RetrieveBuffer(&image);
		image.Convert( PIXEL_FORMAT_MONO8, &convertedImage );
		memcpy(curr->imageData,convertedImage.GetData(),convertedImage.GetDataSize());
		cvResize(curr,output);
		cvShowImage(win_name.c_str(),output);

		t2 = clock();
		res = t2 - t1;
		cout << "res: " << res << endl;
	}
	while(cvWaitKey(1) == -1){
		t1 = clock();
		cam.RetrieveBuffer(&image);
		image.Convert( PIXEL_FORMAT_MONO8, &convertedImage );
		memcpy(prev->imageData,convertedImage.GetData(),convertedImage.GetDataSize());

		cam.RetrieveBuffer(&image);
		image.Convert( PIXEL_FORMAT_MONO8, &convertedImage );
		memcpy(curr->imageData,convertedImage.GetData(),convertedImage.GetDataSize());

		cam.RetrieveBuffer(&image);
		image.Convert( PIXEL_FORMAT_MONO8, &convertedImage );
		memcpy(next->imageData,convertedImage.GetData(),convertedImage.GetDataSize());



		if(find_movement(prev,curr,next,threshold)){
			DEBUG("MOVEMENT!")
			sprintf(img_name,"%s/img (%d).bmp",path.c_str(),counter);
			cvSaveImage(img_name,curr);
			counters.push_back(counter);
		}

		cvResize(curr,output);
		cvShowImage(win_name.c_str(),output);
		t2 = clock();
		counter++;
		cout << "Fps: " << CLOCKS_PER_SEC/(double)(t2 - t1) << endl;
	}

	string file_name = path + "/counters.txt";
	print_to_file(file_name,counters);

	error = cam.StopCapture();
	if (error != PGRERROR_OK){
		error.PrintErrorTrace();
		return;
	}
	error = cam.Disconnect();
	if (error != PGRERROR_OK){
		error.PrintErrorTrace();
		return;
	}
}

bool DirectoryExists(string path) {
  DWORD attribs = ::GetFileAttributesA(path.c_str());
  if (attribs == INVALID_FILE_ATTRIBUTES) {
    return false;
  }
  return (attribs & FILE_ATTRIBUTE_DIRECTORY);
}

bool dir_exists(string path){
	DWORD ftyp = GetFileAttributesA(path.c_str());

	if(ftyp == ERROR_PATH_NOT_FOUND){
		return false;
	}

	else
		return true;
}

bool create_dir(string path){
#ifdef _WINDOWS_
	string cmd = "md " + path;
	system(cmd.c_str());
	return true;
#endif
	return false;
}

int main(int argc, char ** argv){

	char path[200];
	if(argc == 1){
		sprintf(path,"C:/Users/humanif/Documents/cam_test/images");
	}else if( argc > 1){
		sprintf(path,argv[1]);
	}
	vector<int> counters;

	if(argc == 3){

		if(DirectoryExists(path)){
			cout << "This program only works on windows." << endl;
			cout << "Or the path you specified already exists" << endl;
			return -1;
		}
		int threshold = str_to_int(argv[2]);
		if(threshold < 100)
			cout << "WARNING: Tests have shown that threshold values below 100 pick \
					up alot of unwanted movement, for example: leaves" << endl; 
		create_dir(path);
		run(counters,path,threshold);
		background_subtraction(counters,path);
	}else if(argc == 2){
		if(!DirectoryExists(path)){
			cout << "You need to specify an already existing directory with the needed files in \
					to run the program from file" << endl;
			return -1;
		}
		run_from_file(counters,path);
		background_subtraction(counters,path);
	}

	return 0;
}