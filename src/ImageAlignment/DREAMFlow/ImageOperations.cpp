//
// Created by Shane Griffith on 3/20/15.
//
/*
 * Image manipulation class.
 * */
#include "ImageOperations.h"
#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/xfeatures2d.hpp> //SAVE. Uncomment for BRIEF
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;

void ImageOperations::CheckType(Mat& image, int type) {
    /*Make sure the type abides.*/

    if(image.type() != type) {
        cout << "Couldn't handle image type." << endl;
        exit(-1);
    }
}

double * data(Mat & image) {
    return (double *) image.data;
}

double ImageOperations::ValueAtPixel(cv::Mat& image, int x, int y, int c) {
    return data(image)[(y*image.cols + x)*image.channels()+c];
}

void ImageOperations::SetPixel(cv::Mat& image, int x, int y, int c, double val) {
    data(image)[(y*image.cols + x)*image.channels()+c] = val;
}

Mat ImageOperations::imBRIEF(Mat& imsrc){
	/*
    //requires 48/2 + 9/2 = 28.5 => 29 pixels of border to find the descriptor for border pixels.
    int desc_size = 32;
    int border = 29;
    Mat reflect101;
    copyMakeBorder(imsrc,reflect101,border,border,border,border,BORDER_REFLECT_101);
    Ptr<cv::xfeatures2d::BriefDescriptorExtractor> bde = cv::xfeatures2d::BriefDescriptorExtractor::create(desc_size);
    
    //Define the locations where brief should be computed.
    vector<KeyPoint> plocs;
    for(int i=0; i<imsrc.rows; i++){
        for(int j=0; j<imsrc.cols; j++){
            plocs.push_back(KeyPoint(j+border, i+border, 0));
        }
    }
    
    //compute the descriptors
    cv::Mat desc(imsrc.rows, imsrc.cols, CV_8UC(desc_size), cv::Scalar::all(0));
    bde->compute(reflect101, plocs, desc);
    
    cv::Mat res(imsrc.rows, imsrc.cols, CV_8UC(desc_size), cv::Scalar::all(0));
    memcpy(res.data, desc.data, desc.rows*desc_size);
    return res;
    */
	return imsrc;
}

/*
 * imsift = 128 byte sift image.
 A snippit from Liu et al's "SIFT Flow ..." PAMI code.
 * */
Mat ImageOperations::imSIFT(Mat& imsrc, int cellSize, int stepSize, bool IsBoundaryIncluded, int nBins) {
    /*
     * Returns a SIFT image of the same width, height, and with 128 channels. This represents the SIFT descriptor for every
     * pixel in the original image.
     * */
    
    //force the type to double. Almost all the calculations below are floating point.
    Mat imsrc64;
    imsrc.convertTo(imsrc64, CV_64FC(imsrc.channels()));
    
    if(cellSize<=0) {
        cout<<"The cell size must be positive!"<<endl;
        exit(-1);
    }
    
    // this parameter controls decay of the gradient energy falls into a bin
    // run SIFT_weightFunc.m to see why alpha = 9 is the best value
    int alpha = 9;
    
    int width = imsrc.cols;
    int height = imsrc.rows;
    int nchannels = imsrc.channels();
    int nPixels = width*height;
    
    // compute the derivatives;
    Mat imdx = derivative(imsrc64, true);
    Mat imdy = derivative(imsrc64, false);
    
    // get the maximum gradient over the channels and estimate the normalized gradient
    Mat magsrc(height,width,CV_64FC(nchannels), Scalar::all(0));
    Mat mag(height,width,CV_64FC1, Scalar::all(0));
    Mat gradient(height,width,CV_64FC2, Scalar::all(0));
    double Max;
    for(int i=0;i<nPixels;i++) {
        int offset = i*nchannels;
        for(int j = 0;j<nchannels;j++)
            data(magsrc)[offset+j] = sqrt(pow(data(imdx)[offset+j],2.0)+pow(data(imdy)[offset+j],2.0)); //DEBUGGING
        Max = data(magsrc)[offset];
        if(Max!=0) {
            data(gradient)[i*2] = data(imdx)[offset]/Max;
            data(gradient)[i*2+1] = data(imdy)[offset]/Max;
        }
        for(int j = 1;j<nchannels;j++) {
            if(data(magsrc)[offset+j]>Max) {
                Max = data(magsrc)[offset+j];
                data(gradient)[i*2] = data(imdx)[offset+j]/Max;
                data(gradient)[i*2+1] = data(imdy)[offset+j]/Max;
            }
        }
        data(mag)[i] = Max;
    }
    
    // get the pixel-wise energy for each orientation band
    Mat imband(height,width,CV_64FC(nBins));
    double theta = M_PI*2/nBins;
    double _cos,_sin,temp;
    for(int k = 0;k<nBins;k++) {
        _sin    = sin(theta*k);
        _cos   = cos(theta*k);
        for(int i = 0;i<nPixels; i++) {
            temp = max(data(gradient)[i*2]*_cos + data(gradient)[i*2+1]*_sin,0.0);
            if(alpha>1)
                temp = pow(temp,alpha);
            data(imband)[i*nBins+k] = temp*data(mag)[i];
        }
    }
    
    // filter out the SIFT feature (this filter looks weird, but it's what they did)
    double filter[cellSize*2+1];
    for(int i=0; i<cellSize * 2 + 1; i++)
        filter[i] = 0;
    filter[0] = filter[cellSize+1] = 0.25;
    for(int i = 1;i<cellSize+1;i++)
        filter[i+1] = 1;
    for(int i = cellSize+2;i<cellSize*2+1;i++)
        filter[i] = 0;
    
    Mat imband_cell = ImageOperations::imfilter_hv(imband, filter, cellSize);
    
    // allocate buffer for the sift image
    int siftdim = nBins*16;
    int sift_width,sift_height,x_shift=0,y_shift=0;
    
    sift_width = width/stepSize;
    sift_height = height/stepSize;
    
    if(IsBoundaryIncluded == false) {
        sift_width = (width-4*cellSize)/stepSize;
        sift_height= (height-4*cellSize)/stepSize;
        x_shift = 2*cellSize;
        y_shift = 2*cellSize;
    }
    
    Mat imsift(sift_height,sift_width,CV_8UC(siftdim), Scalar::all(0));
    
    // now sample to get SIFT image
    double sift_cell[siftdim];
    for(int i=0;i<sift_height;i++)
        for(int j =0;j<sift_width;j++)
        {
            int count = 0;
            for(int ii = -1;ii<=2;ii++)
                for(int jj=-1;jj<=2;jj++)
                {
                    int y = EnforceRange(y_shift+i*stepSize+ii*cellSize, height);
                    int x = EnforceRange(x_shift+j*stepSize+jj*cellSize, width);
                    
                    memcpy(&sift_cell[count*nBins],&data(imband_cell)[(y*width+x)*nBins],sizeof(double)*nBins);
                    count++;
                }
            // normalize the SIFT descriptor
            double mag = 0;
            for(int m=0; m<siftdim; m++)
            {
                mag += pow(sift_cell[m],2.0);
            }
            mag = sqrt(mag);
            
            int offset = (i*sift_width+j)*siftdim;
            
            for(int k = 0;k<siftdim;k++)
                imsift.data[offset+k] = (unsigned char) min(sift_cell[k]/(mag+0.01)*255,255.0);
        }
    
//    unsigned char * sdata = (unsigned char *) imsift.data;
//    for(int i=0; i<30; i++)
//    {
//        cout << to_string(sdata[i]) << endl;
//    }
//    exit(1);
    return imsift;
}


Mat ImageOperations::imfilter_hv(Mat& source, double *filter, int fsize)
{
    Mat dest(source.rows, source.cols, CV_64FC(source.channels()), Scalar::all(0));
    Mat pTempBuffer(source.rows, source.cols, CV_64FC(source.channels()), Scalar::all(0));
    ImageOperations::filtering(source,pTempBuffer,filter,fsize,true);
    ImageOperations::filtering(pTempBuffer,dest,filter,fsize,false);
    return dest;
}


//------------------------------------------------------------------------------------------
// function to get the derivative of the image
//------------------------------------------------------------------------------------------
Mat ImageOperations::derivative(Mat& source, bool horizontal)
{
    Mat der(source.rows, source.cols, CV_64FC(source.channels()), Scalar::all(0));
    double Filter[5]={1,-8,0,8,-1};
    for(int i=0;i<5;i++)
        Filter[i]/=12;
    ImageOperations::filtering(source,der,Filter,2,horizontal);
    return der;
}

//------------------------------------------------------------------------------------------------------------
//  horizontal direction filtering
//------------------------------------------------------------------------------------------------------------
void ImageOperations::filtering(Mat& source,Mat& pDstImage,double* pfilter1D,int fsize, bool horizontal) {
    int width=source.cols;
    int height=source.rows;
    int nChannels=source.channels();
    double w;
    int i,j,l,k,jj,ii;
    for(i=0;i<height;i++)
        for(j=0;j<width;j++)
            for(k=0;k<nChannels;k++)
                for(l=-fsize;l<=fsize;l++)
                {
                    w=pfilter1D[l+fsize];
                    int srcidx = (i*pDstImage.cols + j)*pDstImage.channels()+k;
                    
                    if(horizontal)
                    {
                        jj=EnforceRange(j+l,width);
                        data(pDstImage)[srcidx] += ImageOperations::ValueAtPixel(source, jj, i, k)*w;
                    }
                    else
                    {
                        ii=EnforceRange(i+l,height);
                        data(pDstImage)[srcidx] += ImageOperations::ValueAtPixel(source, j, ii, k)*w;
                    }
                }
}

Mat ImageOperations::CreateWarpedImage(Mat& image, Mat& v){
    /*Warp the input image using the provided flow.*/
    if(image.rows != v.rows){
        cout << "ImageOperations: Couldn't warp the image. Mismatched image sizes." << endl;
        exit(-1);
    }
    Mat warped(v.rows, v.cols, CV_8UC(image.channels()), Scalar::all(0));
    ImageOperations::warpImage(image, warped, v);
    return warped;
}

inline void ImageOperations::BilinearInterpolate(Mat& image, double x, double y, unsigned char* result){
    int width = image.cols;
    int height = image.rows;
    int nChannels = image.channels();
    unsigned char * ref = (unsigned char *) image.data;
    
    //convert the double to int (which rounds), and in doing so get the amount of overlap
    //with other pixels, whose values are weighted accordingly.
    int xx,yy,m,n,u,v,l,offset;
    xx=floor(x);
    yy=floor(y);
    double dx,dy,s;
    dx=max(min(x-xx,1.0),0.0);
    dy=max(min(y-yy,1.0),0.0);
    
    for(m=0;m<=1;m++)
        for(n=0;n<=1;n++)
        {
            u=EnforceRange(xx+m,width);
            v=EnforceRange(yy+n,height);
            offset=(v*width+u)*nChannels;
            s=fabs(1-m-dx)*fabs(1-n-dy);
            for(l=0;l<nChannels;l++)
            {
                result[l]+=ref[offset+l]*s;
            }
        }
}

void ImageOperations::warpImage(Mat& image, Mat& imgdata, Mat& pV){
    /*
     Maps image[p] to imgdata[p+w(p)]
     */
    int width = image.cols;
    int height = image.rows;
    int nChannels = image.channels();
    
    double * flow = (double *)pV.data;
    
    for(int i=0;i<height;i++)
        for(int j=0;j<width;j++)
        {
            int offset=(i*width+j);
            double x,y;
            x=j+flow[offset*2];
            y=i+flow[offset*2+1];
            offset*=nChannels;
            if(x>=0 && x<width && y>=0 && y<height)
                BilinearInterpolate(image, x, y, imgdata.data+offset);
        }
}

Mat ImageOperations::Load(string path)
{
    /*Loads an image and returns it as a 3 channel double image.
     * */
    
    Mat img = imread(path);
    if(img.empty())
    {
        cout << "Error in loading the image at " << path << endl;
        exit(-1);
    }
    
    return img;
}

void ImageOperations::Undistort(const Camera& _cam, Mat& img){
    Mat temp = img.clone();
    undistort(temp, img, _cam.IntrinsicMatrix(), _cam.Distortion());
}

bool ImageOperations::writeUncompressedImage(Mat& im, string path) {
    //DATATYPE * ref, int cols, int rows, int channels, int type) {
    FILE * file = fopen(path.c_str(), "w");
    int cols = im.cols;
    int rows = im.rows;
    int channels = im.channels();
    int type = (im.type()%8==0)?8:64;
    
    fprintf(file, "PIEH");
    
    if (fwrite(&cols, sizeof(int), 1, file) != 1 ||
        fwrite(&rows, sizeof(int), 1, file) != 1 ||
        fwrite(&channels, sizeof(int), 1, file) != 1 ||
        fwrite(&type, sizeof(int), 1, file) != 1) {
        printf("writeUncompressedImage : problem writing header\n");
        return false;
    }
    
    void * ref = (void *) im.data;
    //    unsigned char * ref = (unsigned char *) simage.data;
    for (int i= 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            for(int k=0; k<channels; k++) {
                int idx = (i*cols + j)*channels + k;
                long ret=0;
                if(type==8) ret = fwrite(&(((unsigned char *)ref)[idx]), sizeof(unsigned char), 1, file);
                else if(type==64) ret = fwrite(&(((double *)ref)[idx]), sizeof(double), 1, file);
                if (ret != 1) {
                    printf("writeUncompressedImage : problem writing data\n");
                    return false;
                }
            }
        }
    }
    fclose(file);
    return true;
}

Mat ImageOperations::readUncompressedImage(string filename) {
    FILE * file = fopen(filename.c_str(), "r");
    
    char header[5];
    if (fread(header, 1, 4, file) < 4 && (std::string)header != "PIEH") {
        cout << "invalid header." << endl;
        exit(1);
    }
    
    int cols, rows, channels, type;
    if (fread(&cols, sizeof(int), 1, file) != 1||
        fread(&rows, sizeof(int), 1, file) != 1||
        fread(&channels, sizeof(int), 1, file) != 1 ||
        fread(&type, sizeof(int), 1, file) != 1) {
        cout << "couldn't read rows/cols." << endl;
        exit(1);
    }
    
    Mat simage;
    if(type==8) simage = cv::Mat::zeros(rows, cols, CV_8UC(channels));
    else if(type==64) simage = cv::Mat::zeros(rows, cols, CV_64FC(channels));
    
    void * ref = (void *) simage.data;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            for(int k=0; k < channels; k++){
                int idx = (i*cols + j)*channels + k;
                
                long ret=0;
                if(type==8) ret = fread(&(((unsigned char *)ref)[idx]), sizeof(unsigned char), 1, file);
                else if(type==64) ret = fread(&(((double *)ref)[idx]), sizeof(double), 1, file);
                if (ret != 1) {
                    printf("readUncompressedImage : problem reading data\n");
                    exit(1);
                }
            }
        }
    }
    fclose(file);
    return simage;
}

/*
 void CompareImages(string sim1, string sim2){
 //"/Users/shane/Desktop/OLD_1.txt"
 Mat im1 = ImageOperations::readUncompressedImage(sim1);
 Mat im2 = ImageOperations::readUncompressedImage(sim2);
 
 void * ref1 = (void *) im1.data;
 void * ref2 = (void *) im2.data;
 
 int type=(im1.type()%8==0)?8:64;
 
 int countunequal = 0;
 int countequal = 0;
 for (int i = 0; i < im1.rows; ++i) {
 for (int j = 0; j < im1.cols; ++j) {
 for(int k=0; k < im1.channels(); k++){
 int idx = (i*im1.cols + j)*im1.channels() + k;
 if(type==8 && (((unsigned char *)ref1)[idx] != ((unsigned char *)ref2)[idx])){
 countunequal++;
 }
 else if(type==64 && (((double *)ref1)[idx] != ((double *)ref2)[idx])){
 countunequal++;
 }
 else countequal++;
 }
 }
 }
 cout << "Image Comparison:\n Equal:\t"<<countequal<<"\n Unequal:\t"<<countunequal<<endl;
 //writeSIFTImageToFile(il.i2.data, il.width(), il.height());
 //CompareImages("/Users/shane/Desktop/OLD_1.jpg", "/Users/shane/Desktop/NEW_1.jpg");
 //exit(1);
}
 */

void ImageOperations::Save(Mat im, string path) {
    /*Converts a double image to a 3 channel unsigned char image.
     * */
//    cout << "path : "<<path<<endl;
    bool success = cv::imwrite(path.c_str(), im);
    if (!success) {
        std::cout << "couldn't write the image to the file: "<< path << std::endl;
        exit(-1);
    }
}

Mat ImageOperations::HalveImage(Mat& im, int g_hsize, double g_sigma) {
    Mat cvimg = Mat(im.rows, im.cols, im.type(), Scalar::all(0));
    Mat cvimghalf = Mat(cvimg.rows/2.0, cvimg.cols/2.0, im.type(), Scalar::all(0));
    
    GaussianBlur(im, cvimg, Size(g_hsize, g_hsize), g_sigma, g_sigma);
    cv::resize(cvimg, cvimghalf, Size(cvimg.cols/2, cvimg.rows/2), 0, 0, CV_INTER_CUBIC);
    
    return cvimghalf;
}

void ImageOperations::DoubleImage(Mat& src, Mat& dest) {
    cv::resize(src, dest, Size(src.cols*2, src.rows*2), 0, 0, CV_INTER_CUBIC);
}

void ImageOperations::DoubleImage(Mat& im) {
    cv::resize(im, im, Size(im.cols*2, im.rows*2), 0, 0, CV_INTER_CUBIC);
}
