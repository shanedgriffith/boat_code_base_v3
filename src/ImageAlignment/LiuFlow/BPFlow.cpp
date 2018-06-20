#include "stdio.h"
//#include "memory.h"
#include "math.h"
#include "stdlib.h"
#include <ImageAlignment/LiuFlow/project.h>
#include <ImageAlignment/LiuFlow/Stochastic.h>

#include "BPFlow.h"


using namespace std;

BPFlow::BPFlow(void) {
   IsDisplay=false;
   IsDataTermTruncated=false;
   IsTRW=false;
	CTRW=(double)1/2;
   //CTRW=0.55;
	Width=Height=Area=0;
	pIm1=pIm2=mask2=NULL;
	for(int i=0;i<2;i++)
	{
		pOffset[i]=NULL;
		pWinSize[i]=NULL;
	}
    pDataTermStore=NULL;
    ptrDataTermStore=NULL;
	pDataTerm=NULL;
	ptrDataTerm=NULL;
	for(int i=0;i<2;i++)
	{
		pRangeTerm[i]=pSpatialMessage[i]=pDualMessage[i]=pBelief[i]=NULL;
		ptrRangeTerm[i]=ptrSpatialMessage[i]=ptrDualMessage[i]=ptrBelief[i]=NULL;
	}
	pX=NULL;
	nNeighbors=4;
}

BPFlow::~BPFlow(void) {
	ReleaseBuffer();
}

void BPFlow::ReleaseBuffer() {
	_Release1DBuffer(pIm1);
	_Release1DBuffer(pIm2);

	for(int i=0;i<2;i++)
	{
		_Release1DBuffer(pOffset[i]); // release the buffer of the offset
		_Release1DBuffer(pWinSize[i]); // release the buffer of the size
	}
    
    _Release1DBuffer(pDataTermStore);
    _Release1DBuffer(ptrDataTermStore);
	_Release1DBuffer(pDataTerm);
	_Release1DBuffer(ptrDataTerm);
	for(int i=0;i<2;i++)
	{
		_Release1DBuffer(pRangeTerm[i]);
		_Release1DBuffer(ptrRangeTerm[i]);
		_Release1DBuffer(pSpatialMessage[i]);
		_Release1DBuffer(ptrSpatialMessage[i]);
		_Release1DBuffer(pDualMessage[i]);
		_Release1DBuffer(ptrDualMessage[i]);
		_Release1DBuffer(pBelief[i]);
		_Release1DBuffer(ptrBelief[i]);
	}
	_Release1DBuffer(pX);
    _Release1DBuffer(mask2);
}

//---------------------------------------------------------------------------------
// set the parameter of the model
// the regularization is a truncated L1 norm: __min(s|v(x,y)-v(x+1,y)|,d)
// sigma is used to penalize large displacement
//---------------------------------------------------------------------------------
void BPFlow::setPara(double _s, double _d)
{
	s=_s;
	d=_d;
	//printf("s: %f, d: %f\n",s,d);
	if(Width>0)
	{
		Im_s.allocate((int)Width,(int)Height,2);
		Im_d.allocate((int)Width,(int)Height,2);
		Im_s.setValue(s);
		Im_d.setValue(d);
	}
	else
		printf("BPFlow: The image dimension has not been specified! Call LoadImages() first\n");
}	

//----------------------------------------------------------------------
// function to load images
//----------------------------------------------------------------------
void BPFlow::LoadImages(int _width, int _height, int _nchannels, vector<T_input*> imgs)
{
	Width=_width;
	Height=_height;
    Width2=Width;
    Height2=Height;
	Area=Width*Height;
	nChannels=_nchannels;

	_Release1DBuffer(pIm1);
	_Release1DBuffer(pIm2);
	pIm1=new T_input[Width*Height*nChannels];
	pIm2=new T_input[Width*Height*nChannels];

	memcpy(pIm1,imgs[0],sizeof(T_input)*Width*Height*nChannels);
	memcpy(pIm2,imgs[1],sizeof(T_input)*Width*Height*nChannels);
	if(imgs.size() >= 3) {
		mask1 = new T_input[Width*Height];
		memcpy(mask1,imgs[2],sizeof(T_input)*Width*Height);
	}
	if(imgs.size() >= 4) {
		mask2 = new T_input[Width*Height];
		memcpy(mask2,imgs[3],sizeof(T_input)*Width*Height);
	}
}

/*
void BPFlow::LoadImages(int _width, int _height, int _nchannels, const T_input *pImage1, const T_input *pImage2)
{
	Width=_width;
	Height=_height;
	Area=Width*Height;
	nChannels=_nchannels;

	_Release1DBuffer(pIm1);
	_Release1DBuffer(pIm2);
	pIm1=new T_input[Width*Height*nChannels];
	pIm2=new T_input[Width*Height*nChannels];

	memcpy(pIm1,pImage1,sizeof(T_input)*Width*Height*nChannels);
	memcpy(pIm2,pImage2,sizeof(T_input)*Width*Height*nChannels);
    masked2=false;
    
    Width2=Width;
    Height2=Height;
}


void BPFlow::LoadImages(int _width, int _height, int _nchannels, const T_input *pImage1, const T_input *pImage2, const T_input * img2mask)
{
	Width=_width;
	Height=_height;
	Area=Width*Height;
	nChannels=_nchannels;
    
	_Release1DBuffer(pIm1);
	_Release1DBuffer(pIm2);
	pIm1=new T_input[Width*Height*nChannels];
	pIm2=new T_input[Width*Height*nChannels];
    
	memcpy(pIm1,pImage1,sizeof(T_input)*Width*Height*nChannels);
	memcpy(pIm2,pImage2,sizeof(T_input)*Width*Height*nChannels);
    
    Width2=Width;
    Height2=Height;
    
    masked2 = true;
//	_Release1DBuffer(mask2);
//    mask2 = img2mask;
    mask2 = new T_input[Width*Height*1];
	memcpy(mask2,img2mask,sizeof(T_input)*Width*Height);
}


void BPFlow::LoadImages(int _width, int _height, int _nchannels, const T_input *pImage1, int _width2,int _height2, const T_input *pImage2)
{
	Width=_width;
	Height=_height;
	Area=Width*Height;
	nChannels=_nchannels;
    Width2=_width2;
    Height2=_height2;

	_Release1DBuffer(pIm1);
	_Release1DBuffer(pIm2);
	pIm1=new T_input[Width*Height*nChannels];
	pIm2=new T_input[Width2*Height2*nChannels];
    masked2=false;

	memcpy(pIm1,pImage1,sizeof(T_input)*Width*Height*nChannels);
	memcpy(pIm2,pImage2,sizeof(T_input)*Width2*Height2*nChannels);
}*/

void BPFlow::setPixelWinsize(int x, int y, int winSizeX, int winSizeY){
    int idx = y*Width + x;
    pWinSize[0][idx] = winSizeX;
    pWinSize[1][idx] = winSizeY;
}

//------------------------------------------------------------------------------------------------
// function to set the homogeneous MRF parameters
// There is no offset, and the window size is identical for each pixel (winSize)
//------------------------------------------------------------------------------------------------
void BPFlow::setHomogeneousMRF(int winSize)
{
	for(int i=0;i<2;i++)
	{
//		_Release1DBuffer(pOffset[i]); // release the buffer of the offset
		_Release1DBuffer(pWinSize[i]); // release the buffer of the size
//		pOffset[i]=new T_state[Area];
//		memset(pOffset[i],0,sizeof(T_state)*Area);

		pWinSize[i]=new T_state[Area];
        for(size_t j=0;j<Area;j++){
            double h = IsAConstraint((int)j);
            if(h>0){
                pWinSize[i][j]=winSize;//round(h);
            }
            else pWinSize[i][j]=winSize;//+CStochastic::UniformSampling(3)-1;
        }
	}
//	// add some disturbance
//	for(int i=0;i<2;i++)
//		for(int j=0;j<Area;j++)
//			pOffset[i][j]=CStochastic::UniformSampling(5)-2;
}


void BPFlow::SetOffset(double* flowOffset)
{
    for(int i=0; i<2; i++)
    {
        _Release1DBuffer(pOffset[i]);
        pOffset[i]=new T_state[Area];
        for(size_t j=0; j<Area; j++)
        {
            if(flowOffset != NULL) pOffset[i][j]=(int) flowOffset[j*2+i];
            else pOffset[i][j]=0;//CStochastic::UniformSampling(5)-2; //uniform sampling of [-2, -1, 0, 1, 2]; adds some disturbance
        }
    }
}


//------------------------------------------------------------------------------------------------
// function to verify whether a point is inside the image boundary or not
//------------------------------------------------------------------------------------------------
template <class T>
bool BPFlow::InsideImage(T x,T y)
{
	if(x>=0 && x<Width2 && y>=0 && y<Height2)
		return true;
	else
		return false;
}

template <class T>
bool BPFlow::Masked(int masknum, T x,T y)
{
//	if(!InsideImage(x,y)) return false;

	int idx = y*Width+x;
	if(masknum==1){
		if(masked1){
			 unsigned char ismasked = mask1[idx];
			 if(ismasked<=50) return true;
		}
	}
	else if(masknum==2){
		if(masked2)
		{
			 unsigned char ismasked = mask2[idx];
			 if(ismasked<=50) return true;
		}
	}

    return false;
}


//------------------------------------------------------------------------------------------------
// function to compute range term
//------------------------------------------------------------------------------------------------
void BPFlow::ComputeRangeTerm(double _gamma) {
	gamma=_gamma;
	for(int i=0;i<2;i++) {
		_Release1DBuffer(pRangeTerm[i]);
		_Release1DBuffer(ptrRangeTerm[i]);
		AllocateBuffer(pRangeTerm[i],1,ptrRangeTerm[i],pWinSize[i]);
	}
	for(ptrdiff_t offset=0;offset<Area;offset++) //area = all the pixels in the image (width*height)
	{
		for(ptrdiff_t plane=0;plane<2;plane++) //u and v planes.
		{
			int winsize=pWinSize[plane][offset];
			for(ptrdiff_t j=-winsize;j<=winsize;j++) //every possible labeling.
            {
//                double re = IsAConstraint(offset);
//                if(re>0){
//                    pRangeTerm[plane][offset].data()[j+winsize]=2*gamma*fabs((double)j);//+pOffset[plane][offset]);
//                }
//                else
                    pRangeTerm[plane][offset].data()[j+winsize]=gamma*fabs((double)j+pOffset[plane][offset]);//);//
            }
		}
	}
}


double BPFlow::GetMedian(int winsize, int i, int j){
	size_t index=i*Width+j;

	vector<double> score;
	//get the med
	for(ptrdiff_t k=-winsize;k<=winsize;k++)  // index over y
		for(ptrdiff_t l=-winsize;l<=winsize;l++)  // index over x
		{
			ptrdiff_t x=j+winsize+l;
			ptrdiff_t y=i+winsize+k;
			if(!InsideImage(x,y)) continue;
			int _ptr=(k+winsize)*winsize+l+winsize;
			score.push_back(pDataTerm[index][_ptr]);
		}

	sort(score.begin(), score.end());

	return score[score.size()/2];
}


//------------------------------------------------------------------------------------------------
// function to compute data term
//------------------------------------------------------------------------------------------------
double BPFlow::ComputeDataTerm()
{
    // allocate the buffer for data term
    AllocateBuffer<PixelBuffer2D<T_message>,T_message>(pDataTerm,ptrDataTerm,pWinSize[0],pWinSize[1]);
    AllocateBuffer<PixelBuffer2D<T_message>,T_message>(pDataTermStore,ptrDataTermStore,pWinSize[0],pWinSize[1]);
    
	T_message HistMin,HistMax;
	double HistInterval;
	double* pHistogramBuffer;
	int nBins=20000;
	int total=0; // total is the total number of plausible matches, used to normalize the histogram
	pHistogramBuffer=new double[nBins];
	memset(pHistogramBuffer,0,sizeof(double)*nBins);
	HistMin= 32767;
	HistMax=0;
	//--------------------------------------------------------------------------------------------------
	// step 1. the first sweep to compute the data term for the visible matches
	//--------------------------------------------------------------------------------------------------
	for(ptrdiff_t i=0;i<Height;i++)			// index over y
		for(ptrdiff_t j=0;j<Width;j++)		// index over x
		{
            //CHECK: MASK HERE? May not be necessary due to step 3.
			size_t index=i*Width+j;
			int XWinLength=pWinSize[0][index]*2+1;
			// loop over a local window
			for(ptrdiff_t k=-pWinSize[1][index]; k<=pWinSize[1][index]; k++)  // index over y
				for(ptrdiff_t l=-pWinSize[0][index]; l<=pWinSize[0][index]; l++)  // index over x
                {
					ptrdiff_t x=j+pOffset[0][index]+l;
					ptrdiff_t y=i+pOffset[1][index]+k;

					// if the point is outside the image boundary then continue
                    if(!InsideImage(x,y)){
						continue;
                    }

                    //CHECK: MASK HERE?
					ptrdiff_t index2=y*Width2+x;
					T_message foo=0;
					for(int n=0;n<nChannels;n++)
                    {
                        //there's a better way to do this, but for now, this might work.
                        if(nChannels==128) //SIFT
                            foo+=abs(pIm1[index*nChannels+n]-pIm2[index2*nChannels+n]); // L1 norm
                        else if (nChannels==32)//BRIEF
                        {
                            unsigned char v = pIm1[index*nChannels+n]^pIm2[index2*nChannels+n];
                            unsigned int c; // c accumulates the total bits set in v
                            // count the number of bits set in v
                            for (c = 0; v; c++)
                            {
                                v &= v - 1; // clear the least significant bit set
                            }
                            foo+=c;
                        }
//                        cout << "foo["<<n<<"] = " << foo << ", im1(" << i << ","<<j<<")= " << (int) pIm1[index*nChannels+n] << ", im2(" <<y<<","<<x<<")= " << (int) pIm2[index2*nChannels+n] << endl;
                    }
//                    if (nChannels==32) foo = foo*30; //for BRIEF

                    pDataTerm[index][(k+pWinSize[1][index])*XWinLength+l+pWinSize[0][index]]=foo;
                    
					HistMin=__min(HistMin,foo);
					HistMax=__max(HistMax,foo);
					total++;
				}
		}

	// compute the histogram info
	HistInterval=(double)(HistMax-HistMin)/nBins;
    if(HistInterval==0)
    {
        cout << "problem, divide by zero ahead." << endl;
        exit(-1);
    }
//    cout << "Max: " << HistMax << ", Min:" << HistMin << endl;
//    exit(1);
    
	//--------------------------------------------------------------------------------------------------
	// step 2. get the histogram of the matching
	//--------------------------------------------------------------------------------------------------
	for(ptrdiff_t i=0;i<Height;i++)			// index over y
		for(ptrdiff_t j=0;j<Width;j++)		// index over x
		{
            //CHECK: MASK HERE? May not be necessary due to step 3.
			size_t index=i*Width+j;
			int XWinLength=pWinSize[0][index]*2+1;
			// loop over a local window
			for(ptrdiff_t k=-pWinSize[1][index];k<=pWinSize[1][index];k++)  // index over y
				for(ptrdiff_t l=-pWinSize[0][index];l<=pWinSize[0][index];l++)  // index over x
				{
					ptrdiff_t x=j+pOffset[0][index]+l;
					ptrdiff_t y=i+pOffset[1][index]+k;

					// if the point is outside the image boundary then continue
					if(!InsideImage(x,y)) continue;
                    
					int foo=__min(pDataTerm[index][(k+pWinSize[1][index])*XWinLength+l+pWinSize[0][index]]/HistInterval,nBins-1);
                    
					pHistogramBuffer[foo]++;
				}
		}
    
    //find the median of the match score.
    // the truncation term is the median of all the match scores for the image
    double DefaultMatchingScore = 0.1;
    double Prob=0;
    double medianpoint = total/2.0;
    for(int i=0;i<nBins;i++) {
        Prob+=pHistogramBuffer[i];
        if(Prob>=medianpoint) // find the matching score
        {
            DefaultMatchingScore=max((double)i, 1.0)*HistInterval+HistMin;
            break;
        }
    }
//    cout << "Min: " << HistMin<<", Default: "<<DefaultMatchingScore<<", "<<HistMax<<endl;
    
	//DefaultMatchingScore=0.1;
	//--------------------------------------------------------------------------------------------------
	// step 3. assigning the default matching score to the outside matches
	//--------------------------------------------------------------------------------------------------

	for(ptrdiff_t i=0;i<Height;i++)	{		// index over y
		for(ptrdiff_t j=0;j<Width;j++)		// index over x
		{
			size_t index=i*Width+j;
			int XWinLength=pWinSize[0][index]*2+1;
            
			//Compute the unaltered data term so that later we can get an accurate estimate of the alignment energy
            for(ptrdiff_t k=-pWinSize[1][index];k<=pWinSize[1][index];k++)  // index over y
				for(ptrdiff_t l=-pWinSize[0][index];l<=pWinSize[0][index];l++)  // index over x
				{
					ptrdiff_t x=j+pOffset[0][index]+l;
					ptrdiff_t y=i+pOffset[1][index]+k;
					int _ptr=(k+pWinSize[1][index])*XWinLength+l+pWinSize[0][index];

					//store the unaltered data term for computing the alignment energy
					pDataTermStore[index][_ptr] = pDataTerm[index][_ptr];

					if(!InsideImage(x,y)) pDataTermStore[index][_ptr] = DefaultMatchingScore;
					else if (IsDataTermTruncated) // truncate the data term
						pDataTermStore[index][_ptr]=__min(pDataTerm[index][_ptr], DefaultMatchingScore);
				}
            
            //get the median value of all the hypothesis for the pixel
            double med = GetMedian(XWinLength, i, j);

            //if the pixel at this flow location is masked, remove its appearance information.
//			if(masked1 && Masked(1, j,i)) {//
			if(got_mask_points && (*mask_points)[index]){
				for(ptrdiff_t k=-pWinSize[1][index];k<=pWinSize[1][index];k++)  // index over y
					for(ptrdiff_t l=-pWinSize[0][index];l<=pWinSize[0][index];l++)  // index over x
					{
						int _ptr=(k+pWinSize[1][index])*XWinLength+l+pWinSize[0][index];
						pDataTerm[index][_ptr]= __min(DefaultMatchingScore, med);
					}
			}

            vector<double> rterms;
            if(rconstraints)
            	rterms = ComputeReprojectionTerms((int) index, pWinSize[0][index], DefaultMatchingScore);

            
			// loop over a local window
			for(ptrdiff_t k=-pWinSize[1][index];k<=pWinSize[1][index];k++)  // index over y
				for(ptrdiff_t l=-pWinSize[0][index];l<=pWinSize[0][index];l++)  // index over x
				{
					ptrdiff_t x=j+pOffset[0][index]+l;
					ptrdiff_t y=i+pOffset[1][index]+k;
                    
                    int _ptr=(k+pWinSize[1][index])*XWinLength+l+pWinSize[0][index];
                    
                    if(!InsideImage(x,y)) pDataTerm[index][_ptr] = DefaultMatchingScore;
                    else if (IsDataTermTruncated) // truncate the data term
                        pDataTerm[index][_ptr]=__min(pDataTerm[index][_ptr], DefaultMatchingScore);
                    
                    //if the pixel to be matched with is masked, remove its appearance information.
                    if(InsideImage(x,y) && !Masked(1, j, i) && masked2 && Masked(2, x, y)){
                    	pDataTerm[index][_ptr]=__min(DefaultMatchingScore, med);
                    }

                    //epipolar constraints
                    if(epipolarconstraints) pDataTerm[index][_ptr] = (1-(*epihypspace)[index][_ptr])*pDataTerm[index][_ptr];

                    //reprojection constraints
                    if(rconstraints && rterms.size() > 0) pDataTerm[index][_ptr] = rterms[_ptr];
                    
                    //cycle consistency
                    if(twocycleconsistency) pDataTerm[index][_ptr] += (*twocyclehypspaceweights)[index][_ptr];
                    
                    //adaptive hypothesis space
                    if(hypspaces.size()>0){
                        if(abs(k)>hypspaces[i][j][1] || abs(l)>hypspaces[i][j][0]) pDataTerm[index][_ptr] = 1000*DefaultMatchingScore;
                    }
				}
		}
	}
	delete[] pHistogramBuffer;
    return DefaultMatchingScore;
}

void BPFlow::CreateHypSpace(int sizex, int sizey){
    if(hypspaces.size()!=sizey || hypspaces[0].size()!=sizex)
        hypspaces = std::vector<std::vector<std::vector<double> > >(sizey, std::vector<std::vector<double> >(sizex, vector<double>(2,0)));
}

void BPFlow::SetHypSpaces(int hx, int hy, int x, int y){
    hypspaces[y][x][0] = hx;
    hypspaces[y][x][1] = hy;
}

void BPFlow::AddHypSpaceWeights(vector<vector<double> >* _two_cycle_hypspace){
	twocycleconsistency = true;
	twocyclehypspaceweights = _two_cycle_hypspace;
}

void BPFlow::AddEpiWeights(vector<vector<double> >* hypspace){
	epipolarconstraints = true;
	epihypspace = hypspace;
}

double BPFlow::EvaluateGaussian(int fx, int fy, double ux, double uy, double stdp){
    return 1.0/(stdp*pow(2*M_PI,0.5))*exp(-1*(pow(fx-ux, 2) + pow(fy-uy,2))/(2*pow(stdp,2)));
}

vector<double> BPFlow::GetConstraintWeights(int c, int win){
	/*Compute the weights as a Gaussian.
	 * */
    vector<double> cflow = cflows[c];
    if(abs(cflow[0]) > win || abs(cflow[1])>win) return {};
    vector<double> res;
    for(int i=-win; i<=win; i++){
        for(int j=-win; j<=win; j++){
            double egauss = EvaluateGaussian(j, i, cflow[0], cflow[1], cflow[2]);
            res.push_back(egauss);
        }
    }
    return res;
}


void Print(vector<double> w, int win){
	for(int i=-win; i<=win; i++){
		for(int j=-win; j<=win; j++){
			int idx = (i+win)*(2*win+1)+(j+win);
			cout << ","<<w[idx];
		}
		cout << endl;
	}
	cout << endl;
}


vector<double> BPFlow::ComputeReprojectionTerms(int index, int win, double DefaultMatchingScore){
    vector<int> constraints = GetConstraint(index);
    vector<double> combweights;
    bool toadd = true;
//    static int firsta = 0;
    //just in case there are multiple constraints per pixel, take the average.
    for(int i=0; i<constraints.size(); i++){
        vector<double> cweights = GetConstraintWeights(constraints[i], win);
//        if(firsta==0){
//        	vector<int> cflow = cflows[constraints[i]];
//        	cout << "For a flow of ("<<cflow[0]<< ","<<cflow[1]<<")"<<endl;
//        	Print(cweights, win);
//        	firsta = 1;
//        }
        if(cweights.size()==0)continue;
        for(int j=0; j<cweights.size(); j++){
            if(toadd){
                combweights.push_back(cweights[j]);
            }
            else{
                combweights[j] += cweights[j];
            }
        }
        toadd = false;
    }
    
    if(combweights.size()==0) return {};
    
    double max = 0;
    for(int i=0; i<combweights.size(); i++){
        if(max < combweights[i])
            max = combweights[i];
    }
    
    //normalize the weights, invert them, and then scale.
    for(int i=0; i<combweights.size(); i++){
        combweights[i] = (1.0 - combweights[i]/max)*DefaultMatchingScore;
    }

//    static int firstb = 0;
//    if(firstb == 0){
//    	firstb = 1;
//        Print(combweights, win);
//    }
    return combweights;
}

void BPFlow::AddMaskPoints(vector<bool>* mp){
	got_mask_points = true;
	mask_points = mp;
}

void BPFlow::AddConstraint(int px, int py, double fx, double fy, double r){
//    int large_value=100000000;
    size_t index=py*Width+px;
//    int XWinLength=pWinSize[0][index]*2+1;
    
    //He programmed it in a way that allows for non-homogenous hypothesis spaces
    //(as it appears, anyway; why else would the winsize be everywhere?)
//    pOffset[0][index] = fx;
//    pOffset[1][index] = fy;

    vector<double> cf = {fx-pOffset[0][index], fy-pOffset[1][index], r};
    cflows.push_back(cf);
    
    isconstraint.push_back((int)index);
    reprojection_errors.push_back(r);
    
//    for(ptrdiff_t k=-pWinSize[1][index];k<=pWinSize[1][index];k++)  // index over y
//        for(ptrdiff_t l=-pWinSize[0][index];l<=pWinSize[0][index];l++)  // index over x
//        {
//            ptrdiff_t x = px+pOffset[0][index]+l;
//            ptrdiff_t y = py+pOffset[1][index]+k;
//            
//            int _ptr= (int)((k+pWinSize[1][index])*XWinLength+l+pWinSize[0][index]);
//            
//            if(!InsideImage(x,y)) continue;
//            else if (k==fy && l==fx) pDataTerm[index][_ptr]=0;
//            else pDataTerm[index][_ptr]=large_value;
//            //if(i==0 && j==0) cout << "score: ["<<k<<","<<l<<"]="<<pDataTerm[index][_ptr]<<endl;
//        }
}


//------------------------------------------------------------------------------------------------
//	function to allocate buffer for the messages
//------------------------------------------------------------------------------------------------
template <class T1,class T2>
size_t BPFlow::AllocateBuffer(T1*& pBuffer,size_t factor,T2*& ptrData,const int* pWinSize) {
	pBuffer=new T1[Area*factor];
	size_t totalElements=0;
	for(ptrdiff_t i=0;i<Area;i++) {
		totalElements+=pWinSize[i]*2+1;
		for(ptrdiff_t j=0;j<factor;j++)
			pBuffer[i*factor+j].allocate(pWinSize[i]*2+1);
	}
	totalElements*=factor;
	ptrData=new T2[totalElements];
	memset(ptrData,0,sizeof(T2)*totalElements);

	T2* ptrDynamic=ptrData;
	size_t total=0;
	for(ptrdiff_t i=0;i<Area*factor;i++)
	{
		pBuffer[i].data()=ptrDynamic;
		ptrDynamic+=pBuffer[i].nElements();
		total+=pBuffer[i].nElements();
	}
	return total;
}

template<class T1,class T2>
size_t BPFlow::AllocateBuffer(T1*& pBuffer,T2*& ptrData,const int* pWinSize1,const int* pWinSize2) {
	pBuffer=new T1[Area];
	size_t totalElements=0;
	for(ptrdiff_t i=0;i<Area;i++)
	{
		totalElements+=(pWinSize1[i]*2+1)*(pWinSize2[i]*2+1);
		pBuffer[i].allocate(pWinSize1[i]*2+1,pWinSize2[i]*2+1);
	}
	ptrData=new T2[totalElements];
	memset(ptrData,0,sizeof(T2)*totalElements);

	T2* ptrDynamic=ptrData;
	size_t total=0;
	for(ptrdiff_t i=0;i<Area;i++)
	{
		pBuffer[i].data()=ptrDynamic;
		ptrDynamic+=pBuffer[i].nElements();
		total+=pBuffer[i].nElements();
	}
	return total;
}

void BPFlow::AllocateMessage() {
	// delete the buffers for the messages
	for(int i=0;i<2;i++) {
		_Release1DBuffer(pSpatialMessage[i]);
		_Release1DBuffer(ptrSpatialMessage[i]);
		_Release1DBuffer(pDualMessage[i]);
		_Release1DBuffer(ptrDualMessage[i]);
		_Release1DBuffer(pBelief[i]);
		_Release1DBuffer(ptrBelief[i]);
	}
	// allocate the buffers for the messages
	for(int i=0;i<2;i++) {
		nTotalSpatialElements[i]=AllocateBuffer(pSpatialMessage[i],nNeighbors,ptrSpatialMessage[i],pWinSize[i]);
		nTotalDualElements[i]=AllocateBuffer(pDualMessage[i],1,ptrDualMessage[i],pWinSize[i]);
		nTotalBelifElements[i]=AllocateBuffer(pBelief[i],1,ptrBelief[i],pWinSize[i]);
	}
}

//------------------------------------------------------------------------------------------------
// function for belief propagation
//------------------------------------------------------------------------------------------------
double BPFlow::MessagePassing(int nIterations,int nHierarchy,double* pEnergyList) {
	AllocateMessage();
	if(nHierarchy>0) {
		BPFlow bp;
		generateCoarserLevel(bp);
		bp.MessagePassing(20,nHierarchy-1);
		bp.propagateFinerLevel(*this);
	}

	if(pX!=NULL)
		_Release1DBuffer(pX);
	pX=new int[Area*2];
	double energy=0;
	for(int count=0;count<nIterations;count++) {
		//Bipartite(count);
		BP_S(count);
		//TRW_S(count);
		//FindOptimalSolutionSequential();
		ComputeBelief();
		FindOptimalSolution();
		//energy=GetEnergy(); //my mask is null in this function for some reason, which causes a seg fault. since this isn't essential, I'm commenting it for a work-around.
		if(pEnergyList!=NULL)
			pEnergyList[count]=energy;
	}
	return energy;
}


//------------------------------------------------------------------------------------------------
// bipartite message update
//------------------------------------------------------------------------------------------------
void BPFlow::Bipartite(int count)
{
	// loop over vx and vy planes to update the message within each grid
	for (int k=0; k<2; k++)
		for (int i=0; i<Height; i++)
			for (int j=0; j<Width; j++)
			{
				// the bipartite update
				if (count%2==0 &&  (i+j)%2==k) // the even count
					continue;
				if (count%2==1 && (i+j)%2==1-k) // the odd count
					continue;

				//------------------------------------------------------------------------------------------------
				// update the message from (j,i,k) to the neighbors on the same plane
				//------------------------------------------------------------------------------------------------
				// the encoding of the direction
				//	0: left to right
				//	1: right to left
				//	2: top down
				//	3: bottom up
				for (int direction = 0; direction<4; direction++)
					UpdateSpatialMessage(j,i,k,direction);

				//-----------------------------------------------------------------------------------------------------
				// update the message from (j,i,k) to the dual node (j,i,1-k)
				//-----------------------------------------------------------------------------------------------------
                //he may have forgone use of the bipartite because they seem to be using it weirdly here. the dual message
                //should always be updated within its set (even or odd). Both planes are always processed, but only half
                //from the dual and half from the regular. (does this defeat the purpose of the dual plane approach?)
				if(count%4<2)
					UpdateDualMessage(j,i,k);
			}
}

void BPFlow::BP_S(int count)
{
	int k=count%2;
	if (count%4<2) //forward update
		for(int i=0;i<Height;i++)
			for(int j=0;j<Width;j++)
			{
				UpdateSpatialMessage(j,i,k,0);
				UpdateSpatialMessage(j,i,k,2);
				if(count%8<4)
					UpdateDualMessage(j,i,k);
			}
	else // backward upate
		for(int i=Height-1;i>=0;i--)
			for(int j=Width-1;j>=0;j--)
			{
				UpdateSpatialMessage(j,i,k,1);
				UpdateSpatialMessage(j,i,k,3);
				if(count%8<4)
					UpdateDualMessage(j,i,k);
			}
}

void BPFlow::TRW_S(int count)
{
	int k=count%2;
	if (k==0) //forward update
		for(int i=0;i<Height;i++)
			for(int j=0;j<Width;j++)
			{
				for(int l=0;l<2;l++)
				{
					UpdateDualMessage(j,i,l);
					UpdateSpatialMessage(j,i,l,0);
					UpdateSpatialMessage(j,i,l,2);
				}
			}
	else // backward upate
		for(int i=(int)Height-1;i>=0;i--)
			for(int j=(int)Width-1;j>=0;j--)
			{
				for(int l=0;l<2;l++)
				{
					UpdateDualMessage(j,i,l);
					UpdateSpatialMessage(j,i,l,1);
					UpdateSpatialMessage(j,i,l,3);
				}					
			}
}


//------------------------------------------------------------------------------------------------
//  update the message from (x0,y0,plane) to the neighbors on the same plane
//    the encoding of the direction
//               2   |
//                   v
//    0 ------> <------- 1
//                   ^
//                3  |
//------------------------------------------------------------------------------------------------
void BPFlow::UpdateSpatialMessage(int x, int y, int plane, int direction)
{
	// eliminate impossible messages
	if (direction==0 && x==Width-1)
		return;
	if (direction==1 && x==0)
		return;
	if (direction==2 && y==Height-1)
		return;
	if (direction==3 && y==0)
		return;

	int offset=(int)y*Width+x;
	int nStates=pWinSize[plane][offset]*2+1;



	T_message* message_org;
   	message_org=new T_message[nStates];

	int x1=x,y1=y; // get the destination
	switch(direction){
		case 0:
			x1++;
			s=Im_s.data()[offset*2+plane];
			d=Im_d.data()[offset*2+plane];
			break;
		case 1:
			x1--;
			s=Im_s.data()[(offset-1)*2+plane];
			d=Im_d.data()[(offset-1)*2+plane];
			break;
		case 2:
			y1++;
			s=Im_s.data()[offset*2+plane];
			d=Im_d.data()[offset*2+plane];
			break;
		case 3:
			y1--;
			s=Im_s.data()[(offset-Width)*2+plane];
			d=Im_d.data()[(offset-Width)*2+plane];
			break;
	}
	//s=m_s;
	//d=m_d;
	int offset1=(int) y1*Width+x1;
	int nStates1=pWinSize[plane][offset1]*2+1; // get the number of states for the destination node
	int wsize=pWinSize[plane][offset];
	int wsize1=pWinSize[plane][offset1];

	T_message*& message=pSpatialMessage[plane][offset1*nNeighbors+direction].data();

	// initialize the message from the dual plane
	if(!IsTRW)
		memcpy(message_org,pDualMessage[plane][offset].data(),sizeof(T_message)*nStates);
	else
	{
		memset(message_org,0,sizeof(T_message)*nStates);
		Add2Message(message_org,pDualMessage[plane][offset].data(),nStates,CTRW);
	}

	// add the range term
	if(!IsTRW)
		Add2Message(message_org,pRangeTerm[plane][offset].data(),nStates);
	else
		Add2Message(message_org,pRangeTerm[plane][offset].data(),nStates,CTRW);
	
	// add spatial messages
	if(!IsTRW)
	{
		if(x>0 && direction!=1) // add left to right
			Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors].data(),nStates);
		if(x<Width-1 && direction!=0) // add right to left 
			Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors+1].data(),nStates);
		if(y>0 && direction!=3) // add top down
			Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors+2].data(),nStates);
		if(y<Height-1 && direction!=2) // add bottom up
			Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors+3].data(),nStates);
	}
	else
	{
		if(x>0) // add left to right
        {
			if(direction==1)
				Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors].data(),nStates,CTRW-1);
			else
				Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors].data(),nStates,CTRW);
        }
		if(x<Width-1) // add right to left
        {
			if(direction==0)
				Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors+1].data(),nStates,CTRW-1);
			else
				Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors+1].data(),nStates,CTRW);
        }
		if(y>0) // add top down
        {
			if(direction==3)
				Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors+2].data(),nStates,CTRW-1);
			else
				Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors+2].data(),nStates,CTRW);
        }
		if(y<Height-1) // add bottom up
        {
			if(direction==2)
				Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors+3].data(),nStates,CTRW-1);
			else
				Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors+3].data(),nStates,CTRW);
        }
	}
    
    bool nonconstraint = 1;
//    for(int i=0; i<nStates1; i++){
//        int * dat = pDataTerm[offset1].data()+i*nStates;
//        for(int j=0; j<nStates; j++){
//            if(dat[i]==0)nonconstraint = 0;
//        }
//    }
    //    if(hasone) cout << "found zero ("<<x<<","<<y<<")" <<endl;
    
	// use distance transform function to impose smoothness compatibility
	T_message Min=CStochastic::Min(nStates,message_org)+d;
	for(ptrdiff_t l=1;l<nStates;l++)
		message_org[l]=__min(message_org[l],message_org[l-1]+s);
	for(ptrdiff_t l=nStates-2;l>=0;l--)
		message_org[l]=__min(message_org[l],message_org[l+1]+s);
    
	// transform the compatibility 
	int shift=pOffset[plane][offset1]-pOffset[plane][offset];
	if(abs(shift)>wsize+wsize1) // the shift is too big that there is no overlap
	{
        if(offset>0){
			for(ptrdiff_t l=0;l<nStates1;l++)
				message[l]=l*s*nonconstraint;
        }
        else{
			for(ptrdiff_t l=0;l<nStates1;l++)
				message[l]=-l*s*nonconstraint;
        }
	}
	else
	{
		int start=__max(-wsize,shift-wsize1);
		int end=__min(wsize,shift+wsize1);
		for(ptrdiff_t i=start;i<=end;i++)
			message[i-shift+wsize1]=message_org[i+wsize];
		if(start-shift+wsize1>0)
			for(ptrdiff_t i=start-shift+wsize1-1;i>=0;i--)
				message[i]=message[i+1]+s*nonconstraint;
		if(end-shift+wsize1<nStates1)
			for(ptrdiff_t i=end-shift+wsize1+1;i<nStates1;i++)
				message[i]=message[i-1]+s*nonconstraint;
	}

	// put back the threshold
	for(ptrdiff_t l=0;l<nStates1;l++)
		message[l]=__min(message[l],Min);

	// normalize the message by subtracting the minimum value
	Min=CStochastic::Min(nStates1,message);
	for(ptrdiff_t l=0;l<nStates1;l++)
		message[l]-=Min;

	delete[] message_org;
}

template<class T>
void BPFlow::Add2Message(T* message,const T* other,int nstates)
{
	for(size_t i=0;i<nstates;i++)
		message[i]+=other[i];
}

template<class T>
void BPFlow::Add2Message(T* message,const T* other,int nstates,double Coeff)
{
	for(size_t i=0;i<nstates;i++)
		message[i]+=other[i]*Coeff;
}

//------------------------------------------------------------------------------------------------
// update dual message passing from one plane to the other
//------------------------------------------------------------------------------------------------
void BPFlow::UpdateDualMessage(int x, int y, int plane)
{
	int offset=(int) y*Width+x;
	int offset1=offset;
	int wsize=pWinSize[plane][offset];
	int nStates=wsize*2+1;
	int wsize1=pWinSize[1-plane][offset];
	int nStates1=wsize1*2+1;

	s=Im_s.data()[offset*2+plane];
	d=Im_d.data()[offset*2+plane];
	//s=m_s;
	//d=m_d;

	T_message* message_org;
	message_org=new T_message[nStates];
	memset(message_org,0,sizeof(T_message)*nStates);
	
	// add the range term
	if(!IsTRW)
		Add2Message(message_org,pRangeTerm[plane][offset].data(),nStates);
	else
		Add2Message(message_org,pRangeTerm[plane][offset].data(),nStates,CTRW);

	// add spatial messages
	if(x>0)  //add left to right
	{
		if(!IsTRW)
			Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors].data(),nStates);
		else
			Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors].data(),nStates,CTRW);
	}
	if(x<Width-1) // add right to left
	{
		if(!IsTRW)
			Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors+1].data(),nStates);
		else
			Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors+1].data(),nStates,CTRW);
	}
	if(y>0) // add top down
	{
		if(!IsTRW)
			Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors+2].data(),nStates);
		else
			Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors+2].data(),nStates,CTRW);
	}
	if(y<Height-1) // add bottom up
	{
		if(!IsTRW)
			Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors+3].data(),nStates);
		else
			Add2Message(message_org,pSpatialMessage[plane][offset*nNeighbors+2].data(),nStates,CTRW);
	}

	if(IsTRW)
		Add2Message(message_org,pDualMessage[plane][offset1].data(),nStates,CTRW-1);

	T_message*& message=pDualMessage[1-plane][offset1].data();
    
	T_message Min;
    // use the data term
    ////testing places to put the constraint.
//    if(plane==0) // from vx plane to vy plane
//        for(size_t l=0;l<nStates1;l++){
//            bool havezero = false;
//            int i;
//            int * dat = pDataTerm[offset].data()+l*nStates;
//            Min =message_org[0]+dat[0];
//            for(i=1;i<nStates;i++){
//                Min=__min(Min,message_org[i]+dat[i]);
//                if(dat[i]==0) havezero = true;
//            }
//            message[l]=havezero?0:Min;
////            if(havezero) cout << " z("<<x<<","<<y<<") " << endl;
//            //return result;
//            //message[l]=CStochastic::Min(nStates,pDataTerm[offset].data()+l*nStates,message_org);
//        }
//    else					// from vy plane to vx plane
//        for(size_t l=0;l<nStates1;l++)
//        {
//            bool havezero = false;
//            Min=message_org[0]+pDataTerm[offset].data()[l];
//            for(size_t h=0;h<nStates;h++){
//                Min=__min(Min,message_org[h]+pDataTerm[offset].data()[h*nStates1+l]);
//                if(pDataTerm[offset].data()[h*nStates1+l]==0) havezero = true;
//            }
//            message[l]=havezero?0:Min;
//        }
    
	// use the data term
	if(plane==0) // from vx plane to vy plane
		for(size_t l=0;l<nStates1;l++)
			message[l]=CStochastic::Min(nStates,pDataTerm[offset].data()+l*nStates,message_org);
	else					// from vy plane to vx plane
		for(size_t l=0;l<nStates1;l++)
		{
//            bool havezero = false;
			Min=message_org[0]+pDataTerm[offset].data()[l];
            for(size_t h=0;h<nStates;h++){
				Min=__min(Min,message_org[h]+pDataTerm[offset].data()[h*nStates1+l]);
//                if(pDataTerm[offset].data()[h*nStates1+l]==0) havezero = true;
            }
			message[l]=Min;
//            if(havezero) cout << " z("<<x<<","<<y<<") " << endl;
		}
    
	// normalize the message
	Min=CStochastic::Min(nStates1,message);
	for(size_t l=0;l<nStates;l++)
		message[l]-=Min;

	delete[] message_org;
}

vector<int> BPFlow::GetConstraint(int offset){
    vector<int> cs;
    for(int i=0; i<isconstraint.size(); i++){
        if(isconstraint[i]==offset){
            cs.push_back(i);
        }
    }
    return cs;
}

double BPFlow::IsAConstraint(int offset){
    for(int i=0; i<isconstraint.size(); i++){
        if(isconstraint[i]==offset)return reprojection_errors[i];
    }
    return -1;
}

double BPFlow::IsAConstraint(int x, int y){
    int offset=y*Width+x;
    return IsAConstraint(offset);
}

//------------------------------------------------------------------------------------------------
// compute belief
//------------------------------------------------------------------------------------------------
void BPFlow::ComputeBelief()
{
    /*
     Accumulate the cost of all the messages for each possible label. This is saved as the `belief', 
     which is what's chosen from to pick the right label.
     */
	for(size_t plane=0;plane<2;plane++)
	{
		memset(ptrBelief[plane],0,sizeof(T_message)*nTotalBelifElements[plane]);
		for(size_t i=0;i<Height;i++)
			for(size_t j=0;j<Width;j++)
			{
				size_t offset=i*Width+j;
				T_message* belief=pBelief[plane][offset].data();
				int nStates=pWinSize[plane][offset]*2+1;

//                if(nStates==1){
//                    //cout << "("<<x<<","<<y<<") with a constraint "<<endl;
//                    if(!IsAConstraint(j, i)){
//                        cout << "something went wrong. a constraint that's not."<<endl;
//                        exit(-1);
//                    }
//                }
				// add range term
				Add2Message(belief,pRangeTerm[plane][offset].data(),nStates);
				// add message from the dual layer
				Add2Message(belief,pDualMessage[plane][offset].data(),nStates);
				if(j>0)
					Add2Message(belief,pSpatialMessage[plane][offset*nNeighbors].data(),nStates);
				if(j<Width-1)
					Add2Message(belief,pSpatialMessage[plane][offset*nNeighbors+1].data(),nStates);
				if(i>0)
					Add2Message(belief,pSpatialMessage[plane][offset*nNeighbors+2].data(),nStates);
				if(i<Height-1)
					Add2Message(belief,pSpatialMessage[plane][offset*nNeighbors+3].data(),nStates);
			}
	}
    
}

void BPFlow::FindOptimalSolution()
{
    /*For each variable, choose the labeling that minimizes the belief.*/
	for(size_t plane=0;plane<2;plane++)
		for(size_t i=0;i<Area;i++)
		{
			int nStates=pWinSize[plane][i]*2+1;
			double Min;
			int index=0;
			T_message* belief=pBelief[plane][i].data();
			Min=belief[0];
			for(int l=1;l<nStates;l++)
				if(Min>belief[l])
				{
					Min=belief[l];
					index=l;
				}
			pX[i*2+plane]=index;
		}
}

void BPFlow::FindOptimalSolutionSequential()
{
	for(size_t plane=0;plane<2;plane++)
		memset(ptrBelief[plane],0,sizeof(T_message)*nTotalBelifElements[plane]);

	for(size_t i=0;i<Height;i++)
		for(size_t j=0;j<Width;j++)
			for(size_t k=0;k<2;k++)
			{
				size_t plane;
				if(j%2==0)
					plane=k;
				else
					plane=1-k;

				size_t offset=i*Width+j;
				int nStates=pWinSize[plane][offset]*2+1;
				T_message* belief=pBelief[plane][offset].data();
				
				// add range term
				Add2Message(belief,pRangeTerm[plane][offset].data(),nStates);

				if (k==0)
					// add message from the dual layer
					Add2Message(belief,pDualMessage[plane][offset].data(),nStates);
				else
					for(int l=0;l<nStates;l++)
					{
						if(plane==0) // if the current is horizontal plane
							belief[l]+=pDataTerm[offset].data()[pX[offset*2+1]*nStates+l];
						else   // if the current is vertical plane
						{
							int nStates1=pWinSize[1-plane][offset]*2+1;
							belief[l]+=pDataTerm[offset].data()[l*nStates1+pX[offset*2]];
						}
					}

				if(j>0) // horizontal energy
					for(int l=0;l<nStates;l++)
						belief[l]+=__min((double)abs(l-pWinSize[plane][offset]+pOffset[plane][offset]-pX[(offset-1)*2+plane]+pWinSize[plane][offset-1]-pOffset[plane][offset+1])*s,d);
				if(i>0) // vertical energy
					for(int l=0;l<nStates;l++)
						belief[l]+=__min((double)abs(l-pWinSize[plane][offset]+pOffset[plane][offset]-pX[(offset-Width)*2+plane]+pWinSize[plane][offset-Width]-pOffset[plane][offset-Width])*s,d);
				if(j<Width-1)
					Add2Message(belief,pSpatialMessage[plane][offset*nNeighbors+1].data(),nStates);
				if(i<Height-1)
					Add2Message(belief,pSpatialMessage[plane][offset*nNeighbors+3].data(),nStates);

				// find the minimum
				int index=0;
				double Min=belief[0];
				for(int l=1;l<nStates;l++)
					if(Min>belief[l])
					{
						Min=belief[l];
						index=l;
					}
				pX[offset*2+plane]=index;
			}
}

//------------------------------------------------------------------------------------------------
// function to get energy
//------------------------------------------------------------------------------------------------
double BPFlow::GetEnergy(bool all)
{
	double energy=0;
	for(size_t i=0;i<Height;i++)
		for(size_t j=0;j<Width;j++)
		{
			size_t offset=i*Width+j;
            if(all){
                //smoothness
                for(size_t k=0;k<2;k++)
                {
                    if(j<Width-1)
                    {
                        s=Im_s.data()[offset*2+k];
                        d=Im_d.data()[offset*2+k];
                        //s=m_s;
                        //d=m_d;
                        energy+=__min((double)abs(pX[offset*2+k]-pWinSize[k][offset]+pOffset[k][offset]-pX[(offset+1)*2+k]+pWinSize[k][offset+1]-pOffset[k][offset+1])*s,d);
                    }
                    if(i<Height-1)
                    {
                        s=Im_s.data()[offset*2+k];
                        d=Im_d.data()[offset*2+k];
                        //s=m_s;
                        //d=m_d;
                        energy+=__min((double)abs(pX[offset*2+k]-pWinSize[k][offset]+pOffset[k][offset]-pX[(offset+Width)*2+k]+pWinSize[k][offset+Width]-pOffset[k][offset+Width])*s,d);
                    }
                }
                
                //range
                for(size_t k=0;k<2;k++)
                    energy+=pRangeTerm[k][offset].data()[pX[offset*2+k]];
            }
            
            //sift
			int vx=pX[offset*2];
			int vy=pX[offset*2+1];
			int nStates=pWinSize[0][offset]*2+1;
			energy+=pDataTermStore[offset].data()[vy*nStates+vx];
            
		}
	return energy;
}


///*An energy function for debugging.*/
//double BPFlow::GetEnergy()
//{
//    cout << "using the debugging energy function." << endl;
////    int wsize = 5;
////    int numLabels = wsize*2+1;
////    double histx[numLabels];
////    double histy[numLabels];
////    for(int i=0; i<numLabels; i++)
////    {
////        histx[i] = 0;
////        histy[i] = 0;
////    }
//    
//    double energy=0;
//    for(size_t i=0;i<Height;i++)
//        for(size_t j=0;j<Width;j++)
//        {
//            size_t offset=i*Width+j;
//            //smoothness
////            for(size_t k=0;k<2;k++)
////            {
////                if(j<Width-1)
////                {
////                    s=Im_s.data()[offset*2+k];
////                    d=Im_d.data()[offset*2+k];
////                    //s=m_s;
////                    //d=m_d;
////                    energy+=__min((double)abs(0-pWinSize[k][offset]+pOffset[k][offset]+pWinSize[k][offset+1]-pOffset[k][offset+1])*s,d);
////                }
////                if(i<Height-1)
////                {
////                    s=Im_s.data()[offset*2+k];
////                    d=Im_d.data()[offset*2+k];
////                    //s=m_s;
////                    //d=m_d;
////                    energy+=__min((double)abs(0-pWinSize[k][offset]+pOffset[k][offset]+pWinSize[k][offset+Width]-pOffset[k][offset+Width])*s,d);
////                }
////            }
//            
////            //range
////            for(size_t k=0;k<2;k++)
////                energy+=pRangeTerm[k][offset].data()[0];
//            
//            
////            if(Masked(j,i)) continue;
//            
//            //sift
//            int vx=pX[offset*2];//pWinSize[0][offset];//
//            int vy=pX[offset*2+1];//pWinSize[1][offset];//
//            int nStates=pWinSize[0][offset]*2+1;
//            energy+=pDataTerm[offset][vy*nStates+vx];
//
//            
//        }
//    
////    for(int i=0; i<numLabels; i++)
////    {
////        cout << i-wsize << ": " << histx[i] << ", " << histy[i] << endl;
////    }
//    
//    return energy;
//}


void BPFlow::GetEnergyImage(double * e)
{
	for(size_t i=0;i<Height;i++)
		for(size_t j=0;j<Width;j++)
		{
			size_t offset=i*Width+j;
            
			int vx=pX[offset*2];
			int vy=pX[offset*2+1];
			int nStates=pWinSize[0][offset]*2+1;
            
            e[offset] = pDataTermStore[offset].data()[vy*nStates+vx];
		}
}


void BPFlow::ComputeVelocity(double * flow)
{
    for(int i=0;i<Area;i++)
    {
//        if(pWinSize[0][i]==0)
        flow[i*2]=pX[i*2]+pOffset[0][i]-pWinSize[0][i];
        flow[i*2+1]=pX[i*2+1]+pOffset[1][i]-pWinSize[1][i];
        
//        if(IsAConstraint(i)){
//            cout << "("<<(i%Width)<<","<<i/Width<<") has ("<<flow[i*2]<<","<<flow[i*2+1]<<")" << endl;
////            cout <<"         x: " << pX[i*2] << " + offset: "<<pOffset[0][i]<<" - win: " <<pWinSize[0][i]<<endl;
////            cout <<"         y: " << pX[i*2+1] << " + offset: "<<pOffset[1][i]<<" - win: " <<pWinSize[1][i]<<endl;
//        }
    }
}


void BPFlow::DebugFlowResult(double * flow) {
    
    /*Output info about the flow for debugging.
     *
     * */
    
    //    double ax, ay, sx=0, sy=0;
    //    int countnonzero=0, counttouched=0;
    
    int numLabels = 2*pWinSize[0][0]+1;
    int wsize = pWinSize[0][0];
    double histx[numLabels];
    double histy[numLabels];
    
    for (int i = 0; i < numLabels; i++) {
        histx[i] = 0;
        histy[i] = 0;
    }
    
    //labels should be between 0 and 120.
    for (int i = 0; i < Height; i++)
        for (int j = 0; j < Width; j++)
        {
            int rvarX = (i * Width + j) * 2 + 0;
            int rvarY = (i * Width + j) * 2 + 1;
            
//            int index = i * Width + j;
//            int wsize = hypSet[index];
            
            histx[(int) flow[rvarX]+wsize]++;
            histy[(int) flow[rvarY]+wsize]++;
        }
    
    std::cout << "----flow histogram----" << std::endl;
    for (int i = 0; i < numLabels; i++) {
        std::cout << "hist[" << i - (numLabels-1)/2 << "] = " << histx[i] << ", " << histy[i] << std::endl;
    }
}


void BPFlow::ComputeVelocity(double * flowX, double * flowY)
{
    for(int i=0;i<Area;i++)
    {
        flowX[i]=pX[i*2]+pOffset[0][i]-pWinSize[0][i];
        flowY[i]=pX[i*2+1]+pOffset[1][i]-pWinSize[1][i];
    }
}


//------------------------------------------------------------------------------------------------
// multi-grid belie propagation
//------------------------------------------------------------------------------------------------
void BPFlow::generateCoarserLevel(BPFlow &bp)
{
	//------------------------------------------------------------------------------------------------
	// set the dimensions and parameters
	//------------------------------------------------------------------------------------------------
	bp.Width=Width/2;
	if(Width%2==1)
		bp.Width++;

	bp.Height=Height/2;
	if(Height%2==1)
		bp.Height++;

	bp.Area=bp.Width*bp.Height;
	bp.s=s;
	bp.d=d;

	DImage foo;
	Im_s.smoothing(foo);
	foo.imresize(bp.Im_s,(int)bp.Width,(int)bp.Height);
	Im_d.smoothing(foo);
	foo.imresize(bp.Im_d,(int)bp.Width,(int)bp.Height);

	bp.IsDisplay=IsDisplay;
	bp.nNeighbors=nNeighbors;

	//------------------------------------------------------------------------------------------------
	// allocate buffers
	//------------------------------------------------------------------------------------------------
	for(int i=0;i<2;i++)
	{
		bp.pOffset[i]=new int[bp.Area];
		bp.pWinSize[i]=new int[bp.Area];
		ReduceImage(bp.pOffset[i],(int)Width,(int)Height,pOffset[i]);
		ReduceImage(bp.pWinSize[i],(int)Width,(int)Height,pWinSize[i]);
	}
	//------------------------------------------------------------------------------------------------
	// generate data term
	//------------------------------------------------------------------------------------------------
	bp.AllocateBuffer(bp.pDataTerm,bp.ptrDataTerm,bp.pWinSize[0],bp.pWinSize[1]);
	for(int i=0;i<bp.Height;i++)
		for(int j=0;j<bp.Width;j++)
		{
			int offset=(int)i*bp.Width+j;
			for(int ii=0;ii<2;ii++)
				for(int jj=0;jj<2;jj++)
				{
					int y=i*2+ii;
					int x=j*2+jj;
					if(y<Height && x<(int)Width)
					{
						int nStates=(bp.pWinSize[0][offset]*2+1)*(bp.pWinSize[1][offset]*2+1);
						for(int k=0;k<nStates;k++)
							bp.pDataTerm[offset].data()[k]+=pDataTerm[y*Width+x].data()[k];
					}
				}
		}
	//------------------------------------------------------------------------------------------------
	// generate range term
	//------------------------------------------------------------------------------------------------
	bp.ComputeRangeTerm(gamma/2);
}


void BPFlow::propagateFinerLevel(BPFlow &bp)
{
	for(int i=0;i<bp.Height;i++)
		for(int j=0;j<bp.Width;j++)
		{
			int y=i/2;
			int x=j/2;
//			int nStates1=pWinSize[0][y*Width+x]*2+1;
//			int nStates2=pWinSize[1][y*Width+x]*2+1;
			for(int k=0;k<2;k++)
			{
				memcpy(bp.pDualMessage[k][i*bp.Width+j].data(),
						pDualMessage[k][y*Width+x].data(),
						sizeof(T_message)*(pWinSize[k][y*Width+x]*2+1));

				for(int l=0;l<nNeighbors;l++)
					memcpy(bp.pSpatialMessage[k][(i*bp.Width+j)*nNeighbors+l].data(),
							pSpatialMessage[k][(y*Width+x)*nNeighbors+l].data(),
							sizeof(T_message)*(pWinSize[k][y*Width+x]*2+1));
			}
		}
}


template<class T>
void BPFlow::ReduceImage(T* pDstData,int width,int height,const T *pSrcData)
{
	int DstWidth=width/2;
	if(width%2==1)
		DstWidth++;
	int DstHeight=height/2;
	if(height%2==1)
		DstHeight++;
	memset(pDstData,0,sizeof(T)*DstWidth*DstHeight);
	int sum=0;
	for(int i=0;i<DstHeight;i++)
		for(int j=0;j<DstWidth;j++)
		{
			int offset=i*DstWidth+j;
			sum=0;
			for(int ii=0;ii<2;ii++)
				for(int jj=0;jj<2;jj++)
				{
					int x=j*2+jj;
					int y=i*2+ii;
					if(y<height && x<width)
					{
						pDstData[offset]+=pSrcData[y*width+x];
						sum++;
					}
				}
			pDstData[offset]/=sum;
		}
}
