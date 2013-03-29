/*  Copyright 2011 AIT Austrian Institute of Technology
*
*   This file is part of OpenTLD.
*
*   OpenTLD is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   OpenTLD is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with OpenTLD.  If not, see <http://www.gnu.org/licenses/>.
*
*/
/*
 * TLD.cpp
 *
 *  Created on: Nov 17, 2011
 *      Author: Georg Nebehay
 */

#include "TLD.h"
#include "NNClassifier.h"
#include "TLDUtil.h"
#include <iostream>
#include <cstdio>
#include <bot_core/bot_core.h>

using namespace std;

namespace tld {

TLD::TLD() {
	trackerEnabled = true;
	detectorEnabled = true;
	learningEnabled = true;
	alternating = true;
	valid = false;
	wasValid = false;
	learning = false;
	currBB = NULL;
        scale_factor = 1.f;
        currpIdx = -1, currnIdx = -1;

	detectorCascade = new DetectorCascade();
	nnClassifier = detectorCascade->nnClassifier;

	medianFlowTracker = new MedianFlowTracker();
}

TLD::~TLD() {
	storeCurrentData();

	delete detectorCascade;
	delete medianFlowTracker;
}

void TLD::release() {
	detectorCascade->release();
	medianFlowTracker->cleanPreviousData();
	delete currBB;
}

void TLD::storeCurrentData() {
	prevImg.release();
	prevImg = currImg; //Store old image (if any)
	prevBB = currBB;		//Store old bounding box (if any)

	detectorCascade->cleanPreviousData(); //Reset detector results
	medianFlowTracker->cleanPreviousData();

	wasValid = valid;
}

void TLD::selectObject(Mat img, Rect * bb) {
	//Delete old object
	detectorCascade->release();

	detectorCascade->objWidth = bb->width;
	detectorCascade->objHeight = bb->height;

	//Init detector cascade
	detectorCascade->init();

        // if (scale_factor != 1.f) { 
        //     int method = cv::INTER_LINEAR; //cv::INTER_AREA : cv::INTER_CUBIC;
        //     cv::resize(img, currImg, cv::Size(), scale_factor, scale_factor, method);
        //     std::cerr << "img: " << img.rows << "x" << img.cols << std::endl;
        //     std::cerr << "input RECT: " << bb->tl() << " " << bb->br() << std::endl;
        //     int w = bb->br().x - bb->tl().x; 
        //     int h = bb->br().y - bb->tl().y; 
        //     bb->x = ((bb->tl().x + bb->br().x) / 2 - w/2) * scale_factor;
        //     bb->y = ((bb->tl().y + bb->br().y) / 2 - h/2) * scale_factor;
        //     bb->width = w;
        //     bb->height = h;
        //     std::cerr << "resized RECT: " << bb->tl() << " " << bb->br() << std::endl;
        //     std::cerr << "currimg: " << currImg.rows << "x" << currImg.cols << std::endl;
        //     currBB = bb;
        // } else { 
            currImg = img;
            currBB = bb;
            //}
	currConf = 1;
	valid = true;

	initialLearning();

}

bool TLD::processImage(Mat img) {
        // if (scale_factor != 1.f) { 
        //     int method = cv::INTER_LINEAR; //cv::INTER_AREA : cv::INTER_CUBIC;
        //     cv::resize(img, img, cv::Size(), scale_factor, scale_factor, method);
        // } else  
        //     img = img;

	storeCurrentData();

	Mat grey_frame;
        if (img.channels() == 1) 
            grey_frame = img.clone(); 
        else 
            cvtColor( img, grey_frame, CV_BGR2GRAY );
	currImg = grey_frame; // Store new image , right after storeCurrentData();

        int64_t tic = bot_timestamp_now(); 
	if(trackerEnabled) {
		medianFlowTracker->track(prevImg, currImg, prevBB);
	}
        printf("MEDIANFLOW (%s): %f ms\n", (medianFlowTracker->trackerBB != NULL) ? "GOOD":"NULL", 
            (bot_timestamp_now() - tic) * 1e-3);

        tic = bot_timestamp_now(); 
	if(detectorEnabled && (!alternating || medianFlowTracker->trackerBB == NULL)) {
		detectorCascade->detect(grey_frame);
	}
        printf("DETECT: %f ms\n", (bot_timestamp_now() - tic) * 1e-3);
        tic = bot_timestamp_now(); 
	fuseHypotheses();
        printf("FUSE: %f ms\n", (bot_timestamp_now() - tic) * 1e-3);

        tic = bot_timestamp_now(); 
	learn();
        printf("LEARN: %f ms\n", (bot_timestamp_now() - tic) * 1e-3);
        
        // drawTemplates();
        return (currBB != NULL);
}

void TLD::drawTemplates() { 
    int tp = detectorCascade->nnClassifier->truePositives->size(); 
    int tp_cols = 20;
    int tp_rows = std::max(tp / tp_cols + 1, 1);
    Mat1b tp_img = Mat1b::zeros(TLD_IMG_PATCH_SIZE * tp_rows, TLD_IMG_PATCH_SIZE * tp_cols);
        
    for (int j=0; j<tp; j++) { 
        NormalizedPatch p = (*(detectorCascade->nnClassifier->truePositives))[j];
        Mat roi(tp_img, Rect((j%tp_cols)*TLD_IMG_PATCH_SIZE,(j/tp_cols)*TLD_IMG_PATCH_SIZE,
                             TLD_IMG_PATCH_SIZE,TLD_IMG_PATCH_SIZE));
        resize(p.img_patch, roi, roi.size());
    }
    
    if (currpIdx >= 0) { 
        Mat roi(tp_img, Rect((currpIdx%tp_cols)*TLD_IMG_PATCH_SIZE,(currpIdx/tp_cols)*TLD_IMG_PATCH_SIZE,
                             TLD_IMG_PATCH_SIZE,TLD_IMG_PATCH_SIZE));
        rectangle(roi, Point(1,1), Point(roi.cols-1,roi.rows-1), Scalar(255), 2);
    }

    imshow("true positives", tp_img);

    int fp = detectorCascade->nnClassifier->falsePositives->size(); 
    int fp_cols = 20;
    int fp_rows = std::max(fp / fp_cols + 1, 1);
    Mat1b fp_img = Mat1b::zeros(TLD_IMG_PATCH_SIZE * fp_rows, TLD_IMG_PATCH_SIZE * fp_cols);
        
    for (int j=0; j<fp; j++) { 
        NormalizedPatch p = (*(detectorCascade->nnClassifier->falsePositives))[j];
        Mat roi(fp_img, Rect((j%fp_cols)*TLD_IMG_PATCH_SIZE,(j/fp_cols)*TLD_IMG_PATCH_SIZE,
                             TLD_IMG_PATCH_SIZE,TLD_IMG_PATCH_SIZE));
        resize(p.img_patch, roi, roi.size());
    }

    if (currnIdx >= 0) { 
        Mat roi(fp_img, Rect((currnIdx%fp_cols)*TLD_IMG_PATCH_SIZE,(currnIdx/fp_cols)*TLD_IMG_PATCH_SIZE,
                             TLD_IMG_PATCH_SIZE,TLD_IMG_PATCH_SIZE));
        rectangle(roi, Point(1,1), Point(roi.cols-1,roi.rows-1), Scalar(255), 2);
    }

    imshow("false positives", fp_img);

    // cout << "NN has now " << detectorCascade->nnClassifier->truePositives->size() << " positives and " << detectorCascade->nnClassifier->falsePositives->size() << " negatives.\n";
}

cv::Mat TLD:: currPosPatch() { 
    if (currpIdx >= 0) { 
        NormalizedPatch p = (*(detectorCascade->nnClassifier->truePositives))[currpIdx];
        return p.img_patch.clone();
    }
    return Mat();
}

void TLD::fuseHypotheses() {
	Rect* trackerBB = medianFlowTracker->trackerBB;
	int numClusters = detectorCascade->detectionResult->numClusters;
	Rect* detectorBB = detectorCascade->detectionResult->detectorBB;


	currBB = NULL;
        currpIdx = -1, currnIdx = -1;

	currConf = 0;
	valid = false;

	float confDetector = 0;

	if(numClusters == 1) {
		confDetector = nnClassifier->classifyBB(currImg, detectorBB, &currpIdx, &currnIdx);
	}

	if(trackerBB != NULL) {
            float confTracker = nnClassifier->classifyBB(currImg, trackerBB, &currpIdx, &currnIdx);

		if(numClusters == 1 && confDetector > confTracker && tldOverlapRectRect(*trackerBB, *detectorBB) < 0.5) {

			currBB = tldCopyRect(detectorBB);
			currConf = confDetector;
		} else {
			currBB = tldCopyRect(trackerBB);
			currConf = confTracker;
			if(confTracker > nnClassifier->thetaTP) {
				valid = true;
			} else if(wasValid && confTracker > nnClassifier->thetaFP) {
				valid = true;
			}
		}
	} else if(numClusters == 1) {
		currBB = tldCopyRect(detectorBB);
		currConf = confDetector;
	}


	/*
	float var = CalculateVariance(patch.values, nn->patch_size*nn->patch_size);

	if(var < min_var) { //TODO: Think about incorporating this
		printf("%f, %f: Variance too low \n", var, classifier->min_var);
		valid = 0;
	}*/
}

void TLD::initialLearning() {
	learning = true; //This is just for display purposes

	DetectionResult* detectionResult = detectorCascade->detectionResult;

	detectorCascade->detect(currImg);

	//This is the positive patch
	NormalizedPatch patch;
	tldExtractNormalizedPatchRect(currImg, currBB, patch.values);
	patch.positive = 1;

        // Mat1b img_patch(currImg, *currBB);
        // resize(img_patch, patch.img_patch, patch.img_patch.size());

	float initVar = tldCalcVariance(patch.values, TLD_PATCH_SIZE*TLD_PATCH_SIZE);
	detectorCascade->varianceFilter->minVar = initVar/2;


	float * overlap = new float[detectorCascade->numWindows];
	tldOverlapRect(detectorCascade->windows, detectorCascade->numWindows, currBB,overlap);

	//Add all bounding boxes with high overlap

	vector< pair<int,float> > positiveIndices;
	vector<int> negativeIndices;

	//First: Find overlapping positive and negative patches

	for(int i = 0; i < detectorCascade->numWindows; i++) {

		if(overlap[i] > 0.6) {
			positiveIndices.push_back(pair<int,float>(i,overlap[i]));
		}

		if(overlap[i] < 0.2) {
			float variance = detectionResult->variances[i];

			if(!detectorCascade->varianceFilter->enabled || variance > detectorCascade->varianceFilter->minVar) { //TODO: This check is unnecessary if minVar would be set before calling detect.
				negativeIndices.push_back(i);
			}
		}
	}

	sort(positiveIndices.begin(), positiveIndices.end(), tldSortByOverlapDesc);

	vector<NormalizedPatch> patches;

	patches.push_back(patch); //Add first patch to patch list

	int numIterations = std::min<size_t>(positiveIndices.size(), 10); //Take at most 10 bounding boxes (sorted by overlap)
	for(int i = 0; i < numIterations; i++) {
		int idx = positiveIndices.at(i).first;
		//Learn this bounding box
		//TODO: Somewhere here image warping might be possible
		detectorCascade->ensembleClassifier->learn(currImg, &detectorCascade->windows[TLD_WINDOW_SIZE*idx], true, &detectionResult->featureVectors[detectorCascade->numTrees*idx]);
	}

	srand(1); //TODO: This is not guaranteed to affect random_shuffle

	random_shuffle(negativeIndices.begin(), negativeIndices.end());

	//Choose 100 random patches for negative examples
	for(size_t i = 0; i < std::min<size_t>(100,negativeIndices.size()); i++) {
		int idx = negativeIndices.at(i);

		NormalizedPatch patch;
		tldExtractNormalizedPatchBB(currImg, &detectorCascade->windows[TLD_WINDOW_SIZE*idx], patch.values);
		patch.positive = 0;
		patches.push_back(patch);

                int* boundary = &detectorCascade->windows[TLD_WINDOW_SIZE*idx];
                int x,y,w,h;
                tldExtractDimsFromArray(boundary, &x,&y,&w,&h);

                Rect bb(x,y,w,h);

                Mat1b img_patch(currImg, bb);
                resize(img_patch, patch.img_patch, patch.img_patch.size());

	}

	detectorCascade->nnClassifier->learn(patches);

	delete[] overlap;

}

//Do this when current trajectory is valid
void TLD::learn() {
	if(!learningEnabled || !valid || !detectorEnabled) {
		learning = false;
		return;
	}
	learning = true;

	DetectionResult* detectionResult = detectorCascade->detectionResult;

	if(!detectionResult->containsValidData) {
		detectorCascade->detect(currImg);
	}

	//This is the positive patch
	NormalizedPatch patch;
	tldExtractNormalizedPatchRect(currImg, currBB, patch.values);

	float * overlap = new float[detectorCascade->numWindows];
	tldOverlapRect(detectorCascade->windows, detectorCascade->numWindows, currBB,overlap);

	//Add all bounding boxes with high overlap

	vector<pair<int,float> > positiveIndices;
	vector<int> negativeIndices;
	vector<int> negativeIndicesForNN;

	//First: Find overlapping positive and negative patches

	for(int i = 0; i < detectorCascade->numWindows; i++) {

		if(overlap[i] > 0.6) {
			positiveIndices.push_back(pair<int,float>(i,overlap[i]));
		}

		if(overlap[i] < 0.2) {
			if(!detectorCascade->ensembleClassifier->enabled || detectionResult->posteriors[i] > 0.1) { //TODO: Shouldn't this read as 0.5?
				negativeIndices.push_back(i);
			}

			if(!detectorCascade->ensembleClassifier->enabled || detectionResult->posteriors[i] > 0.5) {
				negativeIndicesForNN.push_back(i);
			}

		}
	}

	sort(positiveIndices.begin(), positiveIndices.end(), tldSortByOverlapDesc);

	vector<NormalizedPatch> patches;

	patch.positive = 1;
        convert_to_img_patch(patch);
	patches.push_back(patch);

        // Mat1b img_patch(currImg, *currBB);
        // resize(img_patch, patch.img_patch, patch.img_patch.size());

	//TODO: Flip


	int numIterations = std::min<size_t>(positiveIndices.size(), 10); //Take at most 10 bounding boxes (sorted by overlap)

	for(size_t i = 0; i < negativeIndices.size(); i++) {
		int idx = negativeIndices.at(i);
		//TODO: Somewhere here image warping might be possible
		detectorCascade->ensembleClassifier->learn(currImg, &detectorCascade->windows[TLD_WINDOW_SIZE*idx], false, &detectionResult->featureVectors[detectorCascade->numTrees*idx]);
	}

	//TODO: Randomization might be a good idea
	for(int i = 0; i < numIterations; i++) {
		int idx = positiveIndices.at(i).first;
		//TODO: Somewhere here image warping might be possible
		detectorCascade->ensembleClassifier->learn(currImg, &detectorCascade->windows[TLD_WINDOW_SIZE*idx], true, &detectionResult->featureVectors[detectorCascade->numTrees*idx]);
	}

	for(size_t i = 0; i < negativeIndicesForNN.size(); i++) {
		int idx = negativeIndicesForNN.at(i);

		NormalizedPatch patch;
		tldExtractNormalizedPatchBB(currImg, &detectorCascade->windows[TLD_WINDOW_SIZE*idx], patch.values);
		patch.positive = 0;
                convert_to_img_patch(patch);
		patches.push_back(patch);

                // int* boundary = &detectorCascade->windows[TLD_WINDOW_SIZE*idx];
                // int x,y,w,h;
                // tldExtractDimsFromArray(boundary, &x,&y,&w,&h);

                // Rect bb(x,y,w,h);

                // Mat1b img_patch(currImg, bb);
                // resize(img_patch, patch.img_patch, patch.img_patch.size());
	}

	detectorCascade->nnClassifier->learn(patches);

        //cout << "NN has now " << detectorCascade->nnClassifier->truePositives->size() << " positives and " << detectorCascade->nnClassifier->falsePositives->size() << " negatives.\n";

	delete[] overlap;
}

typedef struct {
	int index;
	int P;
	int N;
} TldExportEntry;

void TLD::writeToFile(const char * path) {
	NNClassifier * nn = detectorCascade->nnClassifier;
	EnsembleClassifier* ec = detectorCascade->ensembleClassifier;

	FILE * file = fopen(path, "w");
	fprintf(file,"#Tld ModelExport\n");
	fprintf(file,"%d #width\n", detectorCascade->objWidth);
	fprintf(file,"%d #height\n", detectorCascade->objHeight);
	fprintf(file,"%f #min_var\n", detectorCascade->varianceFilter->minVar);
	fprintf(file,"%d #Positive Sample Size\n", nn->truePositives->size());



	for(size_t s = 0; s < nn->truePositives->size();s++) {
		float * imageData =nn->truePositives->at(s).values;
		for(int i = 0; i < TLD_PATCH_SIZE; i++) {
			for(int j = 0; j < TLD_PATCH_SIZE; j++) {
				fprintf(file, "%f ", imageData[i*TLD_PATCH_SIZE+j]);
			}
			fprintf(file, "\n");
		}
	}

	fprintf(file,"%d #Negative Sample Size\n", nn->falsePositives->size());

	for(size_t s = 0; s < nn->falsePositives->size();s++) {
		float * imageData = nn->falsePositives->at(s).values;
		for(int i = 0; i < TLD_PATCH_SIZE; i++) {
			for(int j = 0; j < TLD_PATCH_SIZE; j++) {
				fprintf(file, "%f ", imageData[i*TLD_PATCH_SIZE+j]);
			}
			fprintf(file, "\n");
		}
	}

	fprintf(file,"%d #numtrees\n", ec->numTrees);
	detectorCascade->numTrees = ec->numTrees;
	fprintf(file,"%d #numFeatures\n", ec->numFeatures);
	detectorCascade->numFeatures = ec->numFeatures;
	for(int i = 0; i < ec->numTrees; i++) {
		fprintf(file, "#Tree %d\n", i);

		for(int j = 0; j < ec->numFeatures; j++) {
			float * features = ec->features + 4*ec->numFeatures*i + 4*j;
			fprintf(file,"%f %f %f %f # Feature %d\n", features[0], features[1], features[2], features[3], j);
		}

		//Collect indices
		vector<TldExportEntry> list;

		for(int index = 0; index < pow(2.0f, ec->numFeatures); index++) {
			int p = ec->positives[i * ec->numIndices + index];
			if(p != 0) {
				TldExportEntry entry;
				entry.index = index;
				entry.P = p;
				entry.N = ec->negatives[i * ec->numIndices + index];
				list.push_back(entry);
			}
		}

		fprintf(file,"%d #numLeaves\n", list.size());
		for(size_t j = 0; j < list.size(); j++) {
			TldExportEntry entry = list.at(j);
			fprintf(file,"%d %d %d\n", entry.index, entry.P, entry.N);
		}
	}

	fclose(file);

}
 
void TLD::convert_to_img_patch(NormalizedPatch& patch) { 
    int len = TLD_PATCH_SIZE*TLD_PATCH_SIZE;
    
    float min = 0;
    for (int j=0; j<len; j++)
         if (patch.values[j] < min)
             min = patch.values[j];
    
    unsigned char patch_vals[len];
    for (int j=0; j<len; j++)
        patch_vals[j] = static_cast<unsigned char>(patch.values[j] - min);
 
    patch.img_patch = cv::Mat1b(TLD_PATCH_SIZE, TLD_PATCH_SIZE, &patch_vals[0]);
    resize(patch.img_patch, patch.img_patch, cv::Size(TLD_IMG_PATCH_SIZE, TLD_IMG_PATCH_SIZE));
}

bool TLD::readFromFile(const char * path) {
	release();

	NNClassifier * nn = detectorCascade->nnClassifier;
	EnsembleClassifier* ec = detectorCascade->ensembleClassifier;

	FILE * file = fopen(path, "r");

	if(file == NULL) {
		printf("Error: Model not found: %s\n", path);
                return false;
		// exit(1);
	}

	int MAX_LEN=255;
	char str_buf[255];
	fgets(str_buf, MAX_LEN, file); /*Skip line*/

	fscanf(file,"%d \n", &detectorCascade->objWidth);
	fgets(str_buf, MAX_LEN, file); /*Skip rest of line*/
	fscanf(file,"%d \n", &detectorCascade->objHeight);
	fgets(str_buf, MAX_LEN, file); /*Skip rest of line*/

	fscanf(file,"%f \n", &detectorCascade->varianceFilter->minVar);
	fgets(str_buf, MAX_LEN, file); /*Skip rest of line*/

	int numPositivePatches;
	fscanf(file, "%d \n", &numPositivePatches);
	fgets(str_buf, MAX_LEN, file); /*Skip line*/


	for(int s = 0; s < numPositivePatches; s++) {
		NormalizedPatch patch;

		for(int i = 0; i < 15; i++) { //Do 15 times

			fgets(str_buf, MAX_LEN, file); /*Read sample*/

			char * pch;
			pch = strtok (str_buf," \n");
			int j = 0;
			while (pch != NULL)
			{
				float val = atof(pch);
				patch.values[i*TLD_PATCH_SIZE+j] = val;

				pch = strtok (NULL, " \n");

				j++;
			}
		}

                convert_to_img_patch(patch); 

		nn->truePositives->push_back(patch);
	}
        std::cerr << "nn: truepos: " << nn->truePositives->size() << std::endl;

	int numNegativePatches;
	fscanf(file, "%d \n", &numNegativePatches);
	fgets(str_buf, MAX_LEN, file); /*Skip line*/


	for(int s = 0; s < numNegativePatches; s++) {
		NormalizedPatch patch;
		for(int i = 0; i < 15; i++) { //Do 15 times

			fgets(str_buf, MAX_LEN, file); /*Read sample*/

			char * pch;
			pch = strtok (str_buf," \n");
			int j = 0;
			while (pch != NULL)
			{
				float val = atof(pch);
				patch.values[i*TLD_PATCH_SIZE+j] = val;

				pch = strtok (NULL, " \n");

				j++;
			}
		}

                convert_to_img_patch(patch);

		nn->falsePositives->push_back(patch);
	}
        std::cerr << "nn: falsepos: " << nn->falsePositives->size() << std::endl;

	fscanf(file,"%d \n", &ec->numTrees);
	detectorCascade->numTrees = ec->numTrees;
	fgets(str_buf, MAX_LEN, file); /*Skip rest of line*/

	fscanf(file,"%d \n", &ec->numFeatures);
	detectorCascade->numFeatures = ec->numFeatures;
	fgets(str_buf, MAX_LEN, file); /*Skip rest of line*/

	int size = 2 * 2 * ec->numFeatures * ec->numTrees;
	ec->features = new float[size];
	ec->numIndices = pow(2.0f, ec->numFeatures);
	ec->initPosteriors();

	for(int i = 0; i < ec->numTrees; i++) {
		fgets(str_buf, MAX_LEN, file); /*Skip line*/

		for(int j = 0; j < ec->numFeatures; j++) {
			float * features = ec->features + 4*ec->numFeatures*i + 4*j;
			fscanf(file, "%f %f %f %f",&features[0], &features[1], &features[2], &features[3]);
			fgets(str_buf, MAX_LEN, file); /*Skip rest of line*/
		}

		/* read number of leaves*/
		int numLeaves;
		fscanf(file,"%d \n", &numLeaves);
		fgets(str_buf, MAX_LEN, file); /*Skip rest of line*/

		for(int j = 0; j < numLeaves; j++) {
			TldExportEntry entry;
			fscanf(file,"%d %d %d \n", &entry.index, &entry.P, &entry.N);
			ec->updatePosterior(i, entry.index, 1, entry.P);
			ec->updatePosterior(i, entry.index, 0, entry.N);
		}
	}

	detectorCascade->initWindowsAndScales();
	detectorCascade->initWindowOffsets();

	detectorCascade->propagateMembers();

	detectorCascade->initialised = true;

	ec->initFeatureOffsets();
        return true;
}

cv::Rect TLD::BB() {
    float x = currBB->tl().x, y = currBB->tl().y;
    float w = currBB->width, h = currBB->height;
    float sw = w / scale_factor, sh = h / scale_factor;
    cv::Rect bb(1.f/scale_factor * (x + w/2 - sw), 
                1.f/scale_factor * (y + h/2 - sh), 
                sw, sh); 
    return bb;                
}

} /* namespace tld */
