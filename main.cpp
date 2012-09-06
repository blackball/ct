/**
 * Tracking single object in video.
 */

#include "cap.h"
#include "ct.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>

static void 
test_video(const char * vname) {
    struct ct_t *ct;
    Capture cap;
    IplImage *frame, *gray = 0;
    /* the target bounding-box in the first frame */
    CvRect obj_box = cvRect(480,290, 75, 95);
    /* create for tracker's parameters */
    ct = ct_new();
    cap.open(vname);
    frame = cap.queryFrame();
    gray = cvCreateImage(cvGetSize(frame), 8, 1);
    cvCvtColor(frame, gray, CV_BGR2GRAY);
    /* initialize */
    ct_init(ct, gray, &obj_box);

    cvNamedWindow("result", 1);
    while (cap.grab()) {
	frame = cap.retrieve();
	cvCvtColor(frame, gray, CV_BGR2GRAY);	
	/* tracking */
	ct_update(ct, gray, &obj_box);
	
	cvRectangle(frame, cvPoint(obj_box.x, obj_box.y), 
		    cvPoint(obj_box.x+obj_box.width, obj_box.y + obj_box.height), cvScalar(0,0,255,0),2,8,0);
	
	cvShowImage("result", frame);
	if (27 == cvWaitKey(1)) break;
    }
    cvDestroyWindow("result");
    /* free all resources in tracker */
    ct_free(&ct);
}

int main(int argc ,char *argv) {
    test_video("videoname.avi");
    return 0;
}
