/**
 * Mouse select multiple objects and track in video
 *
 * @blackball
 */

#include "cap.h"
#include "ct.h"
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#define MAX_OBJECTS 10
struct region_params {
    int n;
    const char * win_name;
    CvPoint loc0[ MAX_OBJECTS ];
    CvPoint loc1[ MAX_OBJECTS ];
    IplImage * objects[ MAX_OBJECTS ];
    IplImage * orig_img;
    IplImage * cur_img;
};

static void
_mouse_cb(int e, int x, int y, int flags, void * _p) {
    struct region_params * p = (struct region_params *)_p;
    CvPoint *loc;
    int n;
    IplImage * tmp;

    static int pressed = FALSE;

    switch( e ) { 
    case CV_EVENT_LBUTTONDOWN:
	n = p->n;
	if( n == MAX_OBJECTS )
	    return;
	loc = p->loc0;
	loc[n].x = x;
	loc[n].y = y;
	pressed = TRUE;
	break;
	/* on left button up, finalize the rectangle and draw it in black */
    case CV_EVENT_LBUTTONUP:
	n = p->n;
	if( n == MAX_OBJECTS )
	    return;
	loc = p->loc1;
	loc[n].x = x;
	loc[n].y = y;
	cvReleaseImage( &(p->cur_img) );
	p->cur_img = NULL;
	cvRectangle( p->orig_img, p->loc0[n], loc[n], CV_RGB(0,0,0), 1, 8, 0 );
	cvShowImage( p->win_name, p->orig_img );
	pressed = FALSE;
	p->n++;
	break;
    case CV_EVENT_MOUSEMOVE:
	if (flags & CV_EVENT_FLAG_LBUTTON) {
	    n = p->n;
	    if( n == MAX_OBJECTS )
		return;
	    tmp = cvCloneImage( p->orig_img );
	    loc = p->loc0;
	    cvRectangle( tmp, loc[n], cvPoint(x, y), CV_RGB(255,255,255), 1, 8, 0 );
	    cvShowImage( p->win_name, tmp );
	    if( p->cur_img )
		cvReleaseImage( &(p->cur_img) );
	    p->cur_img = tmp;
	}
	break;
    }
}

static int
_get_regions(const IplImage *frame, CvRect **regions) {
    const char * win_name = "select regions";
    struct region_params params;
    CvRect *r;
    int i, x0, y0, x1, y1, w, h;

    params.win_name = win_name;
    params.orig_img = cvCloneImage(frame);
    params.cur_img  = NULL;
    params.n = 0;

    cvNamedWindow( win_name, 1 );
    cvShowImage(win_name, frame);
    cvSetMouseCallback( win_name, &_mouse_cb, &params);
    cvWaitKey(0);
    cvDestroyWindow( win_name );
    cvReleaseImage( &(params.orig_img) );
    if ( params.cur_img )
	cvReleaseImage( &(params.cur_img) );

    if ( params.n == 0 ) {
	*regions = NULL;
	return 0;
    }

    r = (CvRect *)malloc( params.n * sizeof(CvRect) );
    /* temporal min, max */
#define _tmin(a,b) ((a) > (b) ? (b) : (a))
#define _tmax(a,b) ((a) < (b) ? (b) : (a))
    
    for (i = 0; i < params.n; ++i) {
	x0 = _tmin(params.loc0[i].x, params.loc1[i].x);
	x1 = _tmax(params.loc0[i].x, params.loc1[i].x);
	y0 = _tmin(params.loc0[i].y, params.loc1[i].y);
	y1 = _tmax(params.loc0[i].y, params.loc1[i].y);
	w = x1 - x0;
	h = y1 - y0;
	r[i] = cvRect(x0, y0, w, h);
    }
    *regions = r;
    return params.n;
}


static void
_test_video_multiple(const char *vname) {
    struct ct_t **pcts;
    CvRect *objects;
    int n = 0;
    int i;
    Capture cap;
    IplImage *frame = 0;
    IplImage *gray = 0;

    cap.open(vname);
    frame = cap.queryFrame();
    gray = cvCreateImage(cvGetSize(frame), 8, 1);
    cvCvtColor(frame, gray, CV_BGR2GRAY);
    
    n = _get_regions(frame, &objects);

    if (n < 1) {
	cvReleaseImage(&gray);
	return ;
    }

    pcts = (struct ct_t **)malloc(n * sizeof(struct ct_t **));
    for (i = 0; i < n; ++i) {
	pcts[i] = ct_new();
	ct_init(pcts[i], gray, &objects[i]);
    }

    cvNamedWindow("tracking", 1);
    while ( cap.grab() ) {
	frame = cap.retrieve();
	cvCvtColor(frame, gray, CV_BGR2GRAY);

#define _draw(f, r)						\
	cvRectangle(f, cvPoint((r).x, (r).y),			\
		    cvPoint((r).x+(r).width, (r).y+(r).height), \
		    cvScalar(255,0,0,0), 2,8,0)
	
	for (i = 0; i < n; ++i) {
	    ct_update(pcts[i], gray, &objects[i]);
	    _draw(frame, objects[i]);
	}
	
	cvShowImage("tracking", frame);
	if (27 == cvWaitKey(1)) break;
    }
#undef _draw
    
    cvDestroyWindow("tracking");
    cvReleaseImage( &gray );
    
    for (i = 0; i < n; ++i) {
	ct_free( &pcts[i] );
    }
    free( pcts );
    free( objects );
}


int main(int argc ,char *argv) {

    test_video_multiple("D:\\video_dataset\\classroom_video\\classroom.wmv");
    return 0;
}
