/**
 * A warpper class to make image list testing easy.
 *
 * @blackball (bugway@gmail.com)
 */

#ifndef IMG_QUEUE_H
#define IMG_QUEUE_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>

class ImgQueue {
public:
    ImgQueue():m_fp(0), m_img(0){}
    ~ImgQueue(){ _release(); }
    void open(const char *fname) {
	_release();
	m_fp = fopen(fname, "r");
	if (!m_fp) {
	    fprintf(stderr, "Can not open file %s!", fname);
	    exit(0);
	}
    }
    IplImage * nextImg(int color_or_gray ) {
	if (!m_fp) return NULL;
	if (m_img) cvReleaseImage( &m_img );

	if (1 == fscanf(m_fp, "%s", m_name)) {
	    m_img = cvLoadImage(m_name, color_or_gray & 0x1 );
	}
	return m_img;
    }
protected:
    void _release() {
	if (m_fp) fclose(m_fp);
	if (m_img) cvReleaseImage( &m_img );
    }
private:
    FILE * m_fp;
    char m_name[ MAX_PATH ];
    IplImage * m_img;
};

#endif 
