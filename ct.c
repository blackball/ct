/**
 * Real-time compressive tracker.
 *
 * @blackball (bugway@gmail.com)
 */

#include "ct.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <limits.h>
#include <float.h>
#include <math.h>


#ifdef __cplusplus
extern "C" {
#endif 

#define pow2(v) ((v) * (v))

/* CT params type */
struct ct_t {
    int outer_positive_radius;
    int search_window_radius;
    int feature_num;
    int feature_rect_num_min;
    int feature_rect_num_max;
    float learning_rate;
    float * mu_positive;
    float * mu_negative;
    float * sigma_positive;
    float * sigma_negative;
    struct rvec_t * detect_box;
    struct rvec_t * positive_box;
    struct rvec_t * negative_box;
    struct wvec_t * features;
    struct mat * detect_values;
    struct mat * positive_values;
    struct mat * negative_values;
    
    IplImage * integral_img; /* move this out in multiple tracking */
};

/*****************/
static unsigned int _rand_trace = 0;
static int
_irand(int min, int max) {
    _rand_trace = 0x00269ec3 + _rand_trace * 0x000343fd;
    return min + ((_rand_trace >> 16) & 0x7FF) % (max - min);
}

typedef struct rvec_t {
    int used;
    int capacity;
    CvRect *data;
} rvec_t;

static rvec_t *
rvec_new(int n) {
    rvec_t * v = (rvec_t *) malloc (sizeof(rvec_t));
    v->data   = (CvRect *) malloc (sizeof(CvRect) * n);
    v->used   = 0;
    v->capacity = n;
    return v;
}

#define rvec_at(v, i) ((v)->data + i)

static void
rvec_free(rvec_t **v) {
    if (!v) return ;
    
    free( (*v)->data );
    free( (*v) );
    *v = 0;
}

static void
rvec_clear(rvec_t *v) {
    v->used = 0;
}

static void
rvec_append(rvec_t *v, CvRect t) {
    if (v->used == v->capacity) {
	/* incremental step was set to the same size of previous */
	v->data = (CvRect *) realloc (v->data, sizeof(CvRect) * (v->capacity * 2));
    }
    v->data[ v->used++ ] = t; 
}

static int
rvec_size(const rvec_t * v) {
    return v->used;
}

struct mat {
    int w,h;
    float * data;
};

static struct mat *
mat_new(int w, int h) {
    struct mat * m = (struct mat *)malloc(sizeof(struct mat));
    m->w = w;
    m->h = h;
    m->data = (float *)malloc(sizeof(float) * w * h);
    return m;
}

static void
mat_realloc(struct mat * m, int nw, int nh) {
    assert(m);

    if (m->w == nw && m->h == nh) return ;
    
    if (m) {
	free(m->data);
	m->data = (float *)malloc(sizeof(float) * nw * nh);
	m->w = nw;
	m->h = nh;
    } 
}

static void
mat_free(struct mat **m) {
    if (!m) return ;
    free( *m );
    *m = 0;
}

#define mat_at(m, r, c)				\
    (*((m->data) + (r) * (m)->w + (c)))
    
struct wrect_t {
    CvRect r;
    float  w;
};

struct wvec_t {
    int *offsets;
    struct wrect_t * rects;
};

#define wvec_at(vec, i, j)			\
    ((vec)->rects + (vec)->offsets[i] + j)

#define wvec_step(vec, i)			\
    ((vec)->offsets[i+1] - (vec)->offsets[i])

struct wvec_t *
wvec_new(const int *arr, int num) {
    struct wvec_t * v;
    int i,sum;
    int sz = sizeof(struct wvec_t) + sizeof(int) * (num + 1); /* + 1 for total length */

    sum = 0;
    for (i = 0; i < num; ++i)
	sum += arr[i]; 

    sz += sizeof(struct wrect_t) * sum;

    v = (struct wvec_t *)malloc(sz);
    v->offsets = (int*)(v + 1);
    v->rects   = (struct wrect_t *)(v->offsets + num + 1);
    
    sum = 0;
    for (i = 0; i < num; ++i) {
	v->offsets[i] = sum;
	sum += arr[i];
    }
    v->offsets[i] = sum;
    
    return v;
}

static void
wvec_free(struct wvec_t **v) {
    if (!v) return ; 
    free( *v );
    *v = 0;
}

/* initialize haar rectangles */
static void 
_ct_inithaar(struct ct_t * ct, int objw, int objh) {
    int i, j, rmin = ct->feature_rect_num_min, rmax = ct->feature_rect_num_max;
    void * pmem = malloc(sizeof(int) * ct->feature_num + sizeof(float) * (rmax- rmin));
    const float _switch[2] = {1.0f, -1.0f};
    int *arr;
    float *cache;

    arr = (int*)pmem;

    cache = (float *)(arr + ct->feature_num);
    
    for (i = 0; i < ct->feature_num; ++i)
	arr[i] = _irand(ct->feature_rect_num_min, ct->feature_rect_num_max);
    
    for (i = 0; i < rmax - rmin; ++i) 
	cache[i] = 1.0f / (float)sqrt((float)(i + rmin));
    
    ct->features = wvec_new(arr, ct->feature_num);

    for (i = 0; i < ct->feature_num; ++i) {
	for (j = 0; j < arr[i]; ++j) {
	    struct wrect_t *p = wvec_at(ct->features, i, j);
	    p->r.x      = _irand(0, objw - 3);
	    p->r.y      = _irand(0, objh - 3);
	    p->r.width  = _irand(0, objw - p->r.x - 2);
	    p->r.height = _irand(0, objh - p->r.y - 2);
	    p->w = _switch[ _irand(0, 2) ] * cache[arr[i] - rmin];
	}
    }

    free( pmem );
}

static void
_ct_get_feature_value(const struct ct_t *ct, struct rvec_t * samples, struct mat * sample_value) {
    int xmin, xmax, ymin, ymax;
    int sample_size = rvec_size(samples);
    IplImage *iimg = ct->integral_img;
    int i, j, k;
    
    mat_realloc(sample_value, sample_size, ct->feature_num);
    
    for (i = 0; i < ct->feature_num; ++i) {
	for (j = 0; j < sample_size; ++j) {
	    float t = .0f;
	    int sz = wvec_step(ct->features, i);
	    for (k = 0; k < sz; ++k){
		CvRect r = samples->data[j];
		struct wrect_t *wr = wvec_at(ct->features, i, k);
		xmin = r.x + wr->r.x;
		xmax = r.x + wr->r.x + wr->r.width;
		ymin = r.y + wr->r.y;
		ymax = r.y + wr->r.y + wr->r.height;
		t += wr->w * (CV_IMAGE_ELEM(iimg, float, ymin, xmin) +
			      CV_IMAGE_ELEM(iimg, float, ymax, xmax) -
			      CV_IMAGE_ELEM(iimg, float, ymin, xmax) -
			      CV_IMAGE_ELEM(iimg, float, ymax, xmin));
	    }
	    mat_at(sample_value, i, j) = t;
	}
    }
}

static void
_ct_sampling(const IplImage *img, const CvRect *obj_box, int swr, struct rvec_t * samples) {
    int colsz = img->width  - obj_box->width - 1;
    int rowsz = img->height - obj_box->height - 1;
    int squared_radius =  pow2(swr);

    int minrow = max(0, obj_box->y - swr);
    int maxrow = min(rowsz - 1, obj_box->y + swr);
    int mincol = max(0, obj_box->x - swr);
    int maxcol = min(colsz - 1, obj_box->x + swr);
    int r,c;
    
    rvec_clear(samples);
    
    for (r = minrow; r <= maxrow; ++r) {
	for (c = mincol; c <= maxcol; ++c) {
	    int dist = pow2( obj_box->y - r ) + pow2( obj_box->x - c );
	    if ( dist < squared_radius ) {
		CvRect rt;
		rt.x = c;
		rt.y = r;
		rt.width  = obj_box->width;
		rt.height = obj_box->height;
                
		rvec_append(samples, rt);
	    }
	}
    }
}


static void
_ct_sampling_io(const IplImage *img, const CvRect *obj_box, int inner_radius, int outer_radius,
		int max_sample_num, struct rvec_t * samples) {
    int rowsz = img->height - obj_box->height - 1;
    int colsz = img->width - obj_box->width - 1;
    int inner_squared_radius = pow2(inner_radius);
    int outer_squared_radius = pow2(outer_radius);

    
    int minrow = max(0, obj_box->y - inner_radius);
    int maxrow = min(rowsz - 1, obj_box->y + inner_radius);
    int mincol = max(0, obj_box->x - inner_radius);
    int maxcol = min(colsz - 1, obj_box->x + inner_radius);
    int r,c, i = 0;
    int area = (maxrow - minrow + 1) * (maxcol - mincol + 1);
    
    rvec_clear( samples );
    for (r = minrow; r <= maxrow; ++r) {
	for (c = mincol; c <= maxcol; ++c) {
	    int dist = pow2( obj_box->y - r ) + pow2( obj_box->x - c );
	    if ( _irand(0, area) < max_sample_num && dist < inner_squared_radius &&
		 dist > outer_squared_radius) {
		CvRect rt;
		rt.x = c;
		rt.y = r;
		rt.width  = obj_box->width;
		rt.height = obj_box->height;
                
		rvec_append(samples, rt);
	    }
	}
    }
}

static void
_row_mean_std_dev(const float *vec, int len, float *mean, float *stddev) {
    int i;
    float tmean, sum = 0.0f;

    if (len < 1) return ;
    
    for (i = 0; i < len; ++i)
	sum += vec[i];

    tmean = sum / len;

    sum = 0.0f;
    for (i = 0; i < len; ++i)
	sum += (vec[i] - tmean) * (vec[i] - tmean);

    *stddev = (float)sqrt( sum / (len - 1));
    *mean = tmean;
}

static void
_ct_update_classifier(const struct mat *sample_value, float *mu, float *sigma, int len, float learning_rate) {
    float tmu, tsigma;
    int i;
    for (i = 0; i < len; ++i) {
	/* calculate mean value and the standard deviation value of a vector */
	_row_mean_std_dev(&mat_at(sample_value, i, 0), sample_value->w, &tmu, &tsigma);

	sigma[i] = (float)sqrt(learning_rate * sigma[i] * sigma[i] + (1.0f-learning_rate) * tsigma * tsigma 
			       + learning_rate * (1.0f - learning_rate) * (mu[i] - tmu) * (mu[i]-tmu));
	mu[i] = mu[i] * learning_rate + (1.0f-learning_rate) * tmu;
    }
}

static void
_ct_ratio_classifier(const struct ct_t *ct, int * ratio_max_idx, float * ratio_max) {
    float sum_ratio;
    int i, j, sample_num, feature_num = ct->feature_num;
    struct mat * pm = ct->detect_values;
    int idx = 0;
    float rmax = -FLT_MAX;
    const float * sigma_pos = ct->sigma_positive;
    const float * sigma_neg = ct->sigma_negative;
    const float * mu_pos    = ct->mu_positive;
    const float * mu_neg    = ct->mu_negative;
    sample_num = pm->w;
	
    for (j = 0; j < sample_num; ++j) {
	sum_ratio = 0.0f;
	for (i = 0; i < feature_num; ++i) {
		
	    float tpos, tneg;
	    tpos = exp( (mat_at(pm,i,j)-mu_pos[i])*(mat_at(pm,i,j)-mu_pos[i]) / -(2.0f*sigma_pos[i]*sigma_pos[i]+1e-30) ) / (sigma_pos[i]+1e-30);
	    tneg = exp( (mat_at(pm,i,j)-mu_neg[i])*(mat_at(pm,i,j)-mu_neg[i]) / -(2.0f*sigma_neg[i]*sigma_neg[i]+1e-30) ) / (sigma_neg[i]+1e-30);
	    sum_ratio += log(tpos+1e-30) - log(tneg+1e-30);	// equation 4
	    /*
	      float tp = mat_at(pm, i, j) - mu_pos[i], tn = mat_at(pm, i, j) - mu_neg[i];
	      tp = (float)exp( pow2( tp ) / (-2.0f * pow2( sigma_pos[i] + FLT_EPSILON))) / (sigma_pos[i] + FLT_EPSILON);
	      tn = (float)exp( pow2( tn ) / (-2.0f * pow2( sigma_neg[i] + FLT_EPSILON))) / (sigma_neg[i] + FLT_EPSILON);
	      sum_ratio += (float)(log (tp) - log(tn));
	    */
	}
	if (rmax < sum_ratio) {
	    idx = j;
	    rmax = sum_ratio;
	}
    }
    *ratio_max_idx = idx;
    *ratio_max = rmax;
}

/**************************/

#define vec_set(vec, vec_len, value)		\
    do{						\
    	int _i;					\
	for(_i = 0; _i < (vec_len); ++_i)	\
	    (vec)[_i] = (value);		\
    } while(0)

struct ct_t *
ct_new() {
    struct ct_t * ct = (struct ct_t*)malloc(sizeof(struct ct_t));
    ct->outer_positive_radius = 4;
    ct->search_window_radius = 25;
    ct->feature_rect_num_min = 2;
    ct->feature_rect_num_max = 4;
    ct->feature_num = 50;
    ct->learning_rate = 0.85f;
    ct->mu_positive = (float *)malloc(sizeof(float) * ct->feature_num);
    ct->mu_negative = (float *)malloc(sizeof(float) * ct->feature_num);
    ct->sigma_positive = (float *)malloc(sizeof(float) * ct->feature_num);
    ct->sigma_negative = (float *)malloc(sizeof(float) * ct->feature_num);
    ct->detect_box   = rvec_new( 1000 );
    ct->positive_box = rvec_new( 1000 );
    ct->negative_box = rvec_new( 1000 );
    ct->features = 0;
    ct->integral_img = 0;
    ct->detect_values = mat_new(1,1);
    ct->positive_values = mat_new(1,1);
    ct->negative_values = mat_new(1,1);

    /* initialize */
    vec_set(ct->mu_positive, ct->feature_num, 0.0f);
    vec_set(ct->mu_negative, ct->feature_num, 0.0f);
    vec_set(ct->sigma_positive, ct->feature_num, 1.0f);
    vec_set(ct->sigma_negative, ct->feature_num, 1.0f);

    return ct;
}
#undef vec_set

void
ct_free(struct ct_t **ct) {
    struct ct_t * p;    
    if (!ct) return ;
    
    p = *ct;
    
#define _safe_free(f, p) f((p) == 0 ? 0 : &(p))    
    _safe_free(free, *p->mu_positive);
    _safe_free(free, *p->mu_negative);
    _safe_free(free, *p->sigma_positive);
    _safe_free(free, *p->sigma_negative);
    _safe_free(rvec_free, p->detect_box);
    _safe_free(rvec_free, p->positive_box);
    _safe_free(rvec_free, p->negative_box);
    _safe_free(wvec_free, p->features);
    _safe_free(mat_free, p->detect_values);
    _safe_free(mat_free, p->positive_values);
    _safe_free(mat_free, p->negative_values);
    _safe_free(cvReleaseImage, p->integral_img);
#undef _safe_free
    
    free(*ct);
    *ct = 0;
}

void
ct_init(struct ct_t *ct, const IplImage * frame, const CvRect *obj_box) {
    _ct_inithaar( ct, obj_box->width, obj_box->height );
    
    _ct_sampling_io(frame, obj_box, ct->outer_positive_radius, 0, 100000, ct->positive_box);
    _ct_sampling_io(frame, obj_box, ct->search_window_radius * 1.5f, ct->outer_positive_radius + 4, 100, ct->negative_box);

    ct->integral_img = cvCreateImage(cvSize(frame->width+1, frame->height+1), IPL_DEPTH_32F, 1);

    cvIntegral(frame, ct->integral_img, 0, 0);

    _ct_get_feature_value(ct, ct->positive_box, ct->positive_values);
    _ct_get_feature_value(ct, ct->negative_box, ct->negative_values);
    _ct_update_classifier(ct->positive_values, ct->mu_positive, ct->sigma_positive, ct->feature_num, ct->learning_rate);
    _ct_update_classifier(ct->negative_values, ct->mu_negative, ct->sigma_negative, ct->feature_num, ct->learning_rate);
}

void
ct_update(struct ct_t *ct, const IplImage * frame, CvRect *obj_box) {
    int ratio_max_idx;
    float ratio_max;
    CvRect *p;
    
    _ct_sampling(frame, obj_box, ct->search_window_radius, ct->detect_box);
    cvIntegral(frame, ct->integral_img, 0, 0);
    _ct_get_feature_value(ct, ct->detect_box, ct->detect_values);
    _ct_ratio_classifier(ct, &ratio_max_idx, &ratio_max);

    p = rvec_at(ct->detect_box, ratio_max_idx);
    obj_box->x = p->x;
    obj_box->y = p->y;
    obj_box->width  = p->width;
    obj_box->height = p->height;

    _ct_sampling_io(frame, obj_box, ct->outer_positive_radius, 0, 1000000, ct->positive_box);
    _ct_sampling_io(frame, obj_box, ct->search_window_radius * 1.5f, ct->outer_positive_radius + 4, 100, ct->negative_box);

    _ct_get_feature_value(ct, ct->positive_box, ct->positive_values);
    _ct_get_feature_value(ct, ct->negative_box, ct->negative_values);
    _ct_update_classifier(ct->positive_values, ct->mu_positive, ct->sigma_positive, ct->feature_num, ct->learning_rate);
    _ct_update_classifier(ct->negative_values, ct->mu_negative, ct->sigma_negative, ct->feature_num, ct->learning_rate);

}

#ifdef __cplusplus
extern "C" {
#endif 
