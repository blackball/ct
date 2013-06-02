#ifndef CT_H
#define CT_H

#ifdef __cplusplus
extern "C" {
#endif 

typedef struct _IplImage IplImage;
typedef struct CvRect CvRect;

struct ct_t;
struct ct_t * ct_new();
void ct_init(struct ct_t *ct, const IplImage *gray, const CvRect *obj_box);
void ct_update(struct ct_t *ct, const IplImage *gray, CvRect *obj_box);
void ct_free(struct ct_t **ct);

#ifdef __cplusplus
}
#endif 

#endif 
