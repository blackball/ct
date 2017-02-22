## Real-time compressive tracker, implemented in C.
====

### Usage
```c
struct ct_t * ct = ct_new(); /* create tracker's parameters */
CvRect obj_box;
gray = query_gray_frame();
ct_init(ct, gray, &obj_box); /* initialize by the bounding box in first frame */
while ( gray = query_gray_frame() ) {
      ct_update(ct, gray, &obj_box); /* update, the obj_box contains new boudning box*/
}
ct_free(&ct); /* free all resources */
```

### Note

* real-time, for tracking at least 5 objects.
* easy to extend and hack.

I didn't carefully align this version with the original version, if there's a bug, feel free to contact or send a push request.

### Reference
[0] "Real-Time Compressive Tracking," Kaihua Zhang, Lei Zhang, Ming-Hsuan Yang, ECCV 2012
