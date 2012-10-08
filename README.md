## Real-time compressive tracker, implemented in C.
====

### Usage
<p>
<code> struct ct_t * ct = ct_new(); /* create tracker's parameters */

CvRect obj_box;

gray = query_gray_frame();

ct_init(ct, gray, &obj_box); /* initialize by the bounding box in first frame */

while ( gray = query_gray_frame() ) {

      ct_update(ct, gray, &obj_box); /* update, the obj_box contains new boudning box*/

}

ct_free(&ct); /* free all resources */
</code>
</p>

### Note

* real-time, for at least 3 targets tracking, in a normal people's PC,
   and in my computer, it could track 10 targets in real time.
* easy to extend and hack.

I did not give an accurate check between this version and  the original version, If there's a bug, feel free to contact me or send a push request.

bugway@gmail.com

### Reference
[0] "Real-Time Compressive Tracking," Kaihua Zhang, Lei Zhang, Ming-Hsuan Yang, ECCV 2012