#include "mex.h"

#include <jpeglib.h>
#include <memory.h>

/*
 * Note: libjpeg must be installed somewhere, say JPEG_PATH.
 * Command to compile this is mex jpeg_decompress.c -I<JPEG_PATH>/include -L<JPEG_PATH>/lib -ljpeg
 */

typedef unsigned char uint8_t;

static void
init_source(j_decompress_ptr cinfo) {}

static boolean
fill_input_buffer(j_decompress_ptr cinfo) {
    return TRUE;
}

static void
skip_input_data(j_decompress_ptr cinfo, long num_bytes) {
    cinfo->src->next_input_byte += num_bytes;
    cinfo->src->bytes_in_buffer -= num_bytes;
}

static void
term_source (j_decompress_ptr cinfo) {}

int do_decompress (const uint8_t * src, int src_size,
        uint8_t** dest, int* w, int* h) {
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    struct jpeg_source_mgr jsrc;
    
    int width;
    int height;

    cinfo.err = jpeg_std_error (&jerr);
    jpeg_create_decompress (&cinfo);

    jsrc.next_input_byte = src;
    jsrc.bytes_in_buffer = src_size;
    jsrc.init_source = init_source;
    jsrc.fill_input_buffer = fill_input_buffer;
    jsrc.skip_input_data = skip_input_data;
    jsrc.resync_to_restart = jpeg_resync_to_restart;
    jsrc.term_source = term_source;
    cinfo.src = &jsrc;

    jpeg_read_header (&cinfo, TRUE);
    cinfo.out_color_space = JCS_RGB;
    jpeg_start_decompress (&cinfo);

    width = cinfo.output_width;
    height = cinfo.output_height;
    *w = width;
    *h = height;
    
    *dest = (uint8_t*)malloc(width*height*3);
    
    while (cinfo.output_scanline < height) {
        uint8_t * row = *dest + cinfo.output_scanline * width * 3;
        jpeg_read_scanlines (&cinfo, &row, 1);
    }
    jpeg_finish_decompress (&cinfo);
    jpeg_destroy_decompress (&cinfo);
    return 0;
}

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  uint8_t *data = NULL;
  uint8_t *img = NULL;
  uint8_t *out = NULL;
  int i, j, k;
  int data_size;
  int dims[3];

  /* Check arguments */
  if (nrhs != 1) { 
    mexErrMsgTxt("One input argument required."); 
  } else if (nlhs > 1) {
    mexErrMsgTxt("Too many output arguments."); 
  } else if (!mxIsNumeric(prhs[0])) {
    mexErrMsgTxt("Argument must be numeric.");
  } else if (mxIsComplex(prhs[0])) {
    mexErrMsgTxt("Argument must be non-complex.");
  }

  data = (uint8_t*)mxGetData(prhs[0]);
  data_size = mxGetNumberOfElements(prhs[0]);

  do_decompress (data, data_size, &img, &(dims[1]), &(dims[0]));
  dims[2] = 3;
  
  /* create the output */
  plhs[0] = mxCreateNumericArray(3, dims, mxUINT8_CLASS, mxREAL);
  out = (uint8_t*)mxGetData(plhs[0]);
  
  for (i = 0; i < dims[0]; ++i) {
      for (j = 0; j < dims[1]; ++j) {
          for (k = 0; k < dims[2]; ++k) {
              out[dims[0]*dims[1]*k + dims[0]*j + i] = img[dims[1]*dims[2]*i + dims[2]*j + k];
          }
      }
  }
          
  free (img);
}
